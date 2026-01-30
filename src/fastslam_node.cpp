#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <mutex>

// INCLUDES LOCAIS (Aspas duplas = mesma pasta src/)
#include "particle.hpp"
#include "motion_model.hpp"
#include "measurement_model.hpp"
#include "grid_mapper.hpp"

using std::placeholders::_1;

/**
 * @class FastSlamNode
 * @brief Nó principal que orquestra o algoritmo FastSLAM 1.0.
 * * Arquitetura baseada no fluxo de processamento do 'GridSlamProcessor' do OpenSLAM GMapping:
 * 1. Prediction (Motion Model)
 * 2. Weighting (Measurement Model)
 * 3. Resampling (Low Variance Sampler)
 * 4. Map Update (Grid Mapping / Bresenham)
 */
class FastSlamNode : public rclcpp::Node {
public:
    FastSlamNode() : Node("fastslam_node") {
        // --- 1. Parâmetros do ROS ---
        this->declare_parameter("particle_count", 30); // 30 partículas para não sobrecarregar a CPU
        this->declare_parameter("map_resolution", 0.05); // 5 cm por pixel
        this->declare_parameter("map_width", 400);       // 20x20 metros
        this->declare_parameter("map_height", 400);
        this->declare_parameter("linear_update", 0.2);   // Atualizar a cada 20cm
        this->declare_parameter("angular_update", 0.2);  // Atualizar a cada ~11 graus

        particle_count_ = this->get_parameter("particle_count").as_int();
        update_dist_linear_ = this->get_parameter("linear_update").as_double();
        update_dist_angular_ = this->get_parameter("angular_update").as_double();
        
        // --- 2. Inicialização dos Componentes (Modern C++) ---
        
        // Motion Model (Baseado no Nav2 AMCL)
        // [alpha1, alpha2, alpha3, alpha4] -> Ruídos de rotação e translação
        motion_model_ = std::make_unique<MotionModel>(0.05, 0.05, 0.05, 0.05); 

        // Measurement Model (Baseado no GMapping Likelihood Field)
        // Parametros: z_hit, z_rand, sigma_hit, max_range
        measurement_model_ = std::make_unique<MeasurementModel>(0.95, 0.05, 0.2, 3.5);

        // Grid Mapper (Baseado no GMapping Bresenham)
        grid_mapper_ = std::make_unique<GridMapper>();

        // --- 3. Inicialização das Partículas ---
        initParticles();

        // --- 4. Configuração do ROS ---
        
        // QoS Profile (Quality of Service) para garantir compatibilidade
        rclcpp::QoS qos(10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&FastSlamNode::scanCallback, this, _1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos, std::bind(&FastSlamNode::odomCallback, this, _1));

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
        particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particles", 1);
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "FastSLAM Node Inicializado com %d particulas.", particle_count_);
    }

private:
    // --- Variáveis de Estado ---
    std::vector<Particle> particles_;
    
    // Controle de Odometria
    bool has_odom_ = false;
    StampedPose2D last_update_odom_;  // Odom na última vez que rodamos o filtro
    StampedPose2D current_odom_pose_; // Odom mais recente recebida

    // Parâmetros de Update
    int particle_count_;
    double update_dist_linear_;
    double update_dist_angular_;

    // Componentes Auxiliares
    std::unique_ptr<MotionModel> motion_model_;
    std::unique_ptr<MeasurementModel> measurement_model_;
    std::unique_ptr<GridMapper> grid_mapper_;

    // ROS Handles
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    std::mutex data_mutex_;

    // --- Funções Principais ---

    void initParticles() {
        particles_.resize(particle_count_);
        
        // Cria um mapa vazio padrão para todas as partículas
        nav_msgs::msg::OccupancyGrid empty_map;
        empty_map.header.frame_id = "map";
        empty_map.info.resolution = this->get_parameter("map_resolution").as_double();
        empty_map.info.width = this->get_parameter("map_width").as_int();
        empty_map.info.height = this->get_parameter("map_height").as_int();
        
        // Centraliza o mapa (0,0 no centro do grid)
        empty_map.info.origin.position.x = -(empty_map.info.width * empty_map.info.resolution) / 2.0;
        empty_map.info.origin.position.y = -(empty_map.info.height * empty_map.info.resolution) / 2.0;
        empty_map.info.origin.orientation.w = 1.0;
        
        // Preenche com -1 (Desconhecido)
        std::fill(empty_map.data.begin(), empty_map.data.end(), -1); 
        // Nota: se usar vector resize, ele preenche com 0, então forçamos -1 aqui se necessário, 
        // mas o construtor do OccupancyGrid já cria o vetor. Garantimos o tamanho:
        empty_map.data.assign(empty_map.info.width * empty_map.info.height, -1);

        for (auto& p : particles_) {
            p.x = 0.0;
            p.y = 0.0;
            p.theta = 0.0;
            p.weight = 1.0 / particle_count_;
            p.map = empty_map; // Cópia profunda (Deep Copy) do mapa vazio
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Extrai Yaw do Quaternion (Matemática básica 2D)
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        current_odom_pose_.x_ = msg->pose.pose.position.x;
        current_odom_pose_.y_ = msg->pose.pose.position.y;
        current_odom_pose_.theta_ = yaw;
        current_odom_pose_.timestamp_ = msg->header.stamp;

        if (!has_odom_) {
            last_update_odom_ = current_odom_pose_;
            has_odom_ = true;
            RCLCPP_INFO(this->get_logger(), "Primeira odometria recebida.");
        }
    }

    // O "Main Loop" do FastSLAM (Equivalente ao GridSlamProcessor::processScan)
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!has_odom_) return; 

        std::lock_guard<std::mutex> lock(data_mutex_);

        // 1. Verificação de Movimento (Keyframe Heuristic)
        // Só rodamos o filtro se o robô se moveu um pouco. Isso economiza CPU e evita "jitter".
        double dx = current_odom_pose_.x_ - last_update_odom_.x_;
        double dy = current_odom_pose_.y_ - last_update_odom_.y_;
        double dth = std::abs(current_odom_pose_.theta_ - last_update_odom_.theta_);
        if (dth > M_PI) dth = 2*M_PI - dth; // Normaliza diferença angular

        double dist_sq = dx*dx + dy*dy;

        if (dist_sq < (update_dist_linear_*update_dist_linear_) && dth < update_dist_angular_) {
            return; // Não moveu o suficiente
        }

        RCLCPP_DEBUG(this->get_logger(), "Executando FastSLAM Step...");

        double total_weight = 0.0;

        // Loop sobre todas as partículas (M partículas)
        for (auto& p : particles_) {
            // A. Passo de Predição (Motion Model - Nav2 Logic)
            StampedPose2D p_pose_struct; 
            p_pose_struct.x_ = p.x; p_pose_struct.y_ = p.y; p_pose_struct.theta_ = p.theta;
            
            // Amostra nova pose baseada no delta da odometria
            StampedPose2D new_pose = motion_model_->sampleMotionModel(p_pose_struct, last_update_odom_, current_odom_pose_);
            p.x = new_pose.x_;
            p.y = new_pose.y_;
            p.theta = new_pose.theta_;

            // B. Passo de Correção (Measurement Model - GMapping Logic)
            // Nota: FastSLAM 1.0 usa o mapa anterior da partícula para calcular o peso
            double w = measurement_model_->computeWeight(msg->ranges, p.x, p.y, p.theta, p.map);
            p.weight = w;
            total_weight += w;
        }

        // C. Normalização e Reamostragem (Low Variance Sampler)
        normalizeWeights(total_weight);
        resampleParticles();

        // D. Atualização do Mapa (Grid Mapper - Bresenham)
        // Agora que reamostramos, atualizamos o mapa das partículas "sobreviventes"
        for (auto& p : particles_) {
            grid_mapper_->updateMap(p.map, msg->ranges, p.x, p.y, p.theta);
        }

        // Atualiza referência de odometria
        last_update_odom_ = current_odom_pose_;

        // 3. Publicação (Visualização)
        publishResults();
    }

    void normalizeWeights(double total_weight) {
        if (total_weight > 0.0) {
            for (auto& p : particles_) {
                p.weight /= total_weight;
            }
        } else {
            // Recuperação de falha: se todos os pesos forem zero (scan muito ruim)
            RCLCPP_WARN(this->get_logger(), "Pesos zerados! Reiniciando uniformemente.");
            for (auto& p : particles_) p.weight = 1.0 / particle_count_;
        }
    }

    // Algoritmo "Low Variance Sampler" (Idêntico ao GMapping / Thrun Tabela 4.4)
    void resampleParticles() {
        std::vector<Particle> new_particles;
        new_particles.reserve(particle_count_);
        
        // Número aleatório entre 0 e 1/N
        double r = ((double)rand() / (RAND_MAX)) * (1.0 / particle_count_);
        double c = particles_[0].weight;
        int i = 0;

        for (int m = 0; m < particle_count_; m++) {
            double u = r + m * (1.0 / particle_count_);
            while (u > c) {
                i = (i + 1) % particle_count_;
                c += particles_[i].weight;
            }
            // AQUI OCORRE O CUSTO COMPUTACIONAL DO FASTSLAM:
            // Copiamos a partícula inteira (incluindo o mapa OccupancyGrid)
            new_particles.push_back(particles_[i]);
            
            // Reseta peso para uniforme após reamostragem
            new_particles.back().weight = 1.0 / particle_count_; 
        }
        particles_ = new_particles;
    }

    void publishResults() {
        // Estratégia simples: A melhor partícula é a primeira da lista reamostrada
        // (pois o resampling favorece as de maior peso)
        const auto& best_p = particles_[0];

        // 1. Publica o Mapa
        auto map_msg = best_p.map;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "map"; // O mapa é fixo no frame 'map'
        map_pub_->publish(map_msg);

        // 2. Publica as Partículas (Para ver nuvem no RViz)
        geometry_msgs::msg::PoseArray poses_msg;
        poses_msg.header.stamp = this->now();
        poses_msg.header.frame_id = "map";
        for(const auto& p : particles_) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            // Yaw to Quat
            tf2::Quaternion q;
            q.setRPY(0, 0, p.theta);
            pose.orientation = tf2::toMsg(q);
            poses_msg.poses.push_back(pose);
        }
        particles_pub_->publish(poses_msg);

        // 3. Publica Transformada TF (map -> odom)
        // Precisamos publicar onde o frame 'odom' está em relação ao frame 'map'.
        // T_map_odom = T_map_base * T_base_odom^(-1)
        // Simplificação 2D:
        // map_to_odom = best_particle_pose - current_odom_pose
        
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "odom";
        
        tf2::Transform t_map_base;
        t_map_base.setOrigin(tf2::Vector3(best_p.x, best_p.y, 0.0));
        tf2::Quaternion q_map_base;
        q_map_base.setRPY(0, 0, best_p.theta);
        t_map_base.setRotation(q_map_base);

        tf2::Transform t_odom_base;
        t_odom_base.setOrigin(tf2::Vector3(current_odom_pose_.x_, current_odom_pose_.y_, 0.0));
        tf2::Quaternion q_odom_base;
        q_odom_base.setRPY(0, 0, current_odom_pose_.theta_);
        t_odom_base.setRotation(q_odom_base);

        // T_map_odom = T_map_base * T_odom_base^(-1)
        tf2::Transform t_map_odom = t_map_base * t_odom_base.inverse();

        tf_msg.transform.translation.x = t_map_odom.getOrigin().x();
        tf_msg.transform.translation.y = t_map_odom.getOrigin().y();
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = tf2::toMsg(t_map_odom.getRotation());

        tf_broadcaster_->sendTransform(tf_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastSlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}