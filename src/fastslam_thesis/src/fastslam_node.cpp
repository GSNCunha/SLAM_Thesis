/*
 * =============================================================================
 * FASTSLAM CORE EXECUTION NODE
 * This file serves as the central orchestrator for the FastSLAM algorithm in ROS 2.
 * It manages the particle cloud, synchronizes the simulated or physical sensor inputs 
 * (/odom and /scan), and executes the recursive Bayesian filter steps: Prediction, 
 * Correction, and Mapping. Finally, it handles the publication of the Occupancy Grid 
 * and the continuous TF2 spatial transforms to correct odometry drift.
 * [See Section 4.3.1: The Core SLAM Node]
 * =============================================================================
 */

#include <rclcpp/rclcpp.hpp>                            //ROS2 main library
#include <sensor_msgs/msg/laser_scan.hpp>               //ROS2 library for reading the laser scan
#include <nav_msgs/msg/odometry.hpp>                    //ROS2 library for reading the odometry
#include <nav_msgs/msg/occupancy_grid.hpp>              //ROS2 library for publishing the occupancy grid (Map)
#include <geometry_msgs/msg/pose_array.hpp>             //ROS2 library for publishing the particles
#include <geometry_msgs/msg/transform_stamped.hpp>      //ROS2 library for fixing the robot odometry based on SLAM in one specific timestamp
#include <tf2_ros/transform_broadcaster.h>              //ROS2 Tool to broadcast teh transform_stamped message 
#include <tf2/LinearMath/Quaternion.h>                  //ROS2 Class for handling rotation math (Quaternions) avoiding gimbal lock (change of reference depending in the robot pose)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>      //ROS2 Tools to convert between TF2 (Quaternions) math types and ROS geometry messages

#include <vector>                                       //Standard dynamic array, used to store the list of particles
#include <cmath>                                        //Standard math functions (sin, cos, atan2, sqrt) for motion models
#include <algorithm>                                    //Standard algorithms, used for sorting particles or finding max weights
#include <random>                                       //Random number generation for resampling and adding noise (Monte Carlo)
#include <mutex>                                        //Mutual exclusion for thread-safe data access between callbacks
#include <limits>                                       //Used to handle special values like NaN or Infinity in sensor data


#include "particle.hpp"                                 //particle filter include
#include "motion_model.hpp"                             //motion model include
#include "measurement_model.hpp"                        //measurement model include
#include "grid_mapper.hpp"                              //grid mapper include

using std::placeholders::_1;                            //this line is telling the code to save in std::placeholders::_1; when _1 is written

class FastSlamNode : public rclcpp::Node {                  // ROS2 Node class 
public:                                                     
    FastSlamNode() : Node("fastslam_node") {
        // =============================================================================
        //                           SET FASTSLAM PARAMETERS
        // Configures the baseline values for the Particle Filter distribution and the 
        // logical boundaries of the Occupancy Grid Map.
        // [See Section 2.4.1: Particle Filters (Monte Carlo Localization - MCL)]
        // [See Section 2.5.1: Occupancy Grid Maps]
        this->declare_parameter("particle_count", 300);     // Number of particles
        this->declare_parameter("map_resolution", 0.05);    // Map pixel size in m
        this->declare_parameter("map_width", 400);         // Map width
        this->declare_parameter("map_height", 400);        // Map height
        this->declare_parameter("linear_update", 0.05);    // Map is updated for every X m traveled
        this->declare_parameter("angular_update", 0.1);    // Map is updated for every X rad traveled
        // =============================================================================
        //                          SET MOVEMENT AND SENSING PARAMENTERS
        
        // --- ADDED: PHYSICAL SENSOR OFFSET ---
        this->declare_parameter("laser_offset_x", 0.0);
        this->declare_parameter("laser_offset_yaw", 0.0);
        laser_offset_x_ = this->get_parameter("laser_offset_x").as_double();
        laser_offset_yaw_ = this->get_parameter("laser_offset_yaw").as_double();
        RCLCPP_INFO(this->get_logger(), "Laser Offset: X=%.3fm, Yaw=%.3frad", laser_offset_x_, laser_offset_yaw_);

        // MOTION MODEL INDEXES:
        // These alpha parameters shape the expected uncertainty cloud (variance) of the robot's 
        // predicted poses. They are crucial to accommodate wheel slippage and real-world drift.
        // [See Section 2.1.2: Motion Model and Odometry]
        // 1 -> alpha1 : Rotational error from rotational motion (turning variance)
        // 2 -> alpha2 : Rotational error from translational motion (drift while driving straight)
        // 3 -> alpha3 : Translational error from translational motion (distance variance)
        // 4 -> alpha4 : Translational error from rotational motion (displacement while turning)
        this->declare_parameter("alpha1", 0.5);
        this->declare_parameter("alpha2", 0.5);
        this->declare_parameter("alpha3", 0.05);
        this->declare_parameter("alpha4", 0.005);
        
        motion_model_ = std::make_unique<MotionModel>(
            this->get_parameter("alpha1").as_double(),
            this->get_parameter("alpha2").as_double(),
            this->get_parameter("alpha3").as_double(),
            this->get_parameter("alpha4").as_double()
        ); 

        // MEASUREMENT MODEL INDEXES:
        // Represents the composite probability distribution components of the Likelihood Field Model.
        // It weights measurement noise (z_hit) against unexplained random noise (z_rand).
        // [See Section 2.2.2: Measurement Model: Likelihood field model]
        // 1 -> P(occupied) : Probability that a cell is occupied if the laser hits it (Black)
        // 2 -> P(free)     : Probability that a cell is occupied if the laser passes through (White)
        // 3 -> P(prior)    : Initial probability for unknown cells (Gray/Unexplored)
        // 4 -> Max Range   : Maximum effective range of the laser sensor in meters
        this->declare_parameter("meas_z_hit", 0.95);
        this->declare_parameter("meas_z_rand", 0.05);
        this->declare_parameter("meas_sigma", 0.5);
        this->declare_parameter("laser_max_range", 3.5);
        this->declare_parameter("beam_skip", 5);
        this->declare_parameter("kernel_size", 1);
        
        measurement_model_ = std::make_unique<MeasurementModel>(
            this->get_parameter("meas_z_hit").as_double(),
            this->get_parameter("meas_z_rand").as_double(),
            this->get_parameter("meas_sigma").as_double(),
            this->get_parameter("laser_max_range").as_double()
        );

        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                
                for (const auto &param : parameters) {
                    if (param.get_name() == "beam_skip") {
                        this->measurement_model_->setBeamSkip(param.as_int());
                    } else if (param.get_name() == "kernel_size") {
                        this->measurement_model_->setKernelSize(param.as_int());
                    } else if (param.get_name() == "alpha1") {
                        this->motion_model_->setAlpha1(param.as_double());
                    } else if (param.get_name() == "alpha2") {
                        this->motion_model_->setAlpha2(param.as_double());
                    } else if (param.get_name() == "alpha3") {
                        this->motion_model_->setAlpha3(param.as_double());
                    } else if (param.get_name() == "alpha4") {
                        this->motion_model_->setAlpha4(param.as_double());
                    }
                }
                return result;
            });
        // =============================================================================

        particle_count_ = this->get_parameter("particle_count").as_int();           // Get arguments
        update_dist_linear_ = this->get_parameter("linear_update").as_double();     // Get arguments
        update_dist_angular_ = this->get_parameter("angular_update").as_double();   // Get arguments

        grid_mapper_ = std::make_unique<GridMapper>();                              // Grid storage for the map
        
        initParticles();                                                            // Call for particles initiation

        rclcpp::QoS qos(10);                                                        // Size of buffer for receiving messages from topics

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(         //scan_sub is receiving the information the subscription in the "/scan" topic, this subscription passes as argument to scanCallback the message received
            "/scan", qos, std::bind(&FastSlamNode::scanCallback, this, _1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(             //odom_sub_is receiving the information the subscription in the "/odom" topic, this subscription passes as argument to odomCallback the message received
            "/odom", qos, std::bind(&FastSlamNode::odomCallback, this, _1));

        // FIX 1: Correct QoS configuration for RViz to accept the map!
        rclcpp::QoS map_qos(1);
        map_qos.reliable();
        map_qos.transient_local();
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
                
        // FIX 2: The Segfault Killer! Creating the particles publisher
        particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particles", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);                   //create an object to handle for handling rotation math (Quaternions)

        RCLCPP_INFO(this->get_logger(), "FastSLAM Node Starting with %d particles.", particle_count_); //print inicializzation
    }

private:

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    std::vector<Particle> particles_;           // Vector of the typr Particle ( from particle.hpp) called particles_
    bool has_odom_ = false;                     // Auxiliary variable for knowing if there is a last odom state 
    StampedPose2D last_update_odom_;            // Struct from motion model for the last odom state(it contains time and pose)
    StampedPose2D current_odom_pose_;           // Struct from motion model for the current odom state(it contains time and pose)

    int particle_count_;                        // Number of particles
    double update_dist_linear_;                 // Map is updated for every X m traveled
    double update_dist_angular_;                // Map is updated for every X rad traveled

    // --- ADDED: PHYSICAL SENSOR OFFSET ---
    double laser_offset_x_;                     // Physical displacement in X
    double laser_offset_yaw_;                   // Physical rotation on its own axis (Yaw)

    std::unique_ptr<MotionModel> motion_model_;               // Instanciation of the class motion model
    std::unique_ptr<MeasurementModel> measurement_model_;     // Instanciation of the class measurement model 
    std::unique_ptr<GridMapper> grid_mapper_;                 // Instanciation of the class grid mapper

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;         // Subscription to the topic /Scan
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;             // Subscription to the topic /Odom
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;            // Publisher to the topic /Map
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;     // Publisher to the topic /Particles
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;                 // Object for tranforming  
    
    std::mutex data_mutex_;          // for managing access of the particle vector between the odomCallback and the scanCallback

    // =============================================================================
    // INITIALIZE PARTICLES
    // Allocates the memory for the required number of particles. Each particle 
    // receives an empty log-odds occupancy grid array representing its independent map.
    // [See Section 2.5.3: SLAM Approaches]
    // =============================================================================
    void initParticles() {
        particles_.resize(particle_count_);                                                             // resize vector with declared particle count 
        
        nav_msgs::msg::OccupancyGrid empty_map;                                                         // creating empty map
        empty_map.header.frame_id = "map";                                                              // setting map ID
        empty_map.info.resolution = this->get_parameter("map_resolution").as_double();                  // settig map resolution 
        empty_map.info.width = this->get_parameter("map_width").as_int();                               // setting map width
        empty_map.info.height = this->get_parameter("map_height").as_int();                             // setting map height    
        
        empty_map.info.origin.position.x = -(empty_map.info.width * empty_map.info.resolution) / 2.0;       // setting initial position of each particle to be in the middle
        empty_map.info.origin.position.y = -(empty_map.info.height * empty_map.info.resolution) / 2.0;      // setting initial position of each particle to be in the middle
        empty_map.info.origin.orientation.w = 1.0;                                                          // setting initial position of each particle to be in the middle
        
        //std::fill(empty_map.data.begin(), empty_map.data.end(), -1);                    
        empty_map.data.assign(empty_map.info.width * empty_map.info.height, -1);        //set all cells to -1 in the initial map for each particle (empty)

        for (auto& p : particles_) {                    //setting initial values for each particle
            p.x = 0.0;
            p.y = 0.0;
            p.theta = 0.0;
            p.weight = 1.0 / particle_count_;
            p.map = empty_map;
        }
    }

    // =============================================================================
    // ODOMETRY CALLBACK
    // Retrieves the dynamic kinematic updates (control variable u_t). Used later in 
    // the prediction step to estimate how far the robot moved.
    // [See Section 4.3.1: The Core SLAM Node]
    // =============================================================================
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {       // "/odom" topic callback(everytime something is published in the topic, this code runs)
        std::lock_guard<std::mutex> lock(data_mutex_);                      // Locking the access of particle for this code to use 
        
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);         // Siny o the yaw angle takem from the Quatérnion
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);     // Cosy o the yaw angle takem from the Quatérnion
        double yaw = std::atan2(siny_cosp, cosy_cosp);  // Tan of the yaw angle

        
        current_odom_pose_.x_ = msg->pose.pose.position.x;     // Coordinate X of the robot current pose, given by "/odom"
        current_odom_pose_.y_ = msg->pose.pose.position.y;     // Coordinate Y of the robot current pose, given by "/odom"
        current_odom_pose_.theta_ = yaw;                       // Yaw of the robot current pose, given by "/odom" Quatérnion
        current_odom_pose_.timestamp_ = msg->header.stamp;     // Timestamp of the robot, given by "/odom"

        if (!has_odom_) {       // Just if it is runnig for the first time, the ododmetry received is declared also as the old one, 
            last_update_odom_ = current_odom_pose_;
            has_odom_ = true;
            RCLCPP_INFO(this->get_logger(), "first odometry received!");
        }
    }

    // =============================================================================
    // LIDAR SCAN CALLBACK (MAIN SLAM PIPELINE)
    // Synchronizes the /scan data (z_t) and triggers the recursive Bayes filter loop:
    // 1. Prediction (via MotionModel)
    // 2. Correction (via MeasurementModel)
    // 3. Mapping (via GridMapper)
    // 4. Resampling 
    // [See Section 2.5.3: SLAM Approaches]
    // =============================================================================
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { // "/odom" topic callback(everytime something is published in the topic, this code runs)
        if (!has_odom_) return; //for the correction step, the measurement model needs at least one odom information first

        std::lock_guard<std::mutex> lock(data_mutex_); // Locking the access of particle for this code to use 

        double dx = current_odom_pose_.x_ - last_update_odom_.x_;                      //  For everytime an scan is received, an delta X is calculated
        double dy = current_odom_pose_.y_ - last_update_odom_.y_;                      //  For everytime an scan is received, an delta Y is calculated
        double dth = std::abs(current_odom_pose_.theta_ - last_update_odom_.theta_);   //  For everytime an scan is received, an delta angle is calculated 
        if (dth > M_PI) dth = 2*M_PI - dth;                                            //  Normalization of the angle

        double dist_sq = dx*dx + dy*dy;                                                // total linear pose change

        if (dist_sq < (update_dist_linear_*update_dist_linear_) && dth < update_dist_angular_) {        
            // FIX 1: Maintains the TF Tree and Map alive in RViz even with the robot stopped!
            publishResults(msg->header.stamp); 
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Executing FastSLAM correction step");

        
        std::vector<float> filtered_ranges = msg->ranges;           // get the ranges from "/scan" messages.
        float range_max = msg->range_max;                           // get the max range from "/scan" messages to filter p_max


        double current_angle_min = msg->angle_min;                  // get the angle min from "/scan" messages
        double current_increment = msg->angle_increment;            // get the delta angle for each laser beam

        for (auto& r : filtered_ranges) {                          // For each laser beam, do this changes to the filtered_range variable...
            if (std::isinf(r) || std::isnan(r)) {
                r = std::numeric_limits<float>::quiet_NaN();       // if the laser beam received is infinite or NaN, ignore this iteration of the for oop and go for the next laser beam
                continue;
            }

            if (r >= (range_max * 0.98)) {                         // filter measures to near from the max range, could be a false or not accurate measurement
                r = range_max; 
            }
            if (r < 0.15) {
                r = std::numeric_limits<float>::quiet_NaN();       // filter measures to near from the robot, could be a false or not accurate measurement, can be also a p_short
            }
        }

        double total_weight = 0.0;                                 //initial weight for an particle           

        for (auto& p : particles_) {                               // For each particle, do this changes to the particles_ variable...

            StampedPose2D p_pose_struct;                                                                                               //Pose struct from Measurement model
            p_pose_struct.x_ = p.x; p_pose_struct.y_ = p.y; p_pose_struct.theta_ = p.theta;                                            // Get the pose information from each particle
            
            StampedPose2D new_pose = motion_model_->sampleMotionModel(p_pose_struct, last_update_odom_, current_odom_pose_);           // Prediction Step, using the motion model, simulate for each particle the new position based on its initial location and the robot position change 
            p.x = new_pose.x_;              // Get new particle pose
            p.y = new_pose.y_;              // Get new particle pose
            p.theta = new_pose.theta_;      // Get new particle pose

            // --- ADDED: APPLICATION OF PHYSICAL SENSOR OFFSET ---
            // Calculates the real Lidar position in space, based on the particle pose
            double laser_x = p.x + (laser_offset_x_ * cos(p.theta));
            double laser_y = p.y + (laser_offset_x_ * sin(p.theta));
            double laser_theta = p.theta + laser_offset_yaw_;

            double w = measurement_model_->computeWeight(   //Corection Step, using the measurement model, estimate the weight or likelihood of one particle being correct using the measurements, map and position change
                filtered_ranges, 
                laser_x,        // <-- ADDED: Passes Lidar X instead of p.x
                laser_y,        // <-- ADDED: Passes Lidar Y instead of p.y
                laser_theta,    // <-- ADDED: Passes Lidar Angle instead of p.theta
                p.map, 
                current_angle_min,
                current_increment  
            );
            
            p.weight = w;           //partice weight
            total_weight += w;      //used for normalization
        }


        normalizeWeights(total_weight);         // Normaization of weights ( all the weights together needs to sum up to 1)
        resampleParticles();                    // Resampling Based on each paricle weight 

        for (auto& p : particles_) {            //  Update the map for each particle
            
            // --- ADDED: APPLICATION OF PHYSICAL SENSOR OFFSET ---
            double laser_x = p.x + (laser_offset_x_ * cos(p.theta));
            double laser_y = p.y + (laser_offset_x_ * sin(p.theta));
            double laser_theta = p.theta + laser_offset_yaw_;

            grid_mapper_->updateMap(
                p.map, 
                filtered_ranges, 
                laser_x,        // <-- ADDED: Passes Lidar X
                laser_y,        // <-- ADDED: Passes Lidar Y
                laser_theta,    // <-- ADDED: Passes Lidar Angle
                current_angle_min,  
                current_increment   
            );
        }

        last_update_odom_ = current_odom_pose_;     // Update the last pose 

        publishResults(msg->header.stamp);                          // Getting the best particle and publishing its information
    }

    // =============================================================================
    // WEIGHT NORMALIZATION
    // Ensures that the sum of all particle weights equals 1.0, treating the cloud
    // as a valid probabilistic distribution before the resampling mechanism is triggered.
    // [See Section 2.4.1: Particle Filters (Monte Carlo Localization - MCL)]
    // =============================================================================
    void normalizeWeights(double total_weight) {   // Normaization of weights ( all the weights together need to sum up to 1)
        if (total_weight > 0.0) {
            for (auto& p : particles_) {
                p.weight /= total_weight;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Weights are reset!");
            for (auto& p : particles_) p.weight = 1.0 / particle_count_;
        }
    }

    // =============================================================================
    // RESAMPLE PARTICLES
    // Selects the fittest particles via adaptive resampling based on their likelihood 
    // weights. High-probability particles are duplicated; low-probability ones vanish.
    // [See Section 2.4.1: Step 3: Resampling (Survival of the Fittest)]
    // =============================================================================
    void resampleParticles() {                      // Resample particles based on their weight 
        std::vector<Particle> new_particles;        // Create new particle vector
        new_particles.reserve(particle_count_);     // Set the size of the vector 
        
        double r = ((double)rand() / (RAND_MAX)) * (1.0 / particle_count_);     //  Initial random value for slight change in resampling
        double c = particles_[0].weight;                                        //  Initial weight
        int i = 0;
        /*
            A comparison particle period is created with the same interval 1/300 summed with the random variable.
            The old particles are divided in intervals dependending in their weight.
            If both compared side by side, the old and the comparison particle period, there is a huge probability of several comparison particle period being within an old particle interval if that old
            particle had a high weight. This is the comparison used to know which particles to resample.
        */
        for (int m = 0; m < particle_count_; m++) {                             // For each new particle...
            double u = r + m * (1.0 / particle_count_);                         
            while (u > c) {                                                     // This new particle is not within the period of the old particle                
                i = (i + 1) % particle_count_;                                  // This old particle is ignored for this iteration and the coparison passes to the next old particle
                c += particles_[i].weight;                                                  
            }
            new_particles.push_back(particles_[i]);                             // This old particle is resampled because the comparison particle period is within its period
            new_particles.back().weight = 1.0 / particle_count_;                // Its weight is reset
        }
        particles_ = new_particles;                                             // Get new particles to the original vector
    }

    // =============================================================================
    // PUBLISH RESULTS AND SPATIAL TRANSFORMS
    // Isolates the particle with the highest weight (best guess), extracts its generated
    // occupancy grid to be rendered in RViz2, and computes the dynamic map-to-odom TF2 
    // correction branch to offset odometry drift.
    // [See Section 3.1.2: Transformation System (TF2)]
    // =============================================================================
    void publishResults(rclcpp::Time current_time) {
        const auto& best_p = particles_[0];         

        static int publish_counter = 0;
        publish_counter++;

        if (publish_counter >= 10) { 
            
            auto map_msg = best_p.map;                  
            map_msg.header.stamp = current_time;    // <--- HERE (Synchronized Clock)     
            map_msg.header.frame_id = "map";            
            map_pub_->publish(map_msg);                 

            geometry_msgs::msg::PoseArray poses_msg;    
            poses_msg.header.stamp = current_time;  // <--- HERE (Synchronized Clock)     
            poses_msg.header.frame_id = "map";          
            for(const auto& p : particles_) {           
                geometry_msgs::msg::Pose pose;
                pose.position.x = p.x;
                pose.position.y = p.y;
                tf2::Quaternion q;
                q.setRPY(0, 0, p.theta);
                pose.orientation = tf2::toMsg(q);
                poses_msg.poses.push_back(pose);
            }
            particles_pub_->publish(poses_msg);         

            publish_counter = 0; 
        }

        geometry_msgs::msg::TransformStamped tf_msg; 
        tf_msg.header.stamp = current_time;         // <--- HERE (Synchronized Clock)           
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

        tf2::Transform t_map_odom = t_map_base * t_odom_base.inverse(); 

        tf_msg.transform.translation.x = t_map_odom.getOrigin().x();        
        tf_msg.transform.translation.y = t_map_odom.getOrigin().y();        
        tf_msg.transform.translation.z = 0.0;                               
        tf_msg.transform.rotation = tf2::toMsg(t_map_odom.getRotation());   

        tf_broadcaster_->sendTransform(tf_msg);                             
    }
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                       // Initialize ROS 2 communication
    auto node = std::make_shared<FastSlamNode>();   // Create the FastSLAM node instance
    rclcpp::spin(node);                             // Keep node alive and wait for callbacks (infinite loop)
    rclcpp::shutdown();                             // Clean ROS 2 shutdown (when Ctrl+C is pressed)
    return 0;                                       // Exit program
}