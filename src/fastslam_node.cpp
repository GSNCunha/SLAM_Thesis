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
        this->declare_parameter("particle_count", 50);     // Number of particles
        this->declare_parameter("map_resolution", 0.1);    // Map pixel size in m
        this->declare_parameter("map_width", 200);          // Map width
        this->declare_parameter("map_height", 200);         // Map height
        this->declare_parameter("linear_update", 0.05);     // Map is updated for every X m traveled
        this->declare_parameter("angular_update", 0.1);     // Map is updated for every X rad traveled
        // =============================================================================
        //                          SET MOVEMENT AND SENSING PARAMENTERS
        // MOTION MODEL INDEXES:
        // 1 -> alpha1 : Rotational error from rotational motion (turning variance)
        // 2 -> alpha2 : Rotational error from translational motion (drift while driving straight)
        // 3 -> alpha3 : Translational error from translational motion (distance variance)
        // 4 -> alpha4 : Translational error from rotational motion (displacement while turning)
        motion_model_ = std::make_unique<MotionModel>(0.05, 0.005, 0.05, 0.005); 
        // MEASUREMENT MODEL INDEXES:
        // 1 -> P(occupied) : Probability that a cell is occupied if the laser hits it (Black)
        // 2 -> P(free)     : Probability that a cell is occupied if the laser passes through (White)
        // 3 -> P(prior)    : Initial probability for unknown cells (Gray/Unexplored)
        // 4 -> Max Range   : Maximum effective range of the laser sensor in meters
        measurement_model_ = std::make_unique<MeasurementModel>(0.95, 0.05, 0.5, 3.5);
        // =============================================================================


        particle_count_ = this->get_parameter("particle_count").as_int();           // Get arguments
        update_dist_linear_ = this->get_parameter("linear_update").as_double();     // Get arguments
        update_dist_angular_ = this->get_parameter("angular_update").as_double();   // Get arguments


        grid_mapper_ = std::make_unique<GridMapper>();                              // Grid storage for the map
        
        initParticles();                                                            // Call for particles initiation

        rclcpp::QoS qos(10);                                                        // Size of buffer for receiving messages from topics

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(                         //scan_sub is receiving the information the subscription in the "/scan" topic, this subscription passes as argument to scanCallback the message received
            "/scan", qos, std::bind(&FastSlamNode::scanCallback, this, _1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(                             //odom_sub_is receiving the information the subscription in the "/odom" topic, this subscription passes as argument to odomCallback the message received
            "/odom", qos, std::bind(&FastSlamNode::odomCallback, this, _1));

        rclcpp::QoS map_qos(1);
        map_qos.best_effort();
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
                
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);                   //create an object to handle for handling rotation math (Quaternions)

        RCLCPP_INFO(this->get_logger(), "FastSLAM Node Starting with %d particles.", particle_count_); //print inicializzation
    }

private:

    std::vector<Particle> particles_;           // Vector of the typr Particle ( from particle.hpp) called particles_
    bool has_odom_ = false;                     // Auxiliary variable for knowing if there is a last odom state 
    StampedPose2D last_update_odom_;            // Struct from motion model for the last odom state(it contains time and pose)
    StampedPose2D current_odom_pose_;           // Struct from motion model for the current odom state(it contains time and pose)

    int particle_count_;                        // Number of particles
    double update_dist_linear_;                 // Map is updated for every X m traveled
    double update_dist_angular_;                // Map is updated for every X rad traveled

    std::unique_ptr<MotionModel> motion_model_;               // Instanciation of the class motion model
    std::unique_ptr<MeasurementModel> measurement_model_;     // Instanciation of the class measurement model 
    std::unique_ptr<GridMapper> grid_mapper_;                 // Instanciation of the class grid mapper

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;         // Subscription to the topic /Scan
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;             // Subscription to the topic /Odom
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;            // Publisher to the topic /Map
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;     // Publisher to the topic /Particles
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;                 // Object for tranforming  
    
    std::mutex data_mutex_;          // for managing access of the particle vector between the odomCallback and the scanCallback

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

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { // "/odom" topic callback(everytime something is published in the topic, this code runs)
        if (!has_odom_) return; //for the correction step, the measurement model needs at least one odom information first

        std::lock_guard<std::mutex> lock(data_mutex_); // Locking the access of particle for this code to use 

        double dx = current_odom_pose_.x_ - last_update_odom_.x_;                      //  For everytime an scan is received, an delta X is calculated
        double dy = current_odom_pose_.y_ - last_update_odom_.y_;                      //  For everytime an scan is received, an delta Y is calculated
        double dth = std::abs(current_odom_pose_.theta_ - last_update_odom_.theta_);   //  For everytime an scan is received, an delta angle is calculated 
        if (dth > M_PI) dth = 2*M_PI - dth;                                            //  Normalization of the angle

        double dist_sq = dx*dx + dy*dy;                                                // total linear pose change

        if (dist_sq < (update_dist_linear_*update_dist_linear_) && dth < update_dist_angular_) {        //if the pose change is not big enough, no further calculation is done
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

            double w = measurement_model_->computeWeight(   //Corection Step, using the measurement model, estimate the weight or likelihood of one particle being correct using the measurements, map and position change
                filtered_ranges, 
                p.x, 
                p.y, 
                p.theta, 
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
            grid_mapper_->updateMap(
                p.map, 
                filtered_ranges, 
                p.x, 
                p.y, 
                p.theta, 
                current_angle_min,  
                current_increment   
            );
        }

        last_update_odom_ = current_odom_pose_;     // Update the last pose 

        publishResults(msg->header.stamp);                          // Getting the best particle and publishing its information
    }

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

    void publishResults(rclcpp::Time current_time) {
        const auto& best_p = particles_[0];         

        static int publish_counter = 0;
        publish_counter++;

        if (publish_counter >= 10) { 
            
            auto map_msg = best_p.map;                  
            map_msg.header.stamp = current_time;    // <--- AQUI (Relógio Sincronizado)     
            map_msg.header.frame_id = "map";            
            map_pub_->publish(map_msg);                 

            geometry_msgs::msg::PoseArray poses_msg;    
            poses_msg.header.stamp = current_time;  // <--- AQUI (Relógio Sincronizado)     
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
        tf_msg.header.stamp = current_time;         // <--- AQUI (Relógio Sincronizado)           
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
