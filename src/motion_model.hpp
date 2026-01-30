/*
 * Copyright (C) 2026  Gabriel Sadigursky Nunes Cunha
 * Based on: "Probabilistic Robotics" (Thrun) & TCC "FastSLAM Algorithm in ROS 2".
 */

#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include <cmath>
#include <random>
#include <rclcpp/time.hpp>

// Usamos uma estrutura compatível com o ROS Time para a odometria
struct StampedPose2D {
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    rclcpp::Time timestamp_;
};

class MotionModel {
public:
    /**
     * Constructor based on TCC Section 2.1.2.
     * @param alpha1 Rotational noise from rotational motion.
     * @param alpha2 Rotational noise from translational motion.
     * @param alpha3 Translational noise from translational motion.
     * @param alpha4 Translational noise from rotational motion.
     */
    MotionModel(double alpha1, double alpha2, double alpha3, double alpha4);

    /**
     * Updates a particle pose based on Odometry Motion Model.
     * Implements the sample_motion_model_odometry algorithm.
     */
    StampedPose2D sampleMotionModel(const StampedPose2D& particle_pose, 
                                    const StampedPose2D& start_odom, 
                                    const StampedPose2D& end_odom);

private:
    double alpha1_, alpha2_, alpha3_, alpha4_;
    std::mt19937 gen_; // Mersenne Twister Engine

    // Helper to sample from Gaussian
    double sampleGaussian(double sigma_sq);
    
    // Helper to normalize angles between -PI and PI
    double normalize_angle(double angle);
};

#endif