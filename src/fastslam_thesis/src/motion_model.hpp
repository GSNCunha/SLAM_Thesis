/*
 * Copyright (C) 2026  Gabriel Sadigursky Nunes Cunha
 * Based on: "Probabilistic Robotics" (Thrun) & TCC "FastSLAM Algorithm in ROS 2".
 */

/*
 * =============================================================================
 * MOTION MODEL HEADER
 * This header defines the MotionModel class and the StampedPose2D structure. 
 * It is responsible for the odometry-based prediction step of the FastSLAM 
 * algorithm, applying the kinematic alpha parameters to model the uncertainty 
 * and drift of the physical robot's differential drive system.
 * [See Section 2.1.2: Motion Model and Odometry]
 * [See Section 4.3.1: The Core SLAM Node]
 * =============================================================================
 */

#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include <cmath>
#include <random>
#include <rclcpp/time.hpp>

// =============================================================================
// STAMPED POSE 2D STRUCTURE
// Represents the spatial state of a particle or odometry reading in a 2D 
// plane (x, y, theta), tightly coupled with a ROS 2 timestamp to ensure 
// precise temporal synchronization across the TF2 tree.
// =============================================================================
struct StampedPose2D {
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    rclcpp::Time timestamp_;
};

// =============================================================================
// MOTION MODEL CLASS DEFINITION
// Implements the probabilistic odometry motion model. It calculates the 
// relative movement (translation and rotation) and injects Gaussian noise 
// based on the tuned alpha variance parameters.
// [See Section 2.1.2: Motion Model and Odometry]
// =============================================================================
class MotionModel {
public:
    // =============================================================================
    // CONSTRUCTOR: KINEMATIC NOISE PARAMETERS
    // Initializes the alpha parameters that define the robot's physical drift 
    // profile, correlating rotational and translational movements to their 
    // respective uncertainty accumulation.
    // [See Section 2.1.2: Modeling Motion Uncertainty]
    // =============================================================================
    MotionModel(double alpha1, double alpha2, double alpha3, double alpha4);

    // =============================================================================
    // SAMPLE MOTION MODEL (PREDICTION STEP)
    // Computes the new state hypothesis for a single particle based on its 
    // previous pose and the robot's physical odometry displacement.
    // [See Section 2.1.2: Eq. 12 - Final odometry motion update]
    // =============================================================================
    StampedPose2D sampleMotionModel(const StampedPose2D& particle_pose, 
                                    const StampedPose2D& start_odom, 
                                    const StampedPose2D& end_odom);

    // =============================================================================
    // DYNAMIC SETTERS
    // Allows real-time parameter tuning of the odometry noise profile.
    // =============================================================================
    void setAlpha1(double a) { alpha1_ = a; }
    void setAlpha2(double a) { alpha2_ = a; }
    void setAlpha3(double a) { alpha3_ = a; }
    void setAlpha4(double a) { alpha4_ = a; }

private:
    // Kinematic uncertainty parameters (alpha1 to alpha4)
    double alpha1_, alpha2_, alpha3_, alpha4_;
    
    // Mersenne Twister Engine for robust pseudo-random number generation
    std::mt19937 gen_; 

    // =============================================================================
    // PRIVATE HELPER FUNCTIONS
    // =============================================================================
    // Helper to generate zero-mean Gaussian noise for the motion variance
    double sampleGaussian(double sigma_sq);
    
    // Helper to normalize angular values to remain within the [-PI, PI] range
    double normalize_angle(double angle);
};

#endif