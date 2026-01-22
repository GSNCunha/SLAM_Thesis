/*
 * Copyright (C) 2026  Gabriel Sadigursky Nunes Cunha
 *
 * This implementation is based on:
 * 1. Source Code: ROS 2 Navigation Stack (nav2_amcl) / Player Project.
 * 2. Textbook: "Probabilistic Robotics" (Thrun, Burgard, Fox, 2005).
 * 3. Thesis: "Development and Evaluation of a FastSLAM Algorithm in ROS 2" (UFRGS).
 */

#ifndef DIFFERENTIAL_MOTION_MODEL_HPP
#define DIFFERENTIAL_MOTION_MODEL_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

// Structure representing the robot's Pose (x, y, theta)
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// Particle Structure for FastSLAM
struct Particle {
    Pose pose;
    double weight = 0.0;
    // Future: OccupancyGridMap map;
};

class DifferentialMotionModel {
public:
    /**
     * Constructor that receives the motion noise parameters.
     * Based on TCC Section 2.1.2 (Motion Model and Odometry):
     * "The variance of this noise is estimated with robot-specific error parameters... They are alpha_1 through alpha_4".
     * @param a1 (alpha_1): "Relates rotational noise to rotational motion".
     * @param a2 (alpha_2): "Relates rotational noise to translational motion".
     * @param a3 (alpha_3): "Relates translational noise to translational motion".
     * @param a4 (alpha_4): "Relates translational noise to rotational motion".
     */
    DifferentialMotionModel(double a1, double a2, double a3, double a4)
        : alpha1_(a1), alpha2_(a2), alpha3_(a3), alpha4_(a4) {
        
        // Initializes the random number engine (Mersenne Twister).
        // Essential for the sampling step described in Section 2.1.2.
        std::random_device rd;
        gen_ = std::mt19937(rd());
    }

    /**
     * Updates particles based on the Odometry Motion Model.
     * Based on TCC Section 2.1.2: 
     * "The Odometry Motion Model... assumes that the true pose is distributed around the odometry-predicted pose".
     */
    void update(std::vector<Particle>& particles, const Pose& odom_new, const Pose& odom_old) {
        
        // --- 1. MOTION DECOMPOSITION ---
        
        // Based on TCC Figure 3 / Section 2.1.2: 
        // "The odometry motion model decomposes relative motion into rotation-translation-rotation".
        
        double dx = odom_new.x - odom_old.x;
        double dy = odom_new.y - odom_old.y;
        
        // delta_rot1 (initial rotation):
        double delta_rot1 = std::atan2(dy, dx) - odom_old.theta;
        delta_rot1 = normalize_angle(delta_rot1);

        // delta_trans (translation):
        double delta_trans = std::sqrt(dx * dx + dy * dy);

        // delta_rot2 (final rotation):
        double delta_rot2 = (odom_new.theta - odom_old.theta) - delta_rot1;
        delta_rot2 = normalize_angle(delta_rot2);

        // Stability adjustment for in-place rotation
        if (delta_trans < 0.01) {
            delta_rot1 = 0.0;
        }

        // Noise symmetry (forward/backward) logic
        double rot1_noise_param = std::min(std::abs(delta_rot1), std::abs(normalize_angle(delta_rot1 - M_PI)));
        double rot2_noise_param = std::min(std::abs(delta_rot2), std::abs(normalize_angle(delta_rot2 - M_PI)));

        // --- 2. NOISE VARIANCE CALCULATION ---
        
        // Based on TCC Section 2.1.2.
        // Calculates the standard deviation (sigma) based on Equations (9), (10), and (11).
        
        // Related to Eq (9): alpha1 * rot + alpha2 * trans
        double sigma_rot1 = std::sqrt(alpha1_ * std::pow(rot1_noise_param, 2) + alpha2_ * std::pow(delta_trans, 2));
        
        // Related to Eq (10): alpha3 * trans + alpha4 * (rot1 + rot2)
        double sigma_trans = std::sqrt(alpha3_ * std::pow(delta_trans, 2) + alpha4_ * std::pow(rot1_noise_param, 2) + alpha4_ * std::pow(rot2_noise_param, 2));
        
        // Related to Eq (11): alpha1 * rot2 + alpha2 * trans
        double sigma_rot2 = std::sqrt(alpha1_ * std::pow(rot2_noise_param, 2) + alpha2_ * std::pow(delta_trans, 2));

        // Generates Gaussian distributions centered at 0.
        // Based on TCC Section 2.1.2: "sample(b) function generates a random number from a zero-mean distribution".
        std::normal_distribution<double> dist_rot1(0.0, sigma_rot1);
        std::normal_distribution<double> dist_trans(0.0, sigma_trans);
        std::normal_distribution<double> dist_rot2(0.0, sigma_rot2);

        // --- 3. SAMPLING AND PARTICLE UPDATE ---
        
        // Based on TCC Section 2.4.1 (MCL - Prediction Step):
        // "This step creates a new 'cloud' of M predicted particles... incorporating the uncertainty".
        
        for (auto& p : particles) {
            // Sampling of the "hat" variables.
            // This implements the "sample()" function described in the equations.
            
            // Implements TCC Equation (9): delta_rot1_hat = delta_rot1 - sample(...)
            double delta_rot1_hat = delta_rot1 - dist_rot1(gen_);
            
            // Implements TCC Equation (10): delta_trans_hat = delta_trans - sample(...)
            double delta_trans_hat = delta_trans - dist_trans(gen_);
            
            // Implements TCC Equation (11): delta_rot2_hat = delta_rot2 - sample(...)
            double delta_rot2_hat = delta_rot2 - dist_rot2(gen_);

            // Applying noisy motion to the previous pose (x_{t-1}) to obtain (x_t)
            // Implements TCC Equation (12): Final update of x', y', theta'
            
            p.pose.x += delta_trans_hat * std::cos(p.pose.theta + delta_rot1_hat);
            p.pose.y += delta_trans_hat * std::sin(p.pose.theta + delta_rot1_hat);
            p.pose.theta += delta_rot1_hat + delta_rot2_hat;
            
            // Final normalization to keep the angle between -PI and PI
            p.pose.theta = normalize_angle(p.pose.theta);
        }
    }

private:
    double alpha1_, alpha2_, alpha3_, alpha4_;
    std::mt19937 gen_;

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

#endif