/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
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
     * * TCC Section 2.1.2 (Motion Model and Odometry):
     * "The variance of this noise is estimated with robot-specific error parameters...
     * [cite_start]They are alpha_1 through alpha_4"[cite: 186, 187, 188].
     * [cite_start]@param a1 (alpha_1): Relates rotational noise to rotational motion[cite: 190].
     * [cite_start]@param a2 (alpha_2): Relates rotational noise to translational motion[cite: 191].
     * [cite_start]@param a3 (alpha_3): Relates translational noise to translational motion[cite: 194].
     * [cite_start]@param a4 (alpha_4): Relates translational noise to rotational motion[cite: 195].
     */
    DifferentialMotionModel(double a1, double a2, double a3, double a4)
        : alpha1_(a1), alpha2_(a2), alpha3_(a3), alpha4_(a4) {
        
        std::random_device rd;
        gen_ = std::mt19937(rd());
    }

    /**
     * Updates particles based on the Odometry Motion Model.
     * * TCC Section 2.1.2: "The Odometry Motion Model... assumes that the true pose 
     * [cite_start]is distributed around the odometry-predicted pose"[cite: 180, 185].
     */
    void update(std::vector<Particle>& particles, const Pose& odom_new, const Pose& odom_old) {
        
        // --- 1. MOTION DECOMPOSITION ---
        
        // TCC Figure 3: "The odometry motion model decomposes relative motion into 
        [cite_start]// rotation-translation-rotation"[cite: 180, 184].
        
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

        // Stability adjustment for in-place rotation (avoids excessive noise when stationary)
        if (delta_trans < 0.01) {
            delta_rot1 = 0.0;
        }

        // Noise symmetry (forward/backward). 
        // Ensures noise is calculated by the magnitude of the turn, regardless of direction.
        double rot1_noise_param = std::min(std::abs(delta_rot1), std::abs(normalize_angle(delta_rot1 - M_PI)));
        double rot2_noise_param = std::min(std::abs(delta_rot2), std::abs(normalize_angle(delta_rot2 - M_PI)));

        // --- 2. NOISE VARIANCE CALCULATION ---
        
        // TCC Section 2.1.2: Calculates the standard deviation (sigma) for the normal distributions
        [cite_start]// based on the alphas defined in equations (9), (10), and (11)[cite: 205, 206, 207].
        
        [cite_start]// Related to Eq (9): alpha1 * rot + alpha2 * trans [cite: 205]
        double sigma_rot1 = std::sqrt(alpha1_ * std::pow(rot1_noise_param, 2) + alpha2_ * std::pow(delta_trans, 2));
        
        [cite_start]// Related to Eq (10): alpha3 * trans + alpha4 * (rot1 + rot2) [cite: 206]
        double sigma_trans = std::sqrt(alpha3_ * std::pow(delta_trans, 2) + alpha4_ * std::pow(rot1_noise_param, 2) + alpha4_ * std::pow(rot2_noise_param, 2));
        
        [cite_start]// Related to Eq (11): alpha1 * rot2 + alpha2 * trans [cite: 207]
        double sigma_rot2 = std::sqrt(alpha1_ * std::pow(rot2_noise_param, 2) + alpha2_ * std::pow(delta_trans, 2));

        // Generates Gaussian distributions centered at 0 with the deviations calculated above
        [cite_start]// TCC: "sample(b) function generates a random number from a zero-mean distribution" [cite: 210]
        std::normal_distribution<double> dist_rot1(0.0, sigma_rot1);
        std::normal_distribution<double> dist_trans(0.0, sigma_trans);
        std::normal_distribution<double> dist_rot2(0.0, sigma_rot2);

        // --- 3. SAMPLING AND PARTICLE UPDATE ---
        
        // TCC Section 2.4.1 (MCL - Prediction Step):
        [cite_start]// "This step creates a new 'cloud' of M predicted particles... incorporating the uncertainty"[cite: 364].
        
        for (auto& p : particles) {
            // Sampling of the "hat" variables
            // The dist(gen_) functions act as the "sample()" function described in the equations.
            
            [cite_start]// TCC Equation (9): delta_rot1_hat = delta_rot1 - sample(...) [cite: 205]
            double delta_rot1_hat = delta_rot1 - dist_rot1(gen_);
            
            [cite_start]// TCC Equation (10): delta_trans_hat = delta_trans - sample(...) [cite: 206]
            double delta_trans_hat = delta_trans - dist_trans(gen_);
            
            [cite_start]// TCC Equation (11): delta_rot2_hat = delta_rot2 - sample(...) [cite: 207]
            double delta_rot2_hat = delta_rot2 - dist_rot2(gen_);

            // Applying noisy motion to the previous pose (x_{t-1}) to obtain (x_t)
            [cite_start]// TCC Equation (12): Final update of x', y', theta' [cite: 216]
            
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