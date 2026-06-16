/*
 * =============================================================================
 * MOTION MODEL (ODOMETRY PREDICTION)
 * This file implements the prediction step of the FastSLAM algorithm. It utilizes
 * the odometry motion model to estimate the robot's pose change over time. By 
 * injecting calibrated Gaussian noise based on the kinematic alpha parameters, 
 * it correctly propagates the uncertainty cloud of the particle distribution.
 * [See Section 2.1.2: Motion Model and Odometry]
 * [See Section 4.3.1: The Core SLAM Node]
 * =============================================================================
 */

#include "motion_model.hpp"

// =============================================================================
// CONSTRUCTOR: RANDOM NUMBER GENERATOR
// Initializes the Mersenne Twister pseudo-random number generator.
// Essential for generating the Gaussian noise applied during the particle 
// sampling step to represent real-world physical uncertainties.
// [See Section 2.1.2: Motion Model and Odometry]
// =============================================================================
MotionModel::MotionModel(double alpha1, double alpha2, double alpha3, double alpha4)
    : alpha1_(alpha1), alpha2_(alpha2), alpha3_(alpha3), alpha4_(alpha4) {
    
    std::random_device rd;
    gen_ = std::mt19937(rd());
}

// =============================================================================
// GAUSSIAN SAMPLING
// Generates a random number from a zero-mean normal distribution with a 
// specified variance. Used to corrupt the ideal odometry measurements.
// [See Section 2.1.2: Eqs. 9, 10, 11]
// =============================================================================
double MotionModel::sampleGaussian(double sigma_sq) {
    if (sigma_sq <= 0) return 0.0;
    std::normal_distribution<double> d(0.0, std::sqrt(sigma_sq));
    return d(gen_);
}

// =============================================================================
// ANGLE NORMALIZATION
// Normalizes an angular value to remain within the standard [-PI, PI] range.
// =============================================================================
double MotionModel::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// =============================================================================
// SAMPLE MOTION MODEL (PREDICTION STEP)
// Calculates the new hypothetical pose for a single particle. It decomposes 
// the relative odometry movement into rotation-translation-rotation, calculates 
// the variance based on the alpha parameters, samples the noise, and updates 
// the final pose.
// [See Section 2.1.2: Motion Model and Odometry]
// =============================================================================
StampedPose2D MotionModel::sampleMotionModel(const StampedPose2D& particle_pose, 
                                             const StampedPose2D& start_odom, 
                                             const StampedPose2D& end_odom) {
    
    // --- 1. MOTION DECOMPOSITION ---
    // Decomposes relative motion into rotation-translation-rotation.
    // [See Section 2.1.2: Figure 5.7 - Odometry Model]

    double dx = end_odom.x_ - start_odom.x_;
    double dy = end_odom.y_ - start_odom.y_;

    // delta_rot1 (initial rotation):
    double d_rot1 = std::atan2(dy, dx) - start_odom.theta_;
    d_rot1 = normalize_angle(d_rot1);

    // delta_trans (translation):
    double d_trans = std::sqrt(dx*dx + dy*dy);

    // delta_rot2 (final rotation):
    double d_rot2 = (end_odom.theta_ - start_odom.theta_) - d_rot1;
    d_rot2 = normalize_angle(d_rot2);

    // Stability adjustment for in-place rotation (Nav2 Logic)
    if (d_trans < 0.01) {
        d_rot1 = 0.0;
    }

    // Noise symmetry (forward/backward) logic (Nav2 Logic)
    // Ensures generic behavior even when reversing
    double rot1_param = std::min(std::abs(d_rot1), std::abs(normalize_angle(d_rot1 - M_PI)));
    double rot2_param = std::min(std::abs(d_rot2), std::abs(normalize_angle(d_rot2 - M_PI)));

    // --- 2. NOISE VARIANCE CALCULATION ---
    // Computes the variance for each motion component based on the tuned alpha parameters.
    // [See Section 2.1.2: Modeling Motion Uncertainty, Eqs. 9, 10, 11]

    double var_rot1  = alpha1_ * rot1_param * rot1_param + alpha2_ * d_trans * d_trans;
    double var_trans = alpha3_ * d_trans * d_trans + alpha4_ * rot1_param * rot1_param + alpha4_ * rot2_param * rot2_param;
    double var_rot2  = alpha1_ * rot2_param * rot2_param + alpha2_ * d_trans * d_trans;

    // --- 3. SAMPLING AND PARTICLE UPDATE ---
    // Calculates "hat" variables by adding sampled zero-mean Gaussian noise.
    
    double noisy_rot1  = d_rot1 - sampleGaussian(var_rot1);
    double noisy_trans = d_trans - sampleGaussian(var_trans);
    double noisy_rot2  = d_rot2 - sampleGaussian(var_rot2);

    // Applying noisy motion to the previous pose (x_{t-1})
    // Implements the final update of the hypothesis state (x', y', theta').
    // [See Section 2.1.2: Eq. 12 - Final odometry motion update]
    StampedPose2D new_pose;
    new_pose.x_ = particle_pose.x_ + noisy_trans * std::cos(particle_pose.theta_ + noisy_rot1);
    new_pose.y_ = particle_pose.y_ + noisy_trans * std::sin(particle_pose.theta_ + noisy_rot1);
    new_pose.theta_ = normalize_angle(particle_pose.theta_ + noisy_rot1 + noisy_rot2);
    
    new_pose.timestamp_ = end_odom.timestamp_;

    return new_pose;
}