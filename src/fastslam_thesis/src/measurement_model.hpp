/*
 * =============================================================================
 * MEASUREMENT MODEL HEADER
 * This header defines the MeasurementModel class, responsible for the 
 * correction step of the FastSLAM algorithm. It declares the methods and 
 * parameters required to implement the Likelihood Field Model, evaluating 
 * the spatial correlation between the LiDAR scans and the generated map.
 * [See Section 2.2.2: Measurement Model: Likelihood field model]
 * [See Section 4.3.1: The Core SLAM Node]
 * =============================================================================
 */

#ifndef MEASUREMENT_MODEL_HPP
#define MEASUREMENT_MODEL_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <nav_msgs/msg/occupancy_grid.hpp>

// =============================================================================
// MEASUREMENT MODEL CLASS DEFINITION
// Implements the Likelihood Field Model using a Local Search (Kernel) approach.
// Instead of pre-computing a full Distance Field, it performs an on-the-fly 
// local neighborhood search for each laser beam to dynamically accommodate 
// the independent Occupancy Grids of each FastSLAM particle.
// [See Section 2.2.2: Measurement Model: Likelihood field model]
// =============================================================================
class MeasurementModel {
public:
    // =============================================================================
    // CONSTRUCTOR: MIXTURE PARAMETERS
    // Configures the composite mixture model variables. It defines the weights 
    // for the correct measurement part (z_hit) and the random noise part (z_rand), 
    // alongside the standard deviation of the sensor (sigma_hit).
    // [See Section 2.2.2: Eq. 13 - Final measurement likelihood]
    // =============================================================================
    MeasurementModel(double z_hit = 0.95, 
                     double z_rand = 0.05, 
                     double sigma_hit = 1, 
                     double laser_max_range = 3.5);

    // =============================================================================
    // COMPUTE WEIGHT (LIKELIHOOD EVALUATION)
    // Calculates the importance weight of a particle by comparing the physical 
    // LiDAR ranges against its specific occupancy map. Returns the computed 
    // probability in a linear scale for the resampling phase.
    // [See Section 2.4.1: Step 2: Correction (Weighting)]
    // =============================================================================
    double computeWeight(const std::vector<float>& ranges, 
                         double pose_x, 
                         double pose_y, 
                         double pose_theta,
                         const nav_msgs::msg::OccupancyGrid& map,
                         double angle_min,      
                         double angle_increment  
                        );

    void setBeamSkip(int skip) { beam_skip_ = skip; }
    void setKernelSize(int size) { kernel_size_ = size; }

private:
    // =============================================================================
    // MEASUREMENT MODEL PARAMETERS & OPTIMIZATIONS
    // Variables storing the probability weights, the sensor's physical limits, 
    // and algorithm optimization settings (beam downsampling and search window).
    // =============================================================================
    double z_hit_;
    double z_rand_;
    double sigma_hit_;
    double laser_max_range_;

    // Optimization: Skip beams to increase performance (e.g., process 1 every 5 beams)
    int beam_skip_; 
    
    // Size of the local search window (Kernel size). 
    // 1 means 3x3 window, 2 means 5x5 window.
    int kernel_size_; 
};

#endif // MEASUREMENT_MODEL_HPP