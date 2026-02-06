#ifndef MEASUREMENT_MODEL_HPP
#define MEASUREMENT_MODEL_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <nav_msgs/msg/occupancy_grid.hpp>

/**
 * @class MeasurementModel
 * @brief Implements the Likelihood Field Model using a Local Search (Kernel) approach.
 * * This implementation is adapted from the GMapping algorithm (ScanMatching Score)
 * to work efficiently with dynamic Occupancy Grids in FastSLAM.
 * instead of pre-computing a full Distance Field (like AMCL), it performs 
 * an on-the-fly local neighborhood search for each laser beam.
 * * References:
 * 1. Thrun, S. et al. "Probabilistic Robotics", Chapter 6.4 (Likelihood Fields).
 * 2. Grisetti, G. et al. "Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters".
 */
class MeasurementModel {
public:
    /**
     * @brief Constructor with model parameters (TCC Section 2.2.2)
     * @param z_hit Weight for the correct measurement part (Gaussian).
     * @param z_rand Weight for random noise part.
     * @param sigma_hit Standard deviation of the measurement noise (meters).
     * @param laser_max_range Maximum range of the LiDAR sensor (meters).
     */
    MeasurementModel(double z_hit = 0.95, 
                     double z_rand = 0.05, 
                     double sigma_hit = 1, 
                     double laser_max_range = 3.5);

    /**
     * @brief Calculates the importance weight of a particle given a scan and its map.
     * * @param ranges Vector of LiDAR range measurements.
     * @param pose_x Particle's X position in the map frame.
     * @param pose_y Particle's Y position in the map frame.
     * @param pose_theta Particle's orientation (yaw) in the map frame.
     * @param map The Occupancy Grid map associated with this specific particle.
     * @return double The calculated weight (probability) in linear scale (not log).
     */
    double computeWeight(const std::vector<float>& ranges, 
                         double pose_x, 
                         double pose_y, 
                         double pose_theta,
                         const nav_msgs::msg::OccupancyGrid& map);

private:
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