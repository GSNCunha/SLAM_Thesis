/*
 * =============================================================================
 * GRID MAPPER HEADER
 * This header defines the GridMapper class, responsible for the Occupancy Grid 
 * Mapping phase of the FastSLAM algorithm. It declares the methods and variables 
 * necessary to implement the Inverse Sensor Model and the log-odds cell updates.
 * [See Section 2.5.1: Occupancy Grid Maps]
 * [See Section 4.3.1: The Core SLAM Node]
 * =============================================================================
 */

#ifndef GRID_MAPPER_HPP
#define GRID_MAPPER_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint> // Required for int8_t data type

// =============================================================================
// GRID MAPPER CLASS DEFINITION
// Implements the Inverse Sensor Model and Grid Line Traversal.
// Derived from foundational SLAM implementations (OpenSLAM GMapping) but 
// modernized for ROS 2. Manages the discrete probability states of the map.
// [See Section 2.5.1: Occupancy Grid Maps]
// =============================================================================
class GridMapper {
public:
    GridMapper();

    // =============================================================================
    // MAP UPDATE FUNCTION
    // Updates the occupancy grid based on a laser scan. Modernized version of 
    // classical scan matching routines, utilizing the physical geometry of the 
    // sensor to project the beams accurately.
    // [See Section 2.5.1: Occupancy Grid Maps]
    // =============================================================================
    void updateMap(nav_msgs::msg::OccupancyGrid& map, 
                   const std::vector<float>& ranges, 
                   double x, double y, double theta,
                   double angle_min, double angle_increment); 

private:
    // =============================================================================
    // LOG-ODDS PARAMETERS (PROBABILISTIC UPDATE)
    // These integer values simulate the recursive Bayesian addition of the log-odds 
    // technique, optimized for fast array manipulation in an integer grid.
    // [See Section 2.5.1: Eq. 15 and Eq. 16]
    // =============================================================================
    int8_t prob_occ_inc_;  // Increment for occupied cells (Hit)
    int8_t prob_free_dec_; // Decrement for free cells (Miss)
    int8_t prob_max_;      // Clamping max (100% Probability)
    int8_t prob_min_;      // Clamping min (0% Probability)

    // =============================================================================
    // BRESENHAM'S RAY TRACING
    // The Core Bresenham Algorithm logic used to trace the continuous path of a 
    // laser beam through the discrete grid cells.
    // =============================================================================
    void traceLine(nav_msgs::msg::OccupancyGrid& map, 
                   int x0, int y0, int x1, int y1, bool hit);
};

#endif // GRID_MAPPER_HPP