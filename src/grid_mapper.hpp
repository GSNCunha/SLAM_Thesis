#ifndef GRID_MAPPER_HPP
#define GRID_MAPPER_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

/**
 * @class GridMapper
 * @brief Implements the Inverse Sensor Model and Grid Line Traversal.
 * Based on OpenSLAM GMapping:
 * - Traversal Logic: 'gridlinetraversal.h' (Bresenham Algorithm)
 * - Update Logic: 'scanmatcher.cpp' (registerScan)
 */
class GridMapper {
public:
    GridMapper();

    /**
     * @brief Updates the occupancy grid based on a laser scan.
     * Modernized version of GMapping's 'ScanMatcher::registerScan'.
     */
    void updateMap(nav_msgs::msg::OccupancyGrid& map, 
                   const std::vector<float>& ranges,
                   double pose_x, double pose_y, double pose_theta);

private:
    // Log-Odds Parameters (Probabilistic Update)
    // Values roughly equivalent to GMapping defaults but adapted for integer grid
    int8_t prob_occ_inc_;  // Increment for occupied cells (Hit)
    int8_t prob_free_dec_; // Decrement for free cells (Miss)
    int8_t prob_max_;      // Clamping max (100)
    int8_t prob_min_;      // Clamping min (0)

    /**
     * @brief The Core Bresenham Algorithm.
     * Direct modernization of 'GridLineTraversal::gridLine' from OpenSLAM.
     * * @param x0, y0 Start point (Sensor position)
     * @param x1, y1 End point (Beam hit position)
     * @param hit True if the laser actually hit an obstacle at (x1,y1)
     */
    void traceLine(nav_msgs::msg::OccupancyGrid& map, 
                   int x0, int y0, int x1, int y1, bool hit);
};

#endif // GRID_MAPPER_HPP