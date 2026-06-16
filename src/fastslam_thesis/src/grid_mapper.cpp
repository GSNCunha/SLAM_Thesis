/*
 * =============================================================================
 * GRID MAPPER (OCCUPANCY GRID MAPPING)
 * This file implements the mapping phase for each individual particle. It uses 
 * the laser scan data (z_t) and the particle's estimated pose (x_t) to update 
 * its specific spatial representation. The algorithm employs a simulated 
 * log-odds approach (Inverse Sensor Model) and Bresenham's line-tracing 
 * algorithm to correctly classify grid cells as free space, occupied boundaries, 
 * or unknown territory.
 * [See Section 2.5.1: Occupancy Grid Maps]
 * [See Section 4.3.1: The Core SLAM Node]
 * =============================================================================
 */

#include "grid_mapper.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GridMapper::GridMapper() 
{
    // =============================================================================
    // CONSTRUCTOR: LOG-ODDS SIMULATION PARAMETERS
    // Configures the increment and decrement steps for the occupancy probability.
    // This simulates the recursive log-odds addition described in the theoretical 
    // framework, preventing complex Bayesian multiplications during real-time execution.
    // [See Section 2.5.1: Occupancy Grid Maps, Eq. 15]
    // =============================================================================
    prob_occ_inc_ = 25;  
    prob_free_dec_ = 10; 
    prob_max_ = 100;
    prob_min_ = 0;
}

// =============================================================================
// UPDATE MAP (INVERSE SENSOR MODEL)
// Executes the spatial state update for the given particle's map. 
// Incorporates the real physical geometry of the LiDAR sensor (offset/yaw) 
// and projects each laser beam into the global coordinate frame.
// [See Section 2.5.1: Occupancy Grid Maps]
// =============================================================================
void GridMapper::updateMap(nav_msgs::msg::OccupancyGrid& map, 
                           const std::vector<float>& ranges,
                           double pose_x, 
                           double pose_y, 
                           double pose_theta,
                           double angle_min,       
                           double angle_increment) 
{
    // Extract occupancy grid metadata variables
    double res = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    int width = map.info.width;
    int height = map.info.height;

    // Calculate the starting discrete grid cell (p0) based on the sensor's origin pose
    int x0 = static_cast<int>((pose_x - origin_x) / res);
    int y0 = static_cast<int>((pose_y - origin_y) / res);

    // Iterate over the complete array of LiDAR beams
    for (size_t i = 0; i < ranges.size(); i++) {
        float r = ranges[i];

        // Discard anomalous sensor values (NaN) and measurements strictly too close to the sensor
        if (std::isnan(r) || r < 0.05) continue;
        
        // Threshold constraint to cap infinite or out-of-bounds laser readings
        double max_range = 10.0; 
        bool hit = true;
        
        if (r >= max_range) {
            r = max_range;
            hit = false; 
        }

        // Determine the absolute global angle of the specific laser beam
        double angle = pose_theta + angle_min + (i * angle_increment);
        
        // Normalize the angular value to remain within the [-PI, PI] operational range
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;

        // Compute the continuous spatial coordinates where the laser intersected an object
        double hit_x = pose_x + r * cos(angle);
        double hit_y = pose_y + r * sin(angle);

        // Convert the continuous coordinates into discrete grid indices (p1)
        int x1 = static_cast<int>((hit_x - origin_x) / res);
        int y1 = static_cast<int>((hit_y - origin_y) / res);

        // =============================================================================
        // BRESENHAM'S RAY TRACING & PROBABILITY UPDATE
        // Traces the trajectory of the laser beam from the sensor origin to the hit point.
        // Applies the inverse sensor model logic: decreasing the occupancy probability 
        // for cells along the path (free space) and increasing it for the final hit 
        // cell (obstacle).
        // =============================================================================
        // Structural optimization: Prevents memory segmentation faults by ensuring 
        // the ray tracing only occurs strictly within the map boundaries.
        if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < height && 
            x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) 
        {
            // Core variables for Bresenham's line algorithm
            int dx = std::abs(x1 - x0);
            int dy = std::abs(y1 - y0);
            int sx = (x0 < x1) ? 1 : -1;
            int sy = (y0 < y1) ? 1 : -1;
            int err = dx - dy;
            
            int x = x0;
            int y = y0;

            while (true) {
                int idx = y * width + x;

                // Determine cell status: Endpoint (Obstacle) vs. Midpoint (Free Space)
                if (x == x1 && y == y1) {
                    // Trajectory termination logic
                    if (hit) {
                        int val = map.data[idx];
                        if (val == -1) val = 50; // Initialize unexplored cell
                        val += prob_occ_inc_;
                        if (val > prob_max_) val = prob_max_;
                        map.data[idx] = static_cast<int8_t>(val);
                    }
                    break;
                } 
                else {
                    // Cells intersected by the beam are cleared
                    int val = map.data[idx];
                    if (val == -1) val = 50; // Initialize unexplored cell
                    val -= prob_free_dec_;
                    if (val < prob_min_) val = prob_min_;
                    map.data[idx] = static_cast<int8_t>(val);
                }

                // Iterative calculation for the next discrete coordinate in the line
                int e2 = 2 * err;
                if (e2 > -dy) {
                    err -= dy;
                    x += sx;
                }
                if (e2 < dx) {
                    err += dx;
                    y += sy;
                }
                
                // Fail-safe structural constraint to prevent infinite loops outside map memory
                if (x < 0 || x >= width || y < 0 || y >= height) break;
            }
        }
    }
}