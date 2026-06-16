/*
 * =============================================================================
 * MEASUREMENT MODEL (LIKELIHOOD FIELD)
 * This file implements the correction step of the FastSLAM algorithm. It evaluates 
 * the likelihood of a particle's pose hypothesis by comparing the actual LiDAR 
 * measurements against the particle's internal occupancy grid map. It utilizes 
 * the Likelihood Field Model, incorporating a mixture of Gaussian measurement 
 * noise and uniform random noise to provide smooth and robust particle weighting.
 * [See Section 2.2.2: Measurement Model: Likelihood field model]
 * [See Section 4.3.1: The Core SLAM Node]
 * =============================================================================
 */

#include "measurement_model.hpp"
#include <limits>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================================================================
// CONSTRUCTOR: MIXTURE PARAMETERS & OPTIMIZATION
// Initializes the probabilistic weights for the likelihood field (z_hit, z_rand) 
// and the standard deviation (sigma_hit). It also sets optimization constants 
// like beam skipping to reduce computational load and the kernel size for the 
// nearest-obstacle search.
// [See Section 2.2.2: Measurement Model: Likelihood field model]
// =============================================================================
MeasurementModel::MeasurementModel(double z_hit, double z_rand, double sigma_hit, double laser_max_range)
    : z_hit_(z_hit), 
      z_rand_(z_rand), 
      sigma_hit_(sigma_hit), 
      laser_max_range_(laser_max_range),
      beam_skip_(5),  // Optimization: Process 1 out of every 5 laser beams
      kernel_size_(1) // Optimization: 3x3 local neighborhood search grid
{
}

// =============================================================================
// COMPUTE PARTICLE WEIGHT
// Calculates the importance weight of a single particle based on its spatial 
// correlation with the actual sensor data.
// [See Section 2.4.1: Step 2: Correction (Weighting)]
// =============================================================================
double MeasurementModel::computeWeight(const std::vector<float>& ranges, 
                                       double pose_x, 
                                       double pose_y, 
                                       double pose_theta,
                                       const nav_msgs::msg::OccupancyGrid& map,
                                       double angle_min,       
                                       double angle_increment) 
{
    double log_weight = 0.0;
    
    // Extract occupancy grid metadata variables
    double resolution = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    int width = map.info.width;
    int height = map.info.height;

    // Constant probability for completely random unexplained measurements (p_rand)
    // Modeled as a uniform distribution over the entire sensor range.
    // [See Section 2.2.2: Measurement Model, p_rand]
    double p_rand_component = z_rand_ * (1.0 / laser_max_range_);

    // Iterate over the laser beams applying the spatial downsampling (beam_skip)
    for (size_t i = 0; i < ranges.size(); i += beam_skip_) {
        float r = ranges[i];

        // 1. Filter invalid ranges (NaN, out-of-bounds, or hardware minimum distance)
        if (std::isnan(r) || r >= laser_max_range_ || r < 0.1) {
            continue;
        }

        // 2. Compute Beam Endpoint (Hit Point) in Global Coordinates
        // Calculates the absolute global angle of the individual laser beam,
        // combining the robot's heading and the sensor's physical sweep geometry.
        double beam_angle = pose_theta + angle_min + (i * angle_increment);
        
        // Normalize angular value to remain within the [-PI, PI] range
        while (beam_angle > M_PI) beam_angle -= 2.0 * M_PI;
        while (beam_angle < -M_PI) beam_angle += 2.0 * M_PI;

        // Compute continuous spatial coordinates of the physical hit
        double hit_x = pose_x + r * cos(beam_angle);
        double hit_y = pose_y + r * sin(beam_angle);

        // 3. Convert to Grid Coordinates (Indices)
        int gx = static_cast<int>((hit_x - origin_x) / resolution);
        int gy = static_cast<int>((hit_y - origin_y) / resolution);

        // 4. Local Neighborhood Search (GMapping Strategy)
        // Dynamically simulates the Likelihood Field by searching for the nearest 
        // occupied cell within a predefined window around the projected hit point.
        double min_dist_sq = std::numeric_limits<double>::max();
        bool obstacle_found = false;

        // Kernel Loop (e.g., from -1 to +1 for a 3x3 window mapping)
        for (int dx = -kernel_size_; dx <= kernel_size_; ++dx) {
            for (int dy = -kernel_size_; dy <= kernel_size_; ++dy) {
                
                int nx = gx + dx;
                int ny = gy + dy;

                // Ensure search index remains within allocated map boundaries
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    // Check occupancy (Log-odds threshold > 50 implies a solid object)
                    int cell_value = map.data[ny * width + nx];
                    
                    if (cell_value > 50) { 
                        // Obstacle found! Calculate Euclidean distance squared.
                        double cell_world_x = origin_x + (nx * resolution) + (resolution / 2.0);
                        double cell_world_y = origin_y + (ny * resolution) + (resolution / 2.0);

                        double dist_sq = (hit_x - cell_world_x) * (hit_x - cell_world_x) +
                                         (hit_y - cell_world_y) * (hit_y - cell_world_y);

                        if (dist_sq < min_dist_sq) {
                            min_dist_sq = dist_sq;
                        }
                        obstacle_found = true;
                    }
                }
            }
        }

        // 5. Calculate Probability (Likelihood Field p_hit)
        // Evaluates the probability of an accurate measurement using a zero-centered 
        // Gaussian distribution over the distance to the nearest mapped obstacle.
        // [See Section 2.2.2: Measurement Model: Likelihood field model]
        double p_hit = 0.0;
        
        if (obstacle_found) {
            // Gaussian distribution implementation
            p_hit = exp(-min_dist_sq / (2.0 * sigma_hit_ * sigma_hit_));
        }

        // Compute the composite mixture model probability for the current beam
        // [See Section 2.2.2: Eq. 13 - Final measurement likelihood]
        double prob = (z_hit_ * p_hit) + p_rand_component;

        // Accumulate Log-Likelihood to avoid numerical underflow errors during 
        // recursive multiplication of small probabilities.
        if (prob > 0.0) {
            log_weight += log(prob);
        } else {
            log_weight += -20.0; // Penalty for highly improbable measurements
        }
    }

    // Return the final importance weight converted back to a linear scale 
    // for the resampling step.
    return exp(log_weight);
}