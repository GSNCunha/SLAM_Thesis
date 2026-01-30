#include "measurement_model.hpp"
#include <limits>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MeasurementModel::MeasurementModel(double z_hit, double z_rand, double sigma_hit, double laser_max_range)
    : z_hit_(z_hit), 
      z_rand_(z_rand), 
      sigma_hit_(sigma_hit), 
      laser_max_range_(laser_max_range),
      beam_skip_(5), // Process only 1 out of 5 beams for real-time performance
      kernel_size_(1) // 3x3 Local Window Search (GMapping default)
{
}

double MeasurementModel::computeWeight(const std::vector<float>& ranges, 
                                       double pose_x, 
                                       double pose_y, 
                                       double pose_theta,
                                       const nav_msgs::msg::OccupancyGrid& map) 
{
    double log_weight = 0.0;
    
    // Map metadata
    double resolution = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    int width = map.info.width;
    int height = map.info.height;

    // Pre-compute angle increment (assuming 360 laser distributed evenly)
    // In a real ROS node, you should get this from sensor_msgs/LaserScan message
    double angle_increment = (2.0 * M_PI) / ranges.size();
    
    // Constant part for Uniform Distribution (p_rand)
    // TCC Equation 13: z_rand * p_rand
    double p_rand_component = z_rand_ * (1.0 / laser_max_range_);

    // Iterate over laser beams
    for (size_t i = 0; i < ranges.size(); i += beam_skip_) {
        float r = ranges[i];

        // 1. Filter invalid ranges
        if (std::isnan(r) || r >= laser_max_range_ || r < 0.1) {
            continue;
        }

        // 2. Compute Beam Endpoint (Hit Point) in Global Coordinates
        // TCC Section 2.2.1: Projecting the scan point
        double beam_angle = pose_theta + (i * angle_increment);
        
        // Normalize angle to [-PI, PI]
        while (beam_angle > M_PI) beam_angle -= 2.0 * M_PI;
        while (beam_angle < -M_PI) beam_angle += 2.0 * M_PI;

        double hit_x = pose_x + r * cos(beam_angle);
        double hit_y = pose_y + r * sin(beam_angle);

        // 3. Convert to Grid Coordinates (Indices)
        int gx = static_cast<int>((hit_x - origin_x) / resolution);
        int gy = static_cast<int>((hit_y - origin_y) / resolution);

        // 4. Local Neighborhood Search (The "GMapping Strategy")
        // Instead of a pre-computed distance map, we search for obstacles 
        // in a small window around the hit point.
        
        double min_dist_sq = std::numeric_limits<double>::max();
        bool obstacle_found = false;

        // Kernel Loop (e.g., from -1 to +1 for a 3x3 window)
        for (int dx = -kernel_size_; dx <= kernel_size_; ++dx) {
            for (int dy = -kernel_size_; dy <= kernel_size_; ++dy) {
                
                int nx = gx + dx;
                int ny = gy + dy;

                // Check bounds
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    // Check occupancy (Threshold > 50 means occupied)
                    int cell_value = map.data[ny * width + nx];
                    
                    if (cell_value > 50) { 
                        // Obstacle found! Calculate Euclidean distance squared.
                        // We convert the cell center back to world coordinates for accuracy.
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

        // 5. Calculate Probability (Likelihood Field)
        // TCC Equation 13: mixture of p_hit and p_rand
        
        double p_hit = 0.0;
        
        if (obstacle_found) {
            // Gaussian distribution: exp(-dist^2 / (2 * sigma^2))
            // Note: We ignore the normalization constant (1/sqrt(2pi...)) for performance 
            // as it is constant for all particles and cancels out during resampling normalization.
            p_hit = exp(-min_dist_sq / (2.0 * sigma_hit_ * sigma_hit_));
        }

        // Calculate total probability for this beam
        double prob = (z_hit_ * p_hit) + p_rand_component;

        // Accumulate Log-Likelihood (sum of logs instead of product of probabilities)
        // This avoids numerical underflow with many beams.
        if (prob > 0.0) {
            log_weight += log(prob);
        } else {
            // Very low probability penalty
            log_weight += -20.0; 
        }
    }

    // Return the final weight in linear scale (required for the Resampling step)
    return exp(log_weight);
}