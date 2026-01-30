#include "grid_mapper.hpp"

GridMapper::GridMapper() 
{
    // GMapping usa log-odds complexos, mas no final converte para probabilidade.
    // No ROS, o grid é int8 [0..100].
    // Ajustamos os incrementos para simular o comportamento do Log-Odds:
    // Uma célula precisa de algumas leituras para virar obstáculo confirmado.
    prob_occ_inc_ = 25;  // Aumenta probabilidade rapidamente se bater
    prob_free_dec_ = 10; // Diminui mais devagar se passar livre
    prob_max_ = 100;
    prob_min_ = 0;
}

// Modernized version of ScanMatcher::registerScan
void GridMapper::updateMap(nav_msgs::msg::OccupancyGrid& map, 
                           const std::vector<float>& ranges,
                           double pose_x, double pose_y, double pose_theta) 
{
    double angle_increment = (2.0 * M_PI) / ranges.size();
    
    // Map metadata
    double res = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    int width = map.info.width;
    int height = map.info.height;

    // Robot Position in Grid (Start Point p0)
    int x0 = static_cast<int>((pose_x - origin_x) / res);
    int y0 = static_cast<int>((pose_y - origin_y) / res);

    // Loop over all beams
    for (size_t i = 0; i < ranges.size(); i++) {
        float r = ranges[i];

        // Validations
        if (std::isnan(r) || r < 0.05) continue;
        
        // Max Range Handling (GMapping: m_usableRange)
        // Se r > max, consideramos livre até o max, mas não marcamos obstáculo no fim.
        double max_range = 10.0; // Exemplo de usableRange
        bool hit = true;
        
        if (r >= max_range) {
            r = max_range;
            hit = false; // Não desenha obstáculo no final, apenas espaço livre
        }

        // Beam Endpoint (End Point p1)
        double angle = pose_theta + (i * angle_increment);
        double hit_x = pose_x + r * cos(angle);
        double hit_y = pose_y + r * sin(angle);

        int x1 = static_cast<int>((hit_x - origin_x) / res);
        int y1 = static_cast<int>((hit_y - origin_y) / res);

        // Check bounds before tracing
        if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < height && 
            x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) 
        {
            traceLine(map, x0, y0, x1, y1, hit);
        }
    }
}

// Modernized version of GridLineTraversal::gridLine
// Uses standard Bresenham integer arithmetic
void GridMapper::traceLine(nav_msgs::msg::OccupancyGrid& map, 
                           int x0, int y0, int x1, int y1, bool hit) 
{
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    int width = map.info.width;

    while (true) {
        // 1. Calculate Index
        int idx = y * width + x;

        // 2. Inverse Sensor Model Logic (GMapping Core)
        // Se chegamos no ponto final (x1, y1)
        if (x == x1 && y == y1) {
            if (hit) {
                // OCCUPIED UPDATE
                int val = map.data[idx];
                if (val == -1) val = 50; // Unknown starts at 50 (0.5)
                
                val += prob_occ_inc_;
                if (val > prob_max_) val = prob_max_;
                
                map.data[idx] = static_cast<int8_t>(val);
            }
            // If !hit (max range), we do nothing at the endpoint (or mark free, depending on config)
            break; // End of line
        } 
        else {
            // FREE SPACE UPDATE (Ray passing through)
            int val = map.data[idx];
            if (val == -1) val = 50;
            
            val -= prob_free_dec_;
            if (val < prob_min_) val = prob_min_;
            
            map.data[idx] = static_cast<int8_t>(val);
        }

        // 3. Bresenham Step
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}