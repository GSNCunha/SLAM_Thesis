#include "grid_mapper.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GridMapper::GridMapper() 
{
    // Ajustamos os incrementos para simular o comportamento do Log-Odds
    prob_occ_inc_ = 25;  
    prob_free_dec_ = 10; 
    prob_max_ = 100;
    prob_min_ = 0;
}

// ATENÇÃO: Assinatura atualizada para receber geometria do laser real
void GridMapper::updateMap(nav_msgs::msg::OccupancyGrid& map, 
                           const std::vector<float>& ranges,
                           double pose_x, 
                           double pose_y, 
                           double pose_theta,
                           double angle_min,       
                           double angle_increment) 
{
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
        
        // Max Range Handling
        double max_range = 10.0; 
        bool hit = true;
        
        if (r >= max_range) {
            r = max_range;
            hit = false; 
        }

        // === CORREÇÃO CRÍTICA AQUI ===
        // Ângulo Global = Theta Robô + (Inicio Laser + Incremento * Indice)
        double angle = pose_theta + angle_min + (i * angle_increment);
        
        // Normaliza ângulo (opcional, mas bom ter)
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;

        double hit_x = pose_x + r * cos(angle);
        double hit_y = pose_y + r * sin(angle);

        int x1 = static_cast<int>((hit_x - origin_x) / res);
        int y1 = static_cast<int>((hit_y - origin_y) / res);

        // Check bounds before tracing
        // Otimização: Só desenha se o robô E o alvo estiverem dentro do mapa (ou lógica de crop)
        if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < height && 
            x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) 
        {
            // Trace Line (Bresenham) from (x0,y0) to (x1,y1)
            int dx = std::abs(x1 - x0);
            int dy = std::abs(y1 - y0);
            int sx = (x0 < x1) ? 1 : -1;
            int sy = (y0 < y1) ? 1 : -1;
            int err = dx - dy;
            
            int x = x0;
            int y = y0;

            while (true) {
                int idx = y * width + x;

                // Lógica Inverse Sensor Model
                if (x == x1 && y == y1) {
                    // FIM DA LINHA (OBSTÁCULO ou MAX RANGE)
                    if (hit) {
                        int val = map.data[idx];
                        if (val == -1) val = 50; 
                        val += prob_occ_inc_;
                        if (val > prob_max_) val = prob_max_;
                        map.data[idx] = static_cast<int8_t>(val);
                    }
                    break;
                } 
                else {
                    // MEIO DA LINHA (LIVRE)
                    int val = map.data[idx];
                    if (val == -1) val = 50;
                    val -= prob_free_dec_;
                    if (val < prob_min_) val = prob_min_;
                    map.data[idx] = static_cast<int8_t>(val);
                }

                // Bresenham step
                int e2 = 2 * err;
                if (e2 > -dy) {
                    err -= dy;
                    x += sx;
                }
                if (e2 < dx) {
                    err += dx;
                    y += sy;
                }
                
                // Safety break (avoid infinite loops if logic fails)
                if (x < 0 || x >= width || y < 0 || y >= height) break;
            }
        }
    }
}