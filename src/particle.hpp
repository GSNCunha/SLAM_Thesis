#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

/**
 * @struct Particle
 * @brief Represents a FastSLAM particle containing a robot pose and its own map.
 * based on Thrun, Burgard, Fox - Probabilistic Robotics (Chapter 13).
 */
struct Particle {
    // 1. Robot Pose Hypothesis
    double x;
    double y;
    double theta;

    // 2. Importance Weight
    double weight;

    // 3. The Particle's Map (The key feature of FastSLAM)
    nav_msgs::msg::OccupancyGrid map;

    // Constructors
    Particle() : x(0.0), y(0.0), theta(0.0), weight(0.0) {}

    // Copy Constructor (Critical for Resampling Step)
    Particle(const Particle& other) {
        x = other.x;
        y = other.y;
        theta = other.theta;
        weight = other.weight;
        map = other.map; // Deep copy of the occupancy grid
    }
};

#endif