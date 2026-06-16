/*
 * =============================================================================
 * PARTICLE STRUCTURE HEADER
 * This header defines the Particle struct, the fundamental data unit of the 
 * FastSLAM algorithm. Unlike standard localization, each FastSLAM particle 
 * represents not only a specific hypothesis of the robot's state (pose), 
 * but also maintains its own independent occupancy grid map.
 * [See Section 2.4.1: Particle Filters (Monte Carlo Localization - MCL)]
 * [See Section 2.5.3: SLAM Approaches]
 * =============================================================================
 */

#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

// =============================================================================
// PARTICLE STRUCT DEFINITION
// Encapsulates a single hypothesis of the joint posterior distribution:
// the robot's pose [x, y, theta] and the map conditioned on that specific path.
// [See Section 2.5.3: Eq. 18 - FastSLAM Factorization]
// =============================================================================
struct Particle {
    // =============================================================================
    // ROBOT POSE HYPOTHESIS
    // The estimated spatial coordinates (x, y) and orientation (theta) of the robot 
    // in the global map frame.
    // =============================================================================
    double x;
    double y;
    double theta;

    // =============================================================================
    // IMPORTANCE WEIGHT
    // The likelihood of this particle's hypothesis being correct, evaluated by 
    // comparing sensor measurements against its internal map.
    // [See Section 2.4.1: Step 2: Correction (Weighting)]
    // =============================================================================
    double weight;

    // =============================================================================
    // THE PARTICLE'S MAP (FASTSLAM CORE FEATURE)
    // An independent spatial representation (Occupancy Grid) built exclusively 
    // from this particle's historical trajectory.
    // [See Section 2.5.3: SLAM Approaches]
    // =============================================================================
    nav_msgs::msg::OccupancyGrid map;

    // =============================================================================
    // DEFAULT CONSTRUCTOR
    // Initializes the hypothesis at the origin with zero weight.
    // =============================================================================
    Particle() : x(0.0), y(0.0), theta(0.0), weight(0.0) {}

    // =============================================================================
    // COPY CONSTRUCTOR (CRITICAL FOR RESAMPLING)
    // Ensures a deep copy of the high-dimensional occupancy grid when a highly 
    // probable particle is cloned during the "Survival of the Fittest" phase.
    // [See Section 2.4.1: Step 3: Resampling (Survival of the Fittest)]
    // =============================================================================
    Particle(const Particle& other) {
        x = other.x;
        y = other.y;
        theta = other.theta;
        weight = other.weight;
        map = other.map; // Deep copy of the occupancy grid
    }
};

#endif