/*
 * motion_model_validation.cpp
 * Program to generate data for the Probabilistic Motion Model validation.
 * Outputs a CSV file to be plotted by the Python script.
 */

#include <iostream>
#include <vector>
#include <fstream>  // To save CSV
#include <cmath>

// Ensure differential_motion_model.hpp is correctly located relative to this file
#include "../../motion_model.hpp" 

int main() {
    // 1. SETUP
    int num_particles = 100;
    
    // Alphas: [rot/rot, rot/trans, trans/trans, trans/rot]
    // Values set to 0.02 to represent a realistic "noisy" robot
    DifferentialMotionModel model(0.002, 0.002, 0.002, 0.002); 

    std::vector<Particle> particles(num_particles);
    // All start at the origin (0,0,0)
    
    // 2. DEFINE MOTION
    // Robot starts at (0,0,0) and moves to (2.0, 0.0, 0.0) -> 2 meters forward
    Pose odom_start = {0.0, 0.0, 0.0};
    Pose odom_end   = {2.0, 0.0, 0.0};

    std::cout << "[C++] Initializing Motion Model Validation...\n";
    std::cout << "[C++] Simulating motion: (0,0) -> (2,0)\n";
    std::cout << "[C++] Number of particles: " << num_particles << "\n";

    // 3. RUN UPDATE
    // This applies the probabilistic motion to all particles
    model.update(particles, odom_end, odom_start);

    // 4. SAVE DATA (CSV)
    // New filename specifically for motion model data
    std::string filename = "motion_model_data.csv";
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "[C++] Error: Could not open file " << filename << " for writing.\n";
        return 1;
    }

    file << "x,y,theta\n";
    for(const auto& p : particles) {
        file << p.pose.x << "," << p.pose.y << "," << p.pose.theta << "\n";
    }
    file.close();

    std::cout << "[C++] Success. Data saved to '" << filename << "'.\n";

    return 0;
}