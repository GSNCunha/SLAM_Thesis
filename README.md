# SLAM Thesis: FastSLAM Implementation in ROS 2

This repository contains the implementation of a FastSLAM 1.0 algorithm (Simultaneous Localization and Mapping) developed as part of a TCC (Trabalho de Conclusão de Curso) for a degree in Engineering. The project focuses on the implementation of a Rao-Blackwellized Particle Filter to estimate robot trajectories and Occupancy Grid Mapping for environment reconstruction.

## Overview

The system is built on ROS 2 Humble and features a hybrid architecture using C++ for performance-critical SLAM processing and Python for high-level path planning and simulation control. This project serves as the technical foundation for the study of probabilistic robotics and mapping algorithms presented in the author's TCC.

### Key Features
* FastSLAM 1.0 Algorithm: Implementation of the particle filter where each particle maintains its own map.
* Hybrid ROS 2 Package: Seamless integration of C++ nodes and Python scripts.
* Custom Motion Model: Validated odometry model with noise injection (Alpha parameters).
* Autonomous Path Following: Python-based node for executing rounded rectangular and square trajectories with starting-point compensation.

## Installation and Setup

### Prerequisites
* ROS 2 Humble
* Python 3.10+
* matplotlib, numpy, scipy
