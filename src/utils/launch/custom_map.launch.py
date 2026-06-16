# =============================================================================
# GAZEBO SIMULATION LAUNCH SCRIPT
# This launch file orchestrates the Gazebo simulation environment. It loads the 
# custom experimental arena, spawns the TurtleBot3 Burger URDF model at a 
# predefined starting pose, and initializes the robot state publisher for TF2.
# [See Section 3.2.1: Gazebo Simulator]
# [See Section 4.4.1: Environment Design and Structural Features]
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # =============================================================================
    # DIRECTORY CONFIGURATION
    # Retrieve the default installation directories for the Gazebo simulator 
    # and the TurtleBot3 packages.
    # =============================================================================
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    # =============================================================================
    # CUSTOM WORLD PATH
    # Points to the custom experimental arena map developed for the simulation.
    # [See Section 4.4.1: Environment Design and Structural Features]
    # =============================================================================
    current_dir = os.getcwd()
    world_path = os.path.join(current_dir, 'src', 'utils', 'worlds', 'gazebo_smaller_map.world')

    # =============================================================================
    # 1. START GAZEBO SERVER
    # Initializes the physics engine and loads the custom world file.
    # [See Section 3.2.1: Gazebo Simulator]
    # =============================================================================
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # =============================================================================
    # 2. START GAZEBO CLIENT (GUI)
    # Initializes the graphical interface for 3D visualization.
    # =============================================================================
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # =============================================================================
    # 3. ROBOT STATE PUBLISHER
    # Essential for broadcasting the static TF2 transforms defined in the URDF.
    # [See Section 3.1.2: Transformation System (TF2)]
    # [See Section 3.3: Robot Description (URDF)]
    # =============================================================================
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # =============================================================================
    # 4. SPAWN TURTLEBOT3 BURGER
    # Injects the robot model into the simulation at the predefined entrance 
    # coordinates of the experimental trajectory.
    # [See Section 4.4.2: Kinematic Path Planning and Trajectory Execution]
    # =============================================================================
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'x_pose': '-0.20', 
            'y_pose': '0.72', 
            'z_pose': '0.05'
            # Removed the yaw line, it will spawn perfectly facing +X
        }.items()
    )

    # =============================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # Aggregates all commands into a single execution sequence.
    # =============================================================================
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld