import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the default installation directories for Gazebo and TurtleBot
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    # THE TRICK: Point to your map in the current working directory
    world_path = os.path.join(os.getcwd(), 'gazebo_smaller_map.world')

    # 1. Start the Gazebo server with YOUR map
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # 2. Start the Gazebo graphical interface (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # 3. Start the robot state publisher (Essential for sensors and TF to work)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 4. Spawn the TurtleBot Burger at the entrance of your hexagon
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'x_pose': '-0.7', 
            'y_pose': '0.0', 
            'z_pose': '0.05'
        }.items()
    )

    # Put it all together and launch
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld