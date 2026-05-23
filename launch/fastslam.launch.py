import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition

def generate_launch_description():
    # 1. A variável mestre que a IHM vai enviar ('real', 'gazebo' ou 'rosbag')
    mode_arg = LaunchConfiguration('robot_mode', default='real')

    # 2. Lógicas Dinâmicas
    # O tempo é simulado? Apenas no gazebo ou no rosbag.
    is_sim_time = PythonExpression(["'true' if '", mode_arg, "' in ['gazebo', 'rosbag'] else 'false'"])
    
    # Onde está o Lidar? -0.032 no gazebo, -0.15 no real e no rosbag.
    laser_offset = PythonExpression(["'-0.032' if '", mode_arg, "' == 'gazebo' else '-0.15'"])
    
    # Qual o giro do Lidar? 0.0 no gazebo, -90 graus (-1.5708 rad) no real e no rosbag.
    laser_yaw = PythonExpression(["'0.0' if '", mode_arg, "' == 'gazebo' else '-1.5708'"])
    
    # Precisamos das nossas pontes TF manuais? Sim, a não ser que estejamos no gazebo.
    run_manual_tfs = PythonExpression(["'false' if '", mode_arg, "' == 'gazebo' else 'true'"])

    # 3. Nó do C++
    fastslam_node = Node(
        package='fastslam_thesis',
        executable='fastslam_node',
        name='fastslam_node',
        output='screen',
        parameters=[{
            'use_sim_time': is_sim_time,
            'laser_offset_x': laser_offset,  
            'laser_offset_yaw': laser_yaw,  # <--- FALTAVA ISTO! Passando o giro para o C++
            'map_width': 400,   
            'map_height': 400,  
            'particle_count': LaunchConfiguration('particle_count'),
            'map_resolution': LaunchConfiguration('map_resolution'),
            'linear_update': LaunchConfiguration('linear_update'),
            'angular_update': LaunchConfiguration('angular_update'),
            'meas_z_hit': LaunchConfiguration('meas_z_hit'),
            'meas_z_rand': LaunchConfiguration('meas_z_rand'),
            'meas_sigma': LaunchConfiguration('meas_sigma'),
            'laser_max_range': LaunchConfiguration('laser_max_range')
        }]
    )
    
    # 4. Ponte do Lidar (Roda no Físico e no Rosbag)
    tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_base_to_laser',
        arguments=[
            '--x', '-0.15',       
            '--y', '0.0', 
            '--z', '0.10',
            '--yaw', '-1.5708',   
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_laser'
        ],
        condition=IfCondition(run_manual_tfs)
    )

    # 5. Ponte da Odometria (Criámos duas versões para respeitar o tempo de cada modo)
    odom_bridge_real = ExecuteProcess(
        cmd=['python3', 'odom_tf_bridge.py', '--ros-args', '-p', 'use_sim_time:=false'],
        output='screen',
        condition=IfCondition(PythonExpression(["'true' if '", mode_arg, "' == 'real' else 'false'"]))
    )
    
    odom_bridge_bag = ExecuteProcess(
        cmd=['python3', 'odom_tf_bridge.py', '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        condition=IfCondition(PythonExpression(["'true' if '", mode_arg, "' == 'rosbag' else 'false'"]))
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_mode', default_value='real'),
        DeclareLaunchArgument('particle_count', default_value='50'),
        DeclareLaunchArgument('map_resolution', default_value='0.1'),
        DeclareLaunchArgument('linear_update', default_value='0.2'),
        DeclareLaunchArgument('angular_update', default_value='0.2'),
        DeclareLaunchArgument('meas_z_hit', default_value='0.95'),
        DeclareLaunchArgument('meas_z_rand', default_value='0.05'),
        DeclareLaunchArgument('meas_sigma', default_value='0.50'),
        DeclareLaunchArgument('laser_max_range', default_value='3.50'),
        
        fastslam_node,
        tf_laser_node,
        odom_bridge_real,
        odom_bridge_bag
    ])