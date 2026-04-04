import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Configuração do Nó FastSLAM
    fastslam_node = Node(
        package='fastslam_thesis', # O nome que está no package.xml
        executable='fastslam_node', # O nome definido no add_executable do CMake
        name='fastslam_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'particle_count': 300,      # 30 Partículas (leve para testar)
            'map_resolution': 0.05,    # 5cm por pixel
            'map_width': 400,          # 20x20 metros
            'map_height': 400,
            'linear_update': 0.2,      # Atualiza mapa a cada 20cm andados
            'angular_update': 0.2      # Atualiza mapa a cada ~11 graus girados
        }]
    )

    # Configuração do RViz (Visualizador)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
        # Dica: Depois que configurar o RViz a primeira vez, você pode salvar 
        # a config num arquivo .rviz e carregar automaticamente aqui.
    )

    return LaunchDescription([
        fastslam_node,
        rviz_node
    ])