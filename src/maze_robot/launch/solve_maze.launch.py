import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('maze_robot')
    nav2_dir = get_package_share_directory('nav2_bringup')

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'spawn.launch.py'))
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'slam.launch.py'))
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'rviz.launch.py'))
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': os.path.join(pkg_dir, 'config', 'maze_params.yaml')
        }.items()
    )

    solver_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='maze_robot',
                executable='maze_solver.py',
                name='maze_solver',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        spawn_launch,
        slam_launch,
        nav2_launch,
        rviz_launch,
        solver_node
    ])
