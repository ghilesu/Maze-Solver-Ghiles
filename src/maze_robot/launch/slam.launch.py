import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('maze_robot')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    # Use official online_async_launch.py which handles ROS 2 Jazzy Lifecycle activation automatically
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([slam_launch])
