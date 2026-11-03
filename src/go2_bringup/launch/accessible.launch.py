"""Launch the repaired, simplified three-floor scene and its matching map."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('go2_bringup')
    worlds = get_package_share_directory('robot_sim_worlds')
    map_file = os.path.abspath(os.path.join(bringup, '../../../../tools/output/building_accessible_5cm.ply'))
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gz_args', default_value='-v 3'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
            launch_arguments={
                'navigation': 'true', 'use_rviz': LaunchConfiguration('use_rviz'),
                'gz_args': LaunchConfiguration('gz_args'),
                'world': os.path.join(worlds, 'worlds', 'building_accessible.sdf'),
                'world_name': 'building_accessible', 'map_file': map_file,
            }.items()),
    ])
