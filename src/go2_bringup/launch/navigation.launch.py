"""Run global A*, SCAN local avoidance and Go2 path tracking."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def navigation_nodes(context):
    share = get_package_share_directory('go2_bringup')
    params = os.path.join(share, 'config', 'jie_navigation.yaml')
    rviz_config = os.path.join(share, 'config', 'navigation.rviz')
    pcd_path = LaunchConfiguration('pcd_path').perform(context)
    if not pcd_path:
        pcd_path = os.path.abspath(os.path.join(
            share, '../../../../tools/output/building_obstacles_5cm.pcd'))
    pcd_path = os.path.abspath(os.path.expanduser(pcd_path))
    if not os.path.isfile(pcd_path) or not pcd_path.lower().endswith('.pcd'):
        raise RuntimeError(f'pcd_path must be an existing PCD file: {pcd_path}')

    frame = LaunchConfiguration('world_frame').perform(context)
    resolution = ParameterValue(
        LaunchConfiguration('octomap_resolution'), value_type=float)
    return [
        Node(
            package='go2_bringup', executable='jie_pcd_octomap_node',
            name='jie_pcd_octomap', output='screen',
            parameters=[params, {
                'use_sim_time': True,
                'pcd_file': pcd_path,
                'frame_id': frame,
                'resolution': resolution,
            }]),
        Node(
            package='octo_planner', executable='jie_path_node',
            name='jie_path_node', output='screen',
            parameters=[params, {'use_sim_time': True, 'frame_id': frame}]),
        Node(
            package='go2_bringup', executable='obstacle_height_filter.py',
            name='obstacle_height_filter', output='screen',
            parameters=[params, {'use_sim_time': True, 'world_frame': frame}]),
        Node(
            package='scan_planner', executable='scan_local_planner',
            name='scan_local_planner', output='screen',
            parameters=[params, {'use_sim_time': True, 'world_frame': frame}]),
        Node(
            package='octo_planner', executable='go2_path_follower',
            name='go2_path_follower', output='screen',
            parameters=[params, {'use_sim_time': True, 'map_frame': frame}]),
        Node(
            package='go2_bringup', executable='jie_go2_navigation.py',
            name='jie_go2_navigation', output='screen',
            parameters=[params, {'use_sim_time': True, 'world_frame': frame}]),
        Node(
            package='rviz2', executable='rviz2', name='navigation_rviz',
            arguments=['-d', rviz_config], output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('go2_bringup')
    workspace_pcd = os.path.abspath(os.path.join(
        share, '../../../../tools/output/building_obstacles_5cm.pcd'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_path', default_value=workspace_pcd,
            description='Static PCD obstacle map in the world frame'),
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument(
            'octomap_resolution', default_value='0.2',
            description='OctoMap voxel size in meters'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=navigation_nodes),
    ])
