"""Start the Go2 planner and Nav2 path controller for an existing simulation."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_navigation(context):
    share = get_package_share_directory('go2_bringup')
    params = os.path.join(share, 'config', 'planner.yaml')
    rviz_config = os.path.join(share, 'config', 'navigation.rviz')
    with open(params) as stream:
        planner_params = yaml.safe_load(stream)['ugv_nav4d_ros2']['ros__parameters']
    map_file = LaunchConfiguration('map_file').perform(context)
    overrides = {'use_sim_time': True, 'load_mls_from_file': bool(map_file)}
    if map_file:
        map_file = os.path.abspath(os.path.expanduser(map_file))
        if not os.path.isfile(map_file) or not map_file.lower().endswith('.ply'):
            raise RuntimeError(f'map_file must be an existing PLY file: {map_file}')
        overrides.update(mls_file_path=map_file, mls_file_type='ply')
    return [
        Node(package='go2_bringup', executable='ground_goal_adapter.py', output='screen',
             parameters=[{'use_sim_time': True,
                          'ground_clearance': planner_params['distToGround'],
                          'world_frame': planner_params['world_frame']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'path_controller.launch.py'))),
        Node(package='ugv_nav4d_ros2', executable='ugv_nav4d_ros2_node',
             name='ugv_nav4d_ros2', output='screen',
             parameters=[params, overrides],
             remappings=[('/ugv_nav4d_ros2/pointcloud', '/go2/points')]),
        Node(package='ugv_nav4d_ros2', executable='follow_path_client.py',
             name='ugv_nav4d_follow_path_client', output='screen',
             parameters=[params, {'use_sim_time': True}]),
        Node(package='ugv_nav4d_ros2', executable='field_operations.py',
             name='ugv_nav4d_field_operations', output='screen',
             parameters=[params, {'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz',
             arguments=['-d', rviz_config], output='screen',
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('go2_bringup')
    workspace_map = os.path.abspath(os.path.join(
        share, '../../../../tools/output/building_obstacles_5cm.ply'))
    default_map_file = workspace_map if os.path.isfile(workspace_map) else ''
    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map_file,
                              description='World-coordinate PLY; empty uses live pointcloud mapping'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz2 with the Go2 navigation configuration'),
        OpaqueFunction(function=launch_navigation),
    ])
