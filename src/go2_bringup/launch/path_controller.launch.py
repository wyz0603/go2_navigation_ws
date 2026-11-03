"""Run a Nav2 FollowPath controller against the Go2 Gazebo simulation."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('go2_bringup')
    config = os.path.join(share, 'config', 'controller.yaml')
    with open(os.path.join(share, 'config', 'planner.yaml')) as stream:
        planner = yaml.safe_load(stream)['ugv_nav4d_ros2']['ros__parameters']
    terrain_params = {'world_frame': planner['world_frame'],
                      'base_frame': planner['robot_frame'],
                      'ground_clearance': planner['distToGround'],
                      'max_step_height': planner['maxStepHeight']}
    return LaunchDescription([
        Node(package='go2_bringup', executable='obstacle_height_filter.py',
             name='obstacle_height_filter', output='screen', parameters=[config, terrain_params]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[config],
             remappings=[('cmd_vel', '/cmd_vel_nav')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_path_controller', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['controller_server']}]),
    ])
