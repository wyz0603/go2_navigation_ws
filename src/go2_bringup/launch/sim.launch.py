"""Launch Go2, Gazebo Sim, ros2_control, and the ONNX RL controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("go2_description")
    gazebo_share = get_package_share_directory("go2_gazebo")
    worlds_share = get_package_share_directory("robot_sim_worlds")
    controller_share = get_package_share_directory("go2_rl_controller")
    bringup_share = get_package_share_directory("go2_bringup")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    model = os.path.join(description_share, "urdf", "go2_gazebo.urdf.xacro")
    world = os.path.join(worlds_share, "worlds", "building_obstacles.sdf")
    bridge = os.path.join(gazebo_share, "config", "bridge.yaml")
    controller_config = os.path.join(controller_share, "config", "policy_controller.yaml")
    policy = os.path.join(controller_share, "policy", "policy.onnx")
    resource_path = os.path.dirname(description_share)
    workspace_map = os.path.abspath(os.path.join(
        bringup_share, "../../../../tools/output/building_obstacles_5cm.ply"))
    default_map_file = workspace_map if os.path.isfile(workspace_map) else ""

    robot_description = ParameterValue(Command(["xacro ", model]), value_type=str)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": [LaunchConfiguration("gz_args"), " ", LaunchConfiguration("world")],
            "on_exit_shutdown": "true",
        }.items(),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge, "use_sim_time": True}],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description", "-name", "go2", "-allow_renaming", "false",
            "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"), "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )
    controllers = Node(
        package="controller_manager", executable="spawner",
        arguments=["go2_effort_controller", "joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--activate-as-group", "--switch-timeout", "30"],
        output="screen",
    )
    policy_controller = Node(
        package="go2_rl_controller",
        executable="onnx_policy_controller",
        parameters=[LaunchConfiguration("controller_config"), {"policy_path": policy, "use_sim_time": True}],
        output="screen",
    )
    unpause = ExecuteProcess(
        cmd=[
            "gz", "service",
            "-s", ["/world/", LaunchConfiguration("world_name"), "/control"],
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", "pause: false",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("navigation", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Start RViz2 when navigation is enabled"),
        DeclareLaunchArgument("map_file", default_value=default_map_file,
                              description="Static world PLY; empty enables live global mapping"),
        DeclareLaunchArgument("controller_config", default_value=controller_config),
        DeclareLaunchArgument("world", default_value=world),
        DeclareLaunchArgument("world_name", default_value="building_obstacles"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.34"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        # Start paused so the robot cannot fall before ros2_control and the
        # policy controller have initialized.
        DeclareLaunchArgument("gz_args", default_value="-v 3"),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[resource_path, os.pathsep, os.path.join(worlds_share, "models"), os.pathsep, EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
        ),
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=[resource_path, os.pathsep, os.path.join(worlds_share, "models"), os.pathsep, EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
        ),
        Node(package="go2_rl_controller", executable="command_mux", output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("go2_bringup"), "launch", "navigation.launch.py")),
            condition=IfCondition(LaunchConfiguration("navigation")),
            launch_arguments={"map_file": LaunchConfiguration("map_file"),
                              "use_rviz": LaunchConfiguration("use_rviz")}.items(),
        ),
        gazebo,
        state_publisher,
        bridge_node,
        spawn,
        # Configure both before unpausing, then activate in the same update.
        TimerAction(period=5.0, actions=[controllers]),
        TimerAction(period=8.0, actions=[policy_controller]),
        TimerAction(period=10.0, actions=[unpause]),
    ])
