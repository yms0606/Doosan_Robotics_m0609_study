#
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
#

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction


def print_launch_configuration_value(context, *args, **kwargs):
    # LaunchConfiguration 값을 평가합니다.
    gz_value = LaunchConfiguration("gz").perform(context)
    # 평가된 값을 콘솔에 출력합니다.
    print(f"LaunchConfiguration gz: {gz_value}")
    return gz_value


def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument("name", default_value="dsr01", description="NAME_SPACE"),
        DeclareLaunchArgument("host", default_value="127.0.0.1", description="ROBOT_IP"),
        DeclareLaunchArgument("port", default_value="12345", description="ROBOT_PORT"),
        DeclareLaunchArgument("mode", default_value="virtual", description="OPERATION MODE"),
        DeclareLaunchArgument("model", default_value="m1013", description="ROBOT_MODEL"),
        DeclareLaunchArgument("color", default_value="white", description="ROBOT_COLOR"),
        DeclareLaunchArgument("gui", default_value="false", description="Start RViz2"),
        DeclareLaunchArgument("gz", default_value="false", description="USE GAZEBO SIM"),
    ]
    xacro_path = os.path.join(get_package_share_directory("dsr_description2"), "xacro")
    # gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dsr_description2"),
                    "xacro",
                    LaunchConfiguration("model"),
                ]
            ),
            ".urdf.xacro",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dsr_controller2"),
            "config",
            "dsr_controller2.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution([FindPackageShare("dsr_description2"), "rviz", "default.rviz"])

    connection_node = Node(
        package="dsr_bringup2",
        executable="connection",
        namespace=LaunchConfiguration("name"),
        parameters=[
            {"name": LaunchConfiguration("name")},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")},
            {"mode": LaunchConfiguration("mode")},
            {"model": LaunchConfiguration("model")},
            {"gripper": "none"},
            {"mobile": "none"},
            # parameters_file_path       # 파라미터 설정을 동일이름으로 launch 파일과 yaml 파일에서 할 경우 yaml 파일로 셋팅된다.
        ],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("name"),
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("name"),
        output="both",
        # remappings=[
        #     (
        #         "/joint_states",
        #         "/dsr/joint_states",
        #     ),
        # ],
        parameters=[
            {"robot_description": Command(["xacro", " ", xacro_path, "/", LaunchConfiguration("model"), ".urdf.xacro color:=", LaunchConfiguration("color")])}
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration("name"),
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        # condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    # joint_trajectory_controller_spawner = Node(
    #     package="controller_manager",
    #     # namespace=LaunchConfiguration('name'),
    #     executable="spawner",
    #     arguments=["dsr_joint_trajectory", "-c", "dsr/controller_manager", "-n", "dsr"],
    # )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        connection_node,
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
