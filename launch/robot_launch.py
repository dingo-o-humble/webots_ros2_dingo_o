#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots and the controller."""

import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node

from launch_param_builder import load_xacro


def generate_launch_description():
    package_dir = get_package_share_directory("webots_ros2_dingo_o")
    world = LaunchConfiguration("world")
    mode = LaunchConfiguration("mode")
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    robot_description_path = os.path.join(package_dir, "urdf", "dingo_o_webots.urdf")

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", world]),
        mode=mode,
        ros2_supervisor=True,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": load_xacro(
                    pathlib.Path(
                        os.path.join(
                            package_dir,
                            "urdf",
                            "dingo_o_webots.urdf.xacro",
                        )
                    )
                )
            }
        ],
    )

    footprint_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )

    # ROS control spawners
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["mecanum_drive_controller"] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_state_broadcaster"] + controller_manager_timeout,
    )
    ros_control_spawners = [
        mecanum_drive_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    ros2_control_params = os.path.join(package_dir, "config", "ros2_control.yaml")
    mappings = [
        ("/mecanum_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ("/mecanum_drive_controller/odom", "/odom"),
    ]

    robot_description_path = os.path.join(
        package_dir, "urdf", "dingo_o_webots_control.urdf"
    )
    dingo_o_driver = WebotsController(
        robot_name="Dingo",
        parameters=[
            {
                "robot_description": robot_description_path,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": False,
            },
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
    )

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=dingo_o_driver,
        nodes_to_start=ros_control_spawners,
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("webots_ros2_dingo_o"),
                "config",
                "ekf.yaml",
            )
        ],
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[
            ("cloud_in", "/Dingo/Velodyne_Puck/point_cloud"),
        ],
        parameters=[
            {
                "transform_tolerance": 0.01,
                "min_height": 0.0,
                "max_height": 1.0,
                "angle_min": -3.14,
                "angle_max": 3.14,
                "angle_increment": 0.00872,
                "scan_time": 0.1,
                "range_min": 0.9,
                "range_max": 100.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="default.wbt",
            ),
            DeclareLaunchArgument(
                "mode", default_value="realtime", description="Webots startup mode"
            ),
            # spawn_URDF_dingo,
            webots,
            webots._supervisor,
            robot_state_publisher,
            footprint_publisher,
            dingo_o_driver,
            waiting_nodes,
            # This action will kill all nodes once the Webots simulation has exited
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
            ekf_node,
            pointcloud_to_laserscan_node,
        ]
    )
