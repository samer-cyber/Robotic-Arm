"""Bring up the VLA pipeline.

    ros2 launch arm_vla vla_bringup.launch.py
    ros2 launch arm_vla vla_bringup.launch.py camera:=false      # camera already up
    ros2 launch arm_vla vla_bringup.launch.py simulate:=true     # no arm attached

The camera is launched last-but-included because depthai takes several seconds
to boot the device, and in WSL it may re-enumerate on the USB bus while doing
so. If it misbehaves, run it in its own terminal with camera:=false here --
that keeps its log separate, which makes the failure much easier to read.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("arm_vla"), "config", "vla_params.yaml"])

    args = [
        DeclareLaunchArgument("camera", default_value="true",
                              description="also launch the depthai driver"),
        DeclareLaunchArgument("oak_config",
                              default_value=os.path.expanduser("~/oak_sr_config.yaml"),
                              description="depthai params file"),
        DeclareLaunchArgument("simulate", default_value="false",
                              description="run the arm in the armlink simulator"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("params_file", default_value=params),
    ]

    params_file = LaunchConfiguration("params_file")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("depthai_ros_driver"),
                                  "launch", "camera.launch.py"])]),
        launch_arguments={"params_file": LaunchConfiguration("oak_config")}.items(),
        condition=IfCondition(LaunchConfiguration("camera")),
    )

    vision = Node(
        package="arm_vla", executable="vision_spatial_node",
        name="vision_spatial_node", output="screen",
        parameters=[params_file],
    )

    commander = Node(
        package="arm_vla", executable="vla_commander_node",
        name="vla_commander_node", output="screen",
        parameters=[params_file,
                    {"simulate": LaunchConfiguration("simulate"),
                     "port": LaunchConfiguration("port")}],
    )

    return LaunchDescription(args + [camera, vision, commander])
