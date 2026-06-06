#!/usr/bin/env python3
"""One-command launch for the whole WaveRider digital-twin system (team item 11).

Brings up (optionally) the Gazebo world plus every twin node, with velocity
commands routed through cmd_mux so there is a single, arbitrated writer of
/cmd_vel (HALT > OVERRIDE > NAV).

Usage:
  ros2 launch my_tb3_world waverider.launch.py
  ros2 launch my_tb3_world waverider.launch.py start_world:=false   # world already running
  ros2 launch my_tb3_world waverider.launch.py goal_x:=2.0 goal_y:=0.5

Command routing:
  go_to_goal     -> /cmd_vel_nav      (lowest priority)
  dt_supervisor  -> /cmd_vel_override (safety override)
  comms_watchdog -> /cmd_vel_halt     (highest priority)
  cmd_mux        -> /cmd_vel          (the only writer the robot sees)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('my_tb3_world')
    start_world = LaunchConfiguration('start_world')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'new_world.launch.py')),
        condition=IfCondition(start_world),
    )

    def n(exe, params=None):
        return Node(package='my_tb3_world', executable=exe, name=exe,
                    output='screen', parameters=[params or {}])

    return LaunchDescription([
        DeclareLaunchArgument('start_world', default_value='true',
                              description='Also launch the Gazebo world'),
        DeclareLaunchArgument('goal_x', default_value='1.5'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),

        world,

        n('ph_sensor'),
        n('dt_supervisor', {'cmd_vel_topic': 'cmd_vel_override'}),
        n('comms_watchdog', {'cmd_vel_topic': 'cmd_vel_halt'}),
        n('go_to_goal', {
            'cmd_vel_topic': 'cmd_vel_nav',
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
        }),
        n('anomaly_zone_viz'),
        n('hazard_manager'),
        n('cmd_mux'),
    ])
