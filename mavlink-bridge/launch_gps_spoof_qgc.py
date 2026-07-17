#!/usr/bin/env python3
"""
Launch file: GPS Spoofing Detector → QGroundControl Integration

Launches the GPS spoofing detector node alongside a MAVLink bridge that
converts spoofing alerts to MAVLink STATUSTEXT messages for display in QGC.

Usage:
    ros2 launch mavlink-bridge launch_gps_spoof_qgc.py system_id:=1 mavlink_port:=14550

Parameters:
    system_id    : MAVLink system ID for this vehicle (default: 1)
    mavlink_port : UDP port for QGC telemetry (default: 14550)
    mavlink_host : Hostname/IP for QGC connection (default: localhost)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    system_id_arg = DeclareLaunchArgument(
        'system_id',
        default_value='1',
        description='MAVLink system ID'
    )

    mavlink_port_arg = DeclareLaunchArgument(
        'mavlink_port',
        default_value='14550',
        description='UDP port for QGC connection'
    )

    mavlink_host_arg = DeclareLaunchArgument(
        'mavlink_host',
        default_value='localhost',
        description='Hostname/IP for MAVLink output'
    )

    # GPS Spoofing Detector Node
    gps_spoof_detector = Node(
        package='my_python_package',  # SAS's actual ROS 2 package name (see SAS/package.xml) -- 'SAS' is just the repo/directory name, not a registered package
        executable='gps_spoof_detector_node',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')}
        ],
        output='screen'
    )

    # GPS Spoof → MAVLink Bridge Node
    mavlink_bridge = Node(
        package='mavlink-bridge',
        executable='gps_spoof_mavlink_bridge',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')},
            {'component_id': 200},  # Custom component ID
            {'mavlink_host': LaunchConfiguration('mavlink_host')},
            {'mavlink_port': LaunchConfiguration('mavlink_port')},
        ],
        output='screen'
    )

    return LaunchDescription([
        system_id_arg,
        mavlink_port_arg,
        mavlink_host_arg,
        gps_spoof_detector,
        mavlink_bridge,
    ])
