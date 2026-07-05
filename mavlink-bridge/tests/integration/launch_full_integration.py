#!/usr/bin/env python3
"""
Launch File: Full Integration Test

Launches the complete pipeline for integration testing:
  1. GPS Spoofing Detector Node
  2. GPS Spoofing MAVLink Bridge
  3. Test UDP Capture (optional, for packet verification)

Usage:
  ros2 launch mavlink-bridge test_full_integration.launch.py

Expected Test Flow:
  1. Detector publishes alerts to /gps_spoof_alert
  2. Bridge receives alerts, converts to MAVLink
  3. Bridge sends MAVLink STATUSTEXT via UDP to localhost:14550
  4. Test capture node (if running) receives packets
  5. Packets are verified with tcpdump or Wireshark

Verification Checklist:
  [] /gps_spoof_alert topic has messages (ros2 topic echo /gps_spoof_alert)
  [] /gps_spoof_detector_node is running (ros2 node list)
  [] /gps_spoof_mavlink_bridge is running (ros2 node list)
  [] UDP packets on port 14550 (tcpdump -i lo udp port 14550)
  [] QGC receives STATUSTEXT messages (visual confirmation in QGC)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    system_id_arg = DeclareLaunchArgument(
        'system_id',
        default_value='1',
        description='MAVLink system ID for test drone'
    )

    enable_qgc_arg = DeclareLaunchArgument(
        'enable_qgc',
        default_value='false',
        description='If true, also connect to QGC on localhost:14550'
    )

    # GPS Spoofing Detector Node
    detector_node = Node(
        package='SAS',
        executable='gps_spoof_detector_node',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')}
        ],
        output='screen',
        emulate_tty=True,
    )

    # GPS Spoof → MAVLink Bridge Node
    bridge_node = Node(
        package='mavlink-bridge',
        executable='gps_spoof_mavlink_bridge',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')},
            {'component_id': 200},
            {'mavlink_host': 'localhost'},
            {'mavlink_port': 14550},
        ],
        output='screen',
        emulate_tty=True,
    )

    # Info messages
    detector_info = LogInfo(msg='GPS Spoofing Detector started. Publishing to /gps_spoof_alert')
    bridge_info = LogInfo(msg='MAVLink Bridge started. Listening on UDP localhost:14550')
    test_info = LogInfo(msg=[
        'Integration test pipeline running.\n',
        'Verify with:\n',
        '  1. ros2 topic echo /gps_spoof_alert\n',
        '  2. tcpdump -i lo udp port 14550\n',
        '  3. Connect QGC to UDP localhost:14550\n',
    ])

    return LaunchDescription([
        system_id_arg,
        enable_qgc_arg,
        detector_info,
        bridge_info,
        test_info,
        detector_node,
        bridge_node,
    ])
