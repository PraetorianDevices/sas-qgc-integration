#!/usr/bin/env python3
"""
Launch File: Complete SAS → QGroundControl Integration

Launches the full MAVLink bridge stack for SAS-QGC integration:
  1. GPS Spoofing Detector → MAVLink STATUSTEXT (alerts)
  2. Offboard Controller → MAVLink Telemetry (position, attitude, battery)
  3. Both bridges publish to UDP localhost:14550 for QGC

Usage:
  ros2 launch mavlink-bridge launch_sas_qgc_integration.py system_id:=1

Parameters:
  system_id    : MAVLink system ID (1-255, default 1)
  drone_id     : ROS 2 namespace for multi-drone (default empty for single drone)
  mavlink_port : UDP port for QGC (default 14550)

Expected Telemetry in QGC:
  - HEARTBEAT: Vehicle armed/disarmed status
  - GLOBAL_POSITION_INT: GPS position, altitude, heading
  - ATTITUDE: Roll, pitch, yaw
  - SYS_STATUS: Battery voltage, CPU load
  - BATTERY_STATUS: Detailed battery info (voltage, current, capacity)
  - STATUSTEXT: GPS spoofing alerts (WARNING/CRITICAL severity)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    system_id_arg = DeclareLaunchArgument(
        'system_id',
        default_value='1',
        description='MAVLink system ID (1-255)'
    )

    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='',
        description='ROS 2 namespace for this drone (empty for single drone)'
    )

    mavlink_port_arg = DeclareLaunchArgument(
        'mavlink_port',
        default_value='14550',
        description='UDP port for QGC telemetry (default 14550)'
    )

    mavlink_host_arg = DeclareLaunchArgument(
        'mavlink_host',
        default_value='localhost',
        description='Host for QGC connection'
    )

    # ===== GPS Spoofing Detector Node =====
    gps_spoof_detector = Node(
        package='SAS',
        executable='gps_spoof_detector_node',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')}
        ],
        output='screen'
    )

    # ===== GPS Spoof → MAVLink Bridge =====
    gps_spoof_bridge = Node(
        package='mavlink-bridge',
        executable='gps_spoof_mavlink_bridge',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')},
            {'component_id': 200},
            {'mavlink_host': LaunchConfiguration('mavlink_host')},
            {'mavlink_port': LaunchConfiguration('mavlink_port')},
        ],
        output='screen'
    )

    # ===== Telemetry → MAVLink Bridge =====
    telemetry_bridge = Node(
        package='mavlink-bridge',
        executable='telemetry_mavlink_bridge',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')},
            {'component_id': 1},  # MAV_COMP_ID_AUTOPILOT
            {'drone_id': LaunchConfiguration('drone_id')},
            {'mavlink_host': LaunchConfiguration('mavlink_host')},
            {'mavlink_port': LaunchConfiguration('mavlink_port')},
        ],
        output='screen'
    )

    # ===== Mission Control Bridge =====
    mission_bridge = Node(
        package='mavlink-bridge',
        executable='mission_control_bridge',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')},
            {'component_id': 1},  # MAV_COMP_ID_AUTOPILOT
            {'drone_id': LaunchConfiguration('drone_id')},
            {'mavlink_host': LaunchConfiguration('mavlink_host')},
            {'mavlink_port': LaunchConfiguration('mavlink_port')},
        ],
        output='screen'
    )

    # ===== Info Messages =====
    detector_started = LogInfo(msg='GPS Spoofing Detector: monitoring for spoofing attacks')
    gps_bridge_started = LogInfo(msg='GPS Spoof Bridge: /gps_spoof_alert -> MAVLink STATUSTEXT')
    telemetry_started = LogInfo(msg='Telemetry Bridge: px4_msgs -> MAVLink HEARTBEAT/POSITION/ATTITUDE/BATTERY')
    startup_complete = LogInfo(msg=[
        '\n',
        '=== SAS-QGC Integration Ready ===\n',
        'QGC Connection: UDP localhost:14550\n',
        'Telemetry:\n',
        '  - HEARTBEAT (armed status)\n',
        '  - GLOBAL_POSITION_INT (GPS, altitude, heading)\n',
        '  - ATTITUDE (roll/pitch/yaw)\n',
        '  - SYS_STATUS (battery, CPU)\n',
        '  - BATTERY_STATUS (detailed power info)\n',
        'Mission Control:\n',
        '  - MISSION_ITEM (waypoint upload/download)\n',
        '  - MISSION_CURRENT (active waypoint tracking)\n',
        '  - MISSION_ACK (mission acknowledgement)\n',
        'Alerts:\n',
        '  - STATUSTEXT (GPS spoofing warnings/critical)\n',
        '\n',
        'Connect QGC:\n',
        '  Settings -> Comm Links -> Add -> UDP\n',
        '  Host: localhost, Port: 14550\n',
        '================================\n'
    ])

    return LaunchDescription([
        system_id_arg,
        drone_id_arg,
        mavlink_port_arg,
        mavlink_host_arg,
        detector_started,
        gps_bridge_started,
        telemetry_started,
        startup_complete,
        gps_spoof_detector,
        gps_spoof_bridge,
        telemetry_bridge,
        mission_bridge,
    ])
