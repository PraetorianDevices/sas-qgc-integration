#!/usr/bin/env python3
"""
Launch File: Complete SAS → QGroundControl Integration

Launches the full MAVLink bridge stack for SAS-QGC integration:
  1. GPS Spoofing Detector → MAVLink STATUSTEXT (alerts)
  2. Offboard Controller → MAVLink Telemetry (position, attitude, battery)
  3. Mission Control (bidirectional waypoint upload/download)
  4. Fleet Manager → MAVLink STATUSTEXT (per-drone mission-state summaries)
  5. Collision (SF45 obstacle sweep) → MAVLink OBSTACLE_DISTANCE
  6. Emergency Wipe (MAVLink COMMAND_LONG → wipe service) — see port note below

Outbound bridges (gps_spoof, telemetry, fleet, collision) `connect()` and only
SEND, so any number of them share the QGC port (14550) fine. INBOUND bridges
BIND the port to receive, and two processes cannot cleanly bind the same UDP
port. mission_control_bridge already binds `mavlink_port` (14550); the emergency
wipe bridge therefore listens on a SEPARATE `wipe_port` (default 14556) here.

  ⚠️ Known limitation: because QGC uses a single UDP comm link per vehicle,
  reaching the wipe bridge on a different port requires either a second QGC
  comm link aimed at wipe_port, or a MAVLink router / single inbound
  demultiplexer that fans one inbound stream out to both inbound bridges. The
  per-message-type one-socket-per-process design does not multiplex inbound on
  one port. Documented in IMPLEMENTATION_STATUS.md Known Limitations.

Usage:
  ros2 launch mavlink-bridge launch_sas_qgc_integration.py system_id:=1

Parameters:
  system_id    : MAVLink system ID (1-255, default 1)
  drone_id     : ROS 2 namespace for multi-drone (default empty for single drone)
  mavlink_port : UDP port for QGC telemetry/mission (default 14550)
  wipe_port    : UDP port the emergency-wipe bridge binds (default 14556)

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

    wipe_port_arg = DeclareLaunchArgument(
        'wipe_port',
        default_value='14556',
        description='UDP port the emergency-wipe bridge binds (separate from '
                    'mavlink_port, which mission_control already binds)'
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

    # ===== Fleet Manager → MAVLink Bridge (outbound STATUSTEXT) =====
    fleet_manager_bridge = Node(
        package='mavlink-bridge',
        executable='fleet_manager_mavlink_bridge',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')},
            {'component_id': 200},  # SAS custom component
            {'mavlink_host': LaunchConfiguration('mavlink_host')},
            {'mavlink_port': LaunchConfiguration('mavlink_port')},
        ],
        output='screen'
    )

    # ===== Collision Avoidance → MAVLink Bridge (outbound OBSTACLE_DISTANCE) =====
    collision_bridge = Node(
        package='mavlink-bridge',
        executable='collision_mavlink_bridge',
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

    # ===== Emergency Wipe Bridge (INBOUND COMMAND_LONG) =====
    # Binds its own wipe_port -- NOT mavlink_port -- since mission_control_bridge
    # already binds mavlink_port and two processes cannot share a UDP bind.
    emergency_wipe_bridge = Node(
        package='mavlink-bridge',
        executable='emergency_wipe_mavlink_bridge',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')},
            {'component_id': 1},  # MAV_COMP_ID_AUTOPILOT
            {'drone_id': LaunchConfiguration('drone_id')},
            {'mavlink_host': LaunchConfiguration('mavlink_host')},
            {'mavlink_port': LaunchConfiguration('wipe_port')},
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
        wipe_port_arg,
        detector_started,
        gps_bridge_started,
        telemetry_started,
        startup_complete,
        gps_spoof_detector,
        gps_spoof_bridge,
        telemetry_bridge,
        mission_bridge,
        fleet_manager_bridge,
        collision_bridge,
        emergency_wipe_bridge,
    ])
