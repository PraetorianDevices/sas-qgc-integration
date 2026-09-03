#!/usr/bin/env python3
"""
Launch File: Complete SAS → QGroundControl Integration

Launches the full MAVLink bridge stack for SAS-QGC integration:
  1. GPS Spoofing Detector → MAVLink STATUSTEXT (alerts)
  2. Offboard Controller → MAVLink Telemetry (position, attitude, battery)
  3. Mission Control (bidirectional waypoint upload/download)
  4. Fleet Manager → MAVLink STATUSTEXT (per-drone mission-state summaries)
  5. Collision (SF45 obstacle sweep) → MAVLink OBSTACLE_DISTANCE
  6. Emergency Wipe (MAVLink COMMAND_LONG → wipe service)
  7. MAVLink Router — fans the single QGC UDP link out to both inbound bridges

Outbound bridges (gps_spoof, telemetry, fleet, collision) `connect()` and only
SEND, so any number of them share the QGC port (14550) fine. INBOUND bridges
(mission_control, emergency_wipe) each need to BIND a socket to receive, and
two processes cannot cleanly bind the same UDP port — but QGC uses a single
UDP comm link per vehicle, so both must somehow be reachable on that one link.

Resolved via mavlink_router_node: it binds the single external port QGC's
comm link targets (`mavlink_port`, 14550) and fans every inbound datagram out
to both inbound bridges, each now listening on its own internal port
(`mission_control_listen_port` 14551, `wipe_port` 14556) instead of the shared
external one. Neither bridge needed any code change for this -- both already
bind whatever port they're configured with and learn their reply address
dynamically from whoever last contacted them, so pointed at the router
instead of directly at QGC, that existing mechanism keeps working unmodified.
See mavlink_router_node.py's module docstring for the full design, and
IMPLEMENTATION_STATUS.md (this limitation is now resolved, not just documented).

Usage:
  ros2 launch mavlink-bridge launch_sas_qgc_integration.py system_id:=1
  ros2 launch mavlink-bridge launch_sas_qgc_integration.py enable_security:=true

Parameters:
  system_id                   : MAVLink system ID (1-255, default 1)
  drone_id                    : namespace for PX4 uXRCE-DDS topics (/fmu/...) -- leave empty for a
                                  single SITL instance, which publishes them unprefixed
  sas_namespace               : namespace the SAS control nodes run under (pass drone_1 to match
                                  SAS/launch/single_drone.launch.py); used by mission_control and
                                  emergency_wipe, which talk to those nodes rather than to PX4
  mavlink_port                : the single external UDP port QGC's comm link targets (default 14550) -- only mavlink_router_node binds this
  mavlink_host                : destination host outbound bridges send to / where QGC is reachable (default localhost).
                                  Set to the other host's address (e.g. a WSL2 default-gateway IP) when QGC runs in a
                                  different network namespace than this launch -- see mavlink_bind_host below.
  mavlink_bind_host           : local address mavlink_router_node/mission_control_bridge/emergency_wipe_bridge bind
                                  their inbound sockets to (default 0.0.0.0). Deliberately separate from mavlink_host
                                  for the WSL2-NAT case above, where mavlink_host is never a locally-assignable address.
  mission_control_listen_port : internal port mission_control_bridge binds, behind the router (default 14551)
  wipe_port                   : internal port emergency_wipe_bridge binds, behind the router (default 14556)
  router_downstream_port      : internal port mavlink_router_node itself binds to talk to both inbound bridges (default 14559)
  enable_security              : set the ROS_SECURITY_* env vars (SROS2) for every node below, same three vars as
                                  SAS/launch/secure_launch.py (default false -- opt-in, so existing runs without a
                                  configured keystore/SROS2 install aren't silently broken)
  keystore_path                : absolute path to the DDS-Security keystore (default: SAS/security/keystore,
                                  resolved as a sibling of this repo's mavlink-bridge/ directory -- override if
                                  your workspace layout differs)

  ⚠️ If you override mission_control_listen_port or wipe_port, you must also
  update mavlink_router_node's `downstream_targets` parameter below to match --
  it is not derived automatically from the other launch arguments.

Security:
  All 7 mavlink-bridge nodes below have enclaves in SAS/security/keystore
  (generated + tailored to least-privilege; see IMPLEMENTATION_STATUS.md), so
  `enable_security:=true` here launches every node in this file -- including
  the SAS-package `gps_spoof_detector_node`, which already had its own
  pre-existing enclave -- under ROS_SECURITY_ENABLE=true,
  ROS_SECURITY_STRATEGY=Enforce, matching SAS/launch/secure_launch.py's own
  env vars exactly. A node started this way with no matching enclave fails to
  launch under Enforce; every node here has one.

Expected Telemetry in QGC:
  - HEARTBEAT: Vehicle armed/disarmed status
  - GLOBAL_POSITION_INT: GPS position, altitude, heading
  - ATTITUDE: Roll, pitch, yaw
  - SYS_STATUS: Battery voltage, CPU load
  - BATTERY_STATUS: Detailed battery info (voltage, current, capacity)
  - STATUSTEXT: GPS spoofing alerts (WARNING/CRITICAL severity)
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# SAS/security/keystore, resolved as a sibling of this file's own package
# directory (mavlink-bridge/) -- matches how SAS/launch/secure_launch.py
# resolves its own keystore path relative to its own file location. Override
# via the keystore_path launch argument if your workspace doesn't have SAS
# and mavlink-bridge as sibling directories.
_DEFAULT_KEYSTORE_PATH = str((Path(__file__).resolve().parent.parent / 'SAS' / 'security' / 'keystore'))


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
        description='Namespace for PX4 uXRCE-DDS topics (/fmu/...). Leave empty '
                    'for a single SITL instance, which publishes them unprefixed.'
    )

    sas_namespace_arg = DeclareLaunchArgument(
        'sas_namespace',
        default_value='',
        description='ROS 2 namespace the SAS control nodes run under, used by the '
                    'bridges that talk to those nodes (mission_control, '
                    'emergency_wipe). Deliberately SEPARATE from drone_id: '
                    'drone_id prefixes PX4 uXRCE-DDS topics (/fmu/...), which a '
                    'single SITL instance publishes UNPREFIXED, while SAS/launch/'
                    'single_drone.launch.py pushes its nodes under /drone_1. One '
                    'shared value cannot satisfy both -- setting drone_id:=drone_1 '
                    'to reach the SAS nodes silently breaks telemetry/collision by '
                    'pointing them at /drone_1/fmu/... topics PX4 never publishes. '
                    'For the standard single-SITL setup pass sas_namespace:=drone_1 '
                    'and leave drone_id empty.'
    )

    mavlink_port_arg = DeclareLaunchArgument(
        'mavlink_port',
        default_value='14550',
        description='Single external UDP port QGC\'s comm link targets. Only '
                    'mavlink_router_node binds this; mission_control and '
                    'emergency_wipe are behind it on their own internal ports.'
    )

    mavlink_host_arg = DeclareLaunchArgument(
        'mavlink_host',
        default_value='localhost',
        description='Destination host outbound bridges send to, and the '
                    'address QGC is reachable at. On a single machine this '
                    'is the same address the inbound bridges bind to, but '
                    'when QGC runs in a different network namespace than '
                    'this launch (e.g. QGC on native Windows, ROS 2 in WSL2 '
                    'NAT) this must be set to that other host\'s address '
                    '(e.g. the WSL2 default-gateway IP) -- see mavlink_bind_host '
                    'for the separate inbound bind address in that case.'
    )

    mavlink_bind_host_arg = DeclareLaunchArgument(
        'mavlink_bind_host',
        default_value='0.0.0.0',
        description='Local address mavlink_router_node, mission_control_bridge, '
                    'and emergency_wipe_bridge bind their inbound-listening '
                    'sockets to. Deliberately separate from mavlink_host: '
                    'under WSL2 NAT (QGC on Windows, this launch inside WSL2), '
                    'mavlink_host is the Windows-side address outbound bridges '
                    'send to, which is never a locally-assignable bind address '
                    'inside the WSL2 VM. Default 0.0.0.0 (all local interfaces) '
                    'works in every topology (native, WSL2 NAT, WSL2 mirrored, '
                    'Docker); only override if you need to restrict which '
                    'interface these sockets listen on.'
    )

    mission_control_listen_port_arg = DeclareLaunchArgument(
        'mission_control_listen_port',
        default_value='14551',
        description='Internal port mission_control_bridge binds, behind the router'
    )

    wipe_port_arg = DeclareLaunchArgument(
        'wipe_port',
        default_value='14556',
        description='Internal port emergency_wipe_bridge binds, behind the router'
    )

    router_downstream_port_arg = DeclareLaunchArgument(
        'router_downstream_port',
        default_value='14559',
        description='Internal port mavlink_router_node itself binds to talk '
                    'to both inbound bridges'
    )

    enable_security_arg = DeclareLaunchArgument(
        'enable_security',
        default_value='false',
        description='Set ROS_SECURITY_ENABLE/STRATEGY/KEYSTORE (SROS2) for '
                    'every node in this file. Opt-in (default false) so '
                    'existing runs without a configured keystore/SROS2 '
                    'install are not silently broken.'
    )

    keystore_path_arg = DeclareLaunchArgument(
        'keystore_path',
        default_value=_DEFAULT_KEYSTORE_PATH,
        description='Absolute path to the DDS-Security keystore (only used '
                    'when enable_security:=true). Default resolves SAS/security/'
                    'keystore as a sibling of this package.'
    )

    # SROS2 environment -- same three variables as SAS/launch/secure_launch.py,
    # applied to the whole launch context (every node below) when enabled.
    # Order matters: these must be declared/added to the LaunchDescription
    # before the Node actions so the environment is set before any process
    # spawns.
    security_env_actions = [
        SetEnvironmentVariable('ROS_SECURITY_ENABLE', 'true', condition=IfCondition(LaunchConfiguration('enable_security'))),
        SetEnvironmentVariable('ROS_SECURITY_STRATEGY', 'Enforce', condition=IfCondition(LaunchConfiguration('enable_security'))),
        SetEnvironmentVariable('ROS_SECURITY_KEYSTORE', LaunchConfiguration('keystore_path'), condition=IfCondition(LaunchConfiguration('enable_security'))),
    ]

    security_status_msg = LogInfo(
        condition=IfCondition(LaunchConfiguration('enable_security')),
        msg=[
            'DDS-Security ENABLED for this launch: ROS_SECURITY_STRATEGY=Enforce, '
            'ROS_SECURITY_KEYSTORE=', LaunchConfiguration('keystore_path'),
            '. Any node below without a matching enclave will fail to start.',
        ]
    )

    # ===== GPS Spoofing Detector Node =====
    gps_spoof_detector = Node(
        package='my_python_package',  # SAS's actual ROS 2 package name (see SAS/package.xml) -- 'SAS' is just the repo/directory name, not a registered package
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

    # ===== Mission Control Bridge (INBOUND: MISSION_COUNT/ITEM_INT/etc.) =====
    # Binds mission_control_listen_port -- NOT the external mavlink_port --
    # since mavlink_router_node owns the external port and fans inbound
    # traffic to this bridge's internal port instead.
    mission_bridge = Node(
        package='mavlink-bridge',
        executable='mission_control_bridge',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')},
            {'component_id': 1},  # MAV_COMP_ID_AUTOPILOT
            {'drone_id': LaunchConfiguration('sas_namespace')},
            {'mavlink_host': LaunchConfiguration('mavlink_host')},
            {'mavlink_bind_host': LaunchConfiguration('mavlink_bind_host')},
            {'mavlink_port': LaunchConfiguration('mission_control_listen_port')},
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
    # Binds its own wipe_port -- NOT the external mavlink_port -- for the same
    # reason as mission_control_bridge above: it's behind the router now.
    emergency_wipe_bridge = Node(
        package='mavlink-bridge',
        executable='emergency_wipe_mavlink_bridge',
        namespace='/',
        parameters=[
            {'system_id': LaunchConfiguration('system_id')},
            {'component_id': 1},  # MAV_COMP_ID_AUTOPILOT
            {'drone_id': LaunchConfiguration('sas_namespace')},
            {'mavlink_host': LaunchConfiguration('mavlink_host')},
            {'mavlink_bind_host': LaunchConfiguration('mavlink_bind_host')},
            {'mavlink_port': LaunchConfiguration('wipe_port')},
        ],
        output='screen'
    )

    # ===== MAVLink Router (fans the single QGC link out to both inbound bridges) =====
    # Binds the external mavlink_port that QGC's comm link actually targets,
    # and forwards every inbound frame to both mission_control_bridge and
    # emergency_wipe_bridge's internal ports. This is what makes it possible
    # for QGC's single UDP link to reach both inbound bridges at once --
    # resolves the previously-documented inbound single-UDP-port limitation.
    #
    # downstream_targets below must match mission_control_listen_port/wipe_port's
    # defaults (14551/14556); it is not derived from those launch args
    # automatically -- update both together if you change either default.
    mavlink_router = Node(
        package='mavlink-bridge',
        executable='mavlink_router_node',
        namespace='/',
        parameters=[
            {'mavlink_host': LaunchConfiguration('mavlink_host')},
            {'mavlink_bind_host': LaunchConfiguration('mavlink_bind_host')},
            {'mavlink_port': LaunchConfiguration('mavlink_port')},
            {'downstream_bind_host': 'localhost'},
            {'downstream_bind_port': LaunchConfiguration('router_downstream_port')},
            {'downstream_targets': ['localhost:14551', 'localhost:14556']},
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
        '  - STATUSTEXT (GPS spoofing warnings/critical, fleet summaries, wipe status)\n',
        'Fleet & Collision:\n',
        '  - Fleet Manager -> STATUSTEXT (per-drone mission state/progress)\n',
        '  - Collision Avoidance -> OBSTACLE_DISTANCE (SF45 sweep)\n',
        'Emergency Wipe:\n',
        '  - COMMAND_LONG (gated) -> COMMAND_ACK\n',
        '\n',
        'A single mavlink_router_node fans the one QGC UDP link out to both\n',
        'inbound bridges (mission_control, emergency_wipe) -- see that node and\n',
        'this file\'s docstring for the resolved single-inbound-port limitation.\n',
        '\n',
        'Connect QGC:\n',
        '  Settings -> Comm Links -> Add -> UDP\n',
        '  Host: localhost, Port: 14550\n',
        '================================\n'
    ])

    return LaunchDescription([
        system_id_arg,
        drone_id_arg,
        sas_namespace_arg,
        mavlink_port_arg,
        mavlink_host_arg,
        mavlink_bind_host_arg,
        mission_control_listen_port_arg,
        wipe_port_arg,
        router_downstream_port_arg,
        enable_security_arg,
        keystore_path_arg,
        *security_env_actions,
        security_status_msg,
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
        mavlink_router,
        emergency_wipe_bridge,
    ])
