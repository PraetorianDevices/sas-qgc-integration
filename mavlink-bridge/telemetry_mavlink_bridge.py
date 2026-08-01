#!/usr/bin/env python3

"""
Telemetry Bridge: px4_msgs → MAVLink for QGroundControl

Subscribes to telemetry topics from offboard_controller_node and other sources,
converts to MAVLink messages, and publishes via UDP to QGroundControl.

Messages Published:
  - HEARTBEAT (MAV_TYPE_QUADROTOR)
  - GLOBAL_POSITION_INT (GPS position, heading, altitude)
  - ATTITUDE (roll, pitch, yaw, angular velocities)
  - SYS_STATUS (battery, CPU, sensor health)
  - BATTERY_STATUS (detailed battery telemetry)

Coordinate Frames:
  - PX4 uses NED: Z positive DOWN, ±X North, ±Y East
  - MAVLink uses NED: Z positive DOWN (same as PX4)
  - Altitude in MAVLink is typically relative to home (m above takeoff)

Hardware:
  Flight Controller: mRo Pixracer Pro (PX4)
  Companion Computer: Orin Nano Super

All frame encoding goes through mavlink_v2.py, which is verified byte-for-byte
against pymavlink. This bridge previously built a non-standard 7-byte frame
header instead of the real 10-byte MAVLink 2.0 header, and its SYS_STATUS
payload packed three 32-bit sensor-bitmask fields as 16-bit (wrong width,
wrong total payload length). Its BATTERY_STATUS payload additionally had a
bug where the real per-cell voltages argument was never used — a loop over
`enumerate([0]*10)` meant every cell was always sent as 0V regardless of the
vehicle's actual battery state.

Found by building this bridge against the real px4_msgs package (not the
hand-shaped test stubs) for the first time: `VehicleStatus.system_status`,
`VehicleStatus.load`, `VehicleAttitude.rollspeed/pitchspeed/yawspeed`, and
`BatteryStatus.energy_consumed_j` were never real px4_msgs fields at all --
confirmed absent as far back as px4_msgs v1.14.0 (2023), not just a newer-
version rename. The existing test suite's stubs supplied exactly the
attributes this code expected, so it never caught that those attributes
don't exist on the real messages -- this would have raised AttributeError
on the very first VehicleAttitude/VehicleStatus callback against a real PX4
instance. Angular rates actually come from a separate topic
(VehicleAngularVelocity.xyz), and CPU load from a separate topic
(Cpuload.load); MAVLink has no PX4 source for `system_status` (its 4-state
DDS enum doesn't map onto MAV_STATE) or `energy_consumed` (PX4 doesn't track
consumed energy in joules), so those two are simplified/marked unknown
rather than invented.
"""

import socket
import time
import math
from typing import Optional
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleAttitude,
    VehicleAngularVelocity,
    VehicleStatus,
    BatteryStatus,
    VehicleGlobalPosition,
    Cpuload,
)

import mavlink_v2 as mav


class MAVType(IntEnum):
    """MAVLink vehicle types."""
    GENERIC = 0
    FIXED_WING = 1
    QUADROTOR = 2
    COAXIAL = 3
    HELICOPTER = 4
    ANTENNA_TRACKER = 5
    GROUND_ROVER = 10
    SURFACE_BOAT = 14


class MAVState(IntEnum):
    """MAVLink system states."""
    UNINIT = 0
    BOOT = 1
    CALIBRATING = 2
    STANDBY = 3
    ACTIVE = 4
    CRITICAL = 5
    EMERGENCY = 6
    POWEROFF = 7
    FLIGHT_TERMINATION = 8


class TelemetryMAVLinkBridge(Node):
    """
    Bridge between ROS 2 px4_msgs and MAVLink telemetry for QGroundControl.

    Converts PX4 sensor data to MAVLink messages and transmits via UDP.
    """

    def __init__(self):
        super().__init__('telemetry_mavlink_bridge')

        # Parameters
        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)  # MAV_COMP_ID_AUTOPILOT
        self.declare_parameter('mavlink_host', 'localhost')
        self.declare_parameter('mavlink_port', 14550)
        self.declare_parameter('drone_id', '')

        self.system_id = self.get_parameter('system_id').value
        self.component_id = self.get_parameter('component_id').value
        mavlink_host = self.get_parameter('mavlink_host').value
        mavlink_port = self.get_parameter('mavlink_port').value
        self.drone_id = self.get_parameter('drone_id').value
        self.topic_prefix = f'/{self.drone_id}' if self.drone_id else ''

        self.get_logger().info(
            f'Telemetry MAVLink Bridge initialized: '
            f'system_id={self.system_id}, component_id={self.component_id}, '
            f'target={mavlink_host}:{mavlink_port}'
        )

        # UDP socket
        self._socket: Optional[socket.socket] = None
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.connect((mavlink_host, mavlink_port))
            self.get_logger().info(f'Connected to MAVLink endpoint {mavlink_host}:{mavlink_port}')
        except OSError as e:
            self.get_logger().error(f'Failed to connect UDP socket: {e}')

        # Sequence counter
        self._sequence = 0

        # MAVLink's time_boot_ms is milliseconds since system boot (uint32), not
        # a Unix timestamp -- time.time()*1000 is ~1.7e12 for any 2020s date,
        # which overflows uint32 (max ~4.3e9) and previously crashed struct.pack
        # on every telemetry publish once a GPS fix was present. Track our own
        # monotonic reference instead so the value starts near 0 and only grows
        # with node uptime.
        self._boot_time = time.monotonic()

        # Telemetry cache
        self._local_pos: Optional[VehicleLocalPosition] = None
        self._attitude: Optional[VehicleAttitude] = None
        self._angular_velocity: Optional[VehicleAngularVelocity] = None
        self._vehicle_status: Optional[VehicleStatus] = None
        self._battery_status: Optional[BatteryStatus] = None
        self._global_pos: Optional[VehicleGlobalPosition] = None
        self._cpuload: Optional[Cpuload] = None

        # Home position (for relative altitude)
        self._home_alt = 0.0

        # QoS profile for PX4
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to telemetry topics
        self.create_subscription(
            VehicleLocalPosition,
            f'{self.topic_prefix}/fmu/out/vehicle_local_position_v1',
            self._cb_local_position,
            qos
        )

        self.create_subscription(
            VehicleAttitude,
            f'{self.topic_prefix}/fmu/out/vehicle_attitude',
            self._cb_attitude,
            qos
        )

        self.create_subscription(
            VehicleAngularVelocity,
            f'{self.topic_prefix}/fmu/out/vehicle_angular_velocity',
            self._cb_angular_velocity,
            qos
        )

        self.create_subscription(
            VehicleStatus,
            f'{self.topic_prefix}/fmu/out/vehicle_status_v2',
            self._cb_vehicle_status,
            qos
        )

        self.create_subscription(
            BatteryStatus,
            f'{self.topic_prefix}/fmu/out/battery_status_v1',
            self._cb_battery_status,
            qos
        )

        self.create_subscription(
            VehicleGlobalPosition,
            f'{self.topic_prefix}/fmu/out/vehicle_global_position',
            self._cb_global_position,
            qos
        )

        self.create_subscription(
            Cpuload,
            f'{self.topic_prefix}/fmu/out/cpuload',
            self._cb_cpuload,
            qos
        )

        # Publish heartbeat at 1 Hz
        self.create_timer(1.0, self._publish_heartbeat)

        # Publish position/attitude telemetry at 10 Hz
        self.create_timer(0.1, self._publish_telemetry)

        self.get_logger().info('Telemetry MAVLink Bridge started')

    # ===== Telemetry Callbacks =====

    def _cb_local_position(self, msg: VehicleLocalPosition):
        """Cache local position."""
        self._local_pos = msg
        if msg.z_valid and self._home_alt == 0.0:
            self._home_alt = -msg.z  # NED: negative Z is altitude above home

    def _cb_attitude(self, msg: VehicleAttitude):
        """Cache attitude."""
        self._attitude = msg

    def _cb_angular_velocity(self, msg: VehicleAngularVelocity):
        """Cache body-frame angular rates (roll/pitch/yaw speed) -- a
        separate topic from VehicleAttitude, which carries orientation only."""
        self._angular_velocity = msg

    def _cb_vehicle_status(self, msg: VehicleStatus):
        """Cache vehicle status."""
        self._vehicle_status = msg

    def _cb_battery_status(self, msg: BatteryStatus):
        """Cache battery status."""
        self._battery_status = msg

    def _cb_global_position(self, msg: VehicleGlobalPosition):
        """Cache fused global position estimate."""
        self._global_pos = msg

    def _cb_cpuload(self, msg: Cpuload):
        """Cache CPU load -- a separate topic from VehicleStatus, which has
        no load field."""
        self._cpuload = msg

    # ===== Publishers =====

    def _publish_heartbeat(self):
        """Publish HEARTBEAT at 1 Hz."""
        if self._vehicle_status is None:
            return

        # Determine system state. VehicleStatus has no `system_status` field
        # (never did, at least as far back as px4_msgs v1.14.0) -- arming
        # state is the only signal available here.
        mav_state = MAVState.ACTIVE if self._vehicle_status.arming_state == 2 else MAVState.STANDBY

        payload = mav.build_heartbeat(
            type_=MAVType.QUADROTOR,
            autopilot=4,  # MAV_AUTOPILOT_PX4
            base_mode=192 if self._vehicle_status.arming_state == 2 else 0,  # ARMED flag
            custom_mode=self._vehicle_status.nav_state,
            system_status=int(mav_state),
            mavlink_version=3
        )

        self._send_mavlink_frame(mav.MAVLINK_MSG_ID_HEARTBEAT, payload)

    def _publish_telemetry(self):
        """Publish position/attitude telemetry at 10 Hz."""
        if self._local_pos is None or self._attitude is None:
            return

        # Global Position. VehicleGlobalPosition (the fused EKF estimate) is
        # used here rather than SensorGps (raw receiver data) because it's
        # reliably exported by PX4's default DDS topic config -- SensorGps
        # is not, in every environment checked so far. Its lat/lon are plain
        # float64 degrees (not the 1e7-scaled int32 SensorGps/MAVLink use
        # directly), so they need explicit scaling here.
        if self._global_pos is not None and self._global_pos.lat_lon_valid:
            payload = mav.build_global_position_int(
                time_boot_ms=self._time_boot_ms(),
                lat=int(self._global_pos.lat * 1e7),
                lon=int(self._global_pos.lon * 1e7),
                alt=int(self._global_pos.alt * 1000),
                relative_alt=int(-self._local_pos.z * 1000),
                vx=int(self._local_pos.vx * 100),
                vy=int(self._local_pos.vy * 100),
                vz=int(self._local_pos.vz * 100),
                hdg=int(self._local_pos.heading * 100),
            )
            self._send_mavlink_frame(mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, payload)

        # Attitude. Angular rates are NOT fields on VehicleAttitude (that
        # message carries orientation only) -- they come from the separate
        # VehicleAngularVelocity topic, which may not have arrived yet even
        # once attitude has, so default to 0.0 rather than blocking on it.
        roll, pitch, yaw = self._quaternion_to_euler(self._attitude.q)
        if self._angular_velocity is not None:
            rollspeed, pitchspeed, yawspeed = self._angular_velocity.xyz
        else:
            rollspeed = pitchspeed = yawspeed = 0.0
        payload = mav.build_attitude(
            time_boot_ms=self._time_boot_ms(),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            rollspeed=float(rollspeed),
            pitchspeed=float(pitchspeed),
            yawspeed=float(yawspeed),
        )
        self._send_mavlink_frame(mav.MAVLINK_MSG_ID_ATTITUDE, payload)

        # System Status. CPU load is NOT a VehicleStatus field -- it comes
        # from the separate Cpuload topic; default to 0 if it hasn't arrived
        # yet rather than blocking SYS_STATUS on a third, unrelated topic.
        if self._vehicle_status is not None:
            load_fraction = self._cpuload.load if self._cpuload is not None else 0.0
            payload = mav.build_sys_status(
                # All 32 bits reported present/enabled/healthy — the bridge has no
                # per-sensor bit mapping, so this is a best-effort "all good" default
                # (previously 0xFFFF, a 16-bit pattern, was packed into a field that
                # was itself incorrectly only 16 bits wide; both are fixed here).
                sensors_present=0xFFFFFFFF,
                sensors_enabled=0xFFFFFFFF,
                sensors_health=0xFFFFFFFF,
                load=int(load_fraction * 1000),
                voltage_battery=int(self._get_battery_voltage() * 1000),
                current_battery=int(self._get_battery_current() * 100),
                battery_remaining=int(self._get_battery_remaining()),
                drop_rate_comm=0,
                errors_comm=0,
                errors_count1=0,
                errors_count2=0,
                errors_count3=0,
                errors_count4=0,
            )
            self._send_mavlink_frame(mav.MAVLINK_MSG_ID_SYS_STATUS, payload)

        # Battery Status. energy_consumed is NOT a BatteryStatus field --
        # PX4 doesn't track joules consumed (only discharged_mah, already
        # used above) -- so report it as unknown (-1), MAVLink's documented
        # sentinel for this field, rather than inventing a value.
        if self._battery_status is not None:
            payload = mav.build_battery_status(
                id_=0,
                battery_function=0,  # MAV_BATTERY_FUNCTION_ALL
                type_=2,  # MAV_BATTERY_TYPE_LIPO
                # MAVLink's temperature field is centidegrees C (int16), with
                # INT16_MAX documented as its "unknown" sentinel. PX4's
                # BatteryStatus.temperature is a plain degC float, and SITL's
                # simulated battery model leaves it as NaN (PX4's own
                # "unknown" convention for float fields) rather than a real
                # reading -- int(nan) raises ValueError, so that has to be
                # mapped to MAVLink's sentinel instead of converted directly.
                # Never exercised before the topic-name fix above, since
                # battery_status was never actually received (dead code path).
                temperature=(
                    32767 if math.isnan(self._battery_status.temperature)
                    else int(self._battery_status.temperature * 100)
                ),
                voltages=[int(v * 1000) for v in self._battery_status.voltage_cell_v[:10]],
                current_battery=int(self._battery_status.current_a * 100),
                current_consumed=int(self._battery_status.discharged_mah),
                energy_consumed=-1,
                battery_remaining=int(self._get_battery_remaining()),
                time_remaining=0,
                charge_state=0,
            )
            self._send_mavlink_frame(mav.MAVLINK_MSG_ID_BATTERY_STATUS, payload)

    # ===== MAVLink Frame Transmission =====

    def _send_mavlink_frame(self, msg_id: int, payload: bytes):
        """Send a spec-compliant MAVLink 2.0 frame."""
        if self._socket is None:
            return

        seq = self._sequence % 256
        self._sequence += 1

        frame = mav.build_frame(msg_id, seq, payload, self.system_id, self.component_id)

        try:
            self._socket.send(frame)
        except OSError as e:
            self.get_logger().warn(f'Failed to send MAVLink packet: {e}')

    # ===== Utility Helpers =====

    def _time_boot_ms(self) -> int:
        """Milliseconds since this node started, wrapped to fit uint32."""
        return int((time.monotonic() - self._boot_time) * 1000) & 0xFFFFFFFF

    def _get_battery_voltage(self) -> float:
        """Get battery voltage in volts."""
        if self._battery_status is None:
            return 0.0
        # voltage_cell_v is already in volts (same field BATTERY_STATUS's
        # per-cell voltages above correctly treat as volts) -- do not divide
        # by 1000 again, or the summed pack voltage comes out ~1000x too low.
        return sum(self._battery_status.voltage_cell_v[:self._battery_status.cell_count])

    def _get_battery_current(self) -> float:
        """Get battery current in amps."""
        if self._battery_status is None:
            return 0.0
        return self._battery_status.current_a

    def _get_battery_remaining(self) -> int:
        """Get battery remaining percentage."""
        if self._battery_status is None:
            return 0
        return int(self._battery_status.remaining * 100)

    @staticmethod
    def _quaternion_to_euler(q) -> tuple:
        """Convert quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw)."""
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        # Roll (X rotation)
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

        # Pitch (Y rotation)
        sin_pitch = 2.0 * (w * y - z * x)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch = math.asin(sin_pitch)

        # Yaw (Z rotation)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return roll, pitch, yaw


def main(args=None):
    """Entry point for telemetry MAVLink bridge."""
    rclpy.init(args=args)
    node = TelemetryMAVLinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
