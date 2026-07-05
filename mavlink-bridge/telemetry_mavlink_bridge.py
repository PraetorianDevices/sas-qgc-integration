#!/usr/bin/env python3

"""
Telemetry Bridge: px4_msgs → MAVLink for QGroundControl

Subscribes to telemetry topics from offboard_controller_node and other sources,
converts to MAVLink messages, and publishes via UDP to QGroundControl.

Messages Published:
  - HEARTBEAT (MAV_TYPE_QUADROTOR)
  - GLOBAL_POSITION_INT (GPS position, heading, altitude)
  - LOCAL_POSITION_NED (relative position, velocity)
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
"""

import struct
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
    VehicleStatus,
    BatteryStatus,
    SensorGps,
)
from std_msgs.msg import String


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

    # MAVLink message IDs
    MAVLINK_MSG_ID_HEARTBEAT = 0
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
    MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32
    MAVLINK_MSG_ID_ATTITUDE = 30
    MAVLINK_MSG_ID_SYS_STATUS = 1
    MAVLINK_MSG_ID_BATTERY_STATUS = 147

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

        # Telemetry cache
        self._local_pos: Optional[VehicleLocalPosition] = None
        self._attitude: Optional[VehicleAttitude] = None
        self._vehicle_status: Optional[VehicleStatus] = None
        self._battery_status: Optional[BatteryStatus] = None
        self._sensor_gps: Optional[SensorGps] = None

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
            f'{self.topic_prefix}/fmu/out/vehicle_local_position',
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
            VehicleStatus,
            f'{self.topic_prefix}/fmu/out/vehicle_status',
            self._cb_vehicle_status,
            qos
        )

        self.create_subscription(
            BatteryStatus,
            f'{self.topic_prefix}/fmu/out/battery_status',
            self._cb_battery_status,
            qos
        )

        self.create_subscription(
            SensorGps,
            f'{self.topic_prefix}/fmu/out/sensor_gps',
            self._cb_sensor_gps,
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

    def _cb_vehicle_status(self, msg: VehicleStatus):
        """Cache vehicle status."""
        self._vehicle_status = msg

    def _cb_battery_status(self, msg: BatteryStatus):
        """Cache battery status."""
        self._battery_status = msg

    def _cb_sensor_gps(self, msg: SensorGps):
        """Cache GPS data."""
        self._sensor_gps = msg

    # ===== Publishers =====

    def _publish_heartbeat(self):
        """Publish HEARTBEAT at 1 Hz."""
        if self._vehicle_status is None:
            return

        # Determine system state
        if self._vehicle_status.arming_state == 2:  # ARMED
            mav_state = MAVState.ACTIVE
        elif self._vehicle_status.system_status == 4:  # Ready
            mav_state = MAVState.STANDBY
        else:
            mav_state = MAVState.STANDBY

        payload = self._build_heartbeat(
            type=MAVType.QUADROTOR,
            autopilot=4,  # MAV_AUTOPILOT_PX4
            base_mode=192 if self._vehicle_status.arming_state == 2 else 0,  # ARMED flag
            custom_mode=self._vehicle_status.nav_state,
            system_status=int(mav_state),
            mavlink_version=3
        )

        self._send_mavlink_frame(self.MAVLINK_MSG_ID_HEARTBEAT, payload)

    def _publish_telemetry(self):
        """Publish position/attitude telemetry at 10 Hz."""
        if self._local_pos is None or self._attitude is None:
            return

        # Global Position
        if self._sensor_gps is not None and self._sensor_gps.fix_type >= 3:
            payload = self._build_global_position_int(
                time_boot_ms=int(time.time() * 1000),
                lat=int(self._sensor_gps.lat),
                lon=int(self._sensor_gps.lon),
                alt=int(self._sensor_gps.alt * 1000),
                relative_alt=int(-self._local_pos.z * 1000),
                vx=int(self._local_pos.vx * 100),
                vy=int(self._local_pos.vy * 100),
                vz=int(self._local_pos.vz * 100),
                hdg=int(self._local_pos.heading * 100),
            )
            self._send_mavlink_frame(self.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, payload)

        # Attitude
        roll, pitch, yaw = self._quaternion_to_euler(self._attitude.q)
        payload = self._build_attitude(
            time_boot_ms=int(time.time() * 1000),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            rollspeed=self._attitude.rollspeed,
            pitchspeed=self._attitude.pitchspeed,
            yawspeed=self._attitude.yawspeed,
        )
        self._send_mavlink_frame(self.MAVLINK_MSG_ID_ATTITUDE, payload)

        # System Status
        if self._vehicle_status is not None:
            payload = self._build_sys_status(
                onboard_control_sensors_present=0xFFFF,
                onboard_control_sensors_enabled=0xFFFF,
                onboard_control_sensors_health=0xFFFF,
                load=int(self._vehicle_status.load * 1000),
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
            self._send_mavlink_frame(self.MAVLINK_MSG_ID_SYS_STATUS, payload)

        # Battery Status
        if self._battery_status is not None:
            payload = self._build_battery_status(
                id=0,
                battery_function=0,  # MAV_BATTERY_FUNCTION_ALL
                type=2,  # MAV_BATTERY_TYPE_LIPO
                temperature=self._battery_status.temperature,
                voltages=[int(v * 1000) for v in self._battery_status.voltage_cell_v[:10]],
                current_battery=int(self._battery_status.current_a * 100),
                current_consumed=int(self._battery_status.discharged_mah),
                energy_consumed=int(self._battery_status.energy_consumed_j * 1000),
                battery_remaining=int(self._get_battery_remaining()),
                time_remaining=0,
                charge_state=0,
            )
            self._send_mavlink_frame(self.MAVLINK_MSG_ID_BATTERY_STATUS, payload)

    # ===== MAVLink Message Builders =====

    def _build_heartbeat(self, type, autopilot, base_mode, custom_mode, system_status, mavlink_version):
        """Build HEARTBEAT payload."""
        return struct.pack('<I B B B B B',
            custom_mode,
            type,
            autopilot,
            base_mode,
            system_status,
            mavlink_version
        )

    def _build_global_position_int(self, time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg):
        """Build GLOBAL_POSITION_INT payload."""
        return struct.pack('<I i i i i h h h H',
            time_boot_ms,
            lat,
            lon,
            alt,
            relative_alt,
            vx,
            vy,
            vz,
            hdg
        )

    def _build_attitude(self, time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed):
        """Build ATTITUDE payload."""
        return struct.pack('<I f f f f f f',
            time_boot_ms,
            roll,
            pitch,
            yaw,
            rollspeed,
            pitchspeed,
            yawspeed
        )

    def _build_sys_status(self, onboard_control_sensors_present, onboard_control_sensors_enabled,
                         onboard_control_sensors_health, load, voltage_battery, current_battery,
                         battery_remaining, drop_rate_comm, errors_comm, errors_count1,
                         errors_count2, errors_count3, errors_count4):
        """Build SYS_STATUS payload."""
        return struct.pack('<H H H H H h b B H H H H H',
            onboard_control_sensors_present,
            onboard_control_sensors_enabled,
            onboard_control_sensors_health,
            load,
            voltage_battery,
            current_battery,
            battery_remaining,
            drop_rate_comm,
            errors_comm,
            errors_count1,
            errors_count2,
            errors_count3,
            errors_count4
        )

    def _build_battery_status(self, id, battery_function, type, temperature, voltages, current_battery,
                             current_consumed, energy_consumed, battery_remaining, time_remaining, charge_state):
        """Build BATTERY_STATUS payload."""
        # Pack voltages (up to 10 cells)
        voltage_data = struct.pack('<10H', *[v if i < len(voltages) else 0 for i, v in enumerate([0]*10)])

        return struct.pack('<i h h h h B B',
            id,
            battery_function,
            type,
            temperature,
            current_battery,
            battery_remaining,
            charge_state
        ) + voltage_data + struct.pack('<h i h',
            current_consumed,
            energy_consumed,
            time_remaining
        )

    # ===== MAVLink Frame Transmission =====

    def _send_mavlink_frame(self, msg_id: int, payload: bytes):
        """Send MAVLink 2.0 frame."""
        if self._socket is None:
            return

        seq = self._sequence % 256
        self._sequence += 1

        # Frame: [STX] [LEN] [INV] [MSG_ID] [SYSID] [COMPID] [SEQ] [PAYLOAD] [CRC]
        frame = self._build_mavlink_frame(msg_id, seq, payload)

        try:
            self._socket.send(frame)
        except OSError as e:
            self.get_logger().warn(f'Failed to send MAVLink packet: {e}')

    def _build_mavlink_frame(self, msg_id: int, seq: int, payload: bytes) -> bytes:
        """Build MAVLink 2.0 frame."""
        stx = 0xFD
        payload_len = len(payload)
        incomp_flags = 0x00

        frame_data = struct.pack(
            '<BBBBBBB',
            stx,
            payload_len,
            incomp_flags,
            msg_id & 0xFF,
            self.system_id,
            self.component_id,
            seq
        ) + payload

        crc = self._compute_mavlink_crc(frame_data[1:], msg_id)
        return frame_data + struct.pack('<H', crc)

    @staticmethod
    def _compute_mavlink_crc(data: bytes, msg_id: int) -> int:
        """Compute MAVLink CRC16-CCITT."""
        CRC_INIT = 0xFFFF
        CRC_POLY = 0xEF01

        # CRC_EXTRA values for common messages
        CRC_EXTRA_MAP = {
            0: 50,      # HEARTBEAT
            1: 124,     # SYS_STATUS
            30: 15,     # ATTITUDE
            32: 49,     # LOCAL_POSITION_NED
            33: 104,    # GLOBAL_POSITION_INT
            147: 60,    # BATTERY_STATUS
        }

        crc_extra = CRC_EXTRA_MAP.get(msg_id, 0)

        crc = CRC_INIT
        for byte in data:
            tmp = byte ^ (crc & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crc &= 0xFFFF

        tmp = crc_extra ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF

        return crc

    # ===== Utility Helpers =====

    def _get_battery_voltage(self) -> float:
        """Get battery voltage in volts."""
        if self._battery_status is None:
            return 0.0
        return sum(self._battery_status.voltage_cell_v[:self._battery_status.cell_count]) / 1000.0

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
