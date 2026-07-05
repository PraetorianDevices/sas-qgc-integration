"""
Shared pytest configuration and fixtures for MAVLink bridge tests.

This module provides:
  - Pytest configuration
  - Shared fixtures for MAVLink frame building
  - ROS 2 node stubs for isolated testing
"""

import struct
import pytest


class MockMAVLinkBuilder:
    """Helper class for building and validating MAVLink frames in tests."""

    @staticmethod
    def build_frame(msg_id: int, seq: int, payload: bytes, system_id: int = 1, component_id: int = 1) -> bytes:
        """Build a complete MAVLink 2.0 frame."""
        stx = 0xFD
        payload_len = len(payload)
        incomp_flags = 0x00

        frame_data = struct.pack(
            '<BBBBBBB',
            stx, payload_len, incomp_flags, msg_id & 0xFF,
            system_id, component_id, seq
        ) + payload

        crc = MockMAVLinkBuilder.compute_crc(frame_data[1:], msg_id)
        return frame_data + struct.pack('<H', crc)

    @staticmethod
    def compute_crc(data: bytes, msg_id: int) -> int:
        """Compute MAVLink CRC16-CCITT."""
        CRC_INIT = 0xFFFF
        CRC_EXTRA_MAP = {
            0: 50,      # HEARTBEAT
            1: 124,     # SYS_STATUS
            30: 15,     # ATTITUDE
            32: 49,     # LOCAL_POSITION_NED
            33: 104,    # GLOBAL_POSITION_INT
            147: 60,    # BATTERY_STATUS
            253: 83,    # STATUSTEXT
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

    @staticmethod
    def parse_frame(frame: bytes) -> dict:
        """Parse a MAVLink 2.0 frame into components."""
        if len(frame) < 10:
            return None

        stx = frame[0]
        payload_len = frame[1]
        msg_id = frame[3]
        system_id = frame[4]
        component_id = frame[5]
        sequence = frame[6]
        payload = frame[7:7+payload_len]
        crc = struct.unpack('<H', frame[-2:])[0]

        return {
            'stx': stx,
            'payload_len': payload_len,
            'msg_id': msg_id,
            'system_id': system_id,
            'component_id': component_id,
            'sequence': sequence,
            'payload': payload,
            'crc': crc,
            'valid': stx == 0xFD and len(frame) == 7 + payload_len + 2,
        }


@pytest.fixture
def mavlink_builder():
    """Provide MAVLink frame builder for tests."""
    return MockMAVLinkBuilder


@pytest.fixture
def sample_heartbeat_payload():
    """Provide a sample HEARTBEAT payload."""
    custom_mode = 4  # Offboard
    type_ = 2  # Quadrotor
    autopilot = 4  # PX4
    base_mode = 0x80  # Armed
    system_status = 4  # Active
    mavlink_version = 3

    return struct.pack('<I B B B B B',
        custom_mode, type_, autopilot, base_mode, system_status, mavlink_version
    )


@pytest.fixture
def sample_global_position_payload():
    """Provide a sample GLOBAL_POSITION_INT payload."""
    time_boot_ms = 1000
    lat = 377_749_000  # San Francisco
    lon = -1_224_194_000
    alt = 500_000  # 500m MSL
    relative_alt = 100_000  # 100m above home
    vx = 250  # 2.5 m/s
    vy = -100
    vz = 50
    hdg = 9000  # 90 degrees

    return struct.pack('<I i i i i h h h H',
        time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg
    )


@pytest.fixture
def sample_attitude_payload():
    """Provide a sample ATTITUDE payload."""
    time_boot_ms = 1000
    roll = 0.1
    pitch = -0.2
    yaw = 1.57
    rollspeed = 0.05
    pitchspeed = 0.02
    yawspeed = 0.01

    return struct.pack('<I f f f f f f',
        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
    )


@pytest.fixture
def sample_battery_status_payload():
    """Provide a sample BATTERY_STATUS payload."""
    id_ = 0
    battery_function = 0
    type_ = 2
    temperature = 25
    current_battery = 250
    battery_remaining = 75
    charge_state = 0

    voltage_data = struct.pack('<10H',
        4200, 4190, 4180, 0, 0, 0, 0, 0, 0, 0
    )

    current_consumed = 2500
    energy_consumed = 45_000
    time_remaining = 0

    header = struct.pack('<i h h h h B B',
        id_, battery_function, type_, temperature,
        current_battery, battery_remaining, charge_state
    )

    footer = struct.pack('<h i h',
        current_consumed, energy_consumed, time_remaining
    )

    return header + voltage_data + footer


# Pytest configuration
def pytest_configure(config):
    """Configure pytest."""
    config.addinivalue_line(
        "markers", "unit: mark test as a unit test"
    )
    config.addinivalue_line(
        "markers", "integration: mark test as an integration test"
    )
    config.addinivalue_line(
        "markers", "ros2: mark test as requiring ROS 2 environment"
    )
