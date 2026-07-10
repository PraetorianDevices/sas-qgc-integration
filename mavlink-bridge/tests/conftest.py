"""
Shared pytest configuration and fixtures for MAVLink bridge tests.

This module provides:
  - Pytest configuration
  - Shared fixtures for MAVLink frame building, delegating to the real,
    pymavlink-verified mavlink_v2 module rather than a parallel
    reimplementation
  - ROS 2 node stubs for isolated testing
"""

import sys
from pathlib import Path

import pytest

# mavlink_v2.py lives at the mavlink-bridge/ root, one level above tests/.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mavlink_v2 as mav


class MAVLinkBuilder:
    """Thin adapter over the real mavlink_v2 codec, kept for tests that were
    already written against this fixture's method names. Delegates entirely
    to mavlink_v2 -- it does not reimplement frame/CRC logic itself, which is
    exactly what let the original 7-byte-header bug go undetected across the
    whole test suite despite 100% pass rate.
    """

    @staticmethod
    def build_frame(msg_id: int, seq: int, payload: bytes, system_id: int = 1, component_id: int = 1) -> bytes:
        return mav.build_frame(msg_id, seq, payload, system_id, component_id)

    @staticmethod
    def compute_crc(data: bytes, msg_id: int) -> int:
        return mav.compute_crc(data, msg_id)

    @staticmethod
    def parse_frame(frame: bytes) -> dict:
        parsed = mav.parse_frame(frame)
        if parsed is None:
            return None
        return {
            'stx': mav.MAVLINK_STX,
            'payload_len': len(parsed.payload),
            'msg_id': parsed.msg_id,
            'system_id': parsed.system_id,
            'component_id': parsed.component_id,
            'sequence': parsed.sequence,
            'payload': parsed.payload,
            'crc': parsed.crc,
            'valid': parsed.valid,
        }


# Backward-compatible alias -- some earlier test files imported this name directly.
MockMAVLinkBuilder = MAVLinkBuilder


@pytest.fixture
def mavlink_builder():
    """Provide the MAVLink frame builder/parser for tests."""
    return MAVLinkBuilder


@pytest.fixture
def sample_heartbeat_payload():
    """Sample HEARTBEAT payload, built via the real, verified codec."""
    return mav.build_heartbeat(
        type_=2, autopilot=4, base_mode=0x80, custom_mode=4, mavlink_version=3)


@pytest.fixture
def sample_global_position_payload():
    """Sample GLOBAL_POSITION_INT payload (San Francisco), built via the
    real, verified codec."""
    return mav.build_global_position_int(
        time_boot_ms=1000, lat=377_749_000, lon=-1_224_194_000, alt=500_000,
        relative_alt=100_000, vx=250, vy=-100, vz=50, hdg=9000)


@pytest.fixture
def sample_attitude_payload():
    """Sample ATTITUDE payload, built via the real, verified codec."""
    return mav.build_attitude(
        time_boot_ms=1000, roll=0.1, pitch=-0.2, yaw=1.57,
        rollspeed=0.05, pitchspeed=0.02, yawspeed=0.01)


@pytest.fixture
def sample_battery_status_payload():
    """Sample BATTERY_STATUS payload, built via the real, verified codec."""
    return mav.build_battery_status(
        id_=0, battery_function=0, type_=2, temperature=25,
        voltages=[4200, 4190, 4180, 0, 0, 0, 0, 0, 0, 0],
        current_battery=250, current_consumed=2500, energy_consumed=45_000,
        battery_remaining=75)


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
