#!/usr/bin/env python3
"""
Unit Test: px4_msgs → MAVLink Conversion (telemetry_mavlink_bridge.py)

Tests the real TelemetryMAVLinkBridge against realistic PX4-shaped telemetry,
plus the pure arithmetic conversion formulas it relies on internally (unit
scaling, coordinate frames) as standalone documentation/consistency checks.

A previous version of this file never imported telemetry_mavlink_bridge at
all -- TestMAVLinkMessagePayloads reimplemented struct.pack calls matching
the bridge's old, broken SYS_STATUS/BATTERY_STATUS field layout (16-bit
sensor bitmasks, scrambled BATTERY_STATUS field order) and asserted on that
copy's byte length, which is exactly why those two tests kept "passing" while
the real bridge was broken. TestMAVLinkMessagePayloads is replaced here with
tests that construct the real bridge and inspect its real output.

NOT REDUNDANT with SAS tests:
  - SAS tests verify offboard_controller outputs px4_msgs correctly
  - These tests verify MAVLink bridge converts those messages correctly
  - Complementary, not overlapping
"""

import math
import struct
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


# rclpy/std_msgs/px4_msgs stubs are installed once in tests/conftest.py,
# shared across every test file -- see that module's docstring for the
# collision this consolidation avoids.
import mavlink_v2 as mav
from telemetry_mavlink_bridge import TelemetryMAVLinkBridge


def _make_bridge():
    """Construct TelemetryMAVLinkBridge bypassing __init__ (socket/ROS setup)."""
    bridge = TelemetryMAVLinkBridge.__new__(TelemetryMAVLinkBridge)
    bridge.system_id = 1
    bridge.component_id = 1
    bridge._sequence = 0
    bridge._boot_time = __import__('time').monotonic()
    bridge.get_logger = lambda: MagicMock()
    bridge._local_pos = None
    bridge._attitude = None
    bridge._angular_velocity = None
    bridge._vehicle_status = None
    bridge._battery_status = None
    bridge._sensor_gps = None
    bridge._cpuload = None
    return bridge


def _capture_socket(bridge):
    """Attach a fake socket to `bridge` and return the list its sent frames
    accumulate into."""
    sent = []
    bridge._socket = types.SimpleNamespace(send=lambda f: sent.append(f))
    return sent


def _parsed(frames, msg_id):
    """Return the ParsedFrame for the first frame in `frames` matching msg_id."""
    for f in frames:
        p = mav.parse_frame(f)
        if p and p.msg_id == msg_id:
            return p
    return None


class TestCoordinateConversionFormulas:
    """Pure-arithmetic documentation/consistency checks for the conversion
    formulas used inline in _publish_telemetry -- there's no separately
    extractable function for these in the bridge, so these test the formula
    itself rather than a real-module call."""

    def test_ned_altitude_from_local_position(self):
        z_ned = -10.5  # 10.5 meters above home
        altitude = -z_ned
        assert altitude == 10.5

    def test_gps_position_scaling_matches_bridge_convention(self):
        lat_deg = 37.7749
        lon_deg = -122.4194
        lat_scaled = int(lat_deg * 1e7)
        lon_scaled = int(lon_deg * 1e7)
        assert lat_scaled == 377_749_000
        assert lon_scaled == -1_224_194_000

    def test_heading_conversion_radians_to_centidegrees(self):
        heading_rad = math.pi / 2
        heading_mavlink = int(math.degrees(heading_rad) * 100)
        assert heading_mavlink == 9_000

    def test_velocity_ms_to_cms(self):
        vel_ms = [2.5, -1.0, 0.5]
        assert [int(v * 100) for v in vel_ms] == [250, -100, 50]


class TestQuaternionToEulerReal:
    """Calls the real TelemetryMAVLinkBridge._quaternion_to_euler."""

    def test_identity_quaternion(self):
        roll, pitch, yaw = TelemetryMAVLinkBridge._quaternion_to_euler([1, 0, 0, 0])
        assert abs(roll) < 1e-6
        assert abs(pitch) < 1e-6
        assert abs(yaw) < 1e-6

    def test_90_degree_roll(self):
        angle = math.pi / 4  # half-angle for 90 deg rotation
        q = [math.cos(angle), math.sin(angle), 0, 0]
        roll, pitch, yaw = TelemetryMAVLinkBridge._quaternion_to_euler(q)
        assert abs(roll - math.pi / 2) < 1e-6
        assert abs(pitch) < 1e-5
        assert abs(yaw) < 1e-5

    def test_non_unit_quaternion_still_bounded(self):
        q_raw = [2, 1, 0.5, 0.5]
        mag = math.sqrt(sum(x ** 2 for x in q_raw))
        q = [x / mag for x in q_raw]
        roll, pitch, yaw = TelemetryMAVLinkBridge._quaternion_to_euler(q)
        assert -math.pi <= roll <= math.pi
        assert -math.pi / 2 <= pitch <= math.pi / 2
        assert -math.pi <= yaw <= math.pi


class TestHeartbeatReal:

    def test_armed_sets_base_mode_192(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._vehicle_status = types.SimpleNamespace(arming_state=2, nav_state=4)

        bridge._publish_heartbeat()

        parsed = mav.parse_frame(sent[-1])
        assert parsed.valid
        _, _, _, base_mode, _, _ = struct.unpack('<IBBBBB', parsed.payload.ljust(9, b'\x00'))
        assert base_mode == 192

    def test_disarmed_sets_base_mode_0(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._vehicle_status = types.SimpleNamespace(arming_state=1, nav_state=0)

        bridge._publish_heartbeat()

        parsed = mav.parse_frame(sent[-1])
        _, _, _, base_mode, _, _ = struct.unpack('<IBBBBB', parsed.payload.ljust(9, b'\x00'))
        assert base_mode == 0

    def test_no_vehicle_status_sends_nothing(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._publish_heartbeat()
        assert sent == []


class TestPublishTelemetryReal:
    """Drives the real _publish_telemetry with realistic PX4-shaped data and
    inspects the real transmitted frames -- this is what would have caught
    the SYS_STATUS field-width bug and the BATTERY_STATUS voltage bug."""

    @pytest.fixture
    def bridge_with_telemetry(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._local_pos = types.SimpleNamespace(
            z=-50.0, z_valid=True, vx=2.5, vy=-1.0, vz=0.1, heading=1.57)
        bridge._attitude = types.SimpleNamespace(q=[0.966, 0.0, 0.0, 0.259])
        bridge._angular_velocity = types.SimpleNamespace(xyz=[0.01, 0.02, 0.03])
        bridge._vehicle_status = types.SimpleNamespace(arming_state=2, nav_state=4)
        bridge._sensor_gps = types.SimpleNamespace(
            fix_type=3, lat=377_749_000, lon=-1_224_194_000, alt=100_500)
        bridge._battery_status = types.SimpleNamespace(
            temperature=25, voltage_cell_v=[4.2, 4.19, 4.18, 4.17, 0, 0, 0, 0, 0, 0],
            cell_count=4, current_a=12.5, discharged_mah=850, remaining=0.72)
        bridge._cpuload = types.SimpleNamespace(load=0.35)
        return bridge, sent

    def test_all_frames_pass_crc_validation(self, bridge_with_telemetry):
        bridge, sent = bridge_with_telemetry
        bridge._publish_telemetry()
        assert len(sent) == 4
        for frame in sent:
            assert mav.parse_frame(frame).valid

    def test_global_position_int_roundtrips_coordinates(self, bridge_with_telemetry):
        bridge, sent = bridge_with_telemetry
        bridge._publish_telemetry()
        parsed = _parsed(sent, mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
        _, lat, lon, *_ = struct.unpack('<Iiiiihhh' + 'H', parsed.payload.ljust(28, b'\x00')[:28])
        assert lat == 377_749_000
        assert lon == -1_224_194_000

    def test_sys_status_uses_full_32_bit_sensor_mask(self, bridge_with_telemetry):
        """Regression test: the old bridge packed these three fields as
        16-bit, silently truncating the value and shrinking the whole
        payload by 6 bytes."""
        bridge, sent = bridge_with_telemetry
        bridge._publish_telemetry()
        parsed = _parsed(sent, mav.MAVLINK_MSG_ID_SYS_STATUS)
        present, enabled, health, *_ = struct.unpack(
            '<IIIHHhHHHHHHb', parsed.payload.ljust(31, b'\x00')[:31])
        assert present == 0xFFFFFFFF
        assert enabled == 0xFFFFFFFF
        assert health == 0xFFFFFFFF

    def test_battery_status_reports_real_cell_voltages_not_zero(self, bridge_with_telemetry):
        """Regression test for the bug where a loop over enumerate([0]*10)
        meant real per-cell voltages were discarded and always sent as 0,
        regardless of the vehicle's actual battery state."""
        bridge, sent = bridge_with_telemetry
        bridge._publish_telemetry()
        parsed = _parsed(sent, mav.MAVLINK_MSG_ID_BATTERY_STATUS)
        _, _, _, *rest = struct.unpack('<iih10HhBBBb', parsed.payload.ljust(36, b'\x00')[:36])
        voltages = rest[:10]
        assert voltages[0] == 4200
        assert voltages[1] == 4190
        assert voltages[2] == 4180
        assert voltages[3] == 4170
        assert any(v != 0 for v in voltages)

    def test_sys_status_battery_voltage_is_pack_voltage_not_1000x_low(self, bridge_with_telemetry):
        """Regression test: _get_battery_voltage() summed already-in-volts
        cell readings and then divided by 1000 again, reporting ~1000x too
        low (e.g. 16.74V pack reported as 0.01674V). voltage_battery is in
        millivolts per the MAVLink spec, so 4 cells at ~4.2V should arrive
        as roughly 16740, not 16 or 17."""
        bridge, sent = bridge_with_telemetry
        bridge._publish_telemetry()
        parsed = _parsed(sent, mav.MAVLINK_MSG_ID_SYS_STATUS)
        _, _, _, load, voltage_battery, *_ = struct.unpack(
            '<IIIHHhHHHHHHb', parsed.payload.ljust(31, b'\x00')[:31])
        assert voltage_battery == pytest.approx(16_740, abs=5)

    def test_no_gps_fix_skips_global_position_int(self, bridge_with_telemetry):
        bridge, sent = bridge_with_telemetry
        bridge._sensor_gps.fix_type = 0  # no fix
        bridge._publish_telemetry()
        assert _parsed(sent, mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT) is None

    def test_no_local_position_sends_nothing(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._attitude = types.SimpleNamespace(q=[1, 0, 0, 0])
        bridge._publish_telemetry()  # _local_pos is still None
        assert sent == []

    def test_attitude_rates_come_from_angular_velocity_topic(self, bridge_with_telemetry):
        """Regression test: rollspeed/pitchspeed/yawspeed are not fields on
        VehicleAttitude at all (confirmed absent from real px4_msgs, not
        just renamed) -- they come from the separate VehicleAngularVelocity
        topic's `xyz` field."""
        bridge, sent = bridge_with_telemetry
        bridge._publish_telemetry()
        parsed = _parsed(sent, mav.MAVLINK_MSG_ID_ATTITUDE)
        _, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = struct.unpack(
            '<Iffffff', parsed.payload.ljust(28, b'\x00')[:28])
        assert rollspeed == pytest.approx(0.01, abs=1e-6)
        assert pitchspeed == pytest.approx(0.02, abs=1e-6)
        assert yawspeed == pytest.approx(0.03, abs=1e-6)

    def test_no_angular_velocity_yet_defaults_rates_to_zero(self, bridge_with_telemetry):
        """Angular velocity is a separate topic from attitude and may not
        have arrived yet even once attitude has -- must not block or crash
        ATTITUDE publishing, just report 0.0 rates."""
        bridge, sent = bridge_with_telemetry
        bridge._angular_velocity = None
        bridge._publish_telemetry()  # must not raise
        parsed = _parsed(sent, mav.MAVLINK_MSG_ID_ATTITUDE)
        assert parsed is not None and parsed.valid
        _, _, _, _, rollspeed, pitchspeed, yawspeed = struct.unpack(
            '<Iffffff', parsed.payload.ljust(28, b'\x00')[:28])
        assert (rollspeed, pitchspeed, yawspeed) == (0.0, 0.0, 0.0)

    def test_no_cpuload_yet_defaults_sys_status_load_to_zero(self, bridge_with_telemetry):
        """CPU load is not a VehicleStatus field -- it comes from the
        separate Cpuload topic, which may not have arrived yet; must not
        block or crash SYS_STATUS publishing."""
        bridge, sent = bridge_with_telemetry
        bridge._cpuload = None
        bridge._publish_telemetry()  # must not raise
        parsed = _parsed(sent, mav.MAVLINK_MSG_ID_SYS_STATUS)
        _, _, _, load, *_ = struct.unpack(
            '<IIIHHhHHHHHHb', parsed.payload.ljust(31, b'\x00')[:31])
        assert load == 0

    def test_battery_status_energy_consumed_reported_unknown(self, bridge_with_telemetry):
        """Regression test: energy_consumed_j is not a BatteryStatus field
        (PX4 doesn't track joules consumed) -- must report MAVLink's
        documented "unknown" sentinel (-1), not a fabricated value."""
        bridge, sent = bridge_with_telemetry
        bridge._publish_telemetry()
        parsed = _parsed(sent, mav.MAVLINK_MSG_ID_BATTERY_STATUS)
        current_consumed, energy_consumed, *_ = struct.unpack(
            '<iih10HhBBBb', parsed.payload.ljust(36, b'\x00')[:36])
        assert energy_consumed == -1

    def test_time_boot_ms_does_not_overflow_uint32(self, bridge_with_telemetry):
        """Regression test: time_boot_ms=int(time.time()*1000) overflows
        uint32 for any real-world timestamp and crashes struct.pack. This
        must stay well under 2**32 for a freshly-constructed bridge."""
        bridge, sent = bridge_with_telemetry
        bridge._publish_telemetry()  # must not raise
        parsed = _parsed(sent, mav.MAVLINK_MSG_ID_ATTITUDE)
        time_boot_ms = struct.unpack('<I', parsed.payload[:4])[0]
        assert 0 <= time_boot_ms < 2**32
        assert time_boot_ms < 60_000  # test runs in well under a minute


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
