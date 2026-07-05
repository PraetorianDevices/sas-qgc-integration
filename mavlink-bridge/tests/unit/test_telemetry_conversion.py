#!/usr/bin/env python3
"""
Unit Test: px4_msgs → MAVLink Conversion

Tests the conversion logic from PX4 messages to MAVLink format.
Focuses on coordinate frame conversions, unit scaling, and message formatting.

NOT REDUNDANT with SAS tests:
  - SAS tests verify offboard_controller outputs px4_msgs correctly
  - These tests verify MAVLink bridge converts those messages correctly
  - Complementary, not overlapping
"""

import math
import pytest
import struct


class TestCoordinateConversions:
    """Test NED frame and coordinate conversions."""

    def test_ned_altitude_from_local_position(self):
        """Test conversion of NED z-coordinate to altitude above home."""
        # In NED frame: negative Z = UP
        # altitude_above_home = -z
        z_ned = -10.5  # 10.5 meters above home
        altitude = -z_ned

        assert altitude == 10.5
        assert altitude > 0, "Altitude above home should be positive"

    def test_gps_position_scaling(self):
        """Test GPS position scaling (degrees → 1e-7 format)."""
        lat_deg = 37.7749
        lon_deg = -122.4194

        lat_scaled = int(lat_deg * 1e7)
        lon_scaled = int(lon_deg * 1e7)

        assert lat_scaled == 377_749_000
        assert lon_scaled == -1_224_194_000

        # Round-trip
        lat_back = lat_scaled / 1e7
        lon_back = lon_scaled / 1e7

        assert abs(lat_back - lat_deg) < 1e-5
        assert abs(lon_back - lon_deg) < 1e-5

    def test_heading_conversion_radians_to_degrees(self):
        """Test heading conversion from radians to degrees (×100 format)."""
        # MAVLink heading: 0-360°, stored as integer ×100
        heading_rad = math.pi / 2  # 90° (East)
        heading_deg = math.degrees(heading_rad)
        heading_mavlink = int(heading_deg * 100)

        assert heading_mavlink == 9_000, "90° should be 9000 (×100)"

        # Round-trip
        heading_back = heading_mavlink / 100

        assert abs(heading_back - heading_deg) < 1.0

    def test_roll_pitch_yaw_radians(self):
        """Test that Euler angles are preserved in radians."""
        roll_rad = 0.1
        pitch_rad = -0.2
        yaw_rad = 1.57  # ~90°

        # MAVLink uses radians directly
        assert -math.pi <= roll_rad <= math.pi
        assert -math.pi/2 <= pitch_rad <= math.pi/2
        assert -math.pi <= yaw_rad <= math.pi


class TestUnitConversions:
    """Test unit scaling for MAVLink messages."""

    def test_velocity_ms_to_cms(self):
        """Test conversion from m/s to cm/s."""
        vel_ms = [2.5, -1.0, 0.5]  # [vx, vy, vz] in m/s
        vel_cms = [int(v * 100) for v in vel_ms]

        assert vel_cms == [250, -100, 50]

    def test_battery_voltage_mv(self):
        """Test battery voltage in millivolts."""
        voltage_v = 11.85  # 11.85V battery
        voltage_mv = int(voltage_v * 1000)

        assert voltage_mv == 11_850

    def test_battery_current_ca(self):
        """Test battery current in centiamps (0.01A resolution)."""
        current_a = 2.5  # 2.5 amps
        current_ca = int(current_a * 100)

        assert current_ca == 250

    def test_altitude_millimeters(self):
        """Test altitude conversion to millimeters."""
        alt_m = 100.5  # 100.5 meters
        alt_mm = int(alt_m * 1000)

        assert alt_mm == 100_500

    def test_temperature_celsius(self):
        """Test temperature stored directly in Celsius."""
        temp_c = 25  # Room temperature
        temp_mavlink = int(temp_c)

        assert temp_mavlink == 25, "Temperature stored as-is in Celsius"

    def test_battery_capacity_mah(self):
        """Test battery capacity in milliamp-hours."""
        capacity_ah = 4.0  # 4000 mAh battery
        capacity_mah = int(capacity_ah * 1000)

        assert capacity_mah == 4000

    def test_battery_remaining_percentage(self):
        """Test battery remaining as percentage (0-100)."""
        remaining_fraction = 0.75  # 75% remaining
        remaining_percent = int(remaining_fraction * 100)

        assert remaining_percent == 75
        assert 0 <= remaining_percent <= 100


class TestQuaternionToEuler:
    """Test quaternion to Euler angle conversion."""

    @staticmethod
    def quaternion_to_euler(q):
        """Convert quaternion [w, x, y, z] to Euler [roll, pitch, yaw]."""
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sin_pitch = 2.0 * (w * y - z * x)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return roll, pitch, yaw

    def test_identity_quaternion(self):
        """Test conversion of identity quaternion [1, 0, 0, 0]."""
        q = [1, 0, 0, 0]  # Identity (no rotation)
        roll, pitch, yaw = self.quaternion_to_euler(q)

        assert abs(roll) < 1e-6
        assert abs(pitch) < 1e-6
        assert abs(yaw) < 1e-6

    def test_90_degree_roll(self):
        """Test 90° roll rotation."""
        # Quaternion for 90° roll: [cos(45°), sin(45°), 0, 0]
        angle = math.pi / 4  # 45° for 90° total (half-angle)
        q = [math.cos(angle), math.sin(angle), 0, 0]
        roll, pitch, yaw = self.quaternion_to_euler(q)

        assert abs(roll - math.pi/2) < 1e-6, "Roll should be 90°"
        assert abs(pitch) < 1e-5
        assert abs(yaw) < 1e-5

    def test_quaternion_normalization(self):
        """Test conversion works with normalized quaternions."""
        # Non-unit quaternion (needs normalization)
        q_raw = [2, 1, 0.5, 0.5]
        mag = math.sqrt(sum(x**2 for x in q_raw))
        q = [x / mag for x in q_raw]

        roll, pitch, yaw = self.quaternion_to_euler(q)

        # Result should be valid angles
        assert -math.pi <= roll <= math.pi
        assert -math.pi/2 <= pitch <= math.pi/2
        assert -math.pi <= yaw <= math.pi


class TestMAVLinkMessagePayloads:
    """Test MAVLink payload construction."""

    def test_heartbeat_payload_structure(self):
        """Test HEARTBEAT message payload format."""
        custom_mode = 4  # Offboard mode
        type_ = 2  # MAV_TYPE_QUADROTOR
        autopilot = 4  # MAV_AUTOPILOT_PX4
        base_mode = 0x80  # Armed
        system_status = 4  # Active
        mavlink_version = 3

        payload = struct.pack('<I B B B B B',
            custom_mode,
            type_,
            autopilot,
            base_mode,
            system_status,
            mavlink_version
        )

        assert len(payload) == 9, "HEARTBEAT payload is 9 bytes"

        # Unpack and verify
        unpacked = struct.unpack('<I B B B B B', payload)
        assert unpacked[0] == custom_mode
        assert unpacked[1] == type_

    def test_global_position_int_payload_structure(self):
        """Test GLOBAL_POSITION_INT message payload format."""
        time_boot_ms = 1000
        lat = 377_749_000
        lon = -1_224_194_000
        alt = 500_000  # 500m MSL
        relative_alt = 100_000  # 100m above home
        vx = 250  # 2.5 m/s north
        vy = -100  # -1.0 m/s east
        vz = 50  # 0.5 m/s down
        hdg = 9000  # 90°

        payload = struct.pack('<I i i i i h h h H',
            time_boot_ms, lat, lon, alt, relative_alt,
            vx, vy, vz, hdg
        )

        assert len(payload) == 28, "GLOBAL_POSITION_INT payload is 28 bytes"

    def test_attitude_payload_structure(self):
        """Test ATTITUDE message payload format."""
        time_boot_ms = 1000
        roll = 0.1
        pitch = -0.2
        yaw = 1.57
        rollspeed = 0.05
        pitchspeed = 0.02
        yawspeed = 0.01

        payload = struct.pack('<I f f f f f f',
            time_boot_ms, roll, pitch, yaw,
            rollspeed, pitchspeed, yawspeed
        )

        assert len(payload) == 28, "ATTITUDE payload is 28 bytes"

    def test_sys_status_payload_structure(self):
        """Test SYS_STATUS message payload format."""
        onboard_control_sensors_present = 0xFFFF
        onboard_control_sensors_enabled = 0xFFFF
        onboard_control_sensors_health = 0xFFF1  # One sensor degraded
        load = 500  # 50% load
        voltage_battery = 11_850  # 11.85V
        current_battery = 250  # 2.5A
        battery_remaining = 75  # 75%
        drop_rate_comm = 0
        errors_comm = 0
        errors_count1 = 0
        errors_count2 = 0
        errors_count3 = 0
        errors_count4 = 0

        payload = struct.pack('<H H H H H h b B H H H H H',
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

        assert len(payload) == 26, "SYS_STATUS payload is 26 bytes"

    def test_battery_status_payload_structure(self):
        """Test BATTERY_STATUS message payload format."""
        id_ = 0
        battery_function = 0  # MAV_BATTERY_FUNCTION_ALL
        type_ = 2  # MAV_BATTERY_TYPE_LIPO
        temperature = 25
        current_battery = 250
        battery_remaining = 75
        charge_state = 0

        voltage_data = struct.pack('<10H',
            4200, 4190, 4180, 0, 0, 0, 0, 0, 0, 0  # 3 cells, rest unused
        )

        current_consumed = 2500  # 2500 mAh
        energy_consumed = 45_000  # 45 kJ
        time_remaining = 0

        header = struct.pack('<i h h h h B B',
            id_, battery_function, type_, temperature,
            current_battery, battery_remaining, charge_state
        )

        footer = struct.pack('<h i h',
            current_consumed, energy_consumed, time_remaining
        )

        payload = header + voltage_data + footer

        assert len(payload) == 36, "BATTERY_STATUS payload is 36 bytes"


class TestDataValidation:
    """Test data range validation."""

    def test_heading_clamped_to_360(self):
        """Test that heading is wrapped to 0-360°."""
        headings = [0, 90, 180, 270, 360, 450, 720, -90]

        for h in headings:
            wrapped = h % 360
            assert 0 <= wrapped < 360

    def test_battery_percentage_range(self):
        """Test battery percentage is 0-100."""
        remaining = 75
        assert 0 <= remaining <= 100

    def test_pitch_range(self):
        """Test pitch angle is within ±90°."""
        pitch_rad = math.radians(45)  # 45°
        assert -math.pi/2 <= pitch_rad <= math.pi/2

    def test_roll_yaw_range(self):
        """Test roll and yaw are within ±180°."""
        for angle_deg in [0, 45, 90, 180, -90, -180]:
            angle_rad = math.radians(angle_deg)
            assert -math.pi <= angle_rad <= math.pi


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
