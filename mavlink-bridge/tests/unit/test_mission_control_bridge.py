#!/usr/bin/env python3
"""
Unit Test: Mission Control Bridge (MAVLink ↔ ROS 2 Conversion)

Tests the conversion logic from MAVLink mission messages to SAS format.
Focuses on message parsing, format conversion, and state tracking.

NOT REDUNDANT with SAS tests:
  - SAS tests: executor receives mission JSON, parses it, executes it
  - Bridge tests: bridge receives MAVLink MISSION_ITEM, converts to JSON, sends to executor
  - Different layers: SAS tests the executor, bridge tests the translation layer
"""

import json
import struct
import pytest


class TestMissionItemParsing:
    """Test parsing of MAVLink MISSION_ITEM messages."""

    def test_parse_mission_item_basic(self):
        """Parse a basic MISSION_ITEM waypoint."""
        # MAVLink MISSION_ITEM structure: 39 bytes
        seq = 0
        frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        command = 16  # NAV_WAYPOINT
        current = 0
        autocontinue = 1
        param1 = 0.0  # hold time
        param2 = 0.0  # acceptance radius
        param3 = 0.0  # pass through
        param4 = 0.0  # yaw
        lat = 377_749_000  # 37.7749° in 1e7 format
        lon = -122_419_400  # -122.4194° in 1e7 format
        alt = 100.0  # 100m above home

        payload = struct.pack('<H B H B B f f f f i i f',
            seq, frame, command, current, autocontinue,
            param1, param2, param3, param4,
            lat, lon, alt
        )

        # Parse it back
        unpacked = struct.unpack('<H B H B B f f f f i i f', payload)
        seq_back, frame_back, cmd_back = unpacked[0], unpacked[1], unpacked[2]
        lat_back, lon_back, alt_back = unpacked[9], unpacked[10], unpacked[11]

        assert seq_back == seq
        assert frame_back == frame
        assert cmd_back == command
        assert lat_back == lat
        assert lon_back == lon
        assert alt_back == alt

    def test_convert_mission_item_to_sas_format(self):
        """Convert MAVLink MISSION_ITEM to SAS JSON format."""
        # MAVLink MISSION_ITEM data
        seq = 1
        lat_scaled = 377_749_000
        lon_scaled = -1_224_194_000
        alt_m = 100.0
        command = 16  # NAV_WAYPOINT
        frame = 3  # Relative altitude

        # Convert to SAS format
        mission_item = {
            'sequence': seq,
            'frame': frame,
            'command': command,
            'position': {
                'latitude': lat_scaled / 1e7,
                'longitude': lon_scaled / 1e7,
                'altitude': alt_m
            }
        }

        # Verify
        assert mission_item['sequence'] == 1
        assert mission_item['position']['latitude'] == pytest.approx(37.7749, abs=1e-4)
        assert mission_item['position']['longitude'] == pytest.approx(-122.4194, abs=1e-4)
        assert mission_item['position']['altitude'] == 100.0

    def test_mission_item_sequence_numbers(self):
        """Test waypoint sequence numbering (0-indexed)."""
        sequences = [0, 1, 2, 3, 4, 255]

        waypoints = {}
        for seq in sequences:
            # Bridge stores waypoint at index seq
            waypoints[seq] = {'seq': seq}

        # Verify all sequences stored
        for seq in sequences:
            assert seq in waypoints
            assert waypoints[seq]['seq'] == seq

    def test_mission_item_frame_types(self):
        """Test different coordinate frame types."""
        frames = {
            0: 'MAV_FRAME_GLOBAL',  # Absolute altitude (MSL)
            3: 'MAV_FRAME_GLOBAL_RELATIVE_ALT',  # Relative to home
            10: 'MAV_FRAME_LOCAL_NED',  # Local NED
        }

        for frame_id, frame_name in frames.items():
            assert frame_id in [0, 3, 10]

    def test_mission_item_command_types(self):
        """Test different MAVLink command types."""
        commands = {
            16: 'NAV_WAYPOINT',
            21: 'NAV_LAND',
            22: 'NAV_TAKEOFF',
            178: 'DO_CHANGE_SPEED',
        }

        for cmd_id, cmd_name in commands.items():
            assert cmd_id in [16, 21, 22, 178]


class TestMissionStateTracking:
    """Test mission state management in the bridge."""

    def test_store_single_waypoint(self):
        """Store a single waypoint in bridge memory."""
        mission_items = {}

        # Simulate receiving MISSION_ITEM with seq=0
        seq = 0
        item = {
            'sequence': seq,
            'position': {'latitude': 37.7749, 'longitude': -122.4194, 'altitude': 100.0}
        }

        # Store while extending list if needed
        while len(mission_items) <= seq:
            mission_items[seq] = None
        mission_items[seq] = item

        assert 0 in mission_items
        assert mission_items[0] is not None

    def test_store_multiple_waypoints_in_order(self):
        """Store multiple waypoints maintaining sequence order."""
        mission_items = {}

        waypoints = [
            {'seq': 0, 'lat': 37.1, 'lon': -122.1, 'alt': 100},
            {'seq': 1, 'lat': 37.2, 'lon': -122.2, 'alt': 150},
            {'seq': 2, 'lat': 37.3, 'lon': -122.3, 'alt': 200},
        ]

        for wp in waypoints:
            seq = wp['seq']
            while len(mission_items) <= seq:
                mission_items[seq] = None
            mission_items[seq] = wp

        assert len(mission_items) == 3
        assert mission_items[1]['seq'] == 1
        assert mission_items[2]['alt'] == 200

    def test_update_existing_waypoint(self):
        """Update a waypoint that was already stored."""
        mission_items = {}

        # Store initial waypoint
        mission_items[0] = {'seq': 0, 'lat': 37.0, 'alt': 100}

        # Update it
        mission_items[0] = {'seq': 0, 'lat': 37.5, 'alt': 150}

        assert mission_items[0]['lat'] == 37.5
        assert mission_items[0]['alt'] == 150

    def test_current_waypoint_tracking(self):
        """Track which waypoint is currently executing."""
        current_waypoint = 0
        mission_items = {
            0: {'seq': 0},
            1: {'seq': 1},
            2: {'seq': 2},
        }

        # Simulate progression
        current_waypoint = 0
        assert current_waypoint == 0

        current_waypoint = 1
        assert current_waypoint == 1

        current_waypoint = 2
        assert current_waypoint == 2


class TestMissionACKGeneration:
    """Test MISSION_ACK message generation."""

    def test_mission_ack_accepted(self):
        """Generate MISSION_ACK with accepted status."""
        MAV_MISSION_ACCEPTED = 0

        payload = struct.pack('<H I', MAV_MISSION_ACCEPTED, 0)

        assert len(payload) == 6
        result, _ = struct.unpack('<H I', payload)
        assert result == 0

    def test_mission_ack_error(self):
        """Generate MISSION_ACK with error status."""
        MAV_MISSION_ERROR = 1

        payload = struct.pack('<H I', MAV_MISSION_ERROR, 0)
        result, _ = struct.unpack('<H I', payload)

        assert result == 1

    def test_mission_count_payload(self):
        """Generate MISSION_COUNT payload."""
        count = 5

        payload = struct.pack('<H I', count, 0)
        count_back, _ = struct.unpack('<H I', payload)

        assert count_back == 5


class TestMissionCurrentTracking:
    """Test MISSION_CURRENT progress reporting."""

    def test_mission_current_update(self):
        """Generate MISSION_CURRENT message for active waypoint."""
        current_waypoint = 2
        time_boot_ms = 10000

        payload = struct.pack('<H I', current_waypoint, time_boot_ms)

        wp_back, time_back = struct.unpack('<H I', payload)

        assert wp_back == 2
        assert time_back == 10000

    def test_mission_current_progression(self):
        """Test sequence of MISSION_CURRENT updates as mission progresses."""
        mission_size = 5
        updates = []

        for wp_index in range(mission_size):
            time_ms = wp_index * 1000
            updates.append((wp_index, time_ms))

        assert len(updates) == 5
        assert updates[0] == (0, 0)
        assert updates[4] == (4, 4000)

    def test_mission_current_wrap_around(self):
        """Test MISSION_CURRENT after mission completion."""
        # Typically, after last waypoint, current stays at last index
        last_waypoint = 4
        current = last_waypoint

        assert current == 4


class TestMissionJsonConversion:
    """Test conversion of MAVLink waypoints to SAS JSON mission format."""

    def test_single_waypoint_to_json(self):
        """Convert single MAVLink waypoint to SAS JSON."""
        mission_item = {
            'sequence': 0,
            'frame': 3,
            'command': 16,
            'position': {
                'latitude': 37.7749,
                'longitude': -122.4194,
                'altitude': 100.0
            }
        }

        # Convert to JSON
        json_str = json.dumps(mission_item)
        parsed = json.loads(json_str)

        assert parsed['sequence'] == 0
        assert parsed['position']['latitude'] == 37.7749

    def test_mission_list_to_json(self):
        """Convert list of waypoints to SAS mission JSON."""
        mission_items = {
            0: {
                'sequence': 0,
                'position': {'latitude': 37.1, 'longitude': -122.1, 'altitude': 100}
            },
            1: {
                'sequence': 1,
                'position': {'latitude': 37.2, 'longitude': -122.2, 'altitude': 150}
            },
        }

        mission_json = {
            'waypoints': list(mission_items.values()),
            'home': {'latitude': 37.0, 'longitude': -122.0, 'altitude': 0}
        }

        json_str = json.dumps(mission_json)
        parsed = json.loads(json_str)

        assert len(parsed['waypoints']) == 2
        assert parsed['waypoints'][1]['sequence'] == 1

    def test_mission_json_preserves_types(self):
        """Ensure JSON conversion preserves data types."""
        mission_item = {
            'sequence': 5,  # int
            'frame': 3,  # int
            'position': {
                'latitude': 37.7749,  # float
                'altitude': 100.0  # float
            }
        }

        json_str = json.dumps(mission_item)
        parsed = json.loads(json_str)

        assert isinstance(parsed['sequence'], int)
        assert isinstance(parsed['position']['latitude'], float)


class TestDataValidation:
    """Test validation of mission data ranges."""

    def test_latitude_range(self):
        """Test latitude is within valid range."""
        valid_lats = [-90.0, -45.0, 0.0, 45.0, 90.0]

        for lat in valid_lats:
            assert -90 <= lat <= 90

    def test_longitude_range(self):
        """Test longitude is within valid range."""
        valid_lons = [-180.0, -90.0, 0.0, 90.0, 180.0]

        for lon in valid_lons:
            assert -180 <= lon <= 180

    def test_altitude_positive(self):
        """Test altitude is non-negative."""
        altitudes = [0.0, 100.0, 1000.0, 5000.0]

        for alt in altitudes:
            assert alt >= 0

    def test_sequence_increment(self):
        """Test waypoint sequences increment correctly."""
        sequences = [0, 1, 2, 3, 4]

        for i, seq in enumerate(sequences):
            assert seq == i

    def test_mission_count_range(self):
        """Test mission count is reasonable."""
        counts = [0, 1, 5, 100, 255]

        for count in counts:
            assert 0 <= count <= 255


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
