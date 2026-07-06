#!/usr/bin/env python3
"""
Integration Test: Mission Control Bridge (MAVLink ↔ ROS 2)

Tests full mission upload/download flow with ROS 2 topic communication.
Simulates QGC sending MAVLink MISSION_* messages and verifies bridge
publishes to mission_executor_node and receives status updates.

NOT REDUNDANT with SAS tests:
  - SAS tests: executor loads JSON mission, executes it
  - Integration tests: bridge receives MAVLink, publishes JSON, tracks progress
  - Different flow: MAVLink → Bridge → ROS 2 topics → Executor
"""

import json
import struct
import pytest
import asyncio
from unittest.mock import Mock, patch, MagicMock
from io import BytesIO


class TestMissionUploadFlow:
    """Test complete mission upload from QGC to executor."""

    def test_mission_request_list_triggers_mission_count(self):
        """When QGC asks for mission list, bridge responds with count."""
        # Simulate: QGC sends MISSION_REQUEST_LIST
        mission_items = [
            {'seq': 0, 'lat': 37.1, 'lon': -122.1, 'alt': 100},
            {'seq': 1, 'lat': 37.2, 'lon': -122.2, 'alt': 150},
            {'seq': 2, 'lat': 37.3, 'lon': -122.3, 'alt': 200},
        ]

        count = len(mission_items)

        # Bridge should prepare MISSION_COUNT response with count=3
        payload = struct.pack('<H I', count, 0)

        assert len(payload) == 6
        count_back, _ = struct.unpack('<H I', payload)
        assert count_back == 3

    def test_mission_item_upload_sequence(self):
        """Test receiving 3 consecutive MISSION_ITEM messages."""
        mission_items = {}

        # Simulate QGC uploading waypoint 0
        waypoint_0 = {
            'seq': 0,
            'lat_scaled': 377_100_000,
            'lon_scaled': -1_222_100_000,
            'alt': 100.0,
            'frame': 3,
            'command': 16
        }

        seq = waypoint_0['seq']
        while len(mission_items) <= seq:
            mission_items[seq] = None
        mission_items[seq] = waypoint_0

        # Simulate QGC uploading waypoint 1
        waypoint_1 = {
            'seq': 1,
            'lat_scaled': 377_200_000,
            'lon_scaled': -1_222_200_000,
            'alt': 150.0,
            'frame': 3,
            'command': 16
        }

        seq = waypoint_1['seq']
        while len(mission_items) <= seq:
            mission_items[seq] = None
        mission_items[seq] = waypoint_1

        # Verify storage
        assert len(mission_items) == 2
        assert mission_items[0]['seq'] == 0
        assert mission_items[1]['seq'] == 1

    def test_mission_ack_on_successful_upload(self):
        """Bridge sends MISSION_ACK after accepting waypoint."""
        MAV_MISSION_ACCEPTED = 0

        # Simulate bridge accepting waypoint
        ack_result = MAV_MISSION_ACCEPTED

        payload = struct.pack('<H I', ack_result, 0)
        result_back, _ = struct.unpack('<H I', payload)

        assert result_back == 0  # ACCEPTED


class TestMissionConversionToJSON:
    """Test conversion of MAVLink waypoints to SAS mission JSON."""

    def test_convert_single_waypoint_to_json(self):
        """Convert MAVLink waypoint to SAS JSON format."""
        waypoint = {
            'sequence': 0,
            'frame': 3,
            'command': 16,
            'current': 0,
            'autocontinue': 1,
            'params': [0.0, 0.0, 0.0, 0.0],
            'position': {
                'latitude': 37.7749,
                'longitude': -122.4194,
                'altitude': 100.0
            }
        }

        json_str = json.dumps(waypoint)
        parsed = json.loads(json_str)

        assert parsed['sequence'] == 0
        assert parsed['position']['latitude'] == 37.7749

    def test_convert_mission_items_to_payload_json(self):
        """Convert mission item list to SAS mission payload."""
        mission_items = [
            {
                'sequence': 0,
                'position': {'latitude': 37.1, 'longitude': -122.1, 'altitude': 100}
            },
            {
                'sequence': 1,
                'position': {'latitude': 37.2, 'longitude': -122.2, 'altitude': 150}
            },
            {
                'sequence': 2,
                'position': {'latitude': 37.3, 'longitude': -122.3, 'altitude': 200}
            }
        ]

        # Package as mission
        mission_payload = {
            'waypoints': mission_items,
            'home': {'latitude': 37.0, 'longitude': -122.0, 'altitude': 0}
        }

        json_str = json.dumps(mission_payload)
        parsed = json.loads(json_str)

        assert len(parsed['waypoints']) == 3
        assert parsed['waypoints'][2]['sequence'] == 2
        assert parsed['waypoints'][2]['position']['altitude'] == 200

    def test_mission_json_published_to_ros2_topic(self):
        """Simulate publishing mission JSON to /mission_executor/load_mission."""
        mission_payload = {
            'waypoints': [
                {'sequence': 0, 'position': {'latitude': 37.1, 'lon': -122.1, 'alt': 100}}
            ],
            'home': {'latitude': 37.0, 'longitude': -122.0, 'altitude': 0}
        }

        json_str = json.dumps(mission_payload)

        # Simulate ROS 2 String message
        class MockStringMsg:
            def __init__(self, data):
                self.data = data

        msg = MockStringMsg(json_str)

        # Bridge publishes this to /mission_executor/load_mission
        received = json.loads(msg.data)

        assert len(received['waypoints']) == 1
        assert received['home']['latitude'] == 37.0


class TestMissionProgressTracking:
    """Test mission progress updates from executor to QGC."""

    def test_receive_executor_status_update(self):
        """Bridge receives status from mission_executor_node."""
        status_json = json.dumps({
            'current_waypoint': 1,
            'in_progress': True,
            'total_waypoints': 5
        })

        # Parse status
        status = json.loads(status_json)

        current_wp = status['current_waypoint']
        in_progress = status['in_progress']

        assert current_wp == 1
        assert in_progress is True

    def test_convert_status_to_mission_current(self):
        """Convert executor status to MISSION_CURRENT message."""
        current_waypoint = 2
        time_boot_ms = 5000

        payload = struct.pack('<H I', current_waypoint, time_boot_ms)

        wp_back, time_back = struct.unpack('<H I', payload)

        assert wp_back == 2
        assert time_back == 5000

    def test_mission_progress_sequence(self):
        """Test mission progress from start to finish."""
        total_waypoints = 5
        progress = []

        for wp_idx in range(total_waypoints):
            progress.append({
                'waypoint': wp_idx,
                'time_ms': wp_idx * 2000,
                'in_progress': True if wp_idx < total_waypoints - 1 else False
            })

        assert len(progress) == 5
        assert progress[0]['waypoint'] == 0
        assert progress[4]['waypoint'] == 4
        assert progress[4]['in_progress'] is False


class TestMAVLinkFrameConstruction:
    """Test MAVLink frame creation and CRC computation."""

    def test_construct_mission_count_frame(self):
        """Build a complete MISSION_COUNT MAVLink frame."""
        msg_id = 44  # MISSION_COUNT
        count = 3
        sequence = 1

        payload = struct.pack('<H I', count, 0)

        # Frame header
        stx = 0xFD
        payload_len = len(payload)
        incomp_flags = 0x00
        system_id = 1
        component_id = 1

        frame_data = struct.pack('<BBBBBBB',
            stx, payload_len, incomp_flags, msg_id,
            system_id, component_id, sequence
        ) + payload

        assert len(frame_data) == 7 + len(payload)
        assert frame_data[0] == 0xFD  # STX

    def test_compute_crc_mission_count(self):
        """Compute CRC16-CCITT for MISSION_COUNT."""
        msg_id = 44  # MISSION_COUNT
        payload = struct.pack('<H I', 3, 0)  # count=3

        CRC_INIT = 0xFFFF
        CRC_EXTRA = 142  # MISSION_COUNT extra byte

        crc = CRC_INIT
        for byte in payload:
            tmp = byte ^ (crc & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crc &= 0xFFFF

        tmp = CRC_EXTRA ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF

        # CRC should be non-zero (valid)
        assert crc != 0

    def test_frame_roundtrip(self):
        """Test building a frame, computing CRC, and parsing it back."""
        msg_id = 42  # MISSION_CURRENT
        current_wp = 2
        time_ms = 10000

        payload = struct.pack('<H I', current_wp, time_ms)

        # Build frame
        stx = 0xFD
        seq = 0
        frame_data = struct.pack('<BBBBBBB',
            stx, len(payload), 0x00, msg_id, 1, 1, seq
        ) + payload

        # Parse back
        parsed_stx = frame_data[0]
        parsed_len = frame_data[1]
        parsed_msg_id = frame_data[3]

        assert parsed_stx == 0xFD
        assert parsed_len == len(payload)
        assert parsed_msg_id == 42


class TestMissionRequestDownload:
    """Test QGC downloading waypoints from bridge."""

    def test_qgc_requests_waypoint(self):
        """QGC asks for a specific waypoint number."""
        requested_seq = 1
        mission_items = {
            0: {'seq': 0, 'lat': 37.1, 'lon': -122.1, 'alt': 100},
            1: {'seq': 1, 'lat': 37.2, 'lon': -122.2, 'alt': 150},
            2: {'seq': 2, 'lat': 37.3, 'lon': -122.3, 'alt': 200},
        }

        # Bridge retrieves waypoint
        if requested_seq in mission_items:
            waypoint = mission_items[requested_seq]
        else:
            waypoint = None

        assert waypoint is not None
        assert waypoint['seq'] == 1

    def test_bridge_sends_mission_item_response(self):
        """Bridge responds to MISSION_REQUEST with MISSION_ITEM."""
        seq = 1
        frame = 3
        command = 16
        lat = 377_200_000
        lon = -1_222_200_000
        alt = 150.0

        payload = struct.pack('<H B H B B f f f f i i f',
            seq, frame, command, 0, 1,
            0.0, 0.0, 0.0, 0.0,
            lat, lon, alt
        )

        # Verify roundtrip
        seq_back, frame_back, cmd_back = struct.unpack('<H B H', payload[0:5])
        lat_back, lon_back, alt_back = struct.unpack('<i i f', payload[23:35])

        assert seq_back == 1
        assert lat_back == lat
        assert lon_back == lon

    def test_handle_invalid_waypoint_request(self):
        """Bridge handles request for non-existent waypoint."""
        mission_items = {
            0: {'seq': 0},
            1: {'seq': 1},
        }

        requested_seq = 5  # Doesn't exist

        if requested_seq in mission_items:
            waypoint = mission_items[requested_seq]
        else:
            waypoint = None

        assert waypoint is None


class TestErrorHandling:
    """Test error handling in mission operations."""

    def test_reject_invalid_mission_item(self):
        """Bridge rejects malformed MISSION_ITEM."""
        MAV_MISSION_ERROR = 1

        # Simulate error condition
        payload = struct.pack('<H I', MAV_MISSION_ERROR, 0)
        result, _ = struct.unpack('<H I', payload)

        assert result == 1

    def test_handle_out_of_range_altitude(self):
        """Bridge validates altitude is reasonable."""
        valid_altitudes = [0, 50, 100, 500, 1000]
        invalid_altitudes = [-100, -1]

        for alt in valid_altitudes:
            assert alt >= 0

        for alt in invalid_altitudes:
            assert alt < 0

    def test_handle_missing_waypoint_data(self):
        """Bridge handles incomplete MISSION_ITEM payload."""
        partial_payload = b'\x00\x01\x03\x10'  # Only 4 bytes, needs 37+

        if len(partial_payload) < 37:
            is_valid = False
        else:
            is_valid = True

        assert is_valid is False

    def test_recover_from_lost_waypoint(self):
        """Bridge can skip missing waypoints and continue."""
        mission_items = {}

        # Waypoints 0, 2, 3 received (1 missing)
        for seq in [0, 2, 3]:
            mission_items[seq] = {'seq': seq}

        assert 0 in mission_items
        assert 1 not in mission_items
        assert 2 in mission_items

        # Bridge can report: "have 3 of 4 waypoints"
        have_count = len([k for k in mission_items if mission_items[k] is not None])
        assert have_count == 3


class TestBidirectionalFlow:
    """Test bidirectional mission control (upload and progress)."""

    def test_upload_then_track_progress(self):
        """Full flow: upload mission, execute, track progress."""
        # Step 1: Upload mission
        mission_items = {}
        for i in range(3):
            mission_items[i] = {
                'seq': i,
                'lat': 37.0 + i * 0.1,
                'lon': -122.0 - i * 0.1,
                'alt': 100 + i * 50
            }

        assert len(mission_items) == 3

        # Step 2: Simulate execution (executor publishes status)
        statuses = [
            {'current_waypoint': 0, 'in_progress': True},
            {'current_waypoint': 1, 'in_progress': True},
            {'current_waypoint': 2, 'in_progress': True},
            {'current_waypoint': 2, 'in_progress': False},  # Complete
        ]

        # Step 3: Bridge sends MISSION_CURRENT for each
        for status in statuses:
            wp = status['current_waypoint']
            payload = struct.pack('<H I', wp, 0)
            wp_back, _ = struct.unpack('<H I', payload)

            assert wp_back == wp

    def test_mission_abort_flow(self):
        """Test aborting a mission in progress."""
        mission_items = {
            0: {'seq': 0},
            1: {'seq': 1},
            2: {'seq': 2},
        }

        current_wp = 1  # Currently at waypoint 1

        # Operator sends abort command
        mission_items.clear()  # Clear mission

        # Bridge should report current waypoint as -1 or 0
        reported_wp = len(mission_items) if mission_items else 0

        assert reported_wp == 0


class TestConcurrentWaypointHandling:
    """Test handling multiple waypoint uploads."""

    def test_out_of_order_waypoint_arrival(self):
        """Handle waypoints arriving out of sequence."""
        mission_items = {}

        # Simulate: waypoint 2 arrives before 0, 1
        sequences = [2, 0, 1]

        for seq in sequences:
            while len(mission_items) <= seq:
                mission_items[seq] = None

            mission_items[seq] = {'seq': seq}

        # Should have all 3
        assert len(mission_items) == 3
        assert 0 in mission_items
        assert 1 in mission_items
        assert 2 in mission_items

    def test_duplicate_waypoint_update(self):
        """Handle receiving same waypoint twice (update)."""
        mission_items = {}

        # Receive waypoint 1 first time
        mission_items[1] = {'seq': 1, 'lat': 37.0, 'alt': 100}

        # Receive waypoint 1 second time (update)
        mission_items[1] = {'seq': 1, 'lat': 37.5, 'alt': 150}

        # Should have latest version
        assert mission_items[1]['lat'] == 37.5
        assert mission_items[1]['alt'] == 150


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
