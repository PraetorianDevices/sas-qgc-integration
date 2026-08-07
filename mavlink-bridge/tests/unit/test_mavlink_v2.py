#!/usr/bin/env python3
"""
Unit Test: mavlink_v2.py codec, verified byte-for-byte against pymavlink.

This is the single source of truth for "is our MAVLink 2.0 wire format
correct" -- every other test file that builds/parses MAVLink frames should
be exercising mavlink_v2.py (directly or through a bridge), not
reimplementing frame logic of its own. That reimplementation pattern is
exactly what let a structurally invalid 7-byte frame header ship with a
100%-passing test suite for an extended period: the tests compared the bug
to a parallel copy of itself instead of to a real MAVLink implementation.

Requires the `pymavlink` package (pip install pymavlink) as ground truth;
skips entirely if it isn't installed rather than failing, since it's a
verification dependency, not a runtime dependency of the bridges themselves.
"""

import pytest

pymavlink_common = pytest.importorskip('pymavlink.dialects.v20.common')

import mavlink_v2 as mav


@pytest.fixture(scope='module')
def mav_ref():
    """A pymavlink MAVLink instance used purely for reference encoding."""
    m = pymavlink_common.MAVLink(None, srcSystem=1, srcComponent=200)
    m.robust_parsing = False
    return m


class TestFrameHeaderAgainstPymavlink:
    """Byte-for-byte comparison of mavlink_v2.build_frame's output against
    pymavlink's own encoder, for every message type the bridges use."""

    def test_heartbeat(self, mav_ref):
        theirs = mav_ref.heartbeat_encode(
            type=2, autopilot=4, base_mode=128, custom_mode=0,
            system_status=4, mavlink_version=3).pack(mav_ref)
        payload = mav.build_heartbeat(type_=2, autopilot=4, base_mode=128,
                                       custom_mode=0, system_status=4, mavlink_version=3)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_HEARTBEAT, 0, payload, 1, 200)
        assert ours == theirs

    def test_sys_status(self, mav_ref):
        theirs = mav_ref.sys_status_encode(
            onboard_control_sensors_present=0xFFFF, onboard_control_sensors_enabled=0xFFFF,
            onboard_control_sensors_health=0xFFFF, load=500, voltage_battery=16800,
            current_battery=250, battery_remaining=75, drop_rate_comm=0, errors_comm=0,
            errors_count1=0, errors_count2=0, errors_count3=0, errors_count4=0).pack(mav_ref)
        payload = mav.build_sys_status(0xFFFF, 0xFFFF, 0xFFFF, 500, 16800, 250, 75, 0, 0, 0, 0, 0, 0)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_SYS_STATUS, 0, payload, 1, 200)
        assert ours == theirs

    def test_attitude(self, mav_ref):
        theirs = mav_ref.attitude_encode(
            time_boot_ms=1000, roll=0.1, pitch=-0.2, yaw=1.57,
            rollspeed=0.05, pitchspeed=0.02, yawspeed=0.01).pack(mav_ref)
        payload = mav.build_attitude(1000, 0.1, -0.2, 1.57, 0.05, 0.02, 0.01)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_ATTITUDE, 0, payload, 1, 200)
        assert ours == theirs

    def test_global_position_int(self, mav_ref):
        theirs = mav_ref.global_position_int_encode(
            time_boot_ms=1000, lat=377749000, lon=-1224194000, alt=500000,
            relative_alt=100000, vx=250, vy=-100, vz=50, hdg=9000).pack(mav_ref)
        payload = mav.build_global_position_int(1000, 377749000, -1224194000, 500000, 100000, 250, -100, 50, 9000)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 0, payload, 1, 200)
        assert ours == theirs

    def test_battery_status(self, mav_ref):
        theirs = mav_ref.battery_status_encode(
            id=0, battery_function=0, type=2, temperature=25,
            voltages=[4200, 4190, 4180, 0, 0, 0, 0, 0, 0, 0], current_battery=250,
            current_consumed=2500, energy_consumed=45000, battery_remaining=75,
            time_remaining=0, charge_state=0).pack(mav_ref)
        payload = mav.build_battery_status(
            0, 0, 2, 25, [4200, 4190, 4180, 0, 0, 0, 0, 0, 0, 0], 250, 2500, 45000, 75, 0, 0)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_BATTERY_STATUS, 0, payload, 1, 200)
        assert ours == theirs

    def test_statustext(self, mav_ref):
        theirs = mav_ref.statustext_encode(
            severity=5, text=b"GPS SPOOF DETECTED: heading diverg", id=0, chunk_seq=0).pack(mav_ref)
        payload = mav.build_statustext("GPS SPOOF DETECTED: heading diverg", 5)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_STATUSTEXT, 0, payload, 1, 200)
        assert ours == theirs

    def test_mission_item_int(self, mav_ref):
        theirs = mav_ref.mission_item_int_encode(
            target_system=1, target_component=1, seq=0, frame=3, command=16,
            current=0, autocontinue=1, param1=0.0, param2=0.0, param3=0.0, param4=0.0,
            x=377749000, y=-1224194000, z=100.0, mission_type=0).pack(mav_ref)
        payload = mav.build_mission_item_int(
            seq=0, frame=3, command=16, current=0, autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
            x=377749000, y=-1224194000, z=100.0,
            target_system=1, target_component=1, mission_type=0)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_ITEM_INT, 0, payload, 1, 200)
        assert ours == theirs

    def test_mission_ack(self, mav_ref):
        theirs = mav_ref.mission_ack_encode(
            target_system=1, target_component=1, type=0, mission_type=0).pack(mav_ref)
        payload = mav.build_mission_ack(result=0, target_system=1, target_component=1, mission_type=0)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_ACK, 0, payload, 1, 200)
        assert ours == theirs

    def test_mission_current(self, mav_ref):
        theirs = mav_ref.mission_current_encode(
            seq=2, total=5, mission_state=3, mission_mode=2).pack(mav_ref)
        payload = mav.build_mission_current(seq=2, total=5, mission_state=3, mission_mode=2)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_CURRENT, 0, payload, 1, 200)
        assert ours == theirs

    def test_mission_count(self, mav_ref):
        theirs = mav_ref.mission_count_encode(
            target_system=255, target_component=0, count=5, mission_type=0).pack(mav_ref)
        payload = mav.build_mission_count(count=5, target_system=255, target_component=0, mission_type=0)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, 0, payload, 1, 200)
        assert ours == theirs

    def test_mission_request_int(self, mav_ref):
        theirs = mav_ref.mission_request_int_encode(
            target_system=1, target_component=1, seq=3, mission_type=0).pack(mav_ref)
        payload = mav.build_mission_request_int(seq=3, target_system=1, target_component=1, mission_type=0)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_REQUEST_INT, 0, payload, 1, 200)
        assert ours == theirs

    def test_mission_item_reached(self, mav_ref):
        theirs = mav_ref.mission_item_reached_encode(seq=4).pack(mav_ref)
        payload = mav.build_mission_item_reached(seq=4)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_ITEM_REACHED, 0, payload, 1, 200)
        assert ours == theirs

    def test_command_ack(self, mav_ref):
        theirs = mav_ref.command_ack_encode(
            command=31010, result=0, progress=0, result_param2=0,
            target_system=255, target_component=0).pack(mav_ref)
        payload = mav.build_command_ack(command=31010, result=0, target_system=255, target_component=0)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_COMMAND_ACK, 0, payload, 1, 200)
        assert ours == theirs

    def test_obstacle_distance(self, mav_ref):
        distances = [65535] * 72
        distances[0] = 150
        distances[18] = 300
        theirs = mav_ref.obstacle_distance_encode(
            time_usec=123456, sensor_type=0, distances=distances, increment=5,
            min_distance=20, max_distance=5000, increment_f=5.0,
            angle_offset=0.0, frame=12).pack(mav_ref)
        payload = mav.build_obstacle_distance(
            time_usec=123456, distances=distances, increment=5,
            min_distance=20, max_distance=5000, increment_f=5.0,
            angle_offset=0.0, sensor_type=0, frame=12)
        ours = mav.build_frame(mav.MAVLINK_MSG_ID_OBSTACLE_DISTANCE, 0, payload, 1, 200)
        assert ours == theirs


class TestFrameHeaderStructure:
    """Structural checks independent of pymavlink -- these describe the
    10-byte MAVLink 2.0 header explicitly, so a regression back to the old
    7-byte header fails here even without pymavlink installed."""

    def test_header_is_10_bytes_before_payload(self):
        payload = b'\x01\x02\x03\x04'
        frame = mav.build_frame(0, seq=5, payload=payload, system_id=7, component_id=8)
        assert frame[0] == 0xFD  # STX
        assert frame[1] == 4     # LEN
        assert frame[2] == 0x00  # INCOMPAT_FLAGS
        assert frame[3] == 0x00  # COMPAT_FLAGS
        assert frame[4] == 5     # SEQ
        assert frame[5] == 7     # SYSID
        assert frame[6] == 8     # COMPID
        # bytes 7-9: 24-bit little-endian msg_id
        assert frame[7:10] == b'\x00\x00\x00'
        assert frame[10:14] == payload

    def test_24_bit_message_id_roundtrip(self):
        # A message ID > 255 would be truncated by the old 8-bit-msg_id bug.
        big_msg_id = 12345  # e.g. a real ArduPilot-namespace message ID
        frame = mav.build_frame(big_msg_id, seq=0, payload=b'', system_id=1, component_id=1)
        parsed = mav.parse_frame(frame)
        assert parsed.msg_id == big_msg_id

    def test_parse_rejects_wrong_magic_byte(self):
        assert mav.parse_frame(b'\xFE' + b'\x00' * 11) is None

    def test_parse_rejects_short_buffer(self):
        assert mav.parse_frame(b'\xFD\x00\x00') is None

    def test_parse_detects_crc_corruption(self):
        frame = mav.build_frame(mav.MAVLINK_MSG_ID_HEARTBEAT, 0,
                                 mav.build_heartbeat(2, 4, 0, 0, 4), 1, 1)
        corrupted = bytearray(frame)
        corrupted[-1] ^= 0xFF  # flip bits in the CRC
        parsed = mav.parse_frame(bytes(corrupted))
        assert parsed is not None
        assert parsed.valid is False


class TestParseFrames:
    """parse_frames() must find every message packed into one buffer, not
    just the first -- regression coverage for the bug that silently broke
    mission upload: QGC bundles its own outgoing HEARTBEAT ahead of
    MISSION_COUNT in a single UDP write, and a receiver using parse_frame()
    alone (which only ever looks at the first message) would process the
    HEARTBEAT and silently discard the MISSION_COUNT with no error."""

    def test_single_frame_buffer_returns_one_frame(self):
        frame = mav.build_frame(mav.MAVLINK_MSG_ID_HEARTBEAT, 0,
                                 mav.build_heartbeat(2, 4, 0, 0, 4), 1, 1)
        frames = mav.parse_frames(frame)
        assert len(frames) == 1
        assert frames[0].msg_id == mav.MAVLINK_MSG_ID_HEARTBEAT
        assert frames[0].valid

    def test_two_bundled_frames_both_found_in_order(self):
        heartbeat = mav.build_frame(mav.MAVLINK_MSG_ID_HEARTBEAT, 0,
                                     mav.build_heartbeat(2, 4, 0, 0, 4), 255, 190)
        count_payload = mav.build_mission_count(count=3, target_system=1,
                                                 target_component=1)
        mission_count = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, 1,
                                         count_payload, 255, 190)
        bundled = heartbeat + mission_count

        frames = mav.parse_frames(bundled)

        assert len(frames) == 2
        assert frames[0].msg_id == mav.MAVLINK_MSG_ID_HEARTBEAT
        assert frames[1].msg_id == mav.MAVLINK_MSG_ID_MISSION_COUNT
        assert frames[0].valid and frames[1].valid

    def test_empty_buffer_returns_no_frames(self):
        assert mav.parse_frames(b'') == []

    def test_trailing_garbage_after_valid_frame_does_not_raise(self):
        frame = mav.build_frame(mav.MAVLINK_MSG_ID_HEARTBEAT, 0,
                                 mav.build_heartbeat(2, 4, 0, 0, 4), 1, 1)
        frames = mav.parse_frames(frame + b'\x00\x01\x02')
        assert len(frames) == 1
        assert frames[0].msg_id == mav.MAVLINK_MSG_ID_HEARTBEAT


class TestMissionItemIntRoundtrip:
    """build -> parse roundtrip for the mission item payload, since this is
    the message type that carries actual waypoint data."""

    def test_roundtrip_preserves_coordinates(self):
        payload = mav.build_mission_item_int(
            seq=3, frame=3, command=16, current=0, autocontinue=1,
            param1=1.0, param2=2.0, param3=3.0, param4=4.0,
            x=377749000, y=-1224194000, z=100.0,
            target_system=1, target_component=1)
        parsed = mav.parse_mission_item_int(payload)
        assert parsed['sequence'] == 3
        assert parsed['command'] == 16
        assert abs(parsed['position']['latitude'] - 37.7749) < 1e-4
        assert abs(parsed['position']['longitude'] - (-122.4194)) < 1e-4
        assert parsed['position']['altitude'] == 100.0
        assert parsed['params'] == [1.0, 2.0, 3.0, 4.0]

    def test_truncated_payload_still_parses(self):
        # mission_type is the only extension field; a real sender may omit it
        # (MAVLink 2 trailing-zero truncation) when it's 0/MISSION.
        payload = mav.build_mission_item_int(
            seq=0, frame=3, command=16, current=0, autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
            x=0, y=0, z=0.0, target_system=1, target_component=1)
        truncated = payload.rstrip(b'\x00')
        parsed = mav.parse_mission_item_int(truncated)
        assert parsed is not None
        assert parsed['mission_type'] == 0


class TestCommandLongParse:
    """parse_command_long against pymavlink-encoded COMMAND_LONG frames --
    this is the inbound message the emergency-wipe bridge gates on, so its
    param/command/confirmation fields must be read from the exact wire order."""

    def test_parses_pymavlink_encoded_command(self, mav_ref):
        frame = mav_ref.command_long_encode(
            target_system=1, target_component=1, command=31010, confirmation=1,
            param1=1.0, param2=2.0, param3=3.0, param4=4.0,
            param5=5.0, param6=6.0, param7=7.0).pack(mav_ref)
        parsed = mav.parse_frame(frame)
        cmd = mav.parse_command_long(parsed.payload)
        assert cmd['command'] == 31010
        assert cmd['target_system'] == 1
        assert cmd['target_component'] == 1
        assert cmd['confirmation'] == 1
        assert cmd['params'] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]

    def test_zeroed_command_survives_truncation(self, mav_ref):
        # command=0, all params 0, confirmation 0 -> payload truncates hard.
        frame = mav_ref.command_long_encode(
            target_system=0, target_component=0, command=0, confirmation=0,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
            param5=0.0, param6=0.0, param7=0.0).pack(mav_ref)
        parsed = mav.parse_frame(frame)
        cmd = mav.parse_command_long(parsed.payload)
        assert cmd is not None
        assert cmd['command'] == 0
        assert cmd['confirmation'] == 0
        assert cmd['params'] == [0.0] * 7


class TestTruncationEdgeCases:
    """MAVLink 2 permits trailing zero bytes to be stripped from a payload.
    All the small mission-protocol messages have an all-zero, common-case
    encoding (result=ACCEPTED=0, count=0, seq=0) that truncates aggressively
    -- these must still parse correctly, not be rejected as too short."""

    def test_mission_ack_accepted_survives_full_truncation(self):
        payload = mav.build_mission_ack(result=0, target_system=0, target_component=0, mission_type=0)
        truncated = payload.rstrip(b'\x00')
        parsed = mav.parse_mission_ack(truncated)
        assert parsed is not None
        assert parsed['result'] == 0

    def test_mission_count_zero_survives_full_truncation(self):
        payload = mav.build_mission_count(count=0, target_system=0, target_component=0, mission_type=0)
        truncated = payload.rstrip(b'\x00')
        parsed = mav.parse_mission_count(truncated)
        assert parsed is not None
        assert parsed['count'] == 0

    def test_mission_request_seq_zero_survives_full_truncation(self):
        payload = mav.build_mission_request_int(seq=0, target_system=0, target_component=0, mission_type=0)
        truncated = payload.rstrip(b'\x00')
        parsed = mav.parse_mission_request(truncated)
        assert parsed is not None
        assert parsed['sequence'] == 0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
