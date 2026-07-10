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
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

# -----------------------------------------------------------------------------
# ROS2 / std_msgs stubs — mission_control_bridge.py imports rclpy at module
# level, which isn't installed outside a ROS 2 environment. Stub just enough
# for the module (and MissionControlBridge.__new__, which bypasses __init__
# entirely) to import cleanly.
# -----------------------------------------------------------------------------

class _DummyString:
    def __init__(self):
        self.data = ''


def _install_ros_stubs() -> None:
    if 'mission_control_bridge' in sys.modules:
        return  # already imported (e.g. re-run in the same session)

    rclpy_mock = MagicMock()
    rclpy_mock.node.Node = object

    qos_mock = MagicMock()
    for name in ('QoSProfile', 'ReliabilityPolicy', 'HistoryPolicy', 'DurabilityPolicy'):
        setattr(qos_mock, name, MagicMock())

    std_msgs_mock = MagicMock()
    std_msgs_mock.String = _DummyString

    sys.modules['rclpy'] = rclpy_mock
    sys.modules['rclpy.node'] = rclpy_mock.node
    sys.modules['rclpy.qos'] = qos_mock
    sys.modules['std_msgs'] = MagicMock()
    sys.modules['std_msgs.msg'] = std_msgs_mock


_install_ros_stubs()

from mission_control_bridge import MissionControlBridge  # noqa: E402
import mavlink_v2 as mav  # noqa: E402


def _make_bridge() -> MissionControlBridge:
    """Construct MissionControlBridge bypassing __init__ (socket/thread setup)
    to unit test _cb_mission_status in isolation."""
    bridge = MissionControlBridge.__new__(MissionControlBridge)
    bridge._current_waypoint = 0
    bridge._mission_in_progress = False
    return bridge


def _status_msg(data: dict) -> _DummyString:
    m = _DummyString()
    m.data = json.dumps(data)
    return m


class TestCbMissionStatusRealModule:
    """Exercises the real mission_control_bridge._cb_mission_status, not a
    reimplementation — covers the fix for the 'in_progress' field, which the
    executor never actually publishes (only 'state')."""

    @pytest.mark.parametrize('state', ['executing', 'paused', 'loading'])
    def test_in_progress_true_for_active_states(self, state):
        bridge = _make_bridge()
        bridge._cb_mission_status(_status_msg({'state': state, 'current_waypoint': 2}))
        assert bridge._mission_in_progress is True

    @pytest.mark.parametrize('state', ['idle', 'completed', 'interrupted', 'error'])
    def test_in_progress_false_for_inactive_states(self, state):
        bridge = _make_bridge()
        bridge._cb_mission_status(_status_msg({'state': state, 'current_waypoint': 2}))
        assert bridge._mission_in_progress is False

    def test_current_waypoint_still_populated(self):
        bridge = _make_bridge()
        bridge._cb_mission_status(_status_msg({'state': 'executing', 'current_waypoint': 3}))
        assert bridge._current_waypoint == 3

    def test_missing_state_key_defaults_to_not_in_progress(self):
        bridge = _make_bridge()
        bridge._cb_mission_status(_status_msg({'current_waypoint': 1}))
        assert bridge._mission_in_progress is False

    def test_malformed_json_leaves_prior_values_unchanged(self):
        bridge = _make_bridge()
        bridge._cb_mission_status(_status_msg({'state': 'executing', 'current_waypoint': 5}))
        assert bridge._mission_in_progress is True

        bad_msg = _DummyString()
        bad_msg.data = 'not json {{{'
        bridge._cb_mission_status(bad_msg)  # must not raise

        assert bridge._mission_in_progress is True
        assert bridge._current_waypoint == 5


def _full_bridge(system_id=1, component_id=1, gcs_system_id=255, gcs_component_id=0):
    """Construct a MissionControlBridge with the full state a real upload/
    download handshake needs, bypassing __init__'s socket/thread setup."""
    bridge = MissionControlBridge.__new__(MissionControlBridge)
    bridge.system_id = system_id
    bridge.component_id = component_id
    bridge._sequence = 0
    bridge._reply_addr = ('localhost', 14550)
    bridge._gcs_system_id = gcs_system_id
    bridge._gcs_component_id = gcs_component_id
    bridge._mission_items = []
    bridge._current_waypoint = 0
    bridge._mission_in_progress = False
    bridge._upload_items = []
    bridge._upload_expected_count = 0
    bridge._upload_in_progress = False
    bridge.get_logger = lambda: MagicMock()
    return bridge


def _capture_socket(bridge):
    sent = []
    bridge._socket = type('S', (), {'sendto': staticmethod(lambda f, a: sent.append(f))})()
    return sent


def _capture_socket_with_addr(bridge):
    """Like _capture_socket, but also records the destination address passed
    to sendto, for tests asserting on reply targeting."""
    sent = []
    targets = []

    def _sendto(f, a):
        sent.append(f)
        targets.append(a)

    bridge._socket = type('S', (), {'sendto': staticmethod(_sendto)})()
    return sent, targets


def _last_sent(sent):
    return mav.parse_frame(sent[-1])


def _qgc_frame(msg_id, payload, seq=0, sysid=255, compid=0):
    """Build a frame as if QGC (sysid=255, compid=0 by default) sent it."""
    return mav.build_frame(msg_id, seq, payload, sysid, compid)


class TestUploadHandshakeReal:
    """Real MISSION_COUNT -> MISSION_REQUEST_INT -> MISSION_ITEM_INT -> ACK
    handshake, driven through the real _handle_mavlink_message dispatcher --
    this is the flow that was entirely unimplemented before (the bridge only
    reacted to unsolicited MISSION_ITEM, which real QGC never sends first)."""

    WAYPOINTS = [
        (377_749_000, -1_224_194_000, 100.0, 16),
        (377_750_000, -1_224_195_000, 150.0, 16),
        (377_751_000, -1_224_196_000, 0.0, 21),
    ]

    def _upload(self, bridge, sent, waypoints):
        count_payload = mav.build_mission_count(len(waypoints), target_system=1, target_component=1)
        bridge._handle_mavlink_message(_qgc_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, count_payload))

        for seq, (lat, lon, alt, cmd) in enumerate(waypoints):
            item_payload = mav.build_mission_item_int(
                seq=seq, frame=3, command=cmd, current=0, autocontinue=1,
                param1=0.0, param2=0.0, param3=0.0, param4=0.0,
                x=lat, y=lon, z=alt, target_system=1, target_component=1)
            bridge._handle_mavlink_message(
                _qgc_frame(mav.MAVLINK_MSG_ID_MISSION_ITEM_INT, item_payload, seq=seq + 1))

    def test_count_triggers_request_for_seq_zero(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)

        count_payload = mav.build_mission_count(3, target_system=1, target_component=1)
        bridge._handle_mavlink_message(_qgc_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, count_payload))

        parsed = _last_sent(sent)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_REQUEST_INT
        req = mav.parse_mission_request(parsed.payload)
        assert req['sequence'] == 0

    def test_each_item_triggers_request_for_next_seq(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        self._upload(bridge, sent, self.WAYPOINTS)

        # The bridge should have requested seq 1 and seq 2 in between items,
        # ending with an ACK after the last -- not a request for seq 3.
        request_frames = [mav.parse_frame(f) for f in sent
                           if mav.parse_frame(f).msg_id == mav.MAVLINK_MSG_ID_MISSION_REQUEST_INT]
        requested_seqs = [mav.parse_mission_request(p.payload)['sequence'] for p in request_frames]
        assert requested_seqs == [0, 1, 2]

    def test_ack_sent_only_once_after_last_item(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        self._upload(bridge, sent, self.WAYPOINTS)

        ack_frames = [mav.parse_frame(f) for f in sent
                      if mav.parse_frame(f).msg_id == mav.MAVLINK_MSG_ID_MISSION_ACK]
        assert len(ack_frames) == 1
        ack = mav.parse_mission_ack(ack_frames[0].payload)
        assert ack['result'] == mav.MAVMissionResult.ACCEPTED

    def test_mission_published_to_executor_in_qgc_plan_format(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        published = []
        bridge._mission_upload_pub = type(
            'P', (), {'publish': staticmethod(lambda m: published.append(json.loads(m.data)))})()

        self._upload(bridge, sent, self.WAYPOINTS)

        assert len(published) == 1
        plan = published[0]
        assert plan['fileType'] == 'Plan'
        items = plan['mission']['items']
        assert len(items) == 3
        assert items[0]['command'] == 16
        assert items[0]['params'][4] == pytest.approx(37.7749, abs=1e-4)  # lat
        assert items[0]['params'][5] == pytest.approx(-122.4194, abs=1e-4)  # lon
        assert items[0]['params'][6] == 100.0  # alt
        assert items[2]['command'] == 21  # NAV_LAND

    def test_bridges_own_mission_items_populated_after_upload(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        bridge._mission_upload_pub = MagicMock()
        self._upload(bridge, sent, self.WAYPOINTS)

        assert len(bridge._mission_items) == 3
        assert bridge._upload_in_progress is False

    def test_empty_mission_count_acked_immediately(self):
        """count=0 with all-zero fields truncates to a 0-byte payload under
        MAVLink 2's trailing-zero rule -- must still be handled, not dropped."""
        bridge = _full_bridge()
        sent = _capture_socket(bridge)

        count_payload = mav.build_mission_count(0, target_system=0, target_component=0)
        bridge._handle_mavlink_message(_qgc_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, count_payload))

        parsed = _last_sent(sent)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_ACK
        ack = mav.parse_mission_ack(parsed.payload)
        assert ack['result'] == mav.MAVMissionResult.ACCEPTED

    def test_unexpected_item_without_prior_count_is_ignored(self):
        """An item arriving with no upload in progress shouldn't crash or
        get stored -- there's nothing to append it to."""
        bridge = _full_bridge()
        sent = _capture_socket(bridge)

        item_payload = mav.build_mission_item_int(
            seq=0, frame=3, command=16, current=0, autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
            x=0, y=0, z=0.0, target_system=1, target_component=1)
        bridge._handle_mavlink_message(_qgc_frame(mav.MAVLINK_MSG_ID_MISSION_ITEM_INT, item_payload))

        assert sent == []
        assert bridge._mission_items == []

    def test_short_item_payload_does_not_crash(self):
        """mavlink_v2.parse_mission_item_int zero-fills short payloads (this
        is required for real MAVLink 2 trailing-zero truncation) and so can
        never return None -- a short/garbage-but-parseable payload for the
        last expected item is absorbed as a (nonsensical) waypoint and still
        completes the upload, rather than crashing the receive loop."""
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        bridge._mission_upload_pub = MagicMock()
        bridge._upload_in_progress = True
        bridge._upload_expected_count = 1
        bridge._upload_items = [None]

        short_frame = _qgc_frame(mav.MAVLINK_MSG_ID_MISSION_ITEM_INT, b'\x01\x02')
        bridge._handle_mavlink_message(short_frame)  # must not raise

        parsed = _last_sent(sent)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_ACK
        ack = mav.parse_mission_ack(parsed.payload)
        assert ack['result'] == mav.MAVMissionResult.ACCEPTED


class TestDownloadHandshakeReal:
    """Real MISSION_REQUEST_LIST / MISSION_REQUEST_INT handling for the
    download direction (QGC pulling our current mission)."""

    def _seed_mission(self, bridge):
        bridge._mission_items = [
            {'sequence': 0, 'frame': 3, 'command': 16, 'current': 0, 'autocontinue': 1,
             'params': [0.0, 0.0, 0.0, 0.0],
             'position': {'latitude': 37.7749, 'longitude': -122.4194, 'altitude': 100.0}},
            {'sequence': 1, 'frame': 3, 'command': 16, 'current': 0, 'autocontinue': 1,
             'params': [0.0, 0.0, 0.0, 0.0],
             'position': {'latitude': 37.7750, 'longitude': -122.4195, 'altitude': 150.0}},
        ]

    def test_request_list_replies_with_count(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        self._seed_mission(bridge)

        bridge._handle_mavlink_message(
            _qgc_frame(mav.MAVLINK_MSG_ID_MISSION_REQUEST_LIST, b'\x01\x01\x00'))

        parsed = _last_sent(sent)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_COUNT
        assert mav.parse_mission_count(parsed.payload)['count'] == 2

    def test_request_list_on_empty_mission_replies_zero(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)

        bridge._handle_mavlink_message(
            _qgc_frame(mav.MAVLINK_MSG_ID_MISSION_REQUEST_LIST, b'\x01\x01\x00'))

        parsed = _last_sent(sent)
        assert mav.parse_mission_count(parsed.payload)['count'] == 0

    def test_request_int_for_known_waypoint_replies_with_item(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        self._seed_mission(bridge)

        req_payload = mav.build_mission_request_int(seq=1, target_system=1, target_component=1)
        bridge._handle_mavlink_message(_qgc_frame(mav.MAVLINK_MSG_ID_MISSION_REQUEST_INT, req_payload))

        parsed = _last_sent(sent)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_ITEM_INT
        item = mav.parse_mission_item_int(parsed.payload)
        assert item['sequence'] == 1
        assert item['position']['latitude'] == pytest.approx(37.7750, abs=1e-4)

    def test_legacy_mission_request_also_handled(self):
        """MISSION_REQUEST (id 40) shares MISSION_REQUEST_INT's wire layout
        and must be accepted the same way, for GCS clients that still send it."""
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        self._seed_mission(bridge)

        req_payload = mav.build_mission_request_int(seq=0, target_system=1, target_component=1)
        bridge._handle_mavlink_message(_qgc_frame(mav.MAVLINK_MSG_ID_MISSION_REQUEST, req_payload))

        parsed = _last_sent(sent)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_ITEM_INT

    def test_request_for_unknown_waypoint_sends_nothing(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        self._seed_mission(bridge)

        req_payload = mav.build_mission_request_int(seq=99, target_system=1, target_component=1)
        bridge._handle_mavlink_message(_qgc_frame(mav.MAVLINK_MSG_ID_MISSION_REQUEST_INT, req_payload))

        assert sent == []


class TestGcsIdentityTracking:
    """The bridge learns the GCS's system/component ID from incoming frames
    and uses it as the target for outbound targeted messages."""

    def test_gcs_identity_captured_from_incoming_frame(self):
        bridge = _full_bridge(gcs_system_id=255, gcs_component_id=0)
        _capture_socket(bridge)

        count_payload = mav.build_mission_count(0)
        frame = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, 0, count_payload, 42, 17)
        bridge._handle_mavlink_message(frame)

        assert bridge._gcs_system_id == 42
        assert bridge._gcs_component_id == 17

    def test_outbound_ack_targets_learned_gcs_identity(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)

        count_payload = mav.build_mission_count(0)
        frame = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, 0, count_payload, 42, 17)
        bridge._handle_mavlink_message(frame)

        parsed = _last_sent(sent)
        ack = mav.parse_mission_ack(parsed.payload)
        assert ack['target_system'] == 42

    def test_reply_targets_actual_sender_address_not_hardcoded_port(self):
        """Regression test: outbound frames were previously hardcoded to
        ('localhost', 14550) regardless of who sent the request or how the
        bridge was configured -- which silently broke the documented
        multi-drone setup (a different mavlink_port per drone)."""
        bridge = _full_bridge()
        sent, targets = _capture_socket_with_addr(bridge)

        count_payload = mav.build_mission_count(0)
        frame = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, 0, count_payload, 1, 1)
        # Simulate a sender on a non-default port, as a second drone's
        # bridge (mavlink_port:=14551) would see.
        bridge._handle_mavlink_message(frame, addr=('192.168.1.50', 14551))

        assert targets[-1] == ('192.168.1.50', 14551)

    def test_reply_falls_back_to_configured_address_before_any_packet_seen(self):
        """Before any inbound packet has been received (e.g. the periodic
        MISSION_CURRENT timer firing early), replies fall back to the
        bridge's own configured (mavlink_host, mavlink_port)."""
        bridge = _full_bridge()
        bridge._reply_addr = ('localhost', 14552)  # as __init__ would set it
        sent, targets = _capture_socket_with_addr(bridge)
        bridge._mission_items = [{'sequence': 0}]

        bridge._publish_mission_current()

        assert targets[-1] == ('localhost', 14552)


class TestPublishMissionCurrentReal:

    def test_no_mission_sends_nothing(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        bridge._publish_mission_current()
        assert sent == []

    def test_publishes_seq_and_total(self):
        bridge = _full_bridge()
        sent = _capture_socket(bridge)
        bridge._mission_items = [{'sequence': 0}, {'sequence': 1}, {'sequence': 2}]
        bridge._current_waypoint = 1

        bridge._publish_mission_current()

        parsed = _last_sent(sent)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_CURRENT
        payload = parsed.payload.ljust(6, b'\x00')
        seq, total = struct.unpack('<HH', payload[:4])
        assert seq == 1
        assert total == 3


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
