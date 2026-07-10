#!/usr/bin/env python3
"""
Integration Test: Mission Control Bridge -- real UDP sockets

Spins up the REAL MissionControlBridge with its actual __init__ (real bound
UDP socket, real background receiver thread), then drives it from a second,
independent UDP socket simulating QGC. This exercises exactly what the unit
tests in tests/unit/test_mission_control_bridge.py cannot: real socket
binding, the background receive thread actually running, and genuine
byte-level round-tripping over loopback UDP -- not direct method calls with
a fake socket object standing in.

A previous version of this file never imported mission_control_bridge at
all; every "integration test" reimplemented the mission upload/ACK handshake
inline and asserted on that copy. None of it would have caught the bridge's
own frame-format bugs, the missing upload handshake, or the hardcoded reply
address bug (fixed in this same session) -- this version would have caught
all three, since it drives the bridge through actual sockets rather than
skipping straight to asserting on hand-built byte strings.

NOT REDUNDANT with SAS tests:
  - SAS tests: executor loads JSON mission, executes it
  - These tests: real UDP bytes in -> bridge -> real UDP bytes out, plus the
    ROS 2 publish call to mission_executor_node
  - Different layer entirely from SAS's own test suite
"""

import json
import socket
import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def _install_ros_stubs():
    if 'mission_control_bridge' in sys.modules:
        return

    class _DummyString:
        def __init__(self):
            self.data = ''

    class _DummyNode:
        def __init__(self, name):
            self._logger = MagicMock()
            self._publishers = {}
            self._subscriptions = {}
            self._timers = []

        def declare_parameter(self, name, default=None):
            pass

        def get_parameter(self, name):
            m = MagicMock()
            m.value = _PARAM_VALUES.get(name, None)
            return m

        def create_publisher(self, msg_type, topic, qos):
            pub = MagicMock()
            self._publishers[topic] = pub
            return pub

        def create_subscription(self, msg_type, topic, callback, qos):
            sub = MagicMock()
            self._subscriptions[topic] = callback
            return sub

        def create_timer(self, period, callback):
            timer = MagicMock()
            self._timers.append(callback)
            return timer

        def get_logger(self):
            return self._logger

        def destroy_node(self):
            pass

    rclpy_mock = MagicMock()
    rclpy_mock.node.Node = _DummyNode
    rclpy_mock.ok.return_value = True
    sys.modules['rclpy'] = rclpy_mock
    sys.modules['rclpy.node'] = rclpy_mock.node
    sys.modules['rclpy.qos'] = MagicMock()

    std_msgs_mock = MagicMock()
    std_msgs_mock.String = _DummyString
    sys.modules['std_msgs'] = MagicMock()
    sys.modules['std_msgs.msg'] = std_msgs_mock


_PARAM_VALUES = {
    'system_id': 1,
    'component_id': 1,
    'mavlink_host': 'localhost',
    'mavlink_port': 0,  # ephemeral -- avoids port collisions between test runs
    'drone_id': '',
}

_install_ros_stubs()

import mavlink_v2 as mav
from mission_control_bridge import MissionControlBridge


@pytest.fixture
def live_bridge():
    """A real MissionControlBridge with a real bound UDP socket and a real
    running receiver thread. Yields (bridge, bridge_addr); tears the thread
    down afterward by flipping rclpy.ok() to False and closing the socket.
    """
    import rclpy
    rclpy.ok.return_value = True

    bridge = MissionControlBridge()
    bridge_port = bridge._socket.getsockname()[1]

    yield bridge, ('localhost', bridge_port)

    rclpy.ok.return_value = False
    if bridge._socket is not None:
        try:
            bridge._socket.close()
        except OSError:
            pass
    bridge._receiver_thread.join(timeout=2.0)


@pytest.fixture
def qgc_socket():
    """A second, independent UDP socket simulating QGroundControl."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    yield sock
    sock.close()


def _send(sock, msg_id, payload, dest_addr, seq=0, sysid=255, compid=0):
    frame = mav.build_frame(msg_id, seq, payload, sysid, compid)
    sock.sendto(frame, dest_addr)


def _recv_parsed(sock):
    data, _ = sock.recvfrom(1024)
    return mav.parse_frame(data)


class TestRealSocketUploadHandshake:
    """Drives a real, running bridge through a genuine UDP round trip."""

    def test_count_triggers_real_request_int_reply(self, live_bridge, qgc_socket):
        bridge, bridge_addr = live_bridge

        count_payload = mav.build_mission_count(2, target_system=1, target_component=1)
        _send(qgc_socket, mav.MAVLINK_MSG_ID_MISSION_COUNT, count_payload, bridge_addr)

        parsed = _recv_parsed(qgc_socket)
        assert parsed.valid
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_REQUEST_INT
        assert mav.parse_mission_request(parsed.payload)['sequence'] == 0

    def test_full_upload_over_real_sockets_publishes_to_executor(self, live_bridge, qgc_socket):
        bridge, bridge_addr = live_bridge
        waypoints = [
            (377_749_000, -1_224_194_000, 100.0, 16),
            (377_750_000, -1_224_195_000, 150.0, 16),
        ]

        count_payload = mav.build_mission_count(len(waypoints), target_system=1, target_component=1)
        _send(qgc_socket, mav.MAVLINK_MSG_ID_MISSION_COUNT, count_payload, bridge_addr)

        for seq, (lat, lon, alt, cmd) in enumerate(waypoints):
            request = _recv_parsed(qgc_socket)
            assert request.msg_id == mav.MAVLINK_MSG_ID_MISSION_REQUEST_INT
            assert mav.parse_mission_request(request.payload)['sequence'] == seq

            item_payload = mav.build_mission_item_int(
                seq=seq, frame=3, command=cmd, current=0, autocontinue=1,
                param1=0.0, param2=0.0, param3=0.0, param4=0.0,
                x=lat, y=lon, z=alt, target_system=1, target_component=1)
            _send(qgc_socket, mav.MAVLINK_MSG_ID_MISSION_ITEM_INT, item_payload, bridge_addr, seq=seq + 1)

        ack = _recv_parsed(qgc_socket)
        assert ack.msg_id == mav.MAVLINK_MSG_ID_MISSION_ACK
        assert mav.parse_mission_ack(ack.payload)['result'] == mav.MAVMissionResult.ACCEPTED

        # Give the receiver thread a moment to process the final item and publish.
        deadline = time.monotonic() + 2.0
        upload_pub = bridge._mission_upload_pub
        while upload_pub.publish.call_count == 0 and time.monotonic() < deadline:
            time.sleep(0.05)

        upload_pub.publish.assert_called_once()
        published = json.loads(upload_pub.publish.call_args[0][0].data)
        assert published['fileType'] == 'Plan'
        assert len(published['mission']['items']) == 2

    def test_reply_addressed_to_actual_udp_sender_not_hardcoded_port(self, live_bridge, qgc_socket):
        """Regression test for the hardcoded-('localhost', 14550) reply bug --
        the reply must actually arrive back at the real ephemeral port the
        test socket is bound to, which is only possible if the bridge
        addresses it correctly."""
        bridge, bridge_addr = live_bridge
        qgc_socket.bind(('localhost', 0))  # claim a specific ephemeral port

        count_payload = mav.build_mission_count(0, target_system=1, target_component=1)
        _send(qgc_socket, mav.MAVLINK_MSG_ID_MISSION_COUNT, count_payload, bridge_addr)

        # If this doesn't arrive, the bridge sent its reply somewhere else.
        parsed = _recv_parsed(qgc_socket)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_ACK


class TestRealSocketDownloadHandshake:

    def test_request_list_over_real_socket(self, live_bridge, qgc_socket):
        bridge, bridge_addr = live_bridge
        bridge._mission_items = [
            {'sequence': 0, 'frame': 3, 'command': 16, 'current': 0, 'autocontinue': 1,
             'params': [0.0, 0.0, 0.0, 0.0],
             'position': {'latitude': 37.0, 'longitude': -122.0, 'altitude': 50.0}},
        ]

        _send(qgc_socket, mav.MAVLINK_MSG_ID_MISSION_REQUEST_LIST, b'\x01\x01\x00', bridge_addr)

        parsed = _recv_parsed(qgc_socket)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_COUNT
        assert mav.parse_mission_count(parsed.payload)['count'] == 1

    def test_request_int_over_real_socket_returns_correct_waypoint(self, live_bridge, qgc_socket):
        bridge, bridge_addr = live_bridge
        bridge._mission_items = [
            {'sequence': 0, 'frame': 3, 'command': 16, 'current': 0, 'autocontinue': 1,
             'params': [0.0, 0.0, 0.0, 0.0],
             'position': {'latitude': 37.7749, 'longitude': -122.4194, 'altitude': 100.0}},
        ]

        req_payload = mav.build_mission_request_int(seq=0, target_system=1, target_component=1)
        _send(qgc_socket, mav.MAVLINK_MSG_ID_MISSION_REQUEST_INT, req_payload, bridge_addr)

        parsed = _recv_parsed(qgc_socket)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_ITEM_INT
        item = mav.parse_mission_item_int(parsed.payload)
        assert item['position']['latitude'] == pytest.approx(37.7749, abs=1e-4)


class TestRealSocketMissionCurrentTimer:

    def test_publish_mission_current_reaches_real_socket(self, live_bridge, qgc_socket):
        bridge, bridge_addr = live_bridge
        qgc_socket.bind(('localhost', 0))
        bridge._mission_items = [{'sequence': 0}, {'sequence': 1}]
        bridge._current_waypoint = 1
        # First inbound packet establishes _reply_addr toward our test socket.
        _send(qgc_socket, mav.MAVLINK_MSG_ID_MISSION_REQUEST_LIST, b'\x01\x01\x00', bridge_addr)
        _recv_parsed(qgc_socket)  # drain the MISSION_COUNT reply to that request

        bridge._publish_mission_current()

        parsed = _recv_parsed(qgc_socket)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_CURRENT
        import struct
        seq, total = struct.unpack('<HH', parsed.payload.ljust(6, b'\x00')[:4])
        assert seq == 1
        assert total == 2


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
