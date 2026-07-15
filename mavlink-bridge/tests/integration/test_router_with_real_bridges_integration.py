#!/usr/bin/env python3
"""
Integration Test: MAVLink Router with the REAL mission_control_bridge and REAL
emergency_wipe_mavlink_bridge sharing it simultaneously.

This is the definitive proof that the originally-flagged limitation is
resolved: "QGroundControl uses a single UDP comm link per vehicle, but
mission_control_bridge and emergency_wipe_mavlink_bridge each need to bind a
UDP socket to receive, and two processes cannot cleanly bind the same UDP
port" (see IMPLEMENTATION_STATUS.md Known Limitations, now resolved).

Both real bridges run behind ONE real MAVLinkRouterNode, reachable from a
SINGLE simulated-QGC socket -- exactly the topology the launch file wires up
for a real deployment. Neither bridge required any code change for this; both
already bind whatever port they're configured with and learn their reply
address dynamically from whoever last contacted them (see each bridge's
_reply_addr) -- pointed at the router instead of directly at QGC, that
existing mechanism keeps working unmodified.

rclpy/std_msgs/std_srvs stubs + the ros_params fixture come from
tests/conftest.py.
"""

import socket
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mavlink_v2 as mav
from mavlink_router_node import MAVLinkRouterNode
from mission_control_bridge import MissionControlBridge
from emergency_wipe_mavlink_bridge import EmergencyWipeMAVLinkBridge, MAV_CMD_USER_1


@pytest.fixture
def qgc_socket():
    """A single simulated QGroundControl UDP endpoint -- the whole point of
    this test is that this ONE socket can reach BOTH bridges."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3.0)
    yield sock
    sock.close()


@pytest.fixture
def topology(ros_params):
    """Real router + real mission_control_bridge + real emergency_wipe_bridge,
    wired together exactly as launch_sas_qgc_integration.py wires them: the
    router owns the single external (QGC-facing) port; both bridges bind
    their own internal ports and are listed as the router's downstream
    targets. All ports are ephemeral (0) to avoid collisions between runs."""
    import rclpy
    rclpy.ok.return_value = True

    ros_params.update({
        'system_id': 1, 'component_id': 1, 'drone_id': '',
        'mavlink_host': 'localhost', 'mavlink_port': 0,
    })
    mc_bridge = MissionControlBridge()
    mc_port = mc_bridge._socket.getsockname()[1]

    ros_params.update({
        'system_id': 1, 'component_id': 1, 'drone_id': '',
        'mavlink_host': 'localhost', 'mavlink_port': 0,
        'wipe_command_id': MAV_CMD_USER_1, 'wipe_magic_param1': 1.0,
    })
    ew_bridge = EmergencyWipeMAVLinkBridge()
    ew_bridge._wipe_client.service_is_ready.return_value = True
    ew_port = ew_bridge._socket.getsockname()[1]

    ros_params.update({
        'mavlink_host': 'localhost', 'mavlink_port': 0,
        'downstream_bind_host': 'localhost', 'downstream_bind_port': 0,
        'downstream_targets': [f'localhost:{mc_port}', f'localhost:{ew_port}'],
    })
    router = MAVLinkRouterNode()

    yield router, mc_bridge, ew_bridge

    rclpy.ok.return_value = False
    router.destroy_node()
    for bridge in (mc_bridge, ew_bridge):
        if bridge._socket is not None:
            try:
                bridge._socket.close()
            except OSError:
                pass
    mc_bridge._receiver_thread.join(timeout=2.0)
    ew_bridge._receiver_thread.join(timeout=2.0)


def _command_long(command, param1, confirmation, target_system=1, target_component=1):
    payload = struct.pack('<fffffffHBBB', param1, 0, 0, 0, 0, 0, 0,
                          command, target_system, target_component, confirmation)
    return mav.build_frame(mav.MAVLINK_MSG_ID_COMMAND_LONG, 0, payload, 255, 0)


class TestEachBridgeThroughTheRouter:

    def test_mission_count_reaches_mission_control_through_router(self, topology, qgc_socket):
        router, _, _ = topology
        router_addr = router._external_socket.getsockname()

        count_payload = mav.build_mission_count(0, target_system=1, target_component=1)
        qgc_socket.sendto(
            mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, 0, count_payload, 255, 0),
            router_addr)

        data, _ = qgc_socket.recvfrom(1024)
        parsed = mav.parse_frame(data)
        assert parsed.valid
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_ACK
        assert mav.parse_mission_ack(parsed.payload)['result'] == mav.MAVMissionResult.ACCEPTED

    def test_command_long_reaches_emergency_wipe_through_router(self, topology, qgc_socket):
        router, _, _ = topology
        router_addr = router._external_socket.getsockname()

        qgc_socket.sendto(_command_long(MAV_CMD_USER_1, param1=1.0, confirmation=1), router_addr)

        # An accepted wipe command produces two replies: COMMAND_ACK first,
        # then a STATUSTEXT announcement (see _handle_wipe_command). Drain both.
        data, _ = qgc_socket.recvfrom(1024)
        parsed = mav.parse_frame(data)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_COMMAND_ACK
        command, result = struct.unpack('<HB', parsed.payload.ljust(3, b'\x00')[:3])
        assert command == MAV_CMD_USER_1
        assert result == int(mav.MAVResult.ACCEPTED)

        data2, _ = qgc_socket.recvfrom(1024)
        assert mav.parse_frame(data2).msg_id == mav.MAVLINK_MSG_ID_STATUSTEXT

    def test_denied_gate_still_acked_through_router(self, topology, qgc_socket):
        router, _, _ = topology
        router_addr = router._external_socket.getsockname()

        qgc_socket.sendto(_command_long(MAV_CMD_USER_1, param1=0.0, confirmation=1), router_addr)

        data, _ = qgc_socket.recvfrom(1024)
        parsed = mav.parse_frame(data)
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_COMMAND_ACK
        _, result = struct.unpack('<HB', parsed.payload.ljust(3, b'\x00')[:3])
        assert result == int(mav.MAVResult.DENIED)


class TestBothBridgesShareOneQgcLink:
    """The definitive regression test: a SINGLE simulated QGC socket talks to
    BOTH bridges through the SAME router -- this is exactly what QGC's
    one-comm-link-per-vehicle constraint requires, and exactly what was
    impossible before this router existed."""

    def test_single_socket_reaches_both_bridges_in_sequence(self, topology, qgc_socket):
        router, _, _ = topology
        router_addr = router._external_socket.getsockname()

        count_payload = mav.build_mission_count(0, target_system=1, target_component=1)
        qgc_socket.sendto(
            mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, 0, count_payload, 255, 0),
            router_addr)
        data1, _ = qgc_socket.recvfrom(1024)
        assert mav.parse_frame(data1).msg_id == mav.MAVLINK_MSG_ID_MISSION_ACK

        qgc_socket.sendto(_command_long(MAV_CMD_USER_1, param1=1.0, confirmation=1), router_addr)
        data2, _ = qgc_socket.recvfrom(1024)
        assert mav.parse_frame(data2).msg_id == mav.MAVLINK_MSG_ID_COMMAND_ACK

    def test_interleaved_traffic_each_reply_goes_to_the_right_place(self, topology, qgc_socket):
        """Both bridges receive every inbound frame (broadcast fan-out), but
        each only acts on and replies to the message types it understands --
        confirm interleaving doesn't cause cross-talk or dropped replies.

        An accepted wipe command produces two replies (COMMAND_ACK, then a
        STATUSTEXT announcement -- see emergency_wipe_mavlink_bridge's
        _handle_wipe_command), so this interleaving produces 3 total frames:
        COMMAND_ACK + STATUSTEXT from the wipe command, MISSION_ACK from the
        mission command."""
        router, _, _ = topology
        router_addr = router._external_socket.getsockname()

        qgc_socket.sendto(_command_long(MAV_CMD_USER_1, param1=1.0, confirmation=1), router_addr)
        count_payload = mav.build_mission_count(0, target_system=1, target_component=1)
        qgc_socket.sendto(
            mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, 1, count_payload, 255, 0),
            router_addr)

        msg_ids = []
        for _ in range(3):
            data, _ = qgc_socket.recvfrom(1024)
            msg_ids.append(mav.parse_frame(data).msg_id)
        assert set(msg_ids) == {
            mav.MAVLINK_MSG_ID_COMMAND_ACK,
            mav.MAVLINK_MSG_ID_STATUSTEXT,
            mav.MAVLINK_MSG_ID_MISSION_ACK,
        }


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
