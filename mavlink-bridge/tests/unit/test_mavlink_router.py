#!/usr/bin/env python3
"""
Unit Test: MAVLink Router (mavlink_router_node.py)

Constructs the real MAVLinkRouterNode (bypassing __init__'s socket/ROS setup)
and drives its real _forward_to_downstream / _relay_to_qgc methods directly
with fake sockets, plus tests parse_targets in isolation. Real-socket,
multi-hop, end-to-end behavior (including with the real mission_control and
emergency_wipe bridges behind it) is covered by
tests/integration/test_mavlink_router_integration.py.

rclpy stub comes from tests/conftest.py.
"""

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mavlink_router_node import MAVLinkRouterNode, parse_targets


def _make_router(targets=None, last_qgc_addr=None):
    router = MAVLinkRouterNode.__new__(MAVLinkRouterNode)
    router._downstream_targets = targets if targets is not None else [
        ('localhost', 14551), ('localhost', 14556)]
    router._last_qgc_addr = last_qgc_addr
    router.get_logger = lambda: MagicMock()
    return router


def _capture_downstream_socket(router):
    sent = []  # list of (data, addr)
    router._downstream_socket = types.SimpleNamespace(
        sendto=lambda data, addr: sent.append((data, addr)))
    return sent


def _capture_external_socket(router):
    sent = []
    router._external_socket = types.SimpleNamespace(
        sendto=lambda data, addr: sent.append((data, addr)))
    return sent


class TestParseTargets:

    def test_parses_host_port_pairs(self):
        assert parse_targets(['localhost:14551', '127.0.0.1:14556']) == [
            ('localhost', 14551), ('127.0.0.1', 14556)]

    def test_skips_entries_without_a_colon(self):
        assert parse_targets(['not-a-target', 'localhost:14551']) == [('localhost', 14551)]

    def test_skips_entries_with_non_numeric_port(self):
        assert parse_targets(['localhost:abc', 'localhost:14551']) == [('localhost', 14551)]

    def test_empty_list_returns_empty(self):
        assert parse_targets([]) == []


class TestForwardToDownstream:

    def test_forwards_identical_bytes_to_every_target(self):
        router = _make_router(targets=[('localhost', 14551), ('localhost', 14556)])
        sent = _capture_downstream_socket(router)
        router._forward_to_downstream(b'hello')
        assert sent == [(b'hello', ('localhost', 14551)), (b'hello', ('localhost', 14556))]

    def test_no_targets_sends_nothing(self):
        router = _make_router(targets=[])
        sent = _capture_downstream_socket(router)
        router._forward_to_downstream(b'hello')
        assert sent == []

    def test_no_downstream_socket_does_not_raise(self):
        router = _make_router()
        router._downstream_socket = None
        router._forward_to_downstream(b'hello')  # must not raise

    def test_one_target_failing_does_not_block_the_others(self):
        router = _make_router(targets=[('bad', 1), ('localhost', 14556)])
        sent = []

        def sendto(data, addr):
            if addr == ('bad', 1):
                raise OSError('unreachable')
            sent.append((data, addr))

        router._downstream_socket = types.SimpleNamespace(sendto=sendto)
        router._forward_to_downstream(b'hello')  # must not raise
        assert sent == [(b'hello', ('localhost', 14556))]


class TestRelayToQgc:

    def test_relays_to_last_known_qgc_address(self):
        router = _make_router(last_qgc_addr=('192.168.1.50', 55123))
        sent = _capture_external_socket(router)
        router._relay_to_qgc(b'reply-bytes')
        assert sent == [(b'reply-bytes', ('192.168.1.50', 55123))]

    def test_no_qgc_seen_yet_sends_nothing(self):
        router = _make_router(last_qgc_addr=None)
        sent = _capture_external_socket(router)
        router._relay_to_qgc(b'reply-bytes')
        assert sent == []

    def test_no_external_socket_does_not_raise(self):
        router = _make_router(last_qgc_addr=('localhost', 1))
        router._external_socket = None
        router._relay_to_qgc(b'reply-bytes')  # must not raise

    def test_send_failure_does_not_raise(self):
        router = _make_router(last_qgc_addr=('localhost', 1))

        def sendto(data, addr):
            raise OSError('gone')

        router._external_socket = types.SimpleNamespace(sendto=sendto)
        router._relay_to_qgc(b'reply-bytes')  # must not raise


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
