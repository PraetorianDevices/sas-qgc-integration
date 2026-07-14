"""
Shared pytest configuration and fixtures for MAVLink bridge tests.

This module provides:
  - Pytest configuration
  - Shared fixtures for MAVLink frame building, delegating to the real,
    pymavlink-verified mavlink_v2 module rather than a parallel
    reimplementation
  - A single, consolidated rclpy/std_msgs/px4_msgs stub, installed once here
    rather than per test file

Why the stub lives here and not per-file: gps_spoof_mavlink_bridge.py is
imported by both test_mavlink_crc.py (unit) and test_gps_spoof_integration.py
(integration); mission_control_bridge.py is imported by both
test_mission_control_bridge.py (unit) and test_mission_control_integration.py
(integration). Python only executes a module's class bodies once and caches
the result in sys.modules -- so whichever test file's rclpy stub happened to
be active at first import "wins" for the rest of the process, regardless of
what any later test file installs. Concretely: unit tests bypass __init__ via
GPSSpoofMAVLinkBridge.__new__() and only need a trivial rclpy.node.Node
stand-in, while integration tests call the real __init__() and need a fully
functional one (declare_parameter/get_parameter/create_subscription/etc).
Having each file install its own, different-capability stub meant the test
suite's pass/fail depended on file *collection order* -- `pytest tests/`
happened to pass only because directory scanning collects tests/integration/
before tests/unit/ alphabetically, installing the capable stub first;
running specific files together in the other order broke it outright. This
single, always-fully-capable stub removes that order dependency.
"""

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

# mavlink_v2.py lives at the mavlink-bridge/ root, one level above tests/.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mavlink_v2 as mav


class DummyString:
    """Stand-in for std_msgs.msg.String."""
    def __init__(self):
        self.data = ''


# Parameter values the DummyNode below returns from get_parameter(); tests
# that construct a real node (rather than bypassing __init__) mutate this via
# the `ros_params` fixture before construction, e.g. to bind an ephemeral port.
ROS_PARAM_OVERRIDES = {}


class DummyNode:
    """A single, fully-capable rclpy.Node stand-in, shared by every test file
    in this suite that constructs a bridge via its real __init__ (see module
    docstring for why this must not be a per-file, varying-capability stub).
    """
    def __init__(self, name):
        self._logger = MagicMock()
        self._name = name

    def declare_parameter(self, name, default=None):
        ROS_PARAM_OVERRIDES.setdefault(name, default)

    def get_parameter(self, name):
        value = ROS_PARAM_OVERRIDES.get(name)
        m = MagicMock()
        m.value = value
        m.get_parameter_value.return_value.string_value = value if isinstance(value, str) else ''
        m.get_parameter_value.return_value.string_array_value = value if isinstance(value, list) else []
        return m

    def create_publisher(self, msg_type, topic, qos):
        return MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos):
        return MagicMock()

    def create_timer(self, period, callback):
        return MagicMock()

    def create_client(self, srv_type, srv_name):
        return MagicMock()

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        pass


def _install_ros_stubs():
    """Install rclpy/std_msgs/px4_msgs stubs exactly once per session."""
    if getattr(sys.modules.get('rclpy'), '_praetorian_stub', False):
        return

    rclpy_mock = MagicMock()
    rclpy_mock._praetorian_stub = True
    rclpy_mock.node.Node = DummyNode
    rclpy_mock.ok.return_value = True
    sys.modules['rclpy'] = rclpy_mock
    sys.modules['rclpy.node'] = rclpy_mock.node
    sys.modules['rclpy.qos'] = MagicMock()

    std_msgs_mock = MagicMock()
    std_msgs_mock.String = DummyString
    sys.modules['std_msgs'] = MagicMock()
    sys.modules['std_msgs.msg'] = std_msgs_mock

    # std_srvs/Trigger — used by emergency_wipe_mavlink_bridge. Provide a real
    # Trigger class with Request/Response so tests can construct genuine
    # responses (success:bool, message:str) rather than opaque mocks.
    class TriggerRequest:
        pass

    class TriggerResponse:
        def __init__(self):
            self.success = False
            self.message = ''

    class Trigger:
        Request = TriggerRequest
        Response = TriggerResponse

    std_srvs_srv_mock = types.ModuleType('std_srvs.srv')
    std_srvs_srv_mock.Trigger = Trigger
    sys.modules['std_srvs'] = types.ModuleType('std_srvs')
    sys.modules['std_srvs.srv'] = std_srvs_srv_mock

    px4_msgs_mock = types.ModuleType('px4_msgs.msg')
    for name in ('VehicleLocalPosition', 'VehicleAttitude', 'VehicleStatus',
                 'BatteryStatus', 'SensorGps', 'ObstacleDistance'):
        setattr(px4_msgs_mock, name, type(name, (), {}))
    sys.modules['px4_msgs'] = types.ModuleType('px4_msgs')
    sys.modules['px4_msgs.msg'] = px4_msgs_mock


_install_ros_stubs()


@pytest.fixture
def ros_params():
    """Mutable dict of parameter overrides consumed by DummyNode.get_parameter.
    Depend on this fixture and populate it before constructing a real node,
    e.g.: `ros_params.update({'mavlink_port': qgc_listener.getsockname()[1]})`.
    Cleared before and after each test so values never leak between tests.
    """
    ROS_PARAM_OVERRIDES.clear()
    yield ROS_PARAM_OVERRIDES
    ROS_PARAM_OVERRIDES.clear()


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
