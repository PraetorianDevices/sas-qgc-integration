# MAVLink Bridge Tests

Comprehensive test suite for GPS spoofing detector and telemetry bridge integration with QGroundControl.

## Test Structure

```
tests/
├── conftest.py                          # Shared pytest fixtures and configuration
├── unit/                                # Unit tests (isolated, no ROS 2 required)
│   ├── test_mavlink_frame_generation.py # MAVLink 2.0 frame structure and CRC
│   ├── test_mavlink_crc.py              # CRC16-CCITT computation validation
│   └── test_telemetry_conversion.py     # px4_msgs → MAVLink conversion logic
├── integration/                         # Integration tests (require ROS 2)
│   ├── test_gps_spoof_integration.py    # GPS spoofing detector → STATUSTEXT bridge
│   ├── test_telemetry_integration.py    # Offboard controller → telemetry bridge (NEW)
│   ├── test_full_pipeline.py            # Complete SAS → QGC pipeline (NEW)
│   └── launch_full_integration.py       # ROS 2 launch file for manual testing
└── fixtures/                            # Test utilities and helper nodes
    └── alert_generator.py               # Synthetic GPS spoofing alert generator
```

## Test Coverage by Node Integration

### GPS Spoofing Detector → MAVLink Bridge

| Test | Type | Coverage | Status |
|------|------|----------|--------|
| `test_gps_spoof_integration.py` | Integration | Real ROS 2 node, synthetic sensor data | ✅ EXISTING |
| `test_integration.py` | Unit | JSON alert parsing, MAVLink conversion | ✅ EXISTING |
| `alert_generator.py` | Fixture | Synthetic alert publisher for testing | ✅ EXISTING |

**Redundancy Check:**
- ❌ NOT REDUNDANT with SAS tests
  - SAS tests verify detector publishes correct JSON
  - Bridge tests verify bridge converts JSON to MAVLink correctly
  - Different layers, complementary coverage

### Offboard Controller → Telemetry Bridge

| Test | Type | Coverage | Status |
|------|------|----------|--------|
| `test_telemetry_conversion.py` | Unit | NED frame, unit scaling, message builders | ✅ NEW |
| `test_telemetry_integration.py` | Integration | ROS 2 topic subscriptions, MAVLink output | ⏳ TODO |
| `test_full_pipeline.py` | Integration | Detector + telemetry bridge together | ⏳ TODO |

**Redundancy Check:**
- ❌ NOT REDUNDANT with SAS tests
  - SAS `test_offboard_controller_node.py` tests node outputs px4_msgs
  - Bridge tests verify conversion of those px4_msgs to MAVLink
  - Complementary, not overlapping

### MAVLink Frame Generation

| Test | Type | Coverage | Status |
|------|------|----------|--------|
| `test_mavlink_frame_generation.py` | Unit | Frame structure, payload formatting | ✅ EXISTING |
| `test_mavlink_crc.py` | Unit | CRC16-CCITT computation per message type | ✅ EXISTING |

**Redundancy Check:**
- ✅ OPTIMIZED (was duplicate, now split by concern)
  - Frame generation tests focus on frame structure
  - CRC tests focus on checksum computation
  - Clear separation of concerns

---

## Running Tests

### Prerequisites

```bash
# For unit tests only
pip install pytest

# For integration tests (with ROS 2)
source /opt/ros/jazzy/setup.bash
cd d:/praetoriandevices/SAS
colcon build --packages-select mavlink-bridge
source install/setup.bash
```

### Unit Tests (No ROS 2 Required)

```bash
# All unit tests
pytest tests/unit/ -v

# Specific test file
pytest tests/unit/test_telemetry_conversion.py -v

# Specific test class
pytest tests/unit/test_telemetry_conversion.py::TestCoordinateConversions -v

# Single test
pytest tests/unit/test_telemetry_conversion.py::TestCoordinateConversions::test_ned_altitude_from_local_position -v
```

### Integration Tests (Requires ROS 2)

```bash
# All integration tests
pytest tests/integration/ -v -s

# Specific integration test
pytest tests/integration/test_gps_spoof_integration.py -v -s

# Flag: -s = show ROS 2 logging output
```

### Run All Tests

```bash
# From mavlink-bridge directory
pytest tests/ -v

# With coverage report (install pytest-cov first)
pytest tests/ --cov=. --cov-report=html
```

### Run with Markers

```bash
# Only unit tests
pytest -m unit tests/

# Only integration tests (requires ROS 2)
pytest -m integration tests/

# Skip ROS 2 tests
pytest -m "not ros2" tests/
```

---

## Test Categories

### Unit Tests

**Purpose:** Test individual components in isolation without external dependencies.

**Location:** `tests/unit/`

**Key Tests:**
1. **test_mavlink_frame_generation.py**
   - MAVLink 2.0 frame structure (STX, LEN, MSG_ID, etc.)
   - Payload padding and truncation
   - Sequence numbering and wrap-around
   - Alert level mapping

2. **test_mavlink_crc.py**
   - CRC16-CCITT computation
   - CRC_EXTRA per message type
   - Round-trip CRC validation

3. **test_telemetry_conversion.py** ← NEW
   - NED frame altitude conversion
   - GPS position scaling (degrees → 1e-7 format)
   - Heading/velocity/battery unit conversions
   - Quaternion to Euler angle conversion
   - MAVLink message payload structure validation

**Run Time:** <1 second

### Integration Tests

**Purpose:** Test full end-to-end pipelines with ROS 2 and real message flows.

**Location:** `tests/integration/`

**Key Tests:**
1. **test_gps_spoof_integration.py** (EXISTING)
   - Detector node publishes JSON alerts
   - Bridge receives and validates JSON format
   - Converts to MAVLink STATUSTEXT
   - Validates severity mapping

2. **test_telemetry_integration.py** (TODO)
   - Offboard controller publishes px4_msgs
   - Bridge subscribes to telemetry topics
   - Converts to MAVLink messages
   - Validates HEARTBEAT, GLOBAL_POSITION_INT, ATTITUDE, SYS_STATUS, BATTERY_STATUS

3. **test_full_pipeline.py** (TODO)
   - Both bridges running together
   - Multiple message types on same UDP port
   - QGC receives without packet loss
   - Sequence numbers continuous

**Run Time:** 5-30 seconds per test

---

## Test Isolation & Dependencies

### What Tests DON'T Do

- ❌ Test SAS node implementations (offboard_controller, gps_spoof_detector)
  - Those are covered by SAS/tests/
  - Bridge tests assume those nodes work correctly

- ❌ Test QGC parsing of MAVLink messages
  - QGC has its own test suite
  - Bridge tests ensure correct frame generation

- ❌ Test UDP network transport
  - Socket libraries are tested by Python/system
  - Bridge tests assume UDP works

### What Tests DO Cover

- ✅ Correct conversion from px4_msgs to MAVLink format
- ✅ Proper coordinate frame handling (NED)
- ✅ Correct unit scaling (m/s → cm/s, V → mV, etc.)
- ✅ MAVLink frame structure and CRC validation
- ✅ JSON parsing and payload validation
- ✅ ROS 2 topic subscription and publishing
- ✅ Multi-message handling on single UDP port

---

## Redundancy Analysis

### GPS Spoofing Path

```
SAS/tests/
  ├─ test_gps_spoof_detector_node.py    # Detector outputs correct JSON
  └─ test_gps_spoof_integration.py       # Real detector with synthetic sensors

mavlink-bridge/tests/
  ├─ test_integration.py                # JSON → MAVLink conversion
  └─ test_gps_spoof_integration.py      # Full pipeline: detector → bridge → UDP
```

**Conclusion:** Complementary, not redundant. Different components tested at different layers.

### Telemetry Path

```
SAS/tests/
  └─ test_offboard_controller_node.py   # Controller outputs correct px4_msgs

mavlink-bridge/tests/
  ├─ test_telemetry_conversion.py       # px4_msgs → MAVLink conversion
  └─ test_telemetry_integration.py      # Full pipeline: controller → bridge → UDP
```

**Conclusion:** Complementary. One tests output, the other tests consumption.

### MAVLink Frames

```
BEFORE (test_integration.py):
  - Frame structure tests
  - CRC tests
  - Alert parsing tests
  ← ALL MIXED TOGETHER

AFTER (split into unit tests):
  - test_mavlink_frame_generation.py    # Frame structure only
  - test_mavlink_crc.py                 # CRC only
  - test_mavlink_*.py                   # Specific message types
```

**Conclusion:** Refactored for clarity, not redundant.

---

## Test Execution Matrix

| Component | Unit | Integration | Manual | Status |
|-----------|------|-------------|--------|--------|
| **GPS Spoof Bridge** | ✅ | ✅ | ✅ | Complete |
| **Telemetry Bridge** | ✅ | ⏳ | ⏳ | In Progress |
| **MAVLink Frames** | ✅ | - | - | Complete |
| **Full Pipeline** | - | ⏳ | ⏳ | TODO |

---

## Adding New Tests

### Unit Test Template

```python
import pytest
from conftest import MockMAVLinkBuilder

class TestNewFeature:
    """Test description."""

    def test_specific_case(self, mavlink_builder):
        """Test case description."""
        # Arrange
        payload = b'...'
        
        # Act
        frame = mavlink_builder.build_frame(msg_id=1, seq=0, payload=payload)
        
        # Assert
        assert frame[0] == 0xFD  # STX byte
        assert len(frame) > 0
```

### Integration Test Template

```python
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

@pytest.mark.ros2
class TestNewIntegration:
    """ROS 2 integration test."""

    def test_ros2_interaction(self):
        """Test ROS 2 topic interaction."""
        rclpy.init()
        
        # Create nodes, subscribe to topics, etc.
        
        rclpy.shutdown()
```

---

## Continuous Integration

Tests are designed to run in CI/CD pipelines:

```yaml
# Example GitHub Actions workflow
- name: Unit tests
  run: pytest tests/unit/ -v

- name: Integration tests (ROS 2 required)
  if: runner.os == 'Linux'
  run: |
    source /opt/ros/jazzy/setup.bash
    pytest tests/integration/ -v -s
```

---

## References

- **pytest documentation:** https://docs.pytest.org/
- **MAVLink spec:** https://mavlink.io/
- **ROS 2 testing:** https://docs.ros.org/en/jazzy/Concepts/About-Testing.html
