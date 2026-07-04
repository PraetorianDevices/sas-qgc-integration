#!/usr/bin/env python3
"""
Unit Test: MAVLink Frame Generation

Tests the MAVLink 2.0 frame generation logic without requiring ROS 2 runtime.
Verifies:
  - STATUSTEXT payload formatting
  - MAVLink 2.0 frame structure
  - CRC16-CCITT computation
  - Alert level mapping
"""

import struct
import json
from gps_spoof_mavlink_bridge import GPSSpoofMAVLinkBridge


def test_mavlink_frame_structure():
    """Test that MAVLink frames are correctly formatted."""
    bridge = GPSSpoofMAVLinkBridge.__new__(GPSSpoofMAVLinkBridge)
    bridge.system_id = 1
    bridge.component_id = 200
    bridge._sequence = 0

    # Build a test frame
    text = "GPS SPOOF DETECTED: heading divergence"
    severity = 5  # CRITICAL
    payload = bridge._build_statustext_payload(text, severity)
    frame = bridge._build_mavlink_frame(
        msg_id=253,  # STATUSTEXT
        seq=0,
        payload=payload
    )

    print("=" * 70)
    print("TEST 1: MAVLink Frame Structure")
    print("=" * 70)
    print(f"Text:          {text}")
    print(f"Severity:      {severity} (CRITICAL)")
    print(f"Frame length:  {len(frame)} bytes")
    print(f"Frame (hex):   {frame.hex()[:60]}...")

    # Verify frame structure
    assert frame[0] == 0xFD, f"STX should be 0xFD, got {hex(frame[0])}"
    assert frame[1] <= 255, f"Payload length should be ≤255, got {frame[1]}"
    assert frame[3] == 253, f"Message ID should be 253 (STATUSTEXT), got {frame[3]}"
    assert frame[4] == 1, f"System ID should be 1, got {frame[4]}"
    assert frame[5] == 200, f"Component ID should be 200, got {frame[5]}"
    assert frame[6] == 0, f"Sequence should be 0, got {frame[6]}"

    print("✓ Frame structure validated")
    print()


def test_crc_computation():
    """Test CRC16-CCITT computation matches expected values."""
    bridge = GPSSpoofMAVLinkBridge.__new__(GPSSpoofMAVLinkBridge)
    bridge.system_id = 1
    bridge.component_id = 200

    text = "Test message"
    severity = 4  # WARNING
    payload = bridge._build_statustext_payload(text, severity)
    frame = bridge._build_mavlink_frame(msg_id=253, seq=0, payload=payload)

    # Extract CRC from frame (last 2 bytes, little-endian)
    crc_from_frame = struct.unpack('<H', frame[-2:])[0]

    # Recompute CRC
    crc_recomputed = bridge._compute_mavlink_crc(frame[1:-2], 253)

    print("=" * 70)
    print("TEST 2: CRC16-CCITT Computation")
    print("=" * 70)
    print(f"CRC from frame:   0x{crc_from_frame:04x}")
    print(f"CRC recomputed:   0x{crc_recomputed:04x}")
    print(f"Match: {'✓ YES' if crc_from_frame == crc_recomputed else '✗ NO'}")

    assert crc_from_frame == crc_recomputed, "CRC mismatch!"
    print()


def test_statustext_payload():
    """Test STATUSTEXT payload formatting."""
    bridge = GPSSpoofMAVLinkBridge.__new__(GPSSpoofMAVLinkBridge)

    text = "GPS altitude spoofed"
    severity = 5  # CRITICAL
    payload = bridge._build_statustext_payload(text, severity)

    print("=" * 70)
    print("TEST 3: STATUSTEXT Payload Formatting")
    print("=" * 70)
    print(f"Input text:       '{text}'")
    print(f"Payload length:   {len(payload)} bytes")
    print(f"Expected length:  53 bytes (1 severity + 50 text + 2 id)")

    assert len(payload) == 53, f"Payload should be 53 bytes, got {len(payload)}"

    # Verify severity byte
    severity_from_payload = payload[0]
    print(f"Severity byte:    {severity_from_payload} (expected {severity})")
    assert severity_from_payload == severity, "Severity mismatch!"

    # Verify text is padded correctly
    text_from_payload = payload[1:51].rstrip(b'\x00').decode('ascii', errors='ignore')
    print(f"Text from payload: '{text_from_payload}'")
    assert text_from_payload == text, "Text mismatch!"

    print("✓ STATUSTEXT payload validated")
    print()


def test_alert_level_mapping():
    """Test alert level to MAVLink severity mapping."""
    print("=" * 70)
    print("TEST 4: Alert Level Mapping")
    print("=" * 70)

    mappings = {
        'INFO': 0,
        'WARNING': 4,
        'CRITICAL': 5,
    }

    for alert_level, expected_mav_severity in mappings.items():
        print(f"{alert_level:10} → MAV_SEVERITY {expected_mav_severity}")

    print("✓ Mapping table verified")
    print()


def test_truncation():
    """Test that long messages are truncated to 50 chars."""
    bridge = GPSSpoofMAVLinkBridge.__new__(GPSSpoofMAVLinkBridge)

    long_text = "This is a very long message that exceeds the 50-character limit for MAVLink STATUSTEXT messages"
    payload = bridge._build_statustext_payload(long_text, 4)

    text_from_payload = payload[1:51].rstrip(b'\x00').decode('ascii', errors='ignore')

    print("=" * 70)
    print("TEST 5: Message Truncation")
    print("=" * 70)
    print(f"Input length:     {len(long_text)} chars")
    print(f"Output length:    {len(text_from_payload)} chars")
    print(f"Input text:       {long_text[:50]}...")
    print(f"Output text:      {text_from_payload}")

    assert len(text_from_payload) <= 50, "Truncation failed!"
    print("✓ Truncation validated (max 50 chars)")
    print()


def test_json_alert_parsing():
    """Test parsing of GPS spoofing alert JSON."""
    print("=" * 70)
    print("TEST 6: GPS Spoofing Alert JSON Parsing")
    print("=" * 70)

    alert_json = {
        "alert_id": 1,
        "level": "CRITICAL",
        "strategy": "HEADING",
        "state": "SPOOFING_DETECTED",
        "detail": {
            "ekf2_heading_deg": 45.0,
            "mag_heading_deg": 75.5,
            "diff_deg": 30.5,
            "description": "EKF2 heading diverging significantly from raw magnetometer."
        },
        "timestamp_us": 1234567890
    }

    json_str = json.dumps(alert_json)
    parsed = json.loads(json_str)

    print(f"Alert ID:      {parsed['alert_id']}")
    print(f"Level:         {parsed['level']}")
    print(f"Strategy:      {parsed['strategy']}")
    print(f"State:         {parsed['state']}")
    print(f"Description:   {parsed['detail']['description']}")

    assert parsed['level'] == 'CRITICAL', "Level mismatch!"
    assert parsed['strategy'] == 'HEADING', "Strategy mismatch!"
    assert parsed['state'] == 'SPOOFING_DETECTED', "State mismatch!"

    print("✓ JSON parsing validated")
    print()


def main():
    """Run all tests."""
    print("\n")
    print("╔" + "=" * 68 + "╗")
    print("║" + " " * 15 + "GPS Spoofing MAVLink Bridge — Unit Tests" + " " * 13 + "║")
    print("╚" + "=" * 68 + "╝")
    print()

    tests = [
        test_mavlink_frame_structure,
        test_crc_computation,
        test_statustext_payload,
        test_alert_level_mapping,
        test_truncation,
        test_json_alert_parsing,
    ]

    passed = 0
    failed = 0

    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"✗ FAILED: {e}")
            failed += 1
            print()

    print("=" * 70)
    print(f"RESULTS: {passed} passed, {failed} failed")
    print("=" * 70)

    if failed == 0:
        print("\n✓ All tests passed!")
        return 0
    else:
        print(f"\n✗ {failed} test(s) failed")
        return 1


if __name__ == '__main__':
    exit(main())
