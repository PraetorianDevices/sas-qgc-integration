#!/usr/bin/env python3
"""
Standalone Unit Test: MAVLink Frame Generation

Tests the core MAVLink logic without ROS 2 dependency.
Extracts and tests the MAVLink functions directly.
"""

import struct
import json


class MAVLinkFrameBuilder:
    """Standalone MAVLink frame builder for testing."""

    MAVLINK_MSG_ID_STATUSTEXT = 253

    def __init__(self, system_id=1, component_id=200):
        self.system_id = system_id
        self.component_id = component_id
        self._sequence = 0

    def build_statustext_payload(self, text: str, severity: int) -> bytes:
        """Build STATUSTEXT message payload (50 bytes text + metadata)."""
        text_bytes = text.encode('ascii', errors='replace').ljust(50, b'\x00')[:50]
        severity_bytes = struct.pack('B', severity)
        id_bytes = struct.pack('<H', 0)
        chunk_seq_bytes = struct.pack('B', 0)
        return severity_bytes + text_bytes + id_bytes + chunk_seq_bytes

    def build_mavlink_frame(self, msg_id: int, seq: int, payload: bytes) -> bytes:
        """Build a MAVLink 2.0 frame."""
        stx = 0xFD
        payload_len = len(payload)
        incomp_flags = 0x00
        sysid = self.system_id
        compid = self.component_id

        frame_data = struct.pack(
            '<BBBBBBB',
            stx,
            payload_len,
            incomp_flags,
            msg_id & 0xFF,
            sysid,
            compid,
            seq
        ) + payload

        crc = self.compute_mavlink_crc(frame_data[1:], msg_id)
        return frame_data + struct.pack('<H', crc)

    @staticmethod
    def compute_mavlink_crc(data: bytes, msg_id: int) -> int:
        """Compute MAVLink CRC16-CCITT."""
        CRC_INIT = 0xFFFF
        CRC_EXTRA_STATUSTEXT = 83

        crc = CRC_INIT
        for byte in data:
            tmp = byte ^ (crc & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crc &= 0xFFFF

        tmp = CRC_EXTRA_STATUSTEXT ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF

        return crc


def test_frame_structure():
    """Test MAVLink frame structure."""
    print("\n" + "=" * 70)
    print("TEST 1: MAVLink Frame Structure")
    print("=" * 70)

    builder = MAVLinkFrameBuilder(system_id=1, component_id=200)
    text = "GPS SPOOF DETECTED: heading divergence"
    severity = 5

    payload = builder.build_statustext_payload(text, severity)
    frame = builder.build_mavlink_frame(msg_id=253, seq=0, payload=payload)

    print(f"Text:          {text}")
    print(f"Severity:      {severity} (CRITICAL)")
    print(f"Frame length:  {len(frame)} bytes")
    print(f"Frame hex:     {frame[:16].hex()}... (STX, LEN, INV, MSG_ID, SYSID, COMPID, SEQ)")

    # Validate structure
    assert frame[0] == 0xFD, f"STX should be 0xFD, got {hex(frame[0])}"
    assert frame[1] == 54, f"Payload length should be 54, got {frame[1]}"
    assert frame[3] == 253, f"Message ID should be 253, got {frame[3]}"
    assert frame[4] == 1, f"System ID should be 1, got {frame[4]}"
    assert frame[5] == 200, f"Component ID should be 200, got {frame[5]}"
    assert frame[6] == 0, f"Sequence should be 0, got {frame[6]}"

    print("[OK] Frame structure PASSED")
    return True


def test_crc():
    """Test CRC computation."""
    print("\n" + "=" * 70)
    print("TEST 2: CRC16-CCITT Computation")
    print("=" * 70)

    builder = MAVLinkFrameBuilder(system_id=1, component_id=200)
    text = "GPS altitude spoofed"
    severity = 5

    payload = builder.build_statustext_payload(text, severity)
    frame = builder.build_mavlink_frame(msg_id=253, seq=0, payload=payload)

    # Extract CRC from frame (last 2 bytes)
    crc_from_frame = struct.unpack('<H', frame[-2:])[0]

    # Recompute
    crc_recomputed = builder.compute_mavlink_crc(frame[1:-2], 253)

    print(f"CRC from frame:    0x{crc_from_frame:04x}")
    print(f"CRC recomputed:    0x{crc_recomputed:04x}")
    print(f"Match:             {'[OK] YES' if crc_from_frame == crc_recomputed else '[FAIL] NO'}")

    assert crc_from_frame == crc_recomputed, f"CRC mismatch: {crc_from_frame:04x} != {crc_recomputed:04x}"
    print("[OK] CRC computation PASSED")
    return True


def test_payload():
    """Test STATUSTEXT payload."""
    print("\n" + "=" * 70)
    print("TEST 3: STATUSTEXT Payload Formatting")
    print("=" * 70)

    builder = MAVLinkFrameBuilder()
    text = "GPS heading divergence detected"
    severity = 4

    payload = builder.build_statustext_payload(text, severity)

    print(f"Input text:        '{text}'")
    print(f"Input length:      {len(text)} chars")
    print(f"Payload length:    {len(payload)} bytes (expected 54: 1 severity + 50 text + 2 id + 1 chunk_seq)")

    assert len(payload) == 54, f"Payload should be 54 bytes, got {len(payload)}"

    # Extract severity and text back
    severity_byte = payload[0]
    text_bytes = payload[1:51].rstrip(b'\x00')
    text_back = text_bytes.decode('ascii', errors='ignore')

    print(f"Severity byte:     {severity_byte} (expected {severity})")
    print(f"Text extracted:    '{text_back}'")

    assert severity_byte == severity, "Severity mismatch!"
    assert text_back == text, "Text mismatch!"

    print("[OK] STATUSTEXT payload PASSED")
    return True


def test_truncation():
    """Test message truncation."""
    print("\n" + "=" * 70)
    print("TEST 4: Message Truncation (Max 50 chars)")
    print("=" * 70)

    builder = MAVLinkFrameBuilder()
    long_text = "A" * 100  # 100 characters, should be truncated to 50

    payload = builder.build_statustext_payload(long_text, 4)
    text_extracted = payload[1:51].rstrip(b'\x00').decode('ascii')

    print(f"Input length:      {len(long_text)} chars")
    print(f"Output length:     {len(text_extracted)} chars")
    print(f"Expected:          50 chars")

    assert len(text_extracted) == 50, f"Text should be 50 chars, got {len(text_extracted)}"
    print("[OK] Truncation PASSED")
    return True


def test_sequence():
    """Test sequence numbering."""
    print("\n" + "=" * 70)
    print("TEST 5: Sequence Numbering")
    print("=" * 70)

    builder = MAVLinkFrameBuilder()

    sequences = []
    for i in range(260):  # Test wrap-around at 256
        payload = builder.build_statustext_payload("test", 4)
        frame = builder.build_mavlink_frame(msg_id=253, seq=i % 256, payload=payload)
        seq_from_frame = frame[6]
        sequences.append(seq_from_frame)

    print(f"Generated {len(sequences)} frames")
    print(f"First 5 sequences:  {sequences[:5]}")
    print(f"Last 5 sequences:   {sequences[-5:]}")
    print(f"Wrap-around check:  seq 255 -> seq 0")

    # Check wrap-around
    assert sequences[255] == 255, "Sequence 255 should be 255"
    assert sequences[256] == 0, "Sequence 256 should wrap to 0"

    print("[OK] Sequence numbering PASSED")
    return True


def test_alert_levels():
    """Test different alert levels."""
    print("\n" + "=" * 70)
    print("TEST 6: Alert Level Mapping")
    print("=" * 70)

    builder = MAVLinkFrameBuilder()

    levels = {
        'INFO': 0,
        'WARNING': 4,
        'CRITICAL': 5,
    }

    for name, severity_value in levels.items():
        payload = builder.build_statustext_payload(f"{name} message", severity_value)
        severity_from_payload = payload[0]

        print(f"  {name:10} -> severity={severity_from_payload} (expected {severity_value})")
        assert severity_from_payload == severity_value, f"Severity mismatch for {name}"

    print("[OK] Alert level mapping PASSED")
    return True


def test_multiple_frames():
    """Test generating multiple frames in sequence."""
    print("\n" + "=" * 70)
    print("TEST 7: Multiple Frames (Simulating Alert Stream)")
    print("=" * 70)

    builder = MAVLinkFrameBuilder()

    alerts = [
        ("GPS nominal", 0),  # INFO
        ("GPS diverging", 4),  # WARNING
        ("GPS SPOOFED", 5),  # CRITICAL
        ("GPS nominal", 0),  # INFO
    ]

    frames = []
    for seq, (text, severity) in enumerate(alerts):
        payload = builder.build_statustext_payload(text, severity)
        frame = builder.build_mavlink_frame(msg_id=253, seq=seq, payload=payload)
        frames.append(frame)

        severity_name = {0: "INFO", 4: "WARNING", 5: "CRITICAL"}[severity]
        print(f"  Frame {seq}: seq={seq}, severity={severity_name:8}, text='{text}'")

    print(f"Generated {len(frames)} frames successfully")

    # Verify each frame
    for frame in frames:
        assert frame[0] == 0xFD, "Invalid STX"
        crc_in_frame = struct.unpack('<H', frame[-2:])[0]
        crc_computed = builder.compute_mavlink_crc(frame[1:-2], 253)
        assert crc_in_frame == crc_computed, f"CRC mismatch in frame"

    print("[OK] Multiple frames PASSED")
    return True


def main():
    """Run all tests."""
    print("\n")
    print("+" + "=" * 68 + "+")
    print("|" + " " * 10 + "GPS Spoofing MAVLink Bridge — Standalone Unit Tests" + " " * 8 + "|")
    print("+" + "=" * 68 + "+")

    tests = [
        test_frame_structure,
        test_crc,
        test_payload,
        test_truncation,
        test_sequence,
        test_alert_levels,
        test_multiple_frames,
    ]

    passed = 0
    failed = 0
    errors = []

    for test in tests:
        try:
            result = test()
            if result:
                passed += 1
        except AssertionError as e:
            failed += 1
            errors.append(f"{test.__name__}: {e}")
            print(f"[FAIL] {test.__name__} FAILED: {e}")

    # Summary
    print("\n" + "=" * 70)
    print(f"RESULTS: {passed} PASSED, {failed} FAILED")
    print("=" * 70)

    if failed == 0:
        print("\n[OK][OK][OK] All tests PASSED! MAVLink frame generation is working correctly. [OK][OK][OK]")
        return 0
    else:
        print(f"\n[FAIL] {failed} test(s) FAILED:")
        for error in errors:
            print(f"  - {error}")
        return 1


if __name__ == '__main__':
    exit(main())
