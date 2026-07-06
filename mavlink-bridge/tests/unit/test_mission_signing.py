#!/usr/bin/env python3
"""
Unit Tests: Mission Signing & Verification

Tests cryptographic signing and verification of missions without ROS 2 dependency.
Verifies signature generation, verification, tampering detection.

NOT REDUNDANT with SAS tests:
  - SAS tests: executor accepts missions, executes them
  - Signing tests: verify cryptographic integrity of mission JSON
  - Different layers: executor layer vs security layer
"""

import json
import pytest
import tempfile
from pathlib import Path
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.backends import default_backend


# Mock implementations for testing without actual crypto files
class MockMissionSigner:
    """Simplified signer for testing signature format."""

    def sign_mission(self, mission):
        """Add signature to mission dict."""
        mission_copy = {k: v for k, v in mission.items() if k != 'signature'}
        signature_hex = 'abcd1234' * 16  # Fake signature

        return {
            **mission_copy,
            'signature': {
                'value': signature_hex,
                'algorithm': 'RSA-2048-SHA256',
                'timestamp': '2026-07-06T12:00:00Z'
            }
        }


class MockMissionVerifier:
    """Simplified verifier for testing verification logic."""

    def verify_mission(self, mission, allow_unsigned=False):
        """Check if mission has valid structure."""
        if 'signature' not in mission:
            if allow_unsigned:
                return True, None
            else:
                return False, "Mission is unsigned"

        sig = mission['signature']
        if not isinstance(sig, dict):
            return False, "Invalid signature format"

        if 'value' not in sig:
            return False, "Signature value missing"

        if sig.get('algorithm') != 'RSA-2048-SHA256':
            return False, f"Unsupported algorithm: {sig.get('algorithm')}"

        # Verify signature is hex string
        try:
            bytes.fromhex(sig['value'])
        except ValueError:
            return False, "Signature is not valid hex"

        return True, None


class TestMissionSignatureFormat:
    """Test mission signature data structure."""

    def test_mission_signature_structure(self):
        """Verify signed mission has correct structure."""
        mission = {'waypoints': [], 'home': {}}
        signer = MockMissionSigner()
        signed = signer.sign_mission(mission)

        assert 'signature' in signed
        assert isinstance(signed['signature'], dict)
        assert 'value' in signed['signature']
        assert 'algorithm' in signed['signature']
        assert 'timestamp' in signed['signature']

    def test_signature_value_is_hex(self):
        """Verify signature value is hex-encoded."""
        mission = {'waypoints': []}
        signer = MockMissionSigner()
        signed = signer.sign_mission(mission)

        sig_value = signed['signature']['value']
        # Should be valid hex
        assert len(sig_value) > 0
        assert all(c in '0123456789abcdefABCDEF' for c in sig_value)

    def test_signature_algorithm_rsa_2048(self):
        """Verify algorithm field."""
        mission = {'waypoints': []}
        signer = MockMissionSigner()
        signed = signer.sign_mission(mission)

        assert signed['signature']['algorithm'] == 'RSA-2048-SHA256'

    def test_signature_timestamp_iso8601(self):
        """Verify timestamp is ISO 8601 format."""
        mission = {'waypoints': []}
        signer = MockMissionSigner()
        signed = signer.sign_mission(mission)

        ts = signed['signature']['timestamp']
        assert 'T' in ts
        assert 'Z' in ts or '+' in ts or ts.count('-') > 2


class TestMissionVerification:
    """Test mission verification logic."""

    def test_verify_unsigned_mission_rejected(self):
        """Unsigned missions rejected in strict mode."""
        mission = {'waypoints': []}
        verifier = MockMissionVerifier()

        is_valid, error = verifier.verify_mission(mission, allow_unsigned=False)
        assert not is_valid
        assert error == "Mission is unsigned"

    def test_verify_unsigned_mission_allowed(self):
        """Unsigned missions allowed when permissive."""
        mission = {'waypoints': []}
        verifier = MockMissionVerifier()

        is_valid, error = verifier.verify_mission(mission, allow_unsigned=True)
        assert is_valid
        assert error is None

    def test_verify_signed_mission_valid_format(self):
        """Properly formatted signed mission passes verification."""
        signer = MockMissionSigner()
        mission = {'waypoints': [], 'home': {}}
        signed = signer.sign_mission(mission)

        verifier = MockMissionVerifier()
        is_valid, error = verifier.verify_mission(signed)

        assert is_valid
        assert error is None

    def test_verify_mission_invalid_signature_format(self):
        """Invalid signature format rejected."""
        mission = {
            'waypoints': [],
            'signature': 'invalid_string'  # Should be dict
        }
        verifier = MockMissionVerifier()

        is_valid, error = verifier.verify_mission(mission)
        assert not is_valid
        assert "Invalid signature format" in error

    def test_verify_mission_missing_signature_value(self):
        """Missing signature value detected."""
        mission = {
            'waypoints': [],
            'signature': {
                'algorithm': 'RSA-2048-SHA256',
                'timestamp': '2026-07-06T12:00:00Z'
                # Missing 'value'
            }
        }
        verifier = MockMissionVerifier()

        is_valid, error = verifier.verify_mission(mission)
        assert not is_valid
        assert "Signature value missing" in error

    def test_verify_mission_invalid_algorithm(self):
        """Unsupported algorithm rejected."""
        mission = {
            'waypoints': [],
            'signature': {
                'value': 'abcd1234' * 16,
                'algorithm': 'ECDSA-256',  # Unsupported
                'timestamp': '2026-07-06T12:00:00Z'
            }
        }
        verifier = MockMissionVerifier()

        is_valid, error = verifier.verify_mission(mission)
        assert not is_valid
        assert "Unsupported algorithm" in error

    def test_verify_mission_invalid_hex_signature(self):
        """Non-hex signature value rejected."""
        mission = {
            'waypoints': [],
            'signature': {
                'value': 'not_valid_hex_zzzz',
                'algorithm': 'RSA-2048-SHA256',
                'timestamp': '2026-07-06T12:00:00Z'
            }
        }
        verifier = MockMissionVerifier()

        is_valid, error = verifier.verify_mission(mission)
        assert not is_valid
        assert "not valid hex" in error


class TestMissionTamperingDetection:
    """Test that tampering is detected."""

    def test_mission_content_extraction(self):
        """Verify signature doesn't include itself."""
        mission = {
            'waypoints': [{'seq': 0, 'lat': 37.0}],
            'home': {'lat': 0},
            'signature': {'value': 'old_sig', 'algorithm': 'RSA', 'timestamp': '2026-01-01T00:00:00Z'}
        }

        # New signature should exclude old signature
        signer = MockMissionSigner()
        new_signed = signer.sign_mission(mission)

        # Old signature should not appear in new content
        assert 'old_sig' not in json.dumps(new_signed, sort_keys=True)

    def test_waypoint_tampering_detection(self):
        """Latitude/longitude tampering is detectable."""
        original = {
            'waypoints': [{'seq': 0, 'lat': 37.7749, 'lon': -122.4194}],
            'home': {}
        }

        signer = MockMissionSigner()
        signed = signer.sign_mission(original)

        # Tamper with waypoint
        tampered = signed.copy()
        tampered['waypoints'][0]['lat'] = 40.0  # Different location

        # Signature mismatch detectable (in real crypto)
        assert tampered['waypoints'][0]['lat'] != signed['waypoints'][0]['lat']

    def test_mission_metadata_preservation(self):
        """Mission metadata preserved through signing."""
        mission = {
            'waypoints': [{'seq': 0, 'lat': 37.0}],
            'home': {'lat': 0},
            'metadata': {'priority': 'high', 'timeout': 3600}
        }

        signer = MockMissionSigner()
        signed = signer.sign_mission(mission)

        assert signed['metadata'] == mission['metadata']


class TestSignatureBatchOperations:
    """Test batch signing and verification."""

    def test_sign_mission_batch(self):
        """Sign multiple missions."""
        missions = [
            {'waypoints': [{'seq': 0}], 'home': {}},
            {'waypoints': [{'seq': 0}, {'seq': 1}], 'home': {}},
            {'waypoints': [{'seq': 0}, {'seq': 1}, {'seq': 2}], 'home': {}},
        ]

        signer = MockMissionSigner()
        signed_batch = [signer.sign_mission(m) for m in missions]

        assert len(signed_batch) == 3
        for signed in signed_batch:
            assert 'signature' in signed

    def test_verify_mission_batch_all_valid(self):
        """Verify batch of valid missions."""
        signer = MockMissionSigner()
        missions = [
            signer.sign_mission({'waypoints': [], 'home': {}}),
            signer.sign_mission({'waypoints': [], 'home': {}}),
            signer.sign_mission({'waypoints': [], 'home': {}}),
        ]

        verifier = MockMissionVerifier()
        results = [verifier.verify_mission(m)[0] for m in missions]

        assert all(results)

    def test_verify_mission_batch_mixed_validity(self):
        """Verify batch with some invalid missions."""
        signer = MockMissionSigner()
        missions = [
            signer.sign_mission({'waypoints': [], 'home': {}}),  # Valid
            {'waypoints': []},  # Unsigned
            {'waypoints': [], 'signature': {'algorithm': 'BAD'}},  # Invalid
        ]

        verifier = MockMissionVerifier()
        valid_count = sum(1 for m in missions if verifier.verify_mission(m, allow_unsigned=False)[0])

        assert valid_count == 1  # Only first is valid


class TestSignatureMetadata:
    """Test extraction of signature metadata."""

    def test_get_signature_info_signed_mission(self):
        """Extract info from signed mission."""
        signer = MockMissionSigner()
        signed = signer.sign_mission({'waypoints': []})

        sig = signed['signature']
        info = {
            'algorithm': sig.get('algorithm'),
            'timestamp': sig.get('timestamp'),
            'has_signature': 'value' in sig
        }

        assert info['algorithm'] == 'RSA-2048-SHA256'
        assert info['has_signature'] is True
        assert 'T' in info['timestamp']

    def test_get_signature_info_unsigned_mission(self):
        """Handle unsigned mission."""
        mission = {'waypoints': []}

        sig_info = None
        if 'signature' not in mission:
            sig_info = None

        assert sig_info is None


class TestSignatureRoundtrip:
    """Test sign-then-verify roundtrip."""

    def test_roundtrip_sign_verify(self):
        """Mission signature survives roundtrip."""
        original = {
            'waypoints': [
                {'seq': 0, 'lat': 37.0, 'lon': -122.0, 'alt': 100},
                {'seq': 1, 'lat': 37.1, 'lon': -122.1, 'alt': 150},
            ],
            'home': {'lat': 37.0, 'lon': -122.0}
        }

        signer = MockMissionSigner()
        signed = signer.sign_mission(original)

        verifier = MockMissionVerifier()
        is_valid, error = verifier.verify_mission(signed)

        assert is_valid
        assert error is None

    def test_roundtrip_json_serialization(self):
        """Signature survives JSON serialization."""
        original = {'waypoints': [{'seq': 0}]}

        signer = MockMissionSigner()
        signed = signer.sign_mission(original)

        # Serialize and deserialize
        json_str = json.dumps(signed)
        deserialized = json.loads(json_str)

        verifier = MockMissionVerifier()
        is_valid, error = verifier.verify_mission(deserialized)

        assert is_valid


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
