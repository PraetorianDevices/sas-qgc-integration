#!/usr/bin/env python3
"""
Unit Tests: Mission Signing & Verification

Tests the real SAS/security/mission_signer.py and
SAS/my_python_package/mission_verifier.py -- RSA-2048 + PKCS1v15 + SHA-256
over canonical JSON. A previous version of this file defined
MockMissionSigner/MockMissionVerifier classes that never touched the real
modules; its "tampering detection" test in particular never even called a
verify function on the tampered mission, just compared two dict values to
each other. None of that would have caught the real signer/verifier being
broken. This version generates a real keypair and exercises the real classes.

NOT REDUNDANT with SAS tests:
  - SAS tests: executor accepts missions, executes them
  - Signing tests: verify cryptographic integrity of mission JSON
  - Different layers: executor layer vs security layer
"""

import json
import sys
from pathlib import Path

import pytest

# security/ and my_python_package/ live under SAS/, a sibling of mavlink-bridge/.
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'SAS'))

from security.mission_signer import MissionSigner
from my_python_package.mission_verifier import MissionVerifier, MissionVerificationError


@pytest.fixture(scope='module')
def keypair(tmp_path_factory):
    """A real RSA-2048 keypair, generated once and shared across this file's
    tests (key generation is the slow part; reuse is safe since tests don't
    mutate key files)."""
    keydir = tmp_path_factory.mktemp('mission_signing_keys')
    private_path = keydir / 'private.pem'
    public_path = keydir / 'public.pem'
    MissionSigner.generate_keypair(str(private_path), str(public_path))
    return str(private_path), str(public_path)


@pytest.fixture
def signer(keypair):
    private_path, _ = keypair
    return MissionSigner(private_path)


@pytest.fixture
def verifier(keypair):
    _, public_path = keypair
    return MissionVerifier(public_path, strict=True)


class TestKeypairGeneration:
    """Sanity-check the generated keypair itself."""

    def test_generate_keypair_creates_both_files(self, keypair):
        private_path, public_path = keypair
        assert Path(private_path).exists()
        assert Path(public_path).exists()

    def test_private_key_is_pem_encoded(self, keypair):
        private_path, _ = keypair
        content = Path(private_path).read_text()
        assert 'PRIVATE KEY' in content

    def test_public_key_is_pem_encoded(self, keypair):
        _, public_path = keypair
        content = Path(public_path).read_text()
        assert 'PUBLIC KEY' in content


class TestMissionSignatureFormat:
    """Test the signed mission's data structure, using the real signer."""

    def test_mission_signature_structure(self, signer):
        mission = {'waypoints': [], 'home': {}}
        signed = signer.sign_mission(mission)

        assert 'signature' in signed
        assert isinstance(signed['signature'], dict)
        assert 'value' in signed['signature']
        assert 'algorithm' in signed['signature']
        assert 'timestamp' in signed['signature']

    def test_signature_value_is_hex(self, signer):
        signed = signer.sign_mission({'waypoints': []})

        sig_value = signed['signature']['value']
        assert len(sig_value) > 0
        assert all(c in '0123456789abcdefABCDEF' for c in sig_value)

    def test_signature_length_matches_rsa_2048(self, signer):
        # RSA-2048 signatures are exactly 256 bytes = 512 hex characters.
        signed = signer.sign_mission({'waypoints': []})
        assert len(signed['signature']['value']) == 512

    def test_signature_algorithm_rsa_2048(self, signer):
        signed = signer.sign_mission({'waypoints': []})
        assert signed['signature']['algorithm'] == 'RSA-2048-SHA256'

    def test_signature_timestamp_iso8601(self, signer):
        signed = signer.sign_mission({'waypoints': []})
        ts = signed['signature']['timestamp']
        assert 'T' in ts
        assert ts.endswith('Z')

    def test_two_signatures_of_same_mission_differ_in_timestamp_only(self, signer):
        # Deterministic PKCS1v15 signing means the signature bytes themselves
        # are identical across calls for identical content+key -- only the
        # timestamp should differ.
        mission = {'waypoints': [{'seq': 0}]}
        first = signer.sign_mission(mission)
        second = signer.sign_mission(mission)
        assert first['signature']['value'] == second['signature']['value']


class TestMissionVerificationReal:
    """Test verification against the real signer/verifier pair -- this is
    where the previous mock version provided zero actual coverage, since it
    never checked a signature against real mission content."""

    def test_verify_unsigned_mission_rejected_strict(self, verifier):
        is_valid, error = verifier.verify_mission({'waypoints': []}, allow_unsigned=False)
        assert not is_valid
        assert 'unsigned' in error.lower()

    def test_verify_unsigned_mission_allowed_permissive(self, verifier):
        is_valid, error = verifier.verify_mission({'waypoints': []}, allow_unsigned=True)
        assert is_valid
        assert error is None

    def test_verify_signed_untampered_mission_passes(self, signer, verifier):
        signed = signer.sign_mission({'waypoints': [{'seq': 0, 'lat': 37.0}], 'home': {}})
        is_valid, error = verifier.verify_mission(signed)
        assert is_valid
        assert error is None

    def test_verify_mission_invalid_signature_format(self, verifier):
        mission = {'waypoints': [], 'signature': 'not_a_dict'}
        is_valid, error = verifier.verify_mission(mission)
        assert not is_valid
        assert 'format' in error.lower()

    def test_verify_mission_missing_signature_value(self, verifier):
        mission = {
            'waypoints': [],
            'signature': {'algorithm': 'RSA-2048-SHA256', 'timestamp': '2026-07-06T12:00:00Z'},
        }
        is_valid, error = verifier.verify_mission(mission)
        assert not is_valid
        assert 'missing' in error.lower()

    def test_verify_mission_unsupported_algorithm_rejected(self, signer, verifier):
        signed = signer.sign_mission({'waypoints': []})
        signed['signature']['algorithm'] = 'ECDSA-256'
        is_valid, error = verifier.verify_mission(signed)
        assert not is_valid
        assert 'algorithm' in error.lower()

    def test_verify_mission_invalid_hex_signature_rejected(self, verifier):
        mission = {
            'waypoints': [],
            'signature': {'value': 'not_valid_hex_zzzz', 'algorithm': 'RSA-2048-SHA256',
                          'timestamp': '2026-07-06T12:00:00Z'},
        }
        is_valid, error = verifier.verify_mission(mission)
        assert not is_valid

    def test_verify_mission_wrong_public_key_rejected(self, signer, tmp_path):
        # Sign with one key, verify with a DIFFERENT key's public half --
        # must fail. This is the case a mock signer/verifier can't represent
        # at all, since there's no real asymmetric relationship to violate.
        signed = signer.sign_mission({'waypoints': [{'seq': 0}]})

        other_private = tmp_path / 'other_private.pem'
        other_public = tmp_path / 'other_public.pem'
        MissionSigner.generate_keypair(str(other_private), str(other_public))
        wrong_verifier = MissionVerifier(str(other_public), strict=True)

        is_valid, error = wrong_verifier.verify_mission(signed)
        assert not is_valid
        assert 'tamper' in error.lower() or 'fail' in error.lower()


class TestTamperingDetectionReal:
    """Real cryptographic tampering detection -- mutate signed mission
    content, then actually re-verify it (the previous mock version's
    equivalent test never called verify_mission on the tampered data at
    all)."""

    def test_tampered_waypoint_latitude_detected(self, signer, verifier):
        original = {'waypoints': [{'seq': 0, 'lat': 37.7749, 'lon': -122.4194}], 'home': {}}
        signed = signer.sign_mission(original)

        tampered = json.loads(json.dumps(signed))  # deep copy
        tampered['waypoints'][0]['lat'] = 40.0

        is_valid, error = verifier.verify_mission(tampered)
        assert not is_valid
        assert 'tamper' in error.lower()

    def test_tampered_altitude_detected(self, signer, verifier):
        original = {'waypoints': [{'seq': 0, 'alt': 100.0}], 'home': {}}
        signed = signer.sign_mission(original)

        tampered = json.loads(json.dumps(signed))
        tampered['waypoints'][0]['alt'] = 9999.0

        is_valid, _ = verifier.verify_mission(tampered)
        assert not is_valid

    def test_added_waypoint_detected(self, signer, verifier):
        # Appending an entire extra waypoint after signing must also fail --
        # not just field-level mutation.
        original = {'waypoints': [{'seq': 0, 'lat': 37.0}], 'home': {}}
        signed = signer.sign_mission(original)

        tampered = json.loads(json.dumps(signed))
        tampered['waypoints'].append({'seq': 1, 'lat': 99.0})

        is_valid, _ = verifier.verify_mission(tampered)
        assert not is_valid

    def test_removed_waypoint_detected(self, signer, verifier):
        original = {'waypoints': [{'seq': 0}, {'seq': 1}], 'home': {}}
        signed = signer.sign_mission(original)

        tampered = json.loads(json.dumps(signed))
        tampered['waypoints'].pop()

        is_valid, _ = verifier.verify_mission(tampered)
        assert not is_valid

    def test_old_signature_excluded_from_new_signature_content(self, signer):
        # Re-signing a mission that already has (a different) signature must
        # exclude the old signature from what gets hashed, or every re-sign
        # would incorporate stale signature bytes into the new one.
        mission = {
            'waypoints': [{'seq': 0}],
            'signature': {'value': 'deadbeef' * 8, 'algorithm': 'RSA-2048-SHA256',
                          'timestamp': '2020-01-01T00:00:00Z'},
        }
        resigned = signer.sign_mission(mission)
        assert 'deadbeef' not in resigned['signature']['value']

    def test_mission_metadata_preserved_through_signing(self, signer, verifier):
        mission = {
            'waypoints': [{'seq': 0}],
            'home': {'lat': 0},
            'metadata': {'priority': 'high', 'timeout': 3600},
        }
        signed = signer.sign_mission(mission)
        assert signed['metadata'] == mission['metadata']
        is_valid, _ = verifier.verify_mission(signed)
        assert is_valid


class TestBatchOperationsReal:

    def test_sign_mission_batch(self, signer):
        missions = [{'waypoints': [{'seq': i}]} for i in range(3)]
        signed_batch = signer.sign_mission_batch(missions)

        assert len(signed_batch) == 3
        for signed in signed_batch:
            assert 'signature' in signed

    def test_verify_mission_batch_all_valid(self, signer, verifier):
        signed_batch = signer.sign_mission_batch(
            [{'waypoints': []}, {'waypoints': [{'seq': 0}]}, {'waypoints': [{'seq': 0}, {'seq': 1}]}])

        valid, failed = verifier.verify_mission_batch(signed_batch)
        assert len(valid) == 3
        assert failed == 0

    def test_verify_mission_batch_mixed_validity(self, signer, verifier):
        good = signer.sign_mission({'waypoints': []})
        tampered = json.loads(json.dumps(good))
        tampered['waypoints'] = [{'seq': 99}]
        unsigned = {'waypoints': []}

        valid, failed = verifier.verify_mission_batch(
            [good, tampered, unsigned], allow_unsigned=False)
        assert len(valid) == 1
        assert failed == 2


class TestSignatureRoundtrip:
    """Sign -> JSON serialize -> deserialize -> verify, matching how a
    mission actually travels (e.g. through mission_executor_node.load_mission_callback)."""

    def test_roundtrip_survives_json_serialization(self, signer, verifier):
        original = {
            'waypoints': [
                {'seq': 0, 'lat': 37.0, 'lon': -122.0, 'alt': 100},
                {'seq': 1, 'lat': 37.1, 'lon': -122.1, 'alt': 150},
            ],
            'home': {'lat': 37.0, 'lon': -122.0},
        }
        signed = signer.sign_mission(original)

        json_str = json.dumps(signed)
        deserialized = json.loads(json_str)

        is_valid, error = verifier.verify_mission(deserialized)
        assert is_valid
        assert error is None

    def test_verify_mission_strict_helper_raises_on_failure(self, signer, keypair):
        from my_python_package.mission_verifier import verify_mission_strict
        _, public_path = keypair

        signed = signer.sign_mission({'waypoints': [{'seq': 0}]})
        tampered = json.loads(json.dumps(signed))
        tampered['waypoints'][0]['seq'] = 999

        with pytest.raises(MissionVerificationError):
            verify_mission_strict(tampered, public_key_path=public_path)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
