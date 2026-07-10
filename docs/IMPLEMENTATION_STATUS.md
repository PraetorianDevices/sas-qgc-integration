# SAS-QGC Integration: Implementation Status Report

**Last Updated:** 2026-07-10
**Overall Status:** Phase 0 (MAVLink protocol correctness) is complete. All three bridges are verified against pymavlink, mission signing is wired into the live upload path, secure_launch.py is fixed, and the entire mavlink-bridge test suite now imports and exercises real production code. Phase 2 (Fleet Manager / Collision / Emergency Wipe bridges) and Phase 3 (QGC plugin) remain not started.

---

## Why This Doc Was Rewritten (history)

An earlier version of this document reported "50% complete," "production-ready," and "zero test redundancy" based on ~130 passing tests. A line-by-line review — cross-checked byte-for-byte against `pymavlink` as ground truth — found the MAVLink v2 frame header was structurally wrong in every bridge (a 7-byte header instead of the real 10-byte one), that most of the ~130 "passing" tests never imported the modules they claimed to test, that mission signing existed but was never wired in, and that `secure_launch.py` set a nonexistent environment variable. All of that has since been fixed; this document now reflects the corrected state.

---

## Part 1: MAVLink Wire Protocol — Bridge-by-Bridge

| Bridge | Frame Header | Payload Correctness | Verified vs pymavlink | Status |
|--------|--------------|---------------------|------------------------|--------|
| `mavlink_v2.py` (shared codec) | ✅ Correct 10-byte header, 3-byte msg_id | ✅ HEARTBEAT, SYS_STATUS, ATTITUDE, GLOBAL_POSITION_INT, BATTERY_STATUS, STATUSTEXT, MISSION_ITEM_INT, MISSION_ACK, MISSION_CURRENT, MISSION_COUNT, MISSION_REQUEST_INT, MISSION_ITEM_REACHED | ✅ Byte-for-byte match, all 12 message types, in a permanent pytest suite (`test_mavlink_v2.py`) | ✅ **DONE** |
| `gps_spoof_mavlink_bridge.py` | ✅ Migrated to `mavlink_v2` | ✅ STATUSTEXT | ✅ Via shared codec | ✅ **DONE** |
| `telemetry_mavlink_bridge.py` | ✅ Migrated to `mavlink_v2` | ✅ SYS_STATUS (32-bit sensor bitmasks, correct field order), BATTERY_STATUS (real per-cell voltages, no longer always 0) | ✅ Via shared codec + real-bridge functional tests | ✅ **DONE** |
| `mission_control_bridge.py` | ✅ Migrated to `mavlink_v2` | ✅ Switched to `MISSION_ITEM_INT`; MISSION_ACK/MISSION_CURRENT/MISSION_COUNT rewritten to the real spec; full upload handshake implemented (inbound `MISSION_COUNT` → sequential `MISSION_REQUEST_INT` pulls → single `MISSION_ACK` after the last item); `_mission_upload_pub.publish()` is now actually called | ✅ Via shared codec + real UDP-socket integration tests | ✅ **DONE** |

### Bugs found and fixed during the telemetry/mission-control migration (beyond the frame header itself)

- **`telemetry_mavlink_bridge.py`:** `time_boot_ms=int(time.time()*1000)` overflowed MAVLink's `uint32` field (Unix-epoch ms is ~1.7 trillion; the field maxes at ~4.3 billion) and would have crashed `struct.pack` on every telemetry publish once a GPS fix was present. Pre-existing, not introduced this pass. Fixed by tracking a monotonic boot-time reference instead.
- **`telemetry_mavlink_bridge.py`:** `_get_battery_voltage()` divides the cell-voltage sum by 1000 assuming millivolt input, but PX4's `voltage_cell_v` is in volts (the same file's `BATTERY_STATUS` conversion correctly treats it as volts elsewhere). Makes `SYS_STATUS.voltage_battery` report ~1000x too low. **Found but not fixed** — a data-accuracy issue, not a protocol/crash issue, left as a known gap rather than silently expanding scope.
- **`mission_control_bridge.py`:** outbound replies were hardcoded to `('localhost', 14550)`, ignoring both the actual UDP sender's address and any configured `mavlink_port` — silently breaking the documented multi-drone setup (a different port per drone). Fixed: the bridge now replies to the address of whoever last contacted it, falling back to its own configured `(mavlink_host, mavlink_port)` before any packet has been received.
- **`mission_control_bridge.py`:** `MAVLINK_MSG_ID_MISSION_ITEM_REACHED` was hardcoded as `61`; the real ID is `46`. Fixed by switching to `mavlink_v2`'s verified constants.
- **`mavlink_v2.py` (caught by testing, not shipped):** `parse_mission_ack`/`parse_mission_count`/`parse_mission_request` originally rejected payloads shorter than their un-truncated size, but MAVLink 2's trailing-zero-truncation rule means a `MISSION_ACK(result=ACCEPTED)` or an empty `MISSION_COUNT(0)` legitimately arrives shorter. Fixed to zero-fill, matching the same fix already applied to `parse_mission_item_int`.

---

## Part 2: Test Suite — Now Testing Real Code

Every mavlink-bridge test file now imports and exercises its corresponding production module. None of the "self-referential mock" pattern from the earlier version of this document remains.

| Test File | Tests | Notes |
|-----------|-------|-------|
| `test_mavlink_v2.py` (new) | 22 | Byte-for-byte vs pymavlink for all 12 message types, plus header-structure and truncation-edge-case tests. Skips gracefully if `pymavlink` isn't installed. |
| `test_mavlink_crc.py` (rewritten) | 14 | Real `GPSSpoofMAVLinkBridge`; absorbed the previously-separate `test_mavlink_frame_generation.py`, which was pure duplication and has been deleted. No longer requires `rclpy` to be installed — stubs it. |
| `test_mission_signing.py` (rewritten) | 28 | Real `MissionSigner`/`MissionVerifier` with a genuinely generated RSA-2048 keypair; tampering tests now actually re-verify mutated signed data instead of comparing two dict values to each other. |
| `test_telemetry_conversion.py` (rewritten) | 17 | Real `TelemetryMAVLinkBridge`, realistic PX4-shaped mock telemetry; includes regression tests for the SYS_STATUS field-width bug, the BATTERY_STATUS voltage bug, and the `time_boot_ms` overflow crash. |
| `test_mission_control_bridge.py` (rewritten) | 29 | Real `MissionControlBridge`; full upload/download handshake, GCS-identity learning, reply-address regression test. |
| `test_mission_control_integration.py` (rewritten) | 6 | Real bridge with a **real bound UDP socket and real background receiver thread**, driven from a second real socket — genuine byte-level round-trip, not a fake socket object. |
| `test_gps_spoof_integration.py` (rewritten) | 8 | Real `GPSSpoofMAVLinkBridge` with a real UDP socket on each end. No longer requires `rclpy` to be installed. |

**Full mavlink-bridge suite: 124 tests passed, zero exclusions** (previously, `test_mavlink_crc.py` had to be excluded because it required a real ROS 2 environment; that's no longer true). Full SAS unit suite: 1215 passed, 3 skipped — unaffected by this work.

---

## Part 3: Mission Signing & Security

| Component | Code Correctness | Wired Into Live Path? |
|-----------|-------------------|------------------------|
| `SAS/security/mission_signer.py` | ✅ RSA-2048 + PKCS1v15 + SHA-256 over canonical JSON | N/A (signing happens off-vehicle) |
| `SAS/my_python_package/mission_verifier.py` | ✅ Correct verification logic, tampering detection confirmed with real crypto | ✅ **Wired into `mission_executor_node.load_mission_callback`** |
| `SAS/launch/secure_launch.py` | ✅ Fixed: `ROS_SECURITY_ENABLE=true`, `ROS_SECURITY_STRATEGY=Enforce`, `ROS_SECURITY_KEYSTORE=<path>` (the real SROS2 variables); removed the `FASTDDS_DEFAULT_PROFILES_FILE` reference to a nonexistent file, which was based on a misunderstanding of how SROS2 activates | Documented prerequisite gap: the pre-existing keystore has no enclaves for the 3 mavlink-bridge nodes yet — must be generated via `ros2 security create_enclave` in a real ROS 2 environment before launching those nodes with security enabled |

**How verification is wired in:** `mission_executor_node.load_mission_callback` (the single choke point every mission passes through, regardless of source) constructs a `MissionVerifier` at startup with `strict=False` — a missing key file disables verification gracefully rather than failing node startup. A mission **with** a signature is verified and rejected if invalid (tampered, or signed by the wrong key). A mission **without** a signature is still accepted, since MAVLink's wire messages have no field to carry a signature blob — this is the expected case for anything uploaded via `mission_control_bridge`. Verified end-to-end with a real generated keypair: signed+untampered → accepted, signed+tampered → rejected, unsigned → accepted, no-key-configured → gracefully degrades to accepted.

**An orphaned EC private key** was found at `SAS/mission_signing.pem` (gitignored, predates `mission_signer.py` by months, referenced by no code, wrong algorithm). Confirmed with the user as unrelated; a fresh RSA-2048 keypair is generated at deployment time instead.

Also found while fixing `secure_launch.py`: its `if __name__ == '__main__':` block had `return` at module level — a syntax error that meant the entire file failed to compile/import, which would have broken `ros2 launch SAS secure_launch.py` regardless of the environment-variable fixes. Fixed alongside the env vars.

---

## Part 4: Gesture Safety Gating — ✅ COMPLETE (separate feature, not part of the original QGC integration plan)

While tracing mission-completion signaling for the MAVLink bridges, a real, unrelated safety gap was found and fixed under its own plan: `gesture_bridge_node`'s detected gestures (via `fleet_manager_node`) could immediately override an actively-running QGC-driven mission, with zero gating anywhere in the pipeline.

**Implemented:**
- `fleet_manager_node.py` — fleet-wide mission-state tracking; gestures blocked unless every configured drone is `COMPLETED` or `IDLE`.
- `navigation_control_node.py` — backstop rejecting gesture-sourced commands while a mission is active, without touching legitimate internal mission traffic on the same topic.
- `mission_executor_node.py` — added `land`/`goto` handling to `mission_control_callback` (previously unhandled — meant `halt_stop`/`assemble_rally` broadcast gestures were silent no-ops), gated identically, plus a startup `IDLE` announcement.
- `mission_control_bridge.py` — the `in_progress`/QoS fixes.

**Two additional bugs found and fixed:** a formation-gesture payload key mismatch (`'command'` vs `'type'`) that made formation gestures dead code, and the QoS durability mismatch above.

**Test results:** 81/119/165 tests in `test_fleet_manager.py`/`test_navigation_control_node.py`/`test_mission_executor_node.py` respectively, all importing real production code.

**Known residual gap:** `hand_gesture_node.py` (legacy, not launched by either launch file) bypasses `fleet_manager_node` entirely — out of scope unless revived.

---

## What's Actually Production-Ready Today

- ✅ All three MAVLink bridges (`gps_spoof_mavlink_bridge.py`, `telemetry_mavlink_bridge.py`, `mission_control_bridge.py`) — verified MAVLink-correct against pymavlink, real upload/download handshake, correct reply addressing.
- ✅ Mission signing — cryptographically sound and wired into the live upload path.
- ✅ Gesture safety gating — fully implemented and tested.
- ✅ `secure_launch.py` — correct SROS2 environment variables (still needs enclaves generated for the 3 bridge nodes before it fully covers them).
- ✅ Full mavlink-bridge test suite (124 tests) and SAS unit suite (1215 tests) — all importing and exercising real code.

## What Is NOT Ready

- ⚠️ `_get_battery_voltage()`'s units bug (reports ~1000x too low) — known, not fixed, low severity (data accuracy, not protocol/crash).
- ⏳ DDS-Security enclaves for the 3 mavlink-bridge nodes — not generated (requires `ros2 security` CLI in a real ROS 2 environment).
- ⏳ Fleet Manager / Collision / Emergency Wipe bridges (Phase 2) — not started.
- ⏳ QGC Custom Plugin (Phase 3) — not started, requires C++/Qt/QML.
- ⏳ Live ROS 2/WSL validation — everything above has been verified via unit/integration tests and real-socket simulation in this environment, but not yet run against a live ROS 2 install, real PX4/QGC, or real hardware.

---

## Immediate Next Steps (Priority Order)

1. **Live ROS 2 validation.** Launch the full bridge stack in WSL against a real QGroundControl instance and a PX4 SITL or real vehicle — everything to this point has been verified via unit tests, byte-for-byte pymavlink comparison, and real-socket simulation, but never against the actual external systems it's meant to interoperate with.
2. Generate DDS-Security enclaves for the 3 mavlink-bridge nodes via `ros2 security create_enclave`.
3. Fix `_get_battery_voltage()`'s units bug, if accurate SYS_STATUS battery voltage reporting matters for the deployment.
4. Phase 2: Fleet Manager / Collision / Emergency Wipe bridges, built on the now-corrected `mavlink_v2.py` from day one.
5. Phase 3: QGC Custom Plugin (separate session, C++/Qt/QML).

---

## Known Limitations

1. **Mission Signer:** private key stored unencrypted on disk (file-permission-restricted only).
2. **DDS-Security:** the pre-existing keystore does not cover the mavlink-bridge nodes; needs new enclaves.
3. **QGC Plugin:** requires C++/Qt/QML, not attempted in any session so far.
4. **`_get_battery_voltage()` units bug:** SYS_STATUS battery voltage reports ~1000x low. Not fixed.
5. **No live validation yet:** all verification to date is unit/integration-level (pymavlink byte comparison, real sockets, real crypto) — not yet run against real QGroundControl, PX4, or hardware.
