# SAS-QGC Integration Implementation Roadmap

## Executive Summary

Full implementation of `SAS_QGC_Integration_Plan.md`, excluding `gesture_bridge_node`'s QGC connection (per original scope). A subsequent protocol audit found the MAVLink layer had a showstopper wire-format bug across all three bridges, so a corrective phase (Phase 0) was inserted ahead of the original Phase 2/3 work. That phase is now complete. A separate, unrelated safety feature (gesture-gating) was also completed under its own plan — see Phase 1.5.

**Phases:**
0. **Phase 0 (✅ Complete):** MAVLink Wire Protocol Correctness — frame-format bug fixed in all 3 bridges, mission signing wired in, secure launch fixed, entire test suite now imports real modules
1. **Phase 1 (✅ Complete):** Security & Mission Signing
2. **Phase 1.5 (✅ Complete):** Gesture Safety Gating — not part of the original plan; added after a real conflict risk was found
3. **Phase 2 (Not started):** Remaining ROS 2 Bridges (Fleet Manager, Collision, Emergency Wipe)
4. **Phase 3 (Not started):** QGC Custom Plugin
5. **Phase 4 (✅ Complete for Phases 0/1/1.5; not started for Phase 2):** Comprehensive Testing

---

## Phase 0: MAVLink Wire Protocol Correctness — ✅ COMPLETE

A line-by-line review, validated byte-for-byte against `pymavlink` as ground truth, found that all three bridges built MAVLink v2 frames with a 7-byte header instead of the real 10-byte header, with the message ID truncated from 24 bits to 8. **None of these bridges could have interoperated with real QGroundControl.** The review also found that the ~130 tests reporting 100% pass rate mostly never imported the modules they claimed to test.

### Completed

1. **`mavlink-bridge/mavlink_v2.py`** (new shared codec) — correct 10-byte header, 3-byte message ID, MAVLink-2 trailing-zero truncation; CRC_EXTRA table and payload layouts for all 12 message types the bridges use, verified byte-for-byte against `pymavlink.dialects.v20.common` (caught two bugs in the first draft this way: a missing truncation step, and a struct-format slicing error).
2. **`gps_spoof_mavlink_bridge.py`** migrated to `mavlink_v2`.
3. **`telemetry_mavlink_bridge.py`** migrated to `mavlink_v2` — `SYS_STATUS` and `BATTERY_STATUS` rewritten to the verified field layouts (fixing the 32-bit-sensor-bitmasks-packed-as-16-bit bug and the always-zero battery voltage bug). Also fixed a `time_boot_ms` `uint32` overflow that would have crashed `struct.pack` on every publish once a GPS fix was present.
4. **`mission_control_bridge.py`** migrated to `mavlink_v2` — switched from a mismatched ID-39-with-scaled-ints hybrid to proper `MISSION_ITEM_INT`; rewrote MISSION_ACK/MISSION_CURRENT/MISSION_COUNT to the real field layouts; implemented the actual upload handshake (inbound `MISSION_COUNT` → sequential `MISSION_REQUEST_INT` pulls → single `MISSION_ACK` after the last item); wired up `_mission_upload_pub.publish()`, which previously existed but was never called. Also fixed a hardcoded reply address (`('localhost', 14550)` regardless of sender or configured port — silently broke the documented multi-drone setup) and a wrong `MISSION_ITEM_REACHED` message ID (61 instead of 46).
5. **Mission signing wired into the live path** — `MissionVerifier` constructed in `mission_executor_node.__init__` (`strict=False`, graceful degradation if no key configured); `load_mission_callback` verifies any mission carrying a signature and rejects it if invalid, while still accepting unsigned missions (required for MAVLink-uploaded missions, which have no signature transport). Verified end-to-end with a real generated RSA-2048 keypair. An unrelated, unused, wrong-algorithm EC key found at `SAS/mission_signing.pem` was confirmed orphaned and left alone.
6. **`secure_launch.py` fixed** — corrected to the real SROS2 environment variables (`ROS_SECURITY_ENABLE`, `ROS_SECURITY_STRATEGY`, `ROS_SECURITY_KEYSTORE`); removed the incorrect `FASTDDS_DEFAULT_PROFILES_FILE` reference to a nonexistent file. Also fixed a `return` outside a function that made the entire file fail to compile/import, independent of the env var fixes.
7. **Test suite rewritten to test real code** — every mavlink-bridge test file now imports and exercises its corresponding production module: `test_mavlink_v2.py` (new, 22 tests, byte-for-byte vs pymavlink), `test_mavlink_crc.py` (rewritten, 14 tests, absorbed and deleted the pure-duplicate `test_mavlink_frame_generation.py`), `test_mission_signing.py` (rewritten, 28 tests, real RSA keypair), `test_telemetry_conversion.py` (rewritten, 17 tests), `test_mission_control_bridge.py` (rewritten, 29 tests), `test_mission_control_integration.py` (rewritten, 6 tests, real bound UDP socket + real background thread), `test_gps_spoof_integration.py` (rewritten, 8 tests, real UDP socket). `conftest.py`'s `MockMAVLinkBuilder` now delegates to the real `mavlink_v2` codec instead of reimplementing it.

**Result:** full mavlink-bridge suite is 124 tests passed, zero exclusions (previously `test_mavlink_crc.py`/`test_gps_spoof_integration.py` required a real ROS 2 environment to even collect; both are now stubbed and portable).

---

## Phase 1: Security & Mission Signing — ✅ COMPLETE

| Component | Location | Code Status | Integration Status |
|-----------|----------|--------------|---------------------|
| Mission Signer | `SAS/security/mission_signer.py` | ✅ Correct (RSA-2048, PKCS1v15, SHA-256) | N/A — signing happens off-vehicle |
| Mission Verifier | `SAS/my_python_package/mission_verifier.py` | ✅ Correct | ✅ Wired into `mission_executor_node.load_mission_callback` |
| Secure Launch Config | `SAS/launch/secure_launch.py` | ✅ Correct env vars | ⏳ Keystore has no enclaves yet for the 3 mavlink-bridge nodes (needs `ros2 security create_enclave` in a real ROS 2 environment) |

---

## Phase 1.5: Gesture Safety Gating — ✅ COMPLETE

Not part of the original `SAS_QGC_Integration_Plan.md`. While tracing mission-completion signaling for Phase 0/1 work, a real, confirmed safety gap was found: `gesture_bridge_node`'s detected gestures could immediately override an actively-running QGC-driven mission, with no gating anywhere in the pipeline. Implemented and fully tested under its own plan (`examine-the-sas-repository-precious-diffie.md`):

- **`fleet_manager_node.py`** — fleet-wide mission-state tracking; a gesture is only acted on if every configured drone currently reports `COMPLETED` or `IDLE`.
- **`navigation_control_node.py`** — backstop rejecting gesture-sourced commands on `navigation_control/mission_command` while a mission is active, without blocking legitimate internal mission traffic sharing the same topic.
- **`mission_executor_node.py`** — added `land`/`goto` handling to `mission_control_callback` (previously unhandled, silently no-opping the `halt_stop`/`assemble_rally` gestures regardless of any gating), gated identically for gesture-sourced commands, plus a startup `IDLE` status announcement.
- **`mission_control_bridge.py`** — `in_progress` field fix + QoS durability mismatch fix (a dependency for `fleet_manager_node`'s state tracking, unrelated to the MAVLink wire-format issues tracked in Phase 0).

Two additional, previously-unknown bugs were found and fixed along the way: a formation-gesture payload key mismatch (`'command'` vs `'type'`) that made formation gestures silent no-ops, and the QoS mismatch noted above.

**Test results:** 1215 SAS unit tests passing (81 in `test_fleet_manager.py`, 119 in `test_navigation_control_node.py`, 165 in `test_mission_executor_node.py`, all new tests importing and exercising real production code), plus 33 in `mavlink-bridge/tests/unit/test_mission_control_bridge.py`.

**Known residual gap:** `hand_gesture_node.py` (legacy, not launched by either launch file) bypasses `fleet_manager_node` entirely and is therefore unaffected by this gate — out of scope unless revived.

---

## Phase 2: Remaining ROS 2 Bridges — NOT STARTED

Phase 0 is complete, so these can now be built directly on the corrected `mavlink_v2.py` from day one, following the pattern established there (byte-for-byte pymavlink verification for new payload builders, real-module tests from the first commit).

### 2a. Fleet Manager Bridge
- **File:** `mavlink-bridge/fleet_manager_mavlink_bridge.py`
- **Input:** `/fleet/status` (aggregated position, battery, mission progress)
- **Output:** MAVLink messages (fleet aggregation)
- **Tests:** 15+ tests, importing the real module from the start

### 2b. Collision Avoidance Bridge
- **File:** `mavlink-bridge/collision_mavlink_bridge.py`
- **Input:** `/fmu/out/obstacle_distance` (SF45 LiDAR)
- **Output:** MAVLink payload or custom message
- **Tests:** 12+ tests

### 2c. Emergency Wipe Bridge
- **File:** `mavlink-bridge/emergency_wipe_mavlink_bridge.py`
- **Input:** MAVLink COMMAND_LONG for emergency operations
- **Output:** ROS 2 service call to `/emergency_wipe/execute`
- **Tests:** 10+ tests

### 2d. Launch File Update
- **File:** `mavlink-bridge/launch_sas_qgc_integration.py` (update)
- Add all three new bridges. `secure_launch.py` is fixed and can be integrated, but generate DDS-Security enclaves for these new nodes first (`ros2 security create_enclave`) or they'll fail to launch under `ROS_SECURITY_STRATEGY=Enforce`.

---

## Phase 3: QGC Custom Plugin — NOT STARTED

### Components (Requires C++/QML)
1. **SASPlugin.h/cpp** — extends QGCCorePlugin, intercepts MAVLink messages (~400 lines)
2. **SASFleetView.qml** — fleet status visualization, formation selector, GPS spoofing alert banner, collision risk heatmap (~500 lines)
3. **CMakeLists.txt** — QGC build integration

**Estimated Effort:** 2-3 weeks (requires Qt/QML expertise, not available in any session to date).

---

## Phase 4: Comprehensive Testing — ✅ COMPLETE for Phases 0/1/1.5

### Current State by Component

| Component | Tests | Import Real Module? | Status |
|-----------|-------|----------------------|--------|
| `mavlink_v2.py` codec (new) | 22 | ✅ Yes (vs pymavlink ground truth) | ✅ Done |
| GPS Spoof Bridge | 14 unit + 8 integration | ✅ Yes (real bridge, real UDP sockets) | ✅ Done |
| Telemetry Bridge | 17 | ✅ Yes | ✅ Done |
| Mission Control Bridge | 29 unit + 6 integration (real socket + real thread) | ✅ Yes | ✅ Done |
| Mission Signing | 28 | ✅ Yes (real RSA-2048 keypair) | ✅ Done |
| Gesture Gating (fleet/nav/executor) | 365 (across 3 SAS test files) | ✅ Yes | ✅ Done (Phase 1.5) |
| Fleet Manager Bridge | 0 | — | Not started (Phase 2) |
| Collision Bridge | 0 | — | Not started (Phase 2) |
| Emergency Wipe Bridge | 0 | — | Not started (Phase 2) |

**Totals:** mavlink-bridge suite 124/124 passing (zero exclusions); SAS unit suite 1215/1215 passing (3 pre-existing skips).

### Test Verification vs SAS Repo

Cross-checked and confirmed no topical redundancy between mavlink-bridge tests and SAS's own test suite (SAS tests executor/node *logic*; bridge tests target MAVLink *translation* — different layers). This finding from the original roadmap still holds. The larger, previously-unstated problem was never redundancy — it was that most bridge tests didn't test anything real at all, regardless of redundancy.

---

## Implementation Priority (Revised)

### Done
- ✅ Phase 0: MAVLink protocol correctness, all 3 bridges, mission signing wired in, secure launch fixed, full test suite rewritten
- ✅ Phase 1.5: Gesture Safety Gating

### Now / Next
- Live ROS 2/WSL validation against real QGroundControl and PX4 — everything to date is unit/integration-tested but not run against the actual external systems
- Generate DDS-Security enclaves for the 3 mavlink-bridge nodes
- Phase 2: remaining bridges (Fleet Manager, Collision, Emergency Wipe), built on the corrected `mavlink_v2.py` from day one

### Later
- Phase 3: QGC Plugin (separate session, requires Qt/QML/C++)

---

## Key Files Summary

### Existing (all complete)
```
mavlink-bridge/
├── mavlink_v2.py                       ✅ verified MAVLink v2 codec
├── gps_spoof_mavlink_bridge.py         ✅ migrated to mavlink_v2
├── telemetry_mavlink_bridge.py         ✅ migrated to mavlink_v2
├── mission_control_bridge.py           ✅ migrated to mavlink_v2; real upload handshake; reply-address fix
└── tests/                              ✅ 124 tests, all import real modules

SAS/
├── security/mission_signer.py          ✅ correct
├── my_python_package/mission_verifier.py  ✅ correct, wired into load_mission_callback
├── launch/secure_launch.py             ✅ correct env vars (enclaves for bridge nodes still needed)
├── my_python_package/fleet_manager_node.py       ✅ gesture-gated (Phase 1.5)
├── my_python_package/navigation_control_node.py  ✅ gesture-gated (Phase 1.5)
└── my_python_package/mission_executor_node.py    ✅ gesture-gated (Phase 1.5) + signature verification (Phase 1)
```

### To Create (Phase 2, unchanged from prior plan)
```
mavlink-bridge/
├── fleet_manager_mavlink_bridge.py
├── collision_mavlink_bridge.py
├── emergency_wipe_mavlink_bridge.py
└── tests/unit/ + tests/integration/ for each, importing real modules from the start
```

---

## Success Criteria

### Phase 0 Complete When: ✅ ALL MET
- [x] `telemetry_mavlink_bridge.py` and `mission_control_bridge.py` both use `mavlink_v2.py`
- [x] `mission_control_bridge.py` implements the real upload handshake and actually publishes uploaded missions to `mission_executor_node`
- [x] `mission_verifier.verify_mission()` is called somewhere in the live upload path
- [x] `secure_launch.py` uses real SROS2 environment variables
- [x] Every mavlink-bridge test file imports and exercises its corresponding production module (no more self-referential mocks)
- [x] A real MAVLink parser (pymavlink) can decode a live frame from each bridge

### Phase 2/3 Complete When: (unchanged from prior roadmap — not yet reached)

### Before calling any of this "production-ready" (not yet done):
- [ ] Full stack launched against real QGroundControl and PX4 (SITL or hardware) in a real ROS 2 environment
- [ ] DDS-Security enclaves generated for the 3 mavlink-bridge nodes
- [ ] `_get_battery_voltage()`'s units bug fixed, if accurate battery reporting matters for the deployment

---

## Notes

1. **Verification methodology going forward:** any new MAVLink payload builder should be checked byte-for-byte against `pymavlink.dialects.v20.common` before being considered correct — this is what caught the Phase 0 bugs, and hand-derivation from the XML spec or from memory is not reliable enough on its own (two bugs were found this way even in the newly-written `mavlink_v2.py`'s first draft).
2. **Test methodology going forward:** a test file's job is to import and exercise the real module. A test suite that reimplements the logic it's meant to verify provides no signal — this is exactly how the frame-header bug went undetected across ~130 "passing" tests.
3. **Backward Compatibility:** all Phase 1.5 gesture-gating changes are additive to existing SAS nodes; no removed functionality.
4. **Git Strategy:** unchanged — logical commits per component, descriptive messages, no long multi-paragraph commit bodies.
