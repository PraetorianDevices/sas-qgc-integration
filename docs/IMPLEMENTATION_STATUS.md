# SAS-QGC Integration: Implementation Status Report

**Last Updated:** 2026-07-14
**Overall Status:** Phases 0, 1, 1.5, and 2 are complete. All six bridges are verified against pymavlink, mission signing is wired into the live upload path, secure_launch.py is fixed, and the entire mavlink-bridge test suite (197 tests) imports and exercises real production code. Phase 2 (Fleet Manager / Collision / Emergency Wipe bridges) is now done, and the inbound single-UDP-port limitation it introduced is resolved via `mavlink_router_node.py`. Phase 3 (QGC plugin) remains not started, and no phase has yet been validated against live QGroundControl/PX4 hardware.

**Branch note:** all of this work lives on `develop` in both the outer repo and the `SAS` submodule (each has its own `develop`). 

---

## Part 1: MAVLink Wire Protocol — Bridge-by-Bridge

| Bridge | Frame Header | Payload Correctness | Verified vs pymavlink | Status |
|--------|--------------|---------------------|------------------------|--------|
| `mavlink_v2.py` (shared codec) | ✅ Correct 10-byte header, 3-byte msg_id | ✅ HEARTBEAT, SYS_STATUS, ATTITUDE, GLOBAL_POSITION_INT, BATTERY_STATUS, STATUSTEXT, MISSION_ITEM_INT, MISSION_ACK, MISSION_CURRENT, MISSION_COUNT, MISSION_REQUEST_INT, MISSION_ITEM_REACHED | ✅ Byte-for-byte match, all 12 message types, in a permanent pytest suite (`test_mavlink_v2.py`) | ✅ **DONE** |
| `gps_spoof_mavlink_bridge.py` | ✅ Migrated to `mavlink_v2` | ✅ STATUSTEXT | ✅ Via shared codec | ✅ **DONE** |
| `telemetry_mavlink_bridge.py` | ✅ Migrated to `mavlink_v2` | ✅ SYS_STATUS (32-bit sensor bitmasks, correct field order), BATTERY_STATUS (real per-cell voltages, no longer always 0) | ✅ Via shared codec + real-bridge functional tests | ✅ **DONE** |
| `mission_control_bridge.py` | ✅ Migrated to `mavlink_v2` | ✅ Switched to `MISSION_ITEM_INT`; MISSION_ACK/MISSION_CURRENT/MISSION_COUNT rewritten to the real spec; full upload handshake implemented (inbound `MISSION_COUNT` → sequential `MISSION_REQUEST_INT` pulls → single `MISSION_ACK` after the last item); `_mission_upload_pub.publish()` is now actually called | ✅ Via shared codec + real UDP-socket integration tests | ✅ **DONE** |
| `fleet_manager_mavlink_bridge.py` (Phase 2) | ✅ Uses `mavlink_v2` | ✅ STATUSTEXT per-drone mission-state/progress summaries from `/fleet/status`, de-duplicated | ✅ Via shared codec + real UDP-socket integration tests | ✅ **DONE** |
| `collision_mavlink_bridge.py` (Phase 2) | ✅ Uses `mavlink_v2` | ✅ OBSTACLE_DISTANCE (330), new codec message verified byte-for-byte; near 1:1 forward of the SF45 sweep from `/fmu/in/obstacle_distance` | ✅ Via shared codec + real UDP-socket + pymavlink round-trip | ✅ **DONE** |
| `emergency_wipe_mavlink_bridge.py` (Phase 2) | ✅ Uses `mavlink_v2` | ✅ COMMAND_LONG (76) parse + COMMAND_ACK (77) build, new codec messages verified byte-for-byte; two-factor gate before calling the wipe service | ✅ Via shared codec + real bound-socket + real receiver-thread integration tests | ✅ **DONE** |
| `mavlink_router_node.py` (Phase 2, inbound-port fix) | N/A — pure byte relay, no MAVLink parsing | ✅ Fans QGC's single external UDP link out to `mission_control_bridge`'s and `emergency_wipe_bridge`'s internal ports, and relays their replies back; neither bridge needed a code change | ✅ Real 3-node topology test (real router + real mission_control_bridge + real emergency_wipe_bridge, one simulated QGC socket) | ✅ **DONE** |

### Bugs found and fixed during the telemetry/mission-control migration (beyond the frame header itself)

- **`telemetry_mavlink_bridge.py`:** `time_boot_ms=int(time.time()*1000)` overflowed MAVLink's `uint32` field (Unix-epoch ms is ~1.7 trillion; the field maxes at ~4.3 billion) and would have crashed `struct.pack` on every telemetry publish once a GPS fix was present. Pre-existing, not introduced this pass. Fixed by tracking a monotonic boot-time reference instead.
- **`telemetry_mavlink_bridge.py`:** `_get_battery_voltage()` divided the cell-voltage sum by 1000 assuming millivolt input, but PX4's `voltage_cell_v` is in volts (the same file's `BATTERY_STATUS` conversion correctly treats it as volts elsewhere), making `SYS_STATUS.voltage_battery` report ~1000x too low. **Fixed** — removed the erroneous divide; added `test_sys_status_battery_voltage_is_pack_voltage_not_1000x_low` regression test.
- **`mission_control_bridge.py`:** outbound replies were hardcoded to `('localhost', 14550)`, ignoring both the actual UDP sender's address and any configured `mavlink_port` — silently breaking the documented multi-drone setup (a different port per drone). Fixed: the bridge now replies to the address of whoever last contacted it, falling back to its own configured `(mavlink_host, mavlink_port)` before any packet has been received.
- **`mission_control_bridge.py`:** `MAVLINK_MSG_ID_MISSION_ITEM_REACHED` was hardcoded as `61`; the real ID is `46`. Fixed by switching to `mavlink_v2`'s verified constants.
- **`mavlink_v2.py` (caught by testing, not shipped):** `parse_mission_ack`/`parse_mission_count`/`parse_mission_request` originally rejected payloads shorter than their un-truncated size, but MAVLink 2's trailing-zero-truncation rule means a `MISSION_ACK(result=ACCEPTED)` or an empty `MISSION_COUNT(0)` legitimately arrives shorter. Fixed to zero-fill, matching the same fix already applied to `parse_mission_item_int`.
- **`gps_spoof_mavlink_bridge.py`:** its `MAVSeverity` enum had INFO/EMERGENCY and ALERT/CRITICAL transposed (`INFO=0`, `CRITICAL=5`, `ALERT=2`, `EMERGENCY=6`, instead of the real MAV_SEVERITY values), so a genuine CRITICAL spoofing alert transmitted at NOTICE priority and an INFO alert at EMERGENCY priority — backwards from QGC's severity-based color coding. Found while building Phase 2's spec-correct severity enums for the fleet/wipe bridges. **Fixed** — all usages in the bridge and its tests referenced the enum symbolically (`MAVSeverity.CRITICAL` etc.), not raw integers, so correcting the values required no other code changes; added `TestSeverityMatchesMavlinkSpec` to `test_mavlink_crc.py` pinning the actual wire values, since the existing tests only checked relative consistency.
- **Inbound single-UDP-port limitation (Phase 2 follow-up), resolved:** `mission_control_bridge` and `emergency_wipe_bridge` both need to bind a socket to receive, but QGC uses one UDP comm link per vehicle and two processes can't cleanly bind the same port. Resolved with a new `mavlink_router_node.py`: it binds the single external port QGC's comm link targets and fans every inbound datagram, byte-for-byte, out to both bridges' own internal ports, relaying their replies back to whichever address most recently contacted the external socket. **Neither existing bridge needed any code change** — both already bind whatever port they're configured with and dynamically learn their reply address from the sender of the last packet they received; pointed at the router instead of directly at QGC, that pre-existing mechanism keeps working unmodified. Verified with a real 3-node topology test: real router + real `MissionControlBridge` + real `EmergencyWipeMAVLinkBridge`, all reachable from one simulated QGC socket, confirming both a `MISSION_COUNT`→`MISSION_ACK` roundtrip and a `COMMAND_LONG`→`COMMAND_ACK` roundtrip through the same link.

---

## Part 2: Test Suite — Now Testing Real Code

Every mavlink-bridge test file now imports and exercises its corresponding production module. None of the "self-referential mock" pattern from the earlier version of this document remains.

| Test File | Tests | Notes |
|-----------|-------|-------|
| `test_mavlink_v2.py` (new) | 26 | Byte-for-byte vs pymavlink for all 15 message types (incl. Phase 2's OBSTACLE_DISTANCE/COMMAND_LONG/COMMAND_ACK), plus header-structure and truncation-edge-case tests. Skips gracefully if `pymavlink` isn't installed. |
| `test_mavlink_crc.py` (rewritten) | 17 | Real `GPSSpoofMAVLinkBridge`; absorbed the previously-separate `test_mavlink_frame_generation.py`, which was pure duplication and has been deleted. No longer requires `rclpy` to be installed — stubs it. Includes a regression test pinning the corrected MAV_SEVERITY wire values. |
| `test_mission_signing.py` (rewritten) | 28 | Real `MissionSigner`/`MissionVerifier` with a genuinely generated RSA-2048 keypair; tampering tests now actually re-verify mutated signed data instead of comparing two dict values to each other. |
| `test_telemetry_conversion.py` (rewritten) | 18 | Real `TelemetryMAVLinkBridge`, realistic PX4-shaped mock telemetry; includes regression tests for the SYS_STATUS field-width bug, the BATTERY_STATUS per-cell-voltage bug, the SYS_STATUS pack-voltage units bug, and the `time_boot_ms` overflow crash. |
| `test_mission_control_bridge.py` (rewritten) | 29 | Real `MissionControlBridge`; full upload/download handshake, GCS-identity learning, reply-address regression test. |
| `test_mission_control_integration.py` (rewritten) | 6 | Real bridge with a **real bound UDP socket and real background receiver thread**, driven from a second real socket — genuine byte-level round-trip, not a fake socket object. |
| `test_gps_spoof_integration.py` (rewritten) | 8 | Real `GPSSpoofMAVLinkBridge` with a real UDP socket on each end. No longer requires `rclpy` to be installed. |
| `test_fleet_manager_bridge.py` + `test_fleet_manager_integration.py` (Phase 2) | 13 + 3 | Real `FleetManagerMAVLinkBridge`; double-encoded `/fleet/status` parsing, per-drone STATUSTEXT, de-duplication, severity mapping, real UDP socket. |
| `test_collision_bridge.py` + `test_collision_integration.py` (Phase 2) | 7 + 3 | Real `CollisionMAVLinkBridge`; ObstacleDistance→OBSTACLE_DISTANCE, distances survive the wire (pymavlink round-trip), real UDP socket. |
| `test_emergency_wipe_bridge.py` + `test_emergency_wipe_integration.py` (Phase 2) | 14 + 4 | Real `EmergencyWipeMAVLinkBridge`; two-factor gate (accept/deny/temporarily-reject), command filtering, CRC rejection, reply addressing over a real bound socket + real receiver thread. |
| `test_mavlink_router.py` (Phase 2, inbound-port fix) | 12 | Real `MAVLinkRouterNode`'s pure relay logic (`_forward_to_downstream`, `_relay_to_qgc`, `parse_targets`) with fake sockets. |
| `test_mavlink_router_integration.py` (Phase 2, inbound-port fix) | 4 | Real router, real UDP sockets, simulated QGC + simulated downstream-bridge endpoints: fan-out, reply relay, no-relay-before-first-inbound-frame, ordering. |
| `test_router_with_real_bridges_integration.py` (Phase 2, inbound-port fix) | 5 | The crown-jewel test: real router + real `MissionControlBridge` + real `EmergencyWipeMAVLinkBridge`, all reachable from ONE simulated QGC socket — the definitive proof the inbound single-UDP-port limitation is resolved. |

**Full mavlink-bridge suite: 197 tests passed, zero exclusions** (previously, `test_mavlink_crc.py` had to be excluded because it required a real ROS 2 environment; that's no longer true). Full SAS unit suite: 1215 passed, 3 skipped — unaffected by this work.

**Test-suite order dependency, found and fixed:** `test_mavlink_crc.py`, `test_gps_spoof_integration.py`, `test_mission_control_bridge.py`, `test_mission_control_integration.py`, and `test_telemetry_conversion.py` each installed their own competing `rclpy`/`std_msgs`/`px4_msgs` stub into `sys.modules`. Because a module's class body only executes once and gets cached, whichever file's stub loaded first "won" for the rest of the process — `pytest tests/` (124 passed) only worked because directory scanning collects `tests/integration/` before `tests/unit/` alphabetically, loading the fully-capable stub first. Running unit-before-integration explicitly (e.g. `pytest tests/unit/test_mission_control_bridge.py tests/integration/test_mission_control_integration.py`) raised `TypeError: object.__init__() takes exactly one argument` — a latent fragility the full-directory run was silently masking. Fixed by consolidating all stubbing into a single, always-fully-capable stub in `tests/conftest.py` (with a new `ros_params` fixture for integration tests to configure e.g. ephemeral ports before construction), which pytest always imports first regardless of file selection or order. Verified with several non-default orderings (unit-first, integration-first, mixed) — all 124 tests now pass in every ordering tried.

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

- ✅ All six MAVLink bridges (`gps_spoof`, `telemetry`, `mission_control`, plus Phase 2's `fleet_manager`, `collision`, `emergency_wipe`) — verified MAVLink-correct against pymavlink, real upload/download handshake, correct reply addressing.
- ✅ Mission signing — cryptographically sound and wired into the live upload path.
- ✅ Gesture safety gating — fully implemented and tested.
- ✅ Emergency-wipe trigger — gated behind a two-factor (magic-param + confirmation) check in the bridge, since the underlying Trigger service has no auth of its own.
- ✅ `secure_launch.py` — correct SROS2 environment variables, and the keystore now has enclaves for all 12 SAS/mavlink-bridge nodes (see below) — though this launch file's security env vars still need merging into `launch_sas_qgc_integration.py` before security is actually enforced for the bridges.
- ✅ **DDS-Security enclaves for all 7 mavlink-bridge nodes** — generated via `ros2 security create_enclave` (WSL Ubuntu 24.04, ROS 2 Jazzy + sros2, the first real ROS 2 tooling used this project) and tailored to least-privilege via `ros2 security create_permission` with a hand-authored policy matching each bridge's actual topic/service usage (not SROS2's broad default template) — e.g. `mission_control_bridge` is scoped to publish `mission_executor/load_mission` and subscribe `mission_executor/status` only, matching the convention already used by the 5 pre-existing SAS enclaves. All 7 certs verified to chain to the identity CA and all 7 `permissions.p7s`/`governance.p7s` verified as validly-signed CMS messages via `openssl cms -verify` against the permissions CA. `mavlink_router_node` (pure UDP relay, zero ROS topics) correctly got the minimal grant (`ros_discovery_info` only).
- ✅ Full mavlink-bridge test suite (197 tests) and SAS unit suite (1215 tests) — all importing and exercising real code.
- ✅ `mavlink-bridge/mavlink_router_node.py` (new) — fans QGC's single UDP comm link out to both inbound bridges (`mission_control_bridge`, `emergency_wipe_bridge`), resolving the inbound single-UDP-port limitation with no code changes to either bridge. See Part 1's bugs-found-and-fixed list for detail.
- ✅ `mavlink-bridge/demo_qgc_wire_protocol.py` (new) — a no-ROS-2, no-QGC-required live demo: the real `GPSSpoofMAVLinkBridge._send_statustext` sends genuine frames over a real UDP socket, decoded live by `pymavlink` (the same reference implementation QGC's own parser is built on). Useful both as a demo and as a fast local sanity check of the wire protocol.
- ✅ `mavlink-bridge/test_gps_spoof_alert_generator.py` (new) — the console script `setup.py` referenced but didn't have; publishes synthetic `/gps_spoof_alert` messages for manual QGC testing (checklist Phases 5.2/5.4/7.1).

## What Is NOT Ready

- ⏳ Enforcing security for the mavlink-bridge launch stack — the enclaves exist and verify correctly, but `launch_sas_qgc_integration.py` doesn't yet set the `ROS_SECURITY_*` env vars the way `secure_launch.py` does for the original 5 SAS nodes; the two launch files need merging (or the bridges relaunched under `secure_launch.py`'s environment) before security is actually enforced, not just available.
- ⏳ QGC Custom Plugin (Phase 3) — not started, requires C++/Qt/QML.
- ⏳ Live ROS 2/WSL validation — everything above has been verified via unit/integration tests and real-socket simulation in this environment, but not yet run against a live ROS 2 install, real PX4/QGC, or real hardware. This now includes the router topology: verified with a real 3-node test in this environment, but never against real QGroundControl.

---

## Immediate Next Steps (Priority Order)

1. **Live ROS 2 validation.** Launch the full bridge stack (now including `mavlink_router_node`) in WSL against a real QGroundControl instance and a PX4 SITL or real vehicle — everything to this point has been verified via unit tests, byte-for-byte pymavlink comparison, and real-socket simulation (plus, now, a live pymavlink-decoded UDP demo and a real 3-node router topology test), but never against the actual external systems it's meant to interoperate with.
2. Merge `secure_launch.py`'s `ROS_SECURITY_*` environment variables into `launch_sas_qgc_integration.py` (or launch the bridges under `secure_launch.py`'s environment) so the now-generated, now-tailored enclaves actually get enforced for the mavlink-bridge nodes, not just present in the keystore.
3. Phase 3: QGC Custom Plugin (separate session, C++/Qt/QML).

---

## Known Limitations

1. **Mission Signer:** private key stored unencrypted on disk (file-permission-restricted only).
2. **DDS-Security enforcement not wired into the mavlink-bridge launch file:** the keystore has correct, verified, least-privilege enclaves for all 7 mavlink-bridge nodes, but `launch_sas_qgc_integration.py` doesn't yet set `ROS_SECURITY_ENABLE`/`ROS_SECURITY_STRATEGY`/`ROS_SECURITY_KEYSTORE` the way `secure_launch.py` does — security is available but not yet turned on for that launch file.
3. **QGC Plugin:** requires C++/Qt/QML, not attempted in any session so far.
4. **No live validation yet:** all verification to date is unit/integration-level (pymavlink byte comparison, real sockets, real crypto) — not yet run against real QGroundControl, PX4, or hardware.
