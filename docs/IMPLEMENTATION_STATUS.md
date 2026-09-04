# SAS-QGC Integration: Implementation Status Report

**Last Updated:** 2026-09-04
**Overall Status:** Phases 0, 1, 1.5, 2 and 5 are complete. All seven bridges are verified
against pymavlink, mission signing is wired into the live upload path, secure_launch.py is
fixed, and the full mavlink-bridge suite (208 tests) plus the SAS unit suite (1215 tests)
import and exercise real production code.

**Live validation is now done.** Every bridge and every QGC-facing SAS node has been
exercised against real PX4 SITL, a real uXRCE-DDS bridge, and real QGroundControl — not
stubs. That included a real armed flight in Gazebo (climb to 5 m, hold, commanded land,
auto-disarm) and a real QGC mission upload landing in `mission_executor_node`. It surfaced
seven bugs that no stub-based test could have caught; see Part 5. Phase 3 (QGC plugin)
remains not started.

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
- **`telemetry_mavlink_bridge.py`, found by finally importing against real `px4_msgs` instead of test stubs:** `VehicleStatus.system_status`, `VehicleStatus.load`, `VehicleAttitude.rollspeed`/`pitchspeed`/`yawspeed`, and `BatteryStatus.energy_consumed_j` are not real px4_msgs fields — confirmed absent as far back as px4_msgs v1.14.0 (2023), not a version-specific rename. This would have raised `AttributeError` on the very first `VehicleAttitude`/`VehicleStatus` callback against any real PX4 instance, at any point in this bridge's history; the test suite's hand-shaped stubs supplied exactly the attributes the code expected, so nothing ever caught it. **Fixed**: angular rates now come from a new `VehicleAngularVelocity` subscription (`xyz` field), CPU load from a new `Cpuload` subscription (`load` field), heartbeat state determination simplified to arming-state-only (`system_status` has no real analogue), and `energy_consumed` reported as MAVLink's documented unknown sentinel (`-1`) rather than a fabricated value. Both new topics default to 0.0 gracefully if not yet received, independent of whether attitude/vehicle_status have arrived. 4 new regression tests added.

---

## Part 2: Test Suite — Now Testing Real Code

Every mavlink-bridge test file now imports and exercises its corresponding production module. None of the "self-referential mock" pattern from the earlier version of this document remains.

| Test File | Tests | Notes |
|-----------|-------|-------|
| `test_mavlink_v2.py` (new) | 26 | Byte-for-byte vs pymavlink for all 15 message types (incl. Phase 2's OBSTACLE_DISTANCE/COMMAND_LONG/COMMAND_ACK), plus header-structure and truncation-edge-case tests. Skips gracefully if `pymavlink` isn't installed. |
| `test_mavlink_crc.py` (rewritten) | 17 | Real `GPSSpoofMAVLinkBridge`; absorbed the previously-separate `test_mavlink_frame_generation.py`, which was pure duplication and has been deleted. No longer requires `rclpy` to be installed — stubs it. Includes a regression test pinning the corrected MAV_SEVERITY wire values. |
| `test_mission_signing.py` (rewritten) | 28 | Real `MissionSigner`/`MissionVerifier` with a genuinely generated RSA-2048 keypair; tampering tests now actually re-verify mutated signed data instead of comparing two dict values to each other. |
| `test_telemetry_conversion.py` (rewritten) | 22 | Real `TelemetryMAVLinkBridge`, realistic PX4-shaped mock telemetry; includes regression tests for the SYS_STATUS field-width bug, the BATTERY_STATUS per-cell-voltage bug, the SYS_STATUS pack-voltage units bug, the `time_boot_ms` overflow crash, and (new) the `system_status`/`load`/`rollspeed`-family/`energy_consumed_j` nonexistent-field bugs found by building against real px4_msgs. |
| `test_mission_control_bridge.py` (rewritten) | 29 | Real `MissionControlBridge`; full upload/download handshake, GCS-identity learning, reply-address regression test. |
| `test_mission_control_integration.py` (rewritten) | 6 | Real bridge with a **real bound UDP socket and real background receiver thread**, driven from a second real socket — genuine byte-level round-trip, not a fake socket object. |
| `test_gps_spoof_integration.py` (rewritten) | 8 | Real `GPSSpoofMAVLinkBridge` with a real UDP socket on each end. No longer requires `rclpy` to be installed. |
| `test_fleet_manager_bridge.py` + `test_fleet_manager_integration.py` (Phase 2) | 13 + 3 | Real `FleetManagerMAVLinkBridge`; double-encoded `/fleet/status` parsing, per-drone STATUSTEXT, de-duplication, severity mapping, real UDP socket. |
| `test_collision_bridge.py` + `test_collision_integration.py` (Phase 2) | 7 + 3 | Real `CollisionMAVLinkBridge`; ObstacleDistance→OBSTACLE_DISTANCE, distances survive the wire (pymavlink round-trip), real UDP socket. |
| `test_emergency_wipe_bridge.py` + `test_emergency_wipe_integration.py` (Phase 2) | 14 + 4 | Real `EmergencyWipeMAVLinkBridge`; two-factor gate (accept/deny/temporarily-reject), command filtering, CRC rejection, reply addressing over a real bound socket + real receiver thread. |
| `test_mavlink_router.py` (Phase 2, inbound-port fix) | 12 | Real `MAVLinkRouterNode`'s pure relay logic (`_forward_to_downstream`, `_relay_to_qgc`, `parse_targets`) with fake sockets. |
| `test_mavlink_router_integration.py` (Phase 2, inbound-port fix) | 4 | Real router, real UDP sockets, simulated QGC + simulated downstream-bridge endpoints: fan-out, reply relay, no-relay-before-first-inbound-frame, ordering. |
| `test_router_with_real_bridges_integration.py` (Phase 2, inbound-port fix) | 5 | The crown-jewel test: real router + real `MissionControlBridge` + real `EmergencyWipeMAVLinkBridge`, all reachable from ONE simulated QGC socket — the definitive proof the inbound single-UDP-port limitation is resolved. |

**Full mavlink-bridge suite: 208 tests passed, zero exclusions** (previously, `test_mavlink_crc.py` had to be excluded because it required a real ROS 2 environment; that's no longer true). Full SAS unit suite: 1215 passed, 3 skipped — unaffected by this work.

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

## Part 5: Live ROS 2 Build & Validation — ✅ COMPLETE

Everything before this phase was correctness-by-construction: pymavlink byte comparison,
real sockets, real crypto, real build tooling. None of it had ever met the live systems it
exists to interoperate with. Running it for real found seven bugs, every one of them
invisible to a stub-based test.

### Build/packaging (earlier in this phase)

Four packaging bugs blocked `colcon build`/`ros2 run` resolution entirely and are covered in
`IMPLEMENTATION_ROADMAP.md` Phase 5: `package='SAS'` in three launch files (the real package
is `my_python_package`), a missing `ament_python` build_type export, a missing `data_files`
block, and a missing `setup.cfg`. Building `px4_msgs` from source for the first time then
surfaced the `telemetry_mavlink_bridge.py` field-mismatch bugs listed in Part 1.

### Bugs found by actually running it

1. **`single_drone.launch.py` passed `drone_id='drone_1'` to `offboard_controller_node`.**
   PX4's uXRCE-DDS bridge publishes `/fmu/out/...` **unprefixed** for a single SITL
   instance, so that node alone subscribed to `/drone_1/fmu/out/...` — topics PX4 never
   publishes. It therefore never received `VehicleLocalPosition`/`VehicleStatus`, and
   arm/takeoff hung forever on "No position data available" while every other node saw real
   PX4 data over the same (correctly unprefixed) topics.

2. **Only the first MAVLink message in a datagram was parsed.** QGC routinely flushes
   several queued messages into a single UDP write. `parse_frame()` inspects one frame and
   has no way to report that more data followed, so anything bundled behind a leading
   message was silently discarded. Fixed with `parse_frames()`.

3. **MAVLink 1.0 frames were rejected outright.** The parser accepted only `STX 0xFD`
   (v2). QGroundControl opens every link in MAVLink **1.0** (`0xFE`) and upgrades only after
   it sees a v2 frame from the vehicle, so its entire opening exchange —
   `PARAM_REQUEST_LIST`, `MISSION_COUNT`, `COMMAND_LONG` — vanished at the magic-byte check
   with no error logged anywhere. This is what made mission upload fail with QGC's "Mission
   write mission count failed, maximum retries exceeded". Confirmed by decoding captured
   bytes (`fe0431ffbe2c04000101fc9a` = v1 `MISSION_COUNT`, count=4, from sysid 255/compid
   190) and fixed by parsing both versions. We still *build* v2, which is what prompts QGC
   to upgrade the link.

4. **`load_mission` was published with incompatible QoS.** The bridge offered
   `BEST_EFFORT` while `mission_executor_node` subscribes with default (`RELIABLE`) QoS.
   Under DDS rules a RELIABLE subscriber never matches a BEST_EFFORT publisher, so a fully
   received mission was assembled, logged as published, and dropped on the floor. Now
   `RELIABLE`.

5. **`drone_id` was overloaded across two incompatible namespaces.**
   `telemetry_mavlink_bridge`/`collision_mavlink_bridge` use it to prefix **PX4** DDS topics
   (`/fmu/...`), which single SITL publishes unprefixed; `mission_control_bridge`/
   `emergency_wipe_mavlink_bridge` use it to prefix **SAS node** topics, which
   `single_drone.launch.py` pushes under `/drone_1`. One value cannot satisfy both — left
   empty, mission uploads went to a topic nobody listened on; set to `drone_1`, telemetry
   and collision would break instead. Split into `drone_id` (PX4) and the new
   `sas_namespace` (SAS).

6. **`emergency_wipe_node` escaped its own namespace.** Its service and status topic were
   declared absolute (`"/emergency_wipe/execute"`), and ROS 2 ignores a node's namespace for
   absolute names — unlike every other SAS node, which uses relative names. So the bridge
   looked for `/drone_1/emergency_wipe/execute`, never matched the server at
   `/emergency_wipe/execute`, and answered QGC `TEMPORARILY_REJECTED` / "service
   unavailable". Under `multi_drone.launch.py` every drone would additionally have collided
   on one global service name, so multi-drone wipe was broken outright.

7. **PX4's own "Normal" mavlink instance collides with the router.** PX4 SITL starts a
   built-in mavlink instance (local port 18570, remote 14550) independent of our bridges,
   which floods and contends with `mavlink_router_node`'s external bind on 14550. It must be
   stopped on every fresh PX4 boot: `mavlink stop -u 18570` at the `pxh>` prompt.

### A long-standing assumption, disproved

Earlier sessions concluded that **WSL2 NAT mode's inbound UDP path was broken at the
platform level**, and QGC-facing inbound testing was set aside on that basis. That is not
correct. A raw UDP packet sent from Windows to the **WSL2 interface IP** (`172.28.61.135:14550`)
was delivered cleanly all the way through the router to both inbound bridges. The same
packet sent to `127.0.0.1:14550` was silently dropped. The real limitation is narrower and
well documented upstream: **WSL2's localhost-forwarding shim does not reliably forward UDP.**
Point QGC at the WSL2 interface IP and inbound works.

### What was verified live

| Component | Verified by |
|---|---|
| `gps_spoof_mavlink_bridge` | spoofing alerts → QGC `STATUSTEXT` |
| `telemetry_mavlink_bridge` | live PX4 telemetry → QGC |
| `fleet_manager_mavlink_bridge` | per-drone status summaries → QGC |
| `collision_mavlink_bridge` | SF45 sweep → `OBSTACLE_DISTANCE` |
| `mavlink_router_node` | logged fan-out of real QGC datagrams to both inbound bridges, and relayed their replies back |
| `mission_control_bridge` | real QGC mission upload: `MISSION_COUNT` → 5 × `MISSION_ITEM_INT` → assembled and published |
| `mission_executor_node` | received that upload: "Parsed QGC mission with 5 waypoints" |
| `emergency_wipe_mavlink_bridge` | real `COMMAND_LONG`: gate correctly DENIES wrong magic param and DENIES `confirmation=0`, ACCEPTS a valid command, invokes the service, reports back over `STATUSTEXT` (STUB_MODE on throughout — nothing was wiped) |
| `offboard_controller_node` | real armed flight in Gazebo: climb to 5 m, hold, commanded land, auto-disarm |
| `navigation_control_node` | drove that arm → takeoff → land sequence |

## What's Actually Production-Ready Today

- ✅ All six MAVLink bridges (`gps_spoof`, `telemetry`, `mission_control`, plus Phase 2's `fleet_manager`, `collision`, `emergency_wipe`) — verified MAVLink-correct against pymavlink, real upload/download handshake, correct reply addressing.
- ✅ Mission signing — cryptographically sound and wired into the live upload path.
- ✅ Gesture safety gating — fully implemented and tested.
- ✅ Emergency-wipe trigger — gated behind a two-factor (magic-param + confirmation) check in the bridge, since the underlying Trigger service has no auth of its own.
- ✅ `secure_launch.py` — correct SROS2 environment variables, and the keystore now has enclaves for all 12 SAS/mavlink-bridge nodes (see below).
- ✅ **DDS-Security wired into `launch_sas_qgc_integration.py`** — a new opt-in `enable_security` launch arg (default `false`, so existing dev/test runs without a configured keystore aren't silently broken) sets the same three `ROS_SECURITY_*` vars `secure_launch.py` uses, for every node in that file (including the SAS-package `gps_spoof_detector_node`). `keystore_path` defaults to `SAS/security/keystore` resolved as a sibling directory, overridable. Verified via the real `ros2 launch` tool in WSL: with `enable_security:=true` the log correctly reports the resolved keystore path and the launch proceeds past all the new env-var/log actions (failing only on `package 'SAS' not found` — expected, since SAS/mavlink-bridge aren't colcon-installed packages in this bare ROS 2 install; that's the separate, already-flagged live-validation gap); with the default `false`, no security-related log line appears at all.
- ✅ **DDS-Security enclaves for all 7 mavlink-bridge nodes** — generated via `ros2 security create_enclave` (WSL Ubuntu 24.04, ROS 2 Jazzy + sros2, the first real ROS 2 tooling used this project) and tailored to least-privilege via `ros2 security create_permission` with a hand-authored policy matching each bridge's actual topic/service usage (not SROS2's broad default template) — e.g. `mission_control_bridge` is scoped to publish `mission_executor/load_mission` and subscribe `mission_executor/status` only, matching the convention already used by the 5 pre-existing SAS enclaves. All 7 certs verified to chain to the identity CA and all 7 `permissions.p7s`/`governance.p7s` verified as validly-signed CMS messages via `openssl cms -verify` against the permissions CA. `mavlink_router_node` (pure UDP relay, zero ROS topics) correctly got the minimal grant (`ros_discovery_info` only).
- ✅ Full mavlink-bridge test suite (208 tests) and SAS unit suite (1215 passed / 3 skipped) — all importing and exercising real code.
- ✅ `mavlink-bridge/mavlink_router_node.py` (new) — fans QGC's single UDP comm link out to both inbound bridges (`mission_control_bridge`, `emergency_wipe_bridge`), resolving the inbound single-UDP-port limitation with no code changes to either bridge. See Part 1's bugs-found-and-fixed list for detail.
- ✅ `mavlink-bridge/demo_qgc_wire_protocol.py` (new) — a no-ROS-2, no-QGC-required live demo: the real `GPSSpoofMAVLinkBridge._send_statustext` sends genuine frames over a real UDP socket, decoded live by `pymavlink` (the same reference implementation QGC's own parser is built on). Useful both as a demo and as a fast local sanity check of the wire protocol.
- ✅ `mavlink-bridge/test_gps_spoof_alert_generator.py` (new) — the console script `setup.py` referenced but didn't have; publishes synthetic `/gps_spoof_alert` messages for manual QGC testing (checklist Phases 5.2/5.4/7.1).
- ✅ **`colcon build` succeeds for both packages** in a real ROS 2 workspace (WSL, ROS 2 Jazzy) — `my_python_package` (SAS) and `mavlink-bridge`, plus `px4_msgs` built from source. Package/executable/launch-file discovery all verified via the real `ros2 pkg`/`ros2 launch` tools, not just `python -m py_compile`. See Part 5.
- ✅ **The whole stack, run live end to end** — PX4 SITL + Gazebo, the uXRCE-DDS bridge, the
  SAS control nodes and all seven MAVLink bridges, against real QGroundControl on Windows.
  Includes a real armed flight (climb to 5 m, hold, commanded land, auto-disarm) and a real
  QGC mission upload reaching `mission_executor_node`. See Part 5 for the bugs this found
  and the per-component verification table.

## What Is NOT Ready

- ⏳ **QGC Custom Plugin (Phase 3)** — not started; requires C++/Qt/QML.
- ⏳ **Emergency wipe is inert in a normal bring-up.** `emergency_wipe_node` is not in any
  launch file, so a standard stack start does not include it and the bridge answers QGC
  "service unavailable". It was started by hand (`ros2 run my_python_package
  emergency_wipe_node --ros-args -r __ns:=/drone_1`) to verify the path.
- ⏳ **The wipe never actually wipes.** `STUB_MODE = True` suppresses execution by design.
  Going live requires populating `DATA_LOCATIONS` and removing the guard, per the node's own
  docstring — deliberately not done.
- ⏳ **DDS-Security has never been run through to enforcement.** `enable_security:=true` is
  confirmed to set the right env vars and log the right message, but no node has been
  started under `ROS_SECURITY_STRATEGY=Enforce`.

---

## Operational Requirements (learned the hard way — read before launching)

These are not optional niceties; each one silently breaks the integration if missed.

1. **Launch with `sas_namespace:=drone_1`** when running alongside
   `SAS/launch/single_drone.launch.py`. Omit it and mission uploads are published to a topic
   nothing subscribes to — no error, uploads just vanish. Leave `drone_id` empty for single
   SITL (PX4 publishes `/fmu/...` unprefixed).

   ```bash
   ros2 launch mavlink-bridge launch_sas_qgc_integration.py      system_id:=1 mavlink_host:=<wsl2-gateway-ip> sas_namespace:=drone_1
   ```

2. **Point QGC's Comm Link at the WSL2 interface IP, not `localhost`.** WSL2's
   localhost-forwarding shim drops UDP silently; the interface IP works. Find it with
   `ip addr show eth0`. **This IP changes when WSL restarts**, so the link needs updating
   after a restart — or switch WSL to mirrored networking mode to make `localhost` work.

3. **Set `mavlink_host` to the WSL2 default gateway** (`ip route | grep default`) — that is
   where the Windows host, and therefore QGC, is reachable from inside WSL2.

4. **Stop PX4's built-in mavlink instance on every fresh SITL boot:** `mavlink stop -u 18570`
   at the `pxh>` prompt. Otherwise it contends with `mavlink_router_node` on port 14550.

5. **QGC's "Plan was created for a different firmware/vehicle type" dialog on upload is
   benign** — click OK. Our `HEARTBEAT` correctly reports `MAV_TYPE_QUADROTOR` /
   `MAV_AUTOPILOT_PX4`; the warning reflects the `.plan` file's saved template, not a
   mismatch in what we send.

---

## Which SAS Nodes Connect to QGC, and Why

Preserved from the original integration plan (now fully implemented, so that document has
been removed). 7 of 15 SAS nodes are QGC-facing; the other 8 are internal plumbing or
sensor I/O with no operator relevance.

| Node | Connected | Route |
|---|---|---|
| `mission_executor_node` | ✅ yes | mission upload/download, progress |
| `offboard_controller_node` | ✅ yes | primary telemetry (position, attitude, battery) |
| `gps_spoof_detector_node` | ✅ yes | security alerts via `STATUSTEXT` |
| `fleet_manager_node` | ✅ yes | per-drone fleet status |
| `collision_offboard_controller_node` | ✅ yes | `OBSTACLE_DISTANCE` |
| `emergency_wipe_node` | ✅ yes | gated `COMMAND_LONG` → wipe service |
| `gesture_bridge_node` | optional | deliberately out of scope; failsafe override only |
| `navigation_control_node` | ✗ no | internal; abstracted by `mission_executor` |
| `sf45_px4_node` | ✗ no | already feeds PX4; exposed via collision bridge |
| `hand_gesture_node` | ✗ no | internal input |
| `image_src_node` / `image_zoom_src_node` | ✗ no | internal sensors |
| `odometry_control_node` | ✗ no | internal sensor fusion; PX4 handles output |
| `test_node` / `mission_test_interface` | ✗ no | development tooling |

---

## Immediate Next Steps (Priority Order)

1. **Decide the emergency-wipe deployment story** — add `emergency_wipe_node` to
   `single_drone.launch.py` (and `multi_drone.launch.py`) so the feature exists in a normal
   bring-up, and decide whether `STUB_MODE` stays on outside of testing.
2. **Make the QGC link durable** — either switch WSL2 to mirrored networking so `localhost`
   works, or script the Comm Link host so it survives a WSL IP change.
3. **Run DDS-Security through to enforcement** — start the stack with
   `enable_security:=true` and confirm nodes actually come up under `Enforce`.
4. **Phase 3: QGC Custom Plugin** (separate effort, C++/Qt/QML).

---

## Known Limitations

1. **Mission signer:** private key stored unencrypted on disk (file-permission-restricted
   only).
2. **QGC plugin:** requires C++/Qt/QML; not attempted in any session so far.
3. **MAVLink-uploaded missions are unsigned.** The mission protocol has no signature
   transport, so `load_mission_callback` accepts unsigned missions by design
   (`strict=False`). Signature verification only protects missions that carry one.
4. **Emergency wipe is not wired into any launch file and runs in STUB_MODE** — see "What Is
   NOT Ready".
5. **The QGC link depends on a dynamic WSL2 IP** — see "Operational Requirements".
6. **Style debt in SAS:** 356 `E501` (line length) and 21 `D401` (docstring imperative mood)
   findings remain, deliberately left as judgment calls rather than mass-rewritten. 17
   `.pyc` files are also still tracked in git despite `__pycache__/` being ignored
   (committed before the ignore rule existed).
