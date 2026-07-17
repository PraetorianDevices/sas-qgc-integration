# SAS-QGC Integration Implementation Roadmap

## Executive Summary

Full implementation of `SAS_QGC_Integration_Plan.md`, excluding `gesture_bridge_node`'s QGC connection (per original scope). A subsequent protocol audit found the MAVLink layer had a showstopper wire-format bug across all three bridges, so a corrective phase (Phase 0) was inserted ahead of the original Phase 2/3 work. That phase is now complete. A separate, unrelated safety feature (gesture-gating) was also completed under its own plan — see Phase 1.5.

**Phases:**
0. **Phase 0 (✅ Complete):** MAVLink Wire Protocol Correctness — frame-format bug fixed in all 3 bridges, mission signing wired in, secure launch fixed, entire test suite now imports real modules
1. **Phase 1 (✅ Complete):** Security & Mission Signing
2. **Phase 1.5 (✅ Complete):** Gesture Safety Gating — not part of the original plan; added after a real conflict risk was found
3. **Phase 2 (✅ Complete):** Remaining ROS 2 Bridges (Fleet Manager, Collision, Emergency Wipe)
4. **Phase 3 (Not started):** QGC Custom Plugin
5. **Phase 4 (✅ Complete for Phases 0/1/1.5/2):** Comprehensive Testing
6. **Phase 5 (In progress — build/import verified, nothing launched yet):** Live ROS 2 Build & Validation

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
7. **Test suite rewritten to test real code** — every mavlink-bridge test file now imports and exercises its corresponding production module: `test_mavlink_v2.py` (new, 22 tests, byte-for-byte vs pymavlink), `test_mavlink_crc.py` (rewritten, 14 tests, absorbed and deleted the pure-duplicate `test_mavlink_frame_generation.py`), `test_mission_signing.py` (rewritten, 28 tests, real RSA keypair), `test_telemetry_conversion.py` (rewritten, 18 tests), `test_mission_control_bridge.py` (rewritten, 29 tests), `test_mission_control_integration.py` (rewritten, 6 tests, real bound UDP socket + real background thread), `test_gps_spoof_integration.py` (rewritten, 8 tests, real UDP socket). `conftest.py`'s `MockMAVLinkBuilder` now delegates to the real `mavlink_v2` codec instead of reimplementing it.

**Result:** full mavlink-bridge suite is 125 tests passed, zero exclusions (previously `test_mavlink_crc.py`/`test_gps_spoof_integration.py` required a real ROS 2 environment to even collect; both are now stubbed and portable).

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

## Phase 2: Remaining ROS 2 Bridges — ✅ COMPLETE

Built directly on the corrected `mavlink_v2.py`, following the Phase 0 pattern: three new MAVLink messages (OBSTACLE_DISTANCE 330, COMMAND_LONG 76, COMMAND_ACK 77) were added to the codec and verified byte-for-byte against pymavlink before use, and every bridge test imports and exercises the real module (unit + real-socket integration) from the first commit.

### 2a. Fleet Manager Bridge — ✅ DONE
- **File:** `mavlink-bridge/fleet_manager_mavlink_bridge.py`
- **Input:** `/fleet/status` (std_msgs/String)
- **Output:** MAVLink STATUSTEXT — one per-drone mission-state/progress summary, de-duplicated so QGC isn't flooded (fleet_manager re-emits the full snapshot on every per-drone update).
- **Design note:** `/fleet/status` carries only mission state + waypoint progress (no position/attitude/battery — confirmed by reading fleet_manager_node and mission_executor_node.publish_mission_status), so STATUSTEXT summaries are the representation honest to the available data, chosen over an empty multi-vehicle HEARTBEAT multiplex.
- **Tests:** 13 unit + 3 integration (real UDP socket).

### 2b. Collision Avoidance Bridge — ✅ DONE
- **File:** `mavlink-bridge/collision_mavlink_bridge.py`
- **Input:** `/fmu/in/obstacle_distance` (px4_msgs/ObstacleDistance, from SF45 via `sf45_px4_node`). NB: `/fmu/in/...`, not `/fmu/out/...` — the latter appears only in stale docs and nothing publishes it.
- **Output:** MAVLink OBSTACLE_DISTANCE (330) — near 1:1 forward of the 72-sector, 5°, body-FRD, centimetre sweep.
- **Tests:** 7 unit + 3 integration (real UDP socket, pymavlink round-trip).

### 2c. Emergency Wipe Bridge — ✅ DONE
- **File:** `mavlink-bridge/emergency_wipe_mavlink_bridge.py`
- **Input:** MAVLink COMMAND_LONG (default command id MAV_CMD_USER_1 / 31010).
- **Output:** ROS 2 service call to `/emergency_wipe/execute` (std_srvs/Trigger), plus COMMAND_ACK back to QGC and `/emergency_wipe/status` → STATUSTEXT.
- **Safety gate:** the wipe fires only if BOTH param1 == a magic confirm value (default 1.0) AND the COMMAND_LONG confirmation byte >= 1; otherwise COMMAND_ACK=DENIED and no service call. This gate lives in the bridge because the Trigger service has an empty request and the node has no auth gate of its own.
- **Tests:** 14 unit + 4 integration (real bound socket + real receiver thread).

### 2d. Launch File Update — ✅ DONE
- **File:** `mavlink-bridge/launch_sas_qgc_integration.py` — all three new bridges added, plus `mavlink_router_node` (2e below).
- DDS-Security enclaves for all 7 mavlink-bridge nodes are now generated, tailored, and enforced via an opt-in `enable_security` arg on this launch file (2f, 2g below).

### 2e. MAVLink Router (inbound single-UDP-port limitation) — ✅ DONE
- **File:** `mavlink-bridge/mavlink_router_node.py`
- **Problem:** the outbound bridges (gps_spoof, telemetry, fleet, collision) only `connect()`/send, so they share port 14550 fine; but `mission_control_bridge` and `emergency_wipe_bridge` both need to BIND to receive, and two processes can't cleanly bind one UDP port — while QGC uses a single UDP comm link per vehicle, so both must be reachable on that one link.
- **Design:** the router binds the single external port QGC's comm link targets and fans every inbound datagram, byte-for-byte, out to a configurable list of downstream targets (both bridges' own internal ports); it relays anything a bridge sends back on its downstream socket out to whichever address most recently contacted the external socket. No MAVLink parsing is needed — pure byte relay, matching how a real MAVLink bus works (every node sees every packet; each bridge already filters for the message types it cares about).
- **Zero changes needed to either existing bridge:** both already (a) bind whatever port they're configured with, and (b) dynamically learn their reply address from whoever last contacted them. Pointed at the router instead of directly at QGC, that pre-existing mechanism (from the Phase 0 hardcoded-reply-address fix) keeps working unmodified — only the launch file's port wiring changed (`mission_control_bridge` and `emergency_wipe_bridge` now bind internal ports 14551/14556; the router binds the external 14550).
- **Tests:** 12 unit (pure relay logic, fake sockets) + 4 integration (real router, simulated QGC/bridge sockets) + 5 integration (the crown-jewel test: real router + real `MissionControlBridge` + real `EmergencyWipeMAVLinkBridge`, one simulated QGC socket, proving both a `MISSION_COUNT`→`MISSION_ACK` and a `COMMAND_LONG`→`COMMAND_ACK` roundtrip through the same shared link).

**Result:** full mavlink-bridge suite is now **197 tests passed, zero exclusions** (was 125 before Phase 2, 176 after the three bridges, 197 after the router).

### 2f. DDS-Security Enclaves for All 7 Nodes — ✅ DONE
- **Where:** `SAS/security/keystore/enclaves/` (gitignored — private key material, never committed; the CA and 5 pre-existing SAS enclaves already lived here).
- **Tooling:** the first real ROS 2 environment used in this whole project — WSL Ubuntu 24.04 with ROS 2 Jazzy + `ros-jazzy-sros2` installed, invoked via `ros2 security create_enclave`/`create_permission`. (Calling `wsl.exe` with inline `$variable`-containing commands from Git Bash silently mangles the variables before they cross the Windows/WSL boundary — writing the commands to a `.sh` file first and invoking `wsl bash /path/to/script.sh`, with `MSYS_NO_PATHCONV=1` to stop MSYS from rewriting `/mnt/...` paths, is what actually works.)
- **Enclave names:** `/gps_spoof_mavlink_bridge`, `/telemetry_mavlink_bridge`, `/mission_control_bridge`, `/fleet_manager_mavlink_bridge`, `/collision_mavlink_bridge`, `/emergency_wipe_mavlink_bridge`, `/mavlink_router_node` — matching each node's exact `super().__init__(...)` name.
- **Tailored, not default:** `create_enclave`'s auto-generated `permissions.xml` grants a broad wildcard template (`rt/*`, `rq/*Request`, etc.) — functionally correct but broader than the 5 pre-existing SAS enclaves' least-privilege convention (each scoped to its own exact topics). Replaced with a hand-authored policy (`ros2 security create_permission <keystore> <enclave> <policy.xml>`, in the sros2 `policy.xsd` schema — a different, higher-level schema than the raw DDS-Security `permissions.xml` grant format the tool ultimately emits) listing each bridge's real topics/services, derived directly from this session's own `create_subscription`/`create_publisher`/`create_client` calls:
  - `gps_spoof_mavlink_bridge`: subscribe `gps_spoof_alert`
  - `telemetry_mavlink_bridge`: subscribe `fmu/out/{vehicle_local_position,vehicle_attitude,vehicle_status,battery_status,sensor_gps}`
  - `mission_control_bridge`: publish `mission_executor/load_mission`, subscribe `mission_executor/status`
  - `fleet_manager_mavlink_bridge`: subscribe `fleet/status`
  - `collision_mavlink_bridge`: subscribe `fmu/in/obstacle_distance`
  - `emergency_wipe_mavlink_bridge`: subscribe `emergency_wipe/status`; service client to `emergency_wipe/execute` (publish+subscribe `rq/.../executeRequest` and `rr/.../executeReply`, matching the existing `emergency_wipe_node` server-side enclave's exact pattern)
  - `mavlink_router_node`: nothing — it has zero ROS topics (pure UDP relay), so its grant is just the standard `ros_discovery_info` rule every enclave gets automatically
- **Verified, not just generated:** all 7 certs verified to chain to `identity_ca.cert.pem` via `openssl verify`; all 7 `permissions.p7s` and `governance.p7s` verified as validly-signed CMS/S-MIME messages via `openssl cms -verify` against `permissions_ca.cert.pem` (these are MIME-wrapped, base64-encoded PKCS7 SignedData, not raw DER — `-inform DER` fails even on the pre-existing, known-good enclaves, which is what first made this look like a real problem before the format was correctly identified).
- **Not yet done (at the time):** `launch_sas_qgc_integration.py` didn't yet set `ROS_SECURITY_*` env vars — resolved in 2g below.

### 2g. Wire Security Into `launch_sas_qgc_integration.py` — ✅ DONE
- **File:** `mavlink-bridge/launch_sas_qgc_integration.py`
- **Added:** an opt-in `enable_security` launch arg (default `false`, so existing dev/test runs without a configured keystore/SROS2 aren't silently broken) and a `keystore_path` arg (default: `SAS/security/keystore` resolved as a sibling directory of `mavlink-bridge/`, overridable). When `enable_security:=true`, three `SetEnvironmentVariable` actions — `ROS_SECURITY_ENABLE=true`, `ROS_SECURITY_STRATEGY=Enforce`, `ROS_SECURITY_KEYSTORE=<keystore_path>` — apply to every node in the file (the 7 mavlink-bridge nodes plus the SAS-package `gps_spoof_detector_node`), each wrapped in `IfCondition(LaunchConfiguration('enable_security'))`, positioned before any `Node` action so the environment is set before any process spawns. Exactly the same three variables `SAS/launch/secure_launch.py` sets, so no new security model was introduced.
- **Verified with the real `ros2 launch` tool (WSL):** `--show-args` shows both new arguments with correct defaults/descriptions; a real launch attempt with `enable_security:=true` prints a log line with the correctly-resolved keystore path and proceeds past all the new actions, failing only on `package 'SAS' not found` (expected — SAS/mavlink-bridge aren't colcon-installed packages in this bare ROS 2 install, the separate live-validation gap); the default (`enable_security:=false`) run prints no security-related log line at all, confirming the conditional actually gates the behavior rather than always firing.

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
| Fleet Manager Bridge | 13 unit + 3 integration | ✅ Yes (real bridge, real UDP socket) | ✅ Done (Phase 2) |
| Collision Bridge | 7 unit + 3 integration | ✅ Yes (real bridge, real UDP socket, pymavlink round-trip) | ✅ Done (Phase 2) |
| Emergency Wipe Bridge | 14 unit + 4 integration | ✅ Yes (real bound socket + real receiver thread) | ✅ Done (Phase 2) |
| MAVLink Router | 12 unit + 9 integration (incl. real 3-node topology) | ✅ Yes (real router, real bridges, real sockets) | ✅ Done (Phase 2) |

**Totals:** mavlink-bridge suite 201/201 passing (zero exclusions); SAS unit suite 1215/1215 passing (3 pre-existing skips).

### Test Verification vs SAS Repo

Cross-checked and confirmed no topical redundancy between mavlink-bridge tests and SAS's own test suite (SAS tests executor/node *logic*; bridge tests target MAVLink *translation* — different layers). This finding from the original roadmap still holds. The larger, previously-unstated problem was never redundancy — it was that most bridge tests didn't test anything real at all, regardless of redundancy.

---

## Phase 5: Live ROS 2 Build & Validation — IN PROGRESS

The first real ROS 2 environment used in this whole project (WSL Ubuntu 24.04, ROS 2 Jazzy, `colcon`) — as opposed to the stubbed `rclpy`/`px4_msgs` every unit/integration test uses — surfaced real bugs that stubs cannot catch, since a stub supplies exactly the attributes the code expects rather than modeling what real tooling and real message schemas actually contain.

### Environment discovered already partially set up (not created this session)
- `~/PX4-Autopilot` already cloned, PX4 SITL already built (`build/px4_sitl_default/bin/px4`, valid ELF binary, built 2026-06-28).
- A colcon workspace at `~/ros2_ws` already had `my_python_package` (SAS) built (from 2026-04-02).
- Gazebo Harmonic already installed (`gz-sim8-cli` etc.).
- QGroundControl confirmed installed as a native Windows application (`C:\Program Files\QGroundControl\bin\QGroundControl.exe`).
- None of the above was rebuilt or altered beyond adding `mavlink-bridge` and `px4_msgs` to the same workspace; nothing has been launched.

### Packaging bugs found and fixed getting `colcon build` to succeed
None of these were caught by 201 passing unit/integration tests, since none of them exercise packaging or `ros2 launch`/`ros2 run` resolution — a genuinely different failure surface than anything covered so far.

1. **`package='SAS'`** in `launch_sas_qgc_integration.py`, `launch_gps_spoof_qgc.py`, and `tests/integration/launch_full_integration.py` — the real registered package name is `my_python_package` (per `SAS/package.xml`/`setup.py`); `SAS` is only the repo directory name. Fixed to `package='my_python_package'` in all three.
2. **`mavlink-bridge/package.xml` missing `<export><build_type>ament_python</build_type></export>`** — colcon couldn't tell it was a Python package and tried to configure it as CMake. Added the export block (matching `SAS/package.xml`) and `<depend>std_srvs</depend>` (used by `emergency_wipe_mavlink_bridge.py`, previously undeclared).
3. **`mavlink-bridge/setup.py` missing the standard `data_files` block** (ament_index resource_index registration + launch file installation) — without it, neither `ros2 pkg list` nor `ros2 launch mavlink-bridge <file>.py` could find the package post-install. Added the block plus a `resource/mavlink-bridge` marker file; the two top-level launch files are installed by name into `share/mavlink-bridge/launch/` without moving them in the source tree (avoids touching the many existing doc references to their current paths).
4. **`mavlink-bridge` missing `setup.cfg`** — without the `[develop]`/`[install]` `script_dir`/`install_scripts` override (present in `SAS/setup.cfg`), setuptools installed all 8 console_scripts into a generic `bin/` instead of ROS 2's expected `lib/mavlink-bridge/`, so `ros2 run`/`ros2 pkg executables mavlink-bridge` found zero executables despite a "successful" build. Added `setup.cfg` matching `SAS`'s convention.

**Result:** `colcon build --symlink-install --packages-select my_python_package mavlink-bridge` succeeds cleanly; `ros2 pkg executables mavlink-bridge` lists all 8 executables; `ros2 launch mavlink-bridge launch_sas_qgc_integration.py --show-args` resolves the launch file by package name for the first time in this project.

### `px4_msgs` built from source — revealed real field-mismatch bugs
`px4_msgs` was entirely missing from the environment (needed at runtime by `telemetry_mavlink_bridge.py`, `collision_mavlink_bridge.py`, several SAS nodes). Cloned (`PX4/px4_msgs`, main branch, v1.17.0) and built via colcon. Importing the real messages for the first time (rather than test stubs) surfaced genuine bugs in `telemetry_mavlink_bridge.py` — see Phase 0/1's bug list update below, since this is a code-correctness finding, not a packaging one:

- `VehicleStatus.system_status`, `VehicleStatus.load`, `VehicleAttitude.rollspeed`/`pitchspeed`/`yawspeed`, and `BatteryStatus.energy_consumed_j` are not real px4_msgs fields — confirmed absent as far back as v1.14.0 (2023), not a version-specific rename. Would have raised `AttributeError` on the very first real `VehicleAttitude`/`VehicleStatus` callback, at any point in this bridge's history. **Fixed**: angular rates now sourced from a new `VehicleAngularVelocity` subscription (`xyz`), CPU load from a new `Cpuload` subscription (`load`), heartbeat state simplified to arming-state-only, `energy_consumed` reported as MAVLink's documented unknown sentinel (`-1`). 4 new regression tests (`test_telemetry_conversion.py`: 18 → 22).
- Checked SAS's own px4_msgs usage for the same class of bug (`grep` for the four confirmed-bad field names across `SAS/my_python_package`) — no other occurrences found.

### Not yet done
Nothing has been launched — no node started, no PX4 SITL run, no QGroundControl opened, no `ros2 launch` beyond `--show-args`. That's the next step.

---

## Implementation Priority (Revised)

### Done
- ✅ Phase 0: MAVLink protocol correctness, all 3 bridges, mission signing wired in, secure launch fixed, full test suite rewritten
- ✅ Phase 1.5: Gesture Safety Gating
- ✅ Phase 2: Fleet Manager, Collision, and Emergency Wipe bridges, built on the corrected `mavlink_v2.py` (OBSTACLE_DISTANCE/COMMAND_LONG/COMMAND_ACK added and pymavlink-verified)
- ✅ Phase 2 follow-up: inbound single-UDP-port limitation resolved via `mavlink_router_node.py`, with no code changes to either existing inbound bridge
- ✅ Phase 2 follow-up: DDS-Security enclaves generated and tailored to least-privilege for all 7 mavlink-bridge nodes, verified cryptographically
- ✅ Phase 2 follow-up: security wired into `launch_sas_qgc_integration.py` via an opt-in `enable_security` arg, verified with the real `ros2 launch` tool
- ✅ Phase 5 (in progress): both packages now `colcon build` cleanly in a real ROS 2 workspace; 4 real packaging bugs found and fixed; `px4_msgs` built from source, revealing (and fixing) real field-mismatch bugs in `telemetry_mavlink_bridge.py` that no stub-based test could have caught

### Now / Next
- **Actually launch the stack.** PX4 SITL is already built, QGroundControl is already installed, both ROS 2 packages now colcon-build cleanly — the next step is a real `ros2 launch mavlink-bridge launch_sas_qgc_integration.py` run, the first genuine end-to-end execution this project has had. This is also the first chance to confirm `enable_security:=true` actually starts nodes under DDS-Security enforcement, not just sets env vars correctly.

### Later
- Phase 3: QGC Plugin (separate session, requires Qt/QML/C++)

---

## Key Files Summary

### Existing (all complete)
```
mavlink-bridge/
├── mavlink_v2.py                       ✅ verified MAVLink v2 codec (+ OBSTACLE_DISTANCE/COMMAND_LONG/COMMAND_ACK)
├── gps_spoof_mavlink_bridge.py         ✅ migrated to mavlink_v2
├── telemetry_mavlink_bridge.py         ✅ migrated to mavlink_v2
├── mission_control_bridge.py           ✅ migrated to mavlink_v2; real upload handshake; reply-address fix
├── fleet_manager_mavlink_bridge.py     ✅ Phase 2: /fleet/status → STATUSTEXT summaries
├── collision_mavlink_bridge.py         ✅ Phase 2: ObstacleDistance → OBSTACLE_DISTANCE
├── emergency_wipe_mavlink_bridge.py    ✅ Phase 2: COMMAND_LONG → wipe service (two-factor gated)
├── mavlink_router_node.py              ✅ Phase 2: fans QGC's one link to both inbound bridges
└── tests/                              ✅ 201 tests, all import real modules

SAS/
├── security/mission_signer.py          ✅ correct
├── my_python_package/mission_verifier.py  ✅ correct, wired into load_mission_callback
├── launch/secure_launch.py             ✅ correct env vars; keystore now has enclaves for all 12 SAS/mavlink-bridge nodes
├── security/keystore/                  ✅ 7 new tailored enclaves (gitignored, not committed -- private key material)
├── my_python_package/fleet_manager_node.py       ✅ gesture-gated (Phase 1.5)
├── my_python_package/navigation_control_node.py  ✅ gesture-gated (Phase 1.5)
└── my_python_package/mission_executor_node.py    ✅ gesture-gated (Phase 1.5) + signature verification (Phase 1)
```

### Created in Phase 2 ✅
```
mavlink-bridge/
├── fleet_manager_mavlink_bridge.py     + tests/unit/test_fleet_manager_bridge.py, tests/integration/test_fleet_manager_integration.py
├── collision_mavlink_bridge.py         + tests/unit/test_collision_bridge.py, tests/integration/test_collision_integration.py
├── emergency_wipe_mavlink_bridge.py    + tests/unit/test_emergency_wipe_bridge.py, tests/integration/test_emergency_wipe_integration.py
└── mavlink_router_node.py              + tests/unit/test_mavlink_router.py, tests/integration/test_mavlink_router_integration.py,
                                           tests/integration/test_router_with_real_bridges_integration.py
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
- [x] DDS-Security enclaves generated and tailored to least-privilege for the 7 mavlink-bridge nodes
- [x] Security wired into `launch_sas_qgc_integration.py` (opt-in `enable_security` arg, verified with the real `ros2 launch` tool)
- [x] Both packages (`my_python_package`, `mavlink-bridge`) colcon-build cleanly in a real ROS 2 workspace, with all executables/launch files discoverable by package name (Phase 5)
- [x] `px4_msgs` built from source and importable by the bridges that need it at runtime
- [ ] `enable_security:=true` actually run through to nodes starting under DDS-Security enforcement (verified so far only that the env vars/log message are correct — the packages weren't installed yet when that was checked; worth re-verifying now that they are)
- [ ] Nodes actually launched and run against PX4 SITL / QGroundControl (both already built/installed in this environment, per Phase 5, but never started)

---

## Notes

1. **Verification methodology going forward:** any new MAVLink payload builder should be checked byte-for-byte against `pymavlink.dialects.v20.common` before being considered correct — this is what caught the Phase 0 bugs, and hand-derivation from the XML spec or from memory is not reliable enough on its own (two bugs were found this way even in the newly-written `mavlink_v2.py`'s first draft).
2. **Test methodology going forward:** a test file's job is to import and exercise the real module. A test suite that reimplements the logic it's meant to verify provides no signal — this is exactly how the frame-header bug went undetected across ~130 "passing" tests.
3. **Backward Compatibility:** all Phase 1.5 gesture-gating changes are additive to existing SAS nodes; no removed functionality.
4. **Git Strategy:** logical commits per component, descriptive messages, no long multi-paragraph commit bodies. **Branch:** all work described in this roadmap now lives on `develop` (both the outer repo and the `SAS` submodule each have their own `develop`) — the outer repo's full history had been committed to `master` by mistake; `master` was left untouched on GitHub, and `develop` was created alongside it with the same history plus everything through Phase 0/1/1.5. Use `develop` going forward.
5. **Test-suite order dependency, found and fixed:** five mavlink-bridge test files each installed their own competing ROS 2 stub into `sys.modules`; whichever loaded first silently won for the rest of the process, so `pytest tests/` passing 124/124 was masking a real fragility that surfaced the moment tests were run in a different order (e.g. unit file before its integration counterpart), raising `TypeError: object.__init__() takes exactly one argument`. Fixed by consolidating all ROS 2 stubbing into a single always-fully-capable stub in `tests/conftest.py`, which pytest loads before any test module regardless of selection/order. See `IMPLEMENTATION_STATUS.md` Part 2 for detail.
6. **Two more quick fixes, found and fixed:** `_get_battery_voltage()` in `telemetry_mavlink_bridge.py` divided an already-in-volts cell-voltage sum by 1000 again, reporting SYS_STATUS battery voltage ~1000x too low — fixed, with a new regression test. Separately, `mavlink-bridge/setup.py`'s dead `test_gps_spoof_alert_generator` console_scripts entry now points at a real file — a standalone CLI tool that publishes synthetic `/gps_spoof_alert` messages for the manual QGC testing phases in `INTEGRATION_TEST_CHECKLIST.md`. A root-level `mavlink-bridge/conftest.py` (`collect_ignore`) keeps pytest from trying to collect that script as a test module, since its name is fixed by the entry point and can't be changed to avoid matching pytest's `test_*.py` discovery pattern.
7. **Inbound single-UDP-port limitation (Phase 2), found and resolved:** each bridge is its own process with its own socket. Outbound bridges only `connect()`/send, so any number share the QGC port (14550). But `mission_control_bridge` and the new `emergency_wipe_bridge` both `bind()` to receive, and two processes can't cleanly bind one UDP port — while QGC uses one comm link per vehicle, so both must be reachable on it. Resolved with `mavlink_router_node.py`: it binds the single external port and fans every inbound datagram out to both bridges' own internal ports, relaying their replies back to whichever address most recently contacted it. Neither existing bridge needed a code change — both already bind whatever port they're configured with and dynamically learn their reply address from the last sender, a mechanism from the Phase 0 hardcoded-reply-address fix that keeps working unmodified when pointed at the router instead of QGC directly. Verified with a real 3-node topology test (real router + real mission_control_bridge + real emergency_wipe_bridge, one simulated QGC socket).
8. **gps_spoof_mavlink_bridge severity numbering was transposed, found and fixed:** its `MAVSeverity` enum had `INFO = 0` (really EMERGENCY in MAV_SEVERITY) and `CRITICAL = 5` (really NOTICE), so a genuine CRITICAL spoof alert transmitted at NOTICE priority and an INFO alert at EMERGENCY — backwards in QGC's color-coding. Fixed to the spec-correct values (matching the fleet/emergency-wipe bridges' enums). Every call site in the bridge and its tests referenced the enum symbolically, so no other code needed to change; added `TestSeverityMatchesMavlinkSpec` to `test_mavlink_crc.py` to pin the actual wire values going forward.
