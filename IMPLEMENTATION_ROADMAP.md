# SAS-QGC Integration Implementation Roadmap

## Executive Summary
Full implementation of SAS_QGC_Integration_Plan per user request, excluding gesture_bridge_node.

**Phases:**
1. **Phase 1 (In Progress):** Security & Mission Signing
2. **Phase 2:** Remaining ROS 2 Bridges  
3. **Phase 3:** QGC Custom Plugin
4. **Phase 4:** Comprehensive Testing

---

## Phase 1: Security & Mission Signing Integration

### Components to Create
1. **mission_signer.py** (security module)
   - Sign mission JSON with private key
   - Support RSA-2048 and ECDSA
   - Location: `SAS/security/mission_signer.py`

2. **mission_verifier.py** (verification module)
   - Verify mission signatures
   - Reject tampered missions
   - Location: `SAS/my_python_package/mission_verifier.py`

3. **Mission Control Bridge Enhancement**
   - Integrate signing into bridge
   - Verify incoming missions from QGC
   - Location: `mavlink-bridge/mission_control_bridge.py` (update)

4. **Secure Launch Config**
   - Enable ROS 2 DDS-Security
   - Deploy keystore
   - Location: `SAS/launch/secure_launch.py`

### Tests Required
- Mission signing/verification unit tests
- Bridge integration with security
- End-to-end signing flow

---

## Phase 2: Remaining ROS 2 Bridges

### 2a. Fleet Manager Bridge
- **File:** `mavlink-bridge/fleet_manager_mavlink_bridge.py`
- **Input:** `/fleet/status` (aggregated position, battery, mission progress)
- **Output:** MAVLink messages (fleet aggregation)
- **Tests:** 15+ tests

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
- Add all three new bridges
- Enable secure communication

---

## Phase 3: QGC Custom Plugin (Next Phase)

### Components (Requires C++/QML)
1. **SASPlugin.h/cpp**
   - Extends QGCCorePlugin
   - Intercepts MAVLink messages
   - 400+ lines

2. **SASFleetView.qml**
   - Fleet status visualization
   - Formation selector UI
   - GPS spoofing alert banner
   - Collision risk heatmap
   - 500+ lines

3. **CMakeLists.txt**
   - Integration with QGC build system
   - Plugin registration

**Estimated Effort:** 2-3 weeks (requires Qt/QML expertise)

---

## Phase 4: Comprehensive Testing

### Test Coverage by Component
| Component | Unit Tests | Integration Tests | Total |
|-----------|-----------|------------------|-------|
| Mission Signing | 12 | 8 | 20 |
| Fleet Manager Bridge | 15 | 10 | 25 |
| Collision Bridge | 12 | 8 | 20 |
| Emergency Wipe Bridge | 10 | 6 | 16 |
| Security Integration | 8 | 6 | 14 |
| **TOTAL (Phase 1-2)** | 57 | 38 | **95** |

### Test Verification vs SAS Repo
- Cross-check all tests against `SAS/tests/`
- Ensure no redundancy with existing unit tests
- Document findings in `REDUNDANCY_ANALYSIS.md`

---

## Implementation Priority

### Now (Session 1)
- ✅ Phase 1: Security & Mission Signing
- ✅ Phase 2: Remaining bridges (a-d)
- ✅ Phase 4: All tests with redundancy checks

### Later (Session 2+)
- ⏳ Phase 3: QGC Plugin

---

## Key Files Summary

### New Files to Create
```
SAS/
├── security/
│   └── mission_signer.py              (150 lines)
└── my_python_package/
    └── mission_verifier.py            (120 lines)

mavlink-bridge/
├── fleet_manager_mavlink_bridge.py    (450 lines)
├── collision_mavlink_bridge.py        (400 lines)
├── emergency_wipe_mavlink_bridge.py   (350 lines)
├── tests/
│   ├── unit/
│   │   ├── test_mission_signing.py    (200 lines)
│   │   ├── test_fleet_bridge.py       (250 lines)
│   │   ├── test_collision_bridge.py   (200 lines)
│   │   └── test_emergency_bridge.py   (150 lines)
│   └── integration/
│       ├── test_signing_integration.py (200 lines)
│       ├── test_fleet_integration.py   (200 lines)
│       ├── test_collision_integration.py (150 lines)
│       └── test_emergency_integration.py (100 lines)
└── launch/
    ├── secure_launch.py               (100 lines)
    └── launch_sas_qgc_integration.py  (update: add 3 bridges)

Documentation/
├── SECURITY_INTEGRATION.md            (integration guide)
├── REDUNDANCY_ANALYSIS.md             (test redundancy report)
└── IMPLEMENTATION_SUMMARY.md          (final status)
```

---

## Execution Timeline

**Estimated Duration:** 4-6 hours for Phases 1-2 + tests
- Security module: 45 min
- Mission signer/verifier: 45 min
- Fleet manager bridge: 60 min
- Collision bridge: 60 min
- Emergency wipe bridge: 45 min
- Tests + redundancy check: 90 min
- Documentation: 30 min

**QGC Plugin (Phase 3):** 2-3 weeks (separate session)

---

## Success Criteria

### Phase 1-2 Complete When:
- [ ] All 95 tests passing
- [ ] Zero redundancy with SAS tests (documented)
- [ ] All bridges launch successfully
- [ ] Secure launch configuration working
- [ ] Documentation complete
- [ ] Git commits clean and well-organized

### Phase 3 Complete When:
- [ ] QGC plugin compiles without errors
- [ ] Custom QML renders correctly
- [ ] Plugin correctly hooks MAVLink messages
- [ ] Fleet visualization working
- [ ] Mission signing UI integrated
- [ ] Emergency buttons functional

---

## Dependencies & Prerequisites

### For Phase 1-2:
- ✅ ROS 2 (available in WSL)
- ✅ Python 3.11+ (available)
- ✅ cryptography library (for signing)
- ✅ struct/json (built-in)

### For Phase 3:
- ⏳ Qt 5.15 or Qt 6.x
- ⏳ QGC source code
- ⏳ C++ compiler
- ⏳ QML knowledge

---

## Notes

1. **Security Implications:** Mission signing uses RSA-2048 with SHA-256. Keys stored in `SAS/security/keystore/`.

2. **Backward Compatibility:** All new bridges are additive; existing nodes unchanged.

3. **Testing Strategy:** Unit tests verify logic in isolation; integration tests verify ROS 2 topic flow.

4. **Documentation:** Each bridge includes inline docstrings + usage examples.

5. **Git Strategy:** Logical commits per component, descriptive messages.
