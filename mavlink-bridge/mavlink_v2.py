
#!/usr/bin/env python3
"""
MAVLink 2.0 wire protocol codec, shared by all SAS-QGC bridges.

Every field layout and CRC_EXTRA value in this module was cross-checked
against `pymavlink.dialects.v20.common` (the reference implementation) rather
than hand-derived from the XML spec, to avoid silent transcription errors.

MAVLink 2.0 frame layout (this is NOT the same as MAVLink 1.0):

    byte 0        magic (0xFD)
    byte 1        payload length
    byte 2        incompat_flags
    byte 3        compat_flags
    byte 4        seq
    byte 5        sysid
    byte 6        compid
    bytes 7-9     msgid (24-bit, little-endian)
    bytes 10..N   payload
    bytes N..N+2  checksum (CRC-16/MCRF4XX over bytes[1:] + payload + crc_extra)

A prior implementation of these bridges used a 7-byte header copied from
MAVLink 1.0 conventions (STX, LEN, INCOMPAT_FLAGS, MSG_ID, SYSID, COMPID, SEQ)
with an 8-bit message ID. That frame is not valid MAVLink 2.0 and is not
parseable by QGroundControl or any spec-compliant MAVLink implementation.
This module replaces that logic.
"""

import struct
from enum import IntEnum
from typing import Optional, NamedTuple


MAVLINK_STX = 0xFD

# CRC_EXTRA per message, verified via pymavlink (crc_extra attribute of each
# MAVLink_<name>_message class in pymavlink.dialects.v20.common).
CRC_EXTRA = {
    0: 50,      # HEARTBEAT
    1: 124,     # SYS_STATUS
    30: 39,     # ATTITUDE
    32: 185,    # LOCAL_POSITION_NED
    33: 104,    # GLOBAL_POSITION_INT
    39: 254,    # MISSION_ITEM
    40: 230,    # MISSION_REQUEST
    42: 28,     # MISSION_CURRENT
    43: 132,    # MISSION_REQUEST_LIST
    44: 221,    # MISSION_COUNT
    46: 11,     # MISSION_ITEM_REACHED
    47: 153,    # MISSION_ACK
    51: 196,    # MISSION_REQUEST_INT
    73: 38,     # MISSION_ITEM_INT
    147: 154,   # BATTERY_STATUS
    253: 83,    # STATUSTEXT
}

# Message IDs used across the SAS-QGC bridges.
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_MISSION_ITEM = 39
MAVLINK_MSG_ID_MISSION_REQUEST = 40
MAVLINK_MSG_ID_MISSION_CURRENT = 42
MAVLINK_MSG_ID_MISSION_REQUEST_LIST = 43
MAVLINK_MSG_ID_MISSION_COUNT = 44
MAVLINK_MSG_ID_MISSION_ITEM_REACHED = 46
MAVLINK_MSG_ID_MISSION_ACK = 47
MAVLINK_MSG_ID_MISSION_REQUEST_INT = 51
MAVLINK_MSG_ID_MISSION_ITEM_INT = 73
MAVLINK_MSG_ID_BATTERY_STATUS = 147
MAVLINK_MSG_ID_STATUSTEXT = 253

HEADER_LEN = 10  # bytes 0..9, i.e. everything before the payload
CRC_LEN = 2


class MAVMissionResult(IntEnum):
    """MAV_MISSION_RESULT values used in MISSION_ACK.type."""
    ACCEPTED = 0
    ERROR = 1
    UNSUPPORTED_FRAME = 2
    UNSUPPORTED = 3
    NO_SPACE = 4
    INVALID = 5


class MissionType(IntEnum):
    """MAV_MISSION_TYPE values."""
    MISSION = 0
    FENCE = 1
    RALLY = 2
    ALL = 255


class ParsedFrame(NamedTuple):
    msg_id: int
    system_id: int
    component_id: int
    sequence: int
    payload: bytes
    crc: int
    valid: bool


def compute_crc(data: bytes, msg_id: int) -> int:
    """Compute MAVLink CRC-16/MCRF4XX (X.25) with message CRC_EXTRA.

    `data` must be bytes[1:] of the frame (everything after STX) through the
    end of the payload -- i.e. it does NOT include STX or the CRC itself.
    """
    crc_extra = CRC_EXTRA.get(msg_id, 0)
    crc = 0xFFFF

    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF

    tmp = crc_extra ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    crc &= 0xFFFF

    return crc


def build_frame(msg_id: int, seq: int, payload: bytes,
                 system_id: int, component_id: int) -> bytes:
    """Build a complete, spec-compliant MAVLink 2.0 frame.

    Per the MAVLink 2.0 spec, trailing zero bytes MAY be (and conventionally
    are) truncated from the payload to save bandwidth; compliant receivers
    zero-fill missing trailing bytes when decoding. This matches pymavlink's
    encoder behavior byte-for-byte and is required for our CRCs to match a
    real MAVLink peer's expectations.
    """
    payload = payload.rstrip(b'\x00')

    incompat_flags = 0x00
    compat_flags = 0x00

    header_and_payload = struct.pack(
        '<BBBBBBBBBB',
        MAVLINK_STX,
        len(payload),
        incompat_flags,
        compat_flags,
        seq & 0xFF,
        system_id & 0xFF,
        component_id & 0xFF,
        msg_id & 0xFF,
        (msg_id >> 8) & 0xFF,
        (msg_id >> 16) & 0xFF,
    ) + payload

    crc = compute_crc(header_and_payload[1:], msg_id)
    return header_and_payload + struct.pack('<H', crc)


def parse_frame(data: bytes) -> Optional[ParsedFrame]:
    """Parse a MAVLink 2.0 frame. Returns None if not a well-formed v2 frame."""
    if len(data) < HEADER_LEN + CRC_LEN:
        return None
    if data[0] != MAVLINK_STX:
        return None

    payload_len = data[1]
    seq = data[4]
    system_id = data[5]
    component_id = data[6]
    msg_id = data[7] | (data[8] << 8) | (data[9] << 16)

    expected_len = HEADER_LEN + payload_len + CRC_LEN
    if len(data) < expected_len:
        return None

    payload = data[HEADER_LEN:HEADER_LEN + payload_len]
    crc = struct.unpack('<H', data[HEADER_LEN + payload_len:expected_len])[0]

    computed_crc = compute_crc(data[1:HEADER_LEN + payload_len], msg_id)
    valid = (crc == computed_crc)

    return ParsedFrame(
        msg_id=msg_id,
        system_id=system_id,
        component_id=component_id,
        sequence=seq,
        payload=payload,
        crc=crc,
        valid=valid,
    )


# ===== Payload builders (field order/width verified against pymavlink) =====

def build_heartbeat(type_: int, autopilot: int, base_mode: int,
                     custom_mode: int, system_status: int,
                     mavlink_version: int = 3) -> bytes:
    """HEARTBEAT (id 0): custom_mode:u32, type:u8, autopilot:u8, base_mode:u8,
    system_status:u8, mavlink_version:u8."""
    return struct.pack('<IBBBBB', custom_mode, type_, autopilot, base_mode,
                        system_status, mavlink_version)


def build_sys_status(sensors_present: int, sensors_enabled: int, sensors_health: int,
                      load: int, voltage_battery: int, current_battery: int,
                      battery_remaining: int, drop_rate_comm: int, errors_comm: int,
                      errors_count1: int, errors_count2: int, errors_count3: int,
                      errors_count4: int) -> bytes:
    """SYS_STATUS (id 1). Wire order (verified): the three sensor bitmasks are
    u32 (NOT u16), and battery_remaining (i8) is the LAST field, not
    positioned next to current_battery."""
    return struct.pack(
        '<IIIHHhHHHHHHb',
        sensors_present, sensors_enabled, sensors_health,
        load, voltage_battery, current_battery,
        drop_rate_comm, errors_comm,
        errors_count1, errors_count2, errors_count3, errors_count4,
        battery_remaining,
    )


def build_attitude(time_boot_ms: int, roll: float, pitch: float, yaw: float,
                    rollspeed: float, pitchspeed: float, yawspeed: float) -> bytes:
    """ATTITUDE (id 30): time_boot_ms:u32, then 6 floats."""
    return struct.pack('<Iffffff', time_boot_ms, roll, pitch, yaw,
                        rollspeed, pitchspeed, yawspeed)


def build_global_position_int(time_boot_ms: int, lat: int, lon: int, alt: int,
                               relative_alt: int, vx: int, vy: int, vz: int,
                               hdg: int) -> bytes:
    """GLOBAL_POSITION_INT (id 33): time_boot_ms:u32, lat/lon/alt/relative_alt:i32,
    vx/vy/vz:i16, hdg:u16."""
    return struct.pack('<IiiiihhhH', time_boot_ms, lat, lon, alt,
                        relative_alt, vx, vy, vz, hdg)


def build_battery_status(id_: int, battery_function: int, type_: int,
                          temperature: int, voltages: list, current_battery: int,
                          current_consumed: int, energy_consumed: int,
                          battery_remaining: int, time_remaining: int = 0,
                          charge_state: int = 0) -> bytes:
    """BATTERY_STATUS (id 147). Verified wire order is NOT declaration order:
    current_consumed(i32), energy_consumed(i32), temperature(i16),
    voltages[10](u16 each), current_battery(i16), id(u8), battery_function(u8),
    type(u8), battery_remaining(i8), then extensions: time_remaining(i32),
    charge_state(u8), voltages_ext[4](u16), mode(u8), fault_bitmask(u32).

    `voltages` must be an iterable of per-cell millivolts; padded/truncated to
    10 entries (0xFFFF marks "no cell" in the real spec, but 0 is accepted by
    QGC for unused trailing cells).
    """
    cells = list(voltages)[:10] + [0] * max(0, 10 - len(voltages))
    return struct.pack(
        '<iih10HhBBBb',
        current_consumed, energy_consumed, temperature,
        *cells,
        current_battery,
        id_, battery_function, type_, battery_remaining,
    ) + struct.pack(
        '<iB4HBI',
        time_remaining, charge_state, 0, 0, 0, 0, 0, 0,
    )


def build_statustext(text: str, severity: int, msg_id_field: int = 0,
                      chunk_seq: int = 0) -> bytes:
    """STATUSTEXT (id 253): severity:u8, text:char[50], id:u16, chunk_seq:u8."""
    text_bytes = text.encode('ascii', errors='replace')[:50].ljust(50, b'\x00')
    return struct.pack('<B50sHB', severity, text_bytes, msg_id_field, chunk_seq)


def build_mission_item_int(seq: int, frame: int, command: int, current: int,
                            autocontinue: int, param1: float, param2: float,
                            param3: float, param4: float, x: int, y: int,
                            z: float, target_system: int = 1,
                            target_component: int = 1,
                            mission_type: int = MissionType.MISSION) -> bytes:
    """MISSION_ITEM_INT (id 73). Verified wire order:
    param1-4:float, x:i32, y:i32, z:float, seq:u16, command:u16,
    target_system:u8, target_component:u8, frame:u8, current:u8,
    autocontinue:u8, mission_type:u8 (extension).

    x/y are latitude/longitude scaled by 1e7 (matches QGC's default mission
    protocol, unlike the float-native MISSION_ITEM message id=39)."""
    return struct.pack(
        '<ffffiifHHBBBBBB',
        param1, param2, param3, param4, x, y, z,
        seq, command,
        target_system, target_component, frame, current, autocontinue,
        mission_type,
    )


def parse_mission_item_int(payload: bytes) -> Optional[dict]:
    """Parse MISSION_ITEM_INT (id 73) payload per verified wire order."""
    if len(payload) < 37:
        # MAVLink 2 payload truncation: zero-fill missing trailing bytes.
        # mission_type is the only extension field and defaults to 0/MISSION.
        payload = payload.ljust(37, b'\x00')
    (param1, param2, param3, param4, x, y, z, seq, command,
     target_system, target_component, frame, current, autocontinue) = struct.unpack(
        '<ffffiifHHBBBBB', payload[:37]
    )
    mission_type = payload[37] if len(payload) > 37 else MissionType.MISSION
    return {
        'sequence': seq,
        'frame': frame,
        'command': command,
        'current': current,
        'autocontinue': autocontinue,
        'params': [param1, param2, param3, param4],
        'target_system': target_system,
        'target_component': target_component,
        'mission_type': mission_type,
        'position': {
            'latitude': x / 1e7,
            'longitude': y / 1e7,
            'altitude': z,
        },
    }


def build_mission_ack(result: int, target_system: int = 255,
                       target_component: int = 0,
                       mission_type: int = MissionType.MISSION) -> bytes:
    """MISSION_ACK (id 47): target_system:u8, target_component:u8, type:u8,
    mission_type:u8 (extension). All four fields are 1 byte -- there is no
    32-bit padding field in the real message."""
    return struct.pack('<BBBB', target_system, target_component, result, mission_type)


def parse_mission_ack(payload: bytes) -> Optional[dict]:
    # MAVLink 2 payload truncation strips trailing zero bytes; since result=0
    # (ACCEPTED) and mission_type=0 are the common case, a real ACK can and
    # does arrive shorter than 3 bytes (e.g. target_component=0 too). Zero-fill
    # rather than rejecting, matching parse_mission_item_int's approach.
    payload = payload.ljust(3, b'\x00')
    target_system, target_component, result = struct.unpack('<BBB', payload[:3])
    mission_type = payload[3] if len(payload) > 3 else MissionType.MISSION
    return {
        'target_system': target_system,
        'target_component': target_component,
        'result': result,
        'mission_type': mission_type,
    }


def build_mission_current(seq: int, total: int = 0, mission_state: int = 0,
                           mission_mode: int = 0) -> bytes:
    """MISSION_CURRENT (id 42): seq:u16, total:u16, mission_state:u8,
    mission_mode:u8. NOT (waypoint, timestamp) -- MISSION_CURRENT carries no
    timestamp field in the real spec."""
    return struct.pack('<HHBB', seq, total, mission_state, mission_mode)


def build_mission_count(count: int, target_system: int = 255,
                         target_component: int = 0,
                         mission_type: int = MissionType.MISSION) -> bytes:
    """MISSION_COUNT (id 44): count:u16, target_system:u8, target_component:u8,
    mission_type:u8 (extension)."""
    return struct.pack('<HBBB', count, target_system, target_component, mission_type)


def parse_mission_count(payload: bytes) -> Optional[dict]:
    # A count=0 ("clear mission") message with target_system/component/mission_type
    # also 0 truncates to a 0-byte payload under MAVLink 2's trailing-zero rule.
    # Zero-fill rather than rejecting, or a legitimate empty-mission announcement
    # would be silently dropped.
    payload = payload.ljust(2, b'\x00')
    count = struct.unpack('<H', payload[0:2])[0]
    target_system = payload[2] if len(payload) > 2 else 0
    target_component = payload[3] if len(payload) > 3 else 0
    mission_type = payload[4] if len(payload) > 4 else MissionType.MISSION
    return {
        'count': count,
        'target_system': target_system,
        'target_component': target_component,
        'mission_type': mission_type,
    }


def build_mission_request_int(seq: int, target_system: int = 1,
                               target_component: int = 1,
                               mission_type: int = MissionType.MISSION) -> bytes:
    """MISSION_REQUEST_INT (id 51): seq:u16, target_system:u8,
    target_component:u8, mission_type:u8 (extension)."""
    return struct.pack('<HBBB', seq, target_system, target_component, mission_type)


def parse_mission_request(payload: bytes) -> Optional[dict]:
    """Shared parser for MISSION_REQUEST (40) and MISSION_REQUEST_INT (51):
    both have identical wire layout (seq:u16, target_system:u8,
    target_component:u8, mission_type:u8)."""
    # seq=0 combined with target_system/component/mission_type=0 truncates to a
    # 0-byte payload under MAVLink 2's trailing-zero rule — zero-fill rather
    # than rejecting, or a request for waypoint 0 would be silently dropped.
    payload = payload.ljust(2, b'\x00')
    seq = struct.unpack('<H', payload[0:2])[0]
    target_system = payload[2] if len(payload) > 2 else 0
    target_component = payload[3] if len(payload) > 3 else 0
    mission_type = payload[4] if len(payload) > 4 else MissionType.MISSION
    return {
        'sequence': seq,
        'target_system': target_system,
        'target_component': target_component,
        'mission_type': mission_type,
    }


def build_mission_item_reached(seq: int) -> bytes:
    """MISSION_ITEM_REACHED (id 46): seq:u16."""
    return struct.pack('<H', seq)
