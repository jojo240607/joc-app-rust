#!/usr/bin/env python3
"""共享 MAVLink v2 工具：CRC、组帧、解析、端口探测。

固件端参考：flyctrl/core/src/comm/mavlink.rs
关键约定（与固件保持一致，否则 CRC 校验全失败）：
  * 反射 CRC-16/ARC 多项式 0x8408，初值 0xFFFF。
  * CRC 计算范围：从 byte[1] (len) 开始，到 payload 末尾，再追加 CRC_EXTRA[msgid]。
  * CRC_EXTRA 表（LOCAL_POSITION_NED=185，勿误用旧 143；COMMAND_ACK=143，勿误用旧 208）。

被以下脚本复用：verify_downlink.py / verify_uplink.py / dump_usb.py / usb_telemetry.py。
"""
import sys
import types

# CRC_EXTRA：与 flyctrl/core/src/comm/mavlink.rs 完全一致
CRC_EXTRA = {
    0: 50,    # HEARTBEAT
    1: 124,   # SYS_STATUS
    21: 159,  # PARAM_REQUEST_LIST
    22: 220,  # PARAM_VALUE
    23: 168,  # PARAM_SET
    20: 214,  # PARAM_REQUEST_READ   (与 flyctrl-core mavlink.rs 一致)
    300: 178, # AUTOPILOT_VERSION     (与 flyctrl-core mavlink.rs 一致)
    30: 39,   # ATTITUDE
    32: 185,  # LOCAL_POSITION_NED  (标准 common.xml；旧脚本误用 143)
    33: 104,  # GLOBAL_POSITION_INT (标准 common.xml)
    74: 20,   # VFR_HUD             (标准 common.xml)
    76: 152,  # COMMAND_LONG
    77: 143,  # COMMAND_ACK         (标准 common.xml；旧脚本误用 208)
    40: 230,  # MISSION_REQUEST
    43: 132,  # MISSION_REQUEST_LIST
    44: 221,  # MISSION_COUNT
    45: 232,  # MISSION_CLEAR_ALL
    47: 153,  # MISSION_ACK
    70: 124,  # RC_CHANNELS_OVERRIDE
    73: 38,   # MISSION_ITEM_INT
    160: 78,   # FENCE_POINT
    161: 68,   # FENCE_FETCH_POINT
    66: 148,    # REQUEST_DATA_STREAM
    67: 21,     # DATA_STREAM
    84: 143,  # SET_POSITION_TARGET_LOCAL_NED  (标准 common.xml；与 mavlink-core frame.rs 一致)
    93: 47,   # HIL_ACTUATOR_CONTROLS          (标准 common.xml)
    107: 108, # HIL_SENSOR                     (标准 common.xml；勿信 DESIGN.md 笔误 90)
}

MAGIC = 0xFD

SYS_ID = 1
COMP_ID = 1
TGT_SYS = 1
TGT_COMP = 1

# MAV_PARAM_TYPE（PARAM_SET/VALUE 的 param_type 字段）
MAV_PARAM_TYPE_REAL32 = 9


def crc16(crc, data):
    """反射 CRC-16/ARC (0x8408)，逐字节。"""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def frame(msgid, payload, seq, sys_id=SYS_ID, comp_id=COMP_ID):
    """构造 MAVLink v2 帧（含 CRC_EXTRA）。"""
    hdr = bytes([MAGIC, len(payload), 0, 0, seq, sys_id, comp_id,
                 msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    body = hdr + bytes(payload)
    c = crc16(0xFFFF, body[1:])          # 从 len 字节起，跳过 magic
    c = crc16(c, [CRC_EXTRA.get(msgid, 0)])
    return body + bytes([c & 0xFF, (c >> 8) & 0xFF])


# ── MISSION / RC_OVERRIDE 上行构造辅助 ──────────────────────────────
def f32(b):
    import struct
    return struct.pack('<f', b)

def enc_command_long(command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, seq=0, confirmation=0):
    """COMMAND_LONG(76)：33B。target_system, target_component, command(u16),
    confirmation, param1-7 (f32 LE)。"""
    p = bytearray(33)
    p[0] = 1; p[1] = 1
    p[2:4] = command.to_bytes(2, 'little')
    p[4] = confirmation
    for i, v in enumerate([p1, p2, p3, p4, p5, p6, p7]):
        p[5 + i*4:9 + i*4] = f32(v)
    return frame(76, bytes(p), seq)

def enc_mission_count(count, seq):
    """MISSION_COUNT(44)：target_system, target_component, count(u16)。"""
    p = bytes([1, 1]) + count.to_bytes(2, 'little')
    return frame(44, p, seq)

def enc_mission_request(seq_req, seq):
    """MISSION_REQUEST(40)：target_system, target_component, seq(u16)。"""
    p = bytes([1, 1]) + seq_req.to_bytes(2, 'little')
    return frame(40, p, seq)

def enc_mission_item_int(item, seq):
    """MISSION_ITEM_INT(73)：37B 标准布局。
    item: dict{seq,command,param1..4,x,y,z,frame,current,autocontinue}
    """
    p = bytearray(37)
    p[0] = 1; p[1] = 1
    p[2:4] = item['seq'].to_bytes(2, 'little')
    p[4] = item.get('frame', 3)         # MAV_FRAME_GLOBAL_INT
    p[5:7] = item['command'].to_bytes(2, 'little')
    p[7] = item.get('current', 0)
    p[8] = item.get('autocontinue', 1)
    p[9:13] = f32(item.get('param1', 0.0))
    p[13:17] = f32(item.get('param2', 0.0))
    p[17:21] = f32(item.get('param3', 0.0))
    p[21:25] = f32(item.get('param4', 0.0))
    p[25:29] = item['x'].to_bytes(4, 'little', signed=True)
    p[29:33] = item['y'].to_bytes(4, 'little', signed=True)
    p[33:37] = f32(item.get('z', 0.0))
    # mission_type 在 v2 扩展字段，此处省略（与固件 decode 一致：未读末尾扩展）
    return frame(73, bytes(p), seq)

def enc_mission_request_list(seq):
    """MISSION_REQUEST_LIST(43)：target_system, target_component。"""
    return frame(43, bytes([1, 1]), seq)

def enc_rc_channels_override(ch, seq):
    """RC_CHANNELS_OVERRIDE(70)：21B。ch=[c1..c8] 为 PWM 微秒值。"""
    p = bytearray(21)
    p[0] = 1; p[1] = 1
    for i in range(8):
        p[2 + i*2:4 + i*2] = ch[i].to_bytes(2, 'little')
    p[18] = 0  # rssi
    return frame(70, bytes(p), seq)

def enc_fence_point(idx, count, lat, lon, seq):
    """FENCE_POINT(160)：12B。idx/count(u8)，lat/lon(i32, *1e7)。"""
    p = bytearray(12)
    p[0] = 1; p[1] = 1
    p[2] = idx
    p[3] = count
    p[4:8] = lat.to_bytes(4, 'little', signed=True)
    p[8:12] = lon.to_bytes(4, 'little', signed=True)
    return frame(160, bytes(p), seq)

def enc_fence_fetch_point(idx, seq):
    """FENCE_FETCH_POINT(161)：3B。idx(u8)。"""
    p = bytes([1, 1, idx])
    return frame(161, p, seq)

def enc_request_data_stream(stream_id, rate_hz, start_stop, seq):
    """REQUEST_DATA_STREAM(66)：6B。stream_id(u8), rate(u16), ts, tc, start_stop(u8)。"""
    p = bytearray(6)
    p[0] = stream_id
    p[1:3] = rate_hz.to_bytes(2, 'little')
    p[3] = 1  # target_system
    p[4] = 1  # target_component
    p[5] = start_stop
    return frame(66, bytes(p), seq)

def enc_set_message_interval(msg_id, interval_us, seq):
    """SET_MESSAGE_INTERVAL 经 COMMAND_LONG(76) 的 MAV_CMD=203。"""
    return enc_command_long(203, msg_id, interval_us, 0, 0, 0, 0, 0, seq)


# ── HIL（硬件在环，仿真器 -> 飞控） ────────────────────────────────
def enc_hil_sensor(time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro,
                   xmag=0.0, ymag=0.0, zmag=0.0,
                   abs_pressure=0.0, diff_pressure=0.0, pressure_alt=0.0,
                   temperature=25, fields_updated=0, seq=0):
    """HIL_SENSOR(107)：标准 62B。IMU/磁/气压真值，仿真器 -> 飞控。
    约定：xacc/yacc/zacc 为机体系比力（悬停 zacc≈+9.81，NED 向下为正），
    xgyro/ygyro/zgyro 为机体角速度(rad/s)，pressure_alt 为气压高度(m)。
    """
    p = bytearray(62)
    p[0:8] = int(time_usec).to_bytes(8, 'little')
    for i, v in enumerate([xacc, yacc, zacc, xgyro, ygyro, zgyro,
                           xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt]):
        p[8 + i*4:12 + i*4] = f32(v)
    p[56:58] = int(temperature).to_bytes(2, 'little', signed=True)
    p[58:62] = int(fields_updated).to_bytes(4, 'little')
    return frame(107, bytes(p), seq)

def enc_set_position_target_local_ned(time_boot_ms, x, y, z, vx=0.0, vy=0.0, vz=0.0,
                                      afx=0.0, afy=0.0, afz=0.0, yaw=0.0, yaw_rate=0.0,
                                      type_mask=0, seq=0):
    """SET_POSITION_TARGET_LOCAL_NED(84)：标准 51B，CRC_EXTRA=143。
    坐标系 MAV_FRAME_LOCAL_NED(1)。HIL 场景下同时承载「期望状态」与
    「机体位置真值」（MCU 端 G_HIL_GPS 取 x/y/z 作为位置测量）。
    type_mask=0 表示位置/速度/加速度/偏航全部使用。
    """
    p = bytearray(51)
    p[0:4] = int(time_boot_ms).to_bytes(4, 'little')
    p[4] = 1  # MAV_FRAME_LOCAL_NED
    p[5:7] = int(type_mask).to_bytes(2, 'little')
    for i, v in enumerate([x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate]):
        p[7 + i*4:11 + i*4] = f32(v)
    return frame(84, bytes(p), seq)

def dec_hil_actuator_controls(payload):
    """HIL_ACTUATOR_CONTROLS(93)：标准 81B，CRC_EXTRA=47。飞控 -> 仿真器。
    返回 (time_usec, controls[16], mode, flags)。controls[0..4] 为四电机归一化推力。
    """
    if len(payload) < 81:
        return None
    import struct
    time_usec = struct.unpack_from('<Q', payload, 0)[0]
    controls = list(struct.unpack_from('<16f', payload, 8))
    mode = payload[72]
    flags = struct.unpack_from('<Q', payload, 73)[0]
    return (time_usec, controls, mode, flags)

def dec_local_position_ned(payload):
    """LOCAL_POSITION_NED(32)：标准 28B，CRC_EXTRA=185。飞控 -> 地面站。
    返回 (time_boot_ms, x, y, z, vx, vy, vz)。NED，单位 m / m/s。
    """
    if len(payload) < 28:
        return None
    import struct
    return struct.unpack_from('<I3f3f', payload, 0)

def dec_heartbeat(payload):
    """HEARTBEAT(0)：标准 9B。返回 (custom_mode, type, autopilot, base_mode,
    system_status, mavlink_version)。base_mode 含 MAV_MODE_FLAG_HIL_ENABLED(0x20) 表示 HIL。"""
    if len(payload) < 9:
        return None
    import struct
    mtype, autopilot, base_mode, custom_mode, sys_status, ver = struct.unpack_from('<BBBI2B', payload, 0)
    return (custom_mode, mtype, autopilot, base_mode, sys_status, ver)


def find_cdc():
    """自动探测 HIL 上行 USB-CDC 端口（STM32 CDC，VID_0483:PID_5740）。

    边界：本工程 HIL 上行走 STM32 USB-CDC（如 COM12），日志走独立 UART
    （CH340，如 COM8）。探测必须把两者分清，否则会拿到日志口而读不到 HIL 数据。
    匹配策略按优先级：
      1. 精确 PID 5740（STM32 USB CDC / ST VCP）——HIL 上行口；
      2. 描述含 "Virtual COM Port" 的 STMicroelectronics 设备；
      3. 兜底：任何 STMicroelectronics 设备。
    返回首个命中端口的 device 名；未找到返回 None。
    """
    try:
        import serial.tools.list_ports
    except Exception:
        return None
    cands = list(serial.tools.list_ports.comports())
    for p in cands:
        hwid = (getattr(p, 'hwid', '') or '').upper()
        if '0483:5740' in hwid:
            return p.device
    for p in cands:
        hwid = (getattr(p, 'hwid', '') or '').upper()
        desc = (p.description or '').upper()
        if 'STMicroelectronics' in desc and 'VIRTUAL COM' in desc:
            return p.device
    for p in cands:
        hwid = (getattr(p, 'hwid', '') or '').upper()
        desc = (p.description or '').upper()
        if 'STMicroelectronics' in desc or '0483' in hwid:
            return p.device
    return None


def scan_frames(buf, start=0):
    """在 buf[start:] 中滑动扫描合法 MAVLink v2 帧。

    返回生成器，逐项为 dict：
        {off, msgid, seq, plen, crc_ok, total, payload}
    只产出「CRC 校验通过」的完整帧；错位/不完整的 0xFD 会被跳过。
    """
    i = start
    n = len(buf)
    while i < n - 11:
        if buf[i] != MAGIC:
            i += 1
            continue
        plen = buf[i + 1]
        if plen > 255:
            i += 1
            continue
        total = 10 + plen + 2
        if i + total > n:
            break  # 帧未收齐，等更多数据
        msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        seq = buf[i + 4]
        crc = crc16(0xFFFF, buf[i + 1:i + 10 + plen])
        crc = crc16(crc, [CRC_EXTRA.get(msgid, 0)])
        fc = buf[i + total - 2] | (buf[i + total - 1] << 8)
        crc_ok = (crc == fc)
        payload = bytes(buf[i + 10:i + 10 + plen])
        yield types.SimpleNamespace(off=i, msgid=msgid, seq=seq, plen=plen,
                                    crc_ok=crc_ok, total=total, payload=payload)
        # 跳过整帧（含其中可能巧合出现的 0xFD），避免误判
        i += total


if __name__ == "__main__":
    # 自检：组一帧 HEARTBEAT 并校验
    f = frame(0, bytes(9), 0)
    ok = any(d.crc_ok for d in scan_frames(f))
    print("self-test:", "PASS" if ok else "FAIL", file=sys.stderr)
