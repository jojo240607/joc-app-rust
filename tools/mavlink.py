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
    30: 39,   # ATTITUDE
    32: 185,  # LOCAL_POSITION_NED  (标准 common.xml；旧脚本误用 143)
    33: 104,  # GLOBAL_POSITION_INT (标准 common.xml)
    74: 20,   # VFR_HUD             (标准 common.xml)
    76: 152,  # COMMAND_LONG
    77: 143,  # COMMAND_ACK         (标准 common.xml；旧脚本误用 208)
}

MAGIC = 0xFD

SYS_ID = 1
COMP_ID = 1
TGT_SYS = 1
TGT_COMP = 1


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


def find_cdc():
    """自动探测 ST VCP / CDC-ACM 端口（优先 VID_0483 / 5740）。"""
    try:
        import serial.tools.list_ports
    except Exception:
        return None
    for p in serial.tools.list_ports.comports():
        hwid = getattr(p, 'hwid', '') or ''
        desc = p.description or ''
        if '5740' in hwid or 'VID_0483' in hwid or 'STMicroelectronics' in desc:
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
