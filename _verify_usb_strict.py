#!/usr/bin/env python3
"""USB 下行(COM12 CDC)严格验证：与 joc-base/tools/_diag_strict.py 同款鲁棒解析。

与 _verify_usb.py 的两点关键差异（避免误报"全 CRC 错"）：
1) CRC_EXTRA 表必须与 flyctrl/core/src/comm/mavlink.rs 完全一致（重要！旧脚本
   LOCAL_POSITION_NED 用了 143，固件实际是 185，导致 LP 帧 CRC 永远校验失败）。
2) 严格滑动解析：只接受「CRC 校验完全通过」的完整帧；payload 里巧合的 0xFD
   （如 SYS_STATUS 传感器掩码里的 1f 00）不会被当成帧头，因此不产生流式错位。

用法：板子已跑 + USB 枚举后直接运行（不要挂 OpenOCD/gdb）。
  python _verify_usb_strict.py [COM12] [SECS]
"""
import sys, time, serial

# ---- CRC_EXTRA：与 flyctrl/core/src/comm/mavlink.rs 的 CRC_EXTRA 表完全一致 ----
CRC = {
    0: 50,    # HEARTBEAT
    1: 124,   # SYS_STATUS
    21: 159,  # PARAM_REQUEST_LIST
    22: 220,  # PARAM_VALUE
    23: 168,  # PARAM_SET
    30: 39,   # ATTITUDE
    32: 185,  # LOCAL_POSITION_NED  ← 标准 common.xml；旧脚本误用 143
    76: 152,  # COMMAND_LONG
    77: 143,  # COMMAND_ACK
}

def crc16_cont(crc, data):
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF

def find_cdc():
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        hwid = (getattr(p, 'hwid', '') or '')
        desc = (p.description or '')
        if '5740' in hwid or 'VID_0483' in hwid or 'STMicroelectronics' in desc:
            return p.device
    return None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else (find_cdc() or "COM12")
    secs = int(sys.argv[2]) if len(sys.argv) > 2 else 15
    print(f"CDC={port} capture={secs}s")

    s = serial.Serial(port, 115200, timeout=1)
    s.dtr = False; s.rts = False
    time.sleep(0.5)
    s.reset_input_buffer()

    buf = bytearray()
    raw = frames = ok = bad = 0
    ids = {}
    seq_last = {}
    seq_dups = seq_jumps = 0
    t0 = time.time()
    while time.time() - t0 < secs:
        b = s.read(256)
        if b:
            buf += b
            raw += len(b)
        # 严格滑动解析：每字节滑，只有 CRC 全通过才算一帧
        i = 0
        consumed = 0
        while i < len(buf) - 12:
            if buf[i] != 0xFD:
                i += 1
                continue
            plen = buf[i + 1]
            if plen > 255:
                i += 1
                continue
            total = 10 + plen + 2
            if i + total > len(buf):
                break  # 帧未收齐，等更多数据
            msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            seq = buf[i + 4]
            crc = crc16_cont(0xFFFF, buf[i + 1:i + 10 + plen])
            crc = crc16_cont(crc, [CRC.get(msgid, 0)])
            fc = buf[i + total - 2] | (buf[i + total - 1] << 8)
            if crc == fc:
                # 真实完整帧
                frames += 1; ok += 1
                ids[msgid] = ids.get(msgid, 0) + 1
                if msgid in seq_last:
                    d = (seq - seq_last[msgid]) & 0xFF
                    if d == 0:
                        seq_dups += 1
                    elif d != 1:
                        seq_jumps += 1
                seq_last[msgid] = seq
                consumed = i + total
                # 跳过这一帧，避免其 payload 内的 0xFD 再被匹配
                i = consumed
                continue
            i += 1
        del buf[:consumed]
        if len(buf) > 1024:
            del buf[:len(buf) - 512]
    s.close()

    print(f"raw={raw} frames={frames} crc_ok={ok} crc_bad={bad} "
          f"seq_dups={seq_dups} seq_jumps={seq_jumps} ids={ids} leftover={len(buf)}")

if __name__ == "__main__":
    main()
