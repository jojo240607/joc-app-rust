#!/usr/bin/env python3
"""阶段四：数据流速率控制验证脚本。

验证板子对以下两条地面站请求的正确应答（联调闭环，板子遥测暂固定 20ms，
此处仅确认请求被接收并正确应答，不验证实际频率变化）：

1) REQUEST_DATA_STREAM(66) -> 板子回 DATA_STREAM(67)
2) SET_MESSAGE_INTERVAL(511, 经 COMMAND_LONG command=203) -> 板子回 COMMAND_ACK(77)

物理链路：板子 USB CDC（ST VCP） <-> 本机 COM 口（默认 COM12）。

用法：
  python tools/verify_stream.py COM12
"""
import sys
import time
import serial
sys.path.insert(0, 'tools')
from mavlink import (find_cdc, frame, crc16, CRC_EXTRA, MAGIC,
                     enc_request_data_stream, enc_set_message_interval,
                     enc_command_long)

PORT_DEFAULT = 'COM12'


def wait_frame(ser, msgid, timeout=3.0):
    """滑动扫描直到出现 msgid 且 CRC 正确的帧；返回 payload 或 None。"""
    buf = b''
    t0 = time.time()
    while time.time() - t0 < timeout:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        i = 0
        while i < len(buf) - 11:
            if buf[i] != MAGIC:
                i += 1
                continue
            plen = buf[i + 1]
            total = 10 + plen + 2
            if i + total > len(buf):
                break
            mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            crc = crc16(0xFFFF, buf[i + 1:i + 10 + plen])
            crc = crc16(crc, [CRC_EXTRA.get(mid, 0)])
            fc = buf[i + total - 2] | (buf[i + total - 1] << 8)
            if mid == msgid and crc == fc:
                return bytes(buf[i + 10:i + 10 + plen])
            i += 1
        time.sleep(0.01)
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else (find_cdc() or PORT_DEFAULT)
    print(f"STREAM verify: opening {port} ...")
    ser = serial.Serial(port, 115200, timeout=0.1, dsrdtr=False)
    ser.dtr = False
    ser.rts = False
    time.sleep(0.3)

    seq = 0
    # ── 测试 1：REQUEST_DATA_STREAM(66) -> DATA_STREAM(67) ──
    print("[1] REQUEST_DATA_STREAM(stream=0 ALL, 10Hz) -> expect DATA_STREAM(67)")
    seq += 1
    ser.write(enc_request_data_stream(0, 10, 1, seq % 256))
    pl = wait_frame(ser, 67, timeout=2.0)
    if pl is None:
        print("    NO DATA_STREAM reply (FAIL)")
        r1 = False
    else:
        stream_id = pl[0]
        rate = int.from_bytes(pl[1:3], 'little')
        on_off = pl[7]
        r1 = (stream_id == 0 and rate == 10 and on_off == 1)
        print(f"    DATA_STREAM stream={stream_id} rate={rate}Hz on={on_off} -> {'OK' if r1 else 'MISMATCH'}")

    # ── 测试 2：SET_MESSAGE_INTERVAL(511) via COMMAND_LONG command=203 -> COMMAND_ACK(77) ──
    print("[2] SET_MESSAGE_INTERVAL(msg=33 GLOBAL_POS, 100000us=10Hz) -> expect COMMAND_ACK(77)")
    seq += 1
    ser.write(enc_set_message_interval(33, 100000, seq % 256))
    pl = wait_frame(ser, 77, timeout=2.0)
    if pl is None:
        print("    NO COMMAND_ACK reply (FAIL)")
        r2 = False
    else:
        # COMMAND_ACK 布局：command(u16), result(u8), ...
        cmd = int.from_bytes(pl[0:2], 'little')
        result = pl[2]
        r2 = (cmd == 203 and result == 0)
        print(f"    COMMAND_ACK command={cmd} result={result} (0=ACCEPTED) -> {'OK' if r2 else 'MISMATCH'}")

    print("[RESULT]", "OK" if (r1 and r2) else "FAIL")
    ser.close()


if __name__ == "__main__":
    main()
