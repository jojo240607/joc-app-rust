#!/usr/bin/env python3
"""USB CDC 原始字节 dump：抓 COM 端口原始流，打印前若干 0xFD 帧的 hex 与解析，
辅助判断字节污染 / CRC 表错误 / 上行是否收到 ACK。

要求：板子已运行 + USB 已枚举（不要挂 OpenOCD/gdb）。
复用 tools/mavlink.py。

用法：
  python tools/dump_usb.py [PORT] [SECS] [MAXFRAMES]
  python tools/dump_usb.py COM12 6 8
"""
import argparse
import sys
import time

import serial

import mavlink as ml

MSG_NAMES = {0: "HEARTBEAT", 1: "SYS_STATUS", 21: "PARAM_REQUEST_LIST",
             22: "PARAM_VALUE", 23: "PARAM_SET", 30: "ATTITUDE",
             32: "LOCAL_POSITION_NED", 76: "COMMAND_LONG", 77: "COMMAND_ACK"}


def main():
    ap = argparse.ArgumentParser(description="USB CDC 原始字节 dump 诊断")
    ap.add_argument("port", nargs="?", default=None, help="串口 (默认自动探测 ST VCP)")
    ap.add_argument("secs", nargs="?", type=int, default=6, help="捕获时长秒")
    ap.add_argument("maxframes", nargs="?", type=int, default=8, help="最多展示帧数")
    args = ap.parse_args()
    port = args.port or (ml.find_cdc() or "COM12")
    secs, maxframes = args.secs, args.maxframes
    print(f"CDC={port} capture={secs}s maxframes={maxframes}")

    s = serial.Serial(port, 115200, timeout=1)
    s.dtr = False
    s.rts = False
    time.sleep(0.5)
    s.reset_input_buffer()

    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < secs:
        b = s.read(512)
        if b:
            buf += b
        if len(buf) >= 1200:
            break
    s.close()

    shown = 0
    for d in ml.scan_frames(buf):
        seg = bytes(buf[d.off:d.off + min(d.total, 40)])
        name = MSG_NAMES.get(d.msgid, f"MSG_{d.msgid}")
        print(f"@off={d.off} {name} len={d.total} plen={d.plen} "
              f"seq={d.seq} crc_ok={d.crc_ok}")
        print(f"    hex={seg.hex()}")
        shown += 1
        if shown >= maxframes:
            break
    print(f"\n[total_buf={len(buf)} shown={shown}]")


if __name__ == "__main__":
    main()
