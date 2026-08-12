#!/usr/bin/env python3
"""USB CDC 下行(MATLink v2)严格验证：抓 COM 端口原始流，只接受 CRC 校验通过的完整帧，
统计帧数 / 各 msgid 计数 / seq 连续度（seq 重复=字节污染，seq 跳变>1=丢帧）。

要求：板子已运行 + USB 已枚举（不要挂 OpenOCD/gdb，halt 会让 CDC 端口掉线）。
复用 tools/mavlink.py 的 CRC 与扫描逻辑。

用法：
  python tools/verify_downlink.py [PORT] [SECS]
  python tools/verify_downlink.py COM12 15
端口缺省自动探测 ST VCP (VID_0483/PID_5740)。
"""
import argparse
import sys
import time

import serial

import mavlink as ml


def main():
    ap = argparse.ArgumentParser(description="USB CDC 下行 MAVLink v2 严格校验")
    ap.add_argument("port", nargs="?", default=None, help="串口 (默认自动探测 ST VCP)")
    ap.add_argument("secs", nargs="?", type=int, default=15, help="捕获时长秒")
    args = ap.parse_args()
    port = args.port or (ml.find_cdc() or "COM12")
    secs = args.secs
    print(f"CDC={port} capture={secs}s")

    s = serial.Serial(port, 115200, timeout=1)
    s.dtr = False
    s.rts = False
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
        for d in ml.scan_frames(buf):
            frames += 1
            if d.crc_ok:
                ok += 1
            else:
                bad += 1
            ids[d.msgid] = ids.get(d.msgid, 0) + 1
            if d.msgid in seq_last:
                delta = (d.seq - seq_last[d.msgid]) & 0xFF
                if delta == 0:
                    seq_dups += 1
                elif delta != 1:
                    seq_jumps += 1
            seq_last[d.msgid] = d.seq
        # scan_frames 已逐整帧跳过；丢弃已处理部分，仅保留可能残缺的尾帧(<=64B)
        if len(buf) > 64:
            del buf[:len(buf) - 64]
    s.close()

    print(f"raw={raw} frames={frames} crc_ok={ok} crc_bad={bad} "
          f"seq_dups={seq_dups} seq_jumps={seq_jumps} ids={ids} leftover={len(buf)}")


if __name__ == "__main__":
    main()
