#!/usr/bin/env python3
"""USB CDC 下行(MAVLink v2)严格验证：抓 COM 端口原始流，只接受 CRC 校验通过的完整帧，
统计帧数 / 各 msgid 计数 / seq 连续度（seq 重复=字节污染，seq 跳变>1=丢帧）。

要求：板子已运行 + USB 已枚举（不要挂 OpenOCD/gdb，halt 会让 CDC 端口掉线）。
复用 tools/mavlink.py 的 CRC 与扫描逻辑。

注意（已修复重复计数缺陷）：旧版每读一批就对整个 buf 从头 scan_frames() 再
del buf[:len-64]，而最小帧(HEARTBEAT 21B)<64B，导致靠近 buf 末尾的完整帧被
保留并在下一轮【重复计数】→ seq_dups 假阳性。现改为维护解析游标 scan_pos，
每帧只解析一次；只丢弃已解析的字节，不重复扫。若仍报 seq_dups，才是真字节重复。

用法：
  python tools/verify_downlink.py [PORT] [SECS]
"""
import argparse
import sys
import time

import serial

import mavlink as ml

# 遥测帧 msgid：每 20ms 下行 6 帧(HB=0/LP=32/SS=1/ATTITUDE=30/VFR_HUD=74/
# GLOBAL_POSITION_INT=33)共享同一 seq 计数器，逐周期 +1。
# 序列连续性只对核心三路(HB/LP/SS=0/32/1)统计：它们最先写、被 USB 流控丢帧的
# 概率最低；下行流里混入的上行应答帧(COMMAND_ACK=77 / PARAM_VALUE=22 /
# AUTOPILOT_VERSION=300)是一次性插入，seq 无连续性，参与统计会造出假阳性
# seq_jumps。任一路遥测连续即代表链路无字节污染。
TELEMETRY_IDS = frozenset((0, 32, 1))


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
    scan_pos = 0               # 已解析到的字节偏移（单调，不回头）
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
        # 只在 buf[scan_pos:] 里滑动解析，每个 0xFD 尝试一帧。
        # 防级联错位：CRC 校验失败说明这个 0xFD 大概率是某帧 payload 里的数据字节、
        # 不是真帧头，此时【只前移 1 字节】继续找，绝不能按伪帧长整段消费——
        # 否则会把后续真帧头吞掉，产生一连串假 crc_bad + 假 seq_jumps。
        i = scan_pos
        n = len(buf)
        while i < n - 11:
            if buf[i] != ml.MAGIC:
                i += 1
                continue
            plen = buf[i + 1]
            total = 10 + plen + 2
            if i + total > n:
                break                       # 帧未收齐，等更多数据
            msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            seq = buf[i + 4]
            c = ml.crc16(0xFFFF, buf[i + 1:i + 10 + plen])
            c = ml.crc16(c, [ml.CRC_EXTRA.get(msgid, 0)])
            fc = buf[i + total - 2] | (buf[i + total - 1] << 8)
            crc_ok = (c == fc)
            if not crc_ok:
                # 伪帧头：只跳过这个字节，继续向后找真帧头
                bad += 1
                i += 1
                scan_pos = i
                continue
            frames += 1
            ok += 1
            ids[msgid] = ids.get(msgid, 0) + 1
            # 序列连续性仅对核心遥测帧统计（非遥测帧是一次性插入，无连续性可言）
            if msgid in TELEMETRY_IDS:
                if msgid in seq_last:
                    delta = (seq - seq_last[msgid]) & 0xFF
                    if delta == 0:
                        seq_dups += 1
                    elif delta != 1:
                        seq_jumps += 1
                seq_last[msgid] = seq
            scan_pos = i + total            # 前进到本帧末尾，避免重复计数
            i += total
        # 丢弃已解析的字节，只保留未解析/可能残缺的尾部
        if scan_pos > 0:
            del buf[:scan_pos]
            scan_pos = 0
    s.close()

    print(f"raw={raw} frames={frames} crc_ok={ok} crc_bad={bad} "
          f"seq_dups={seq_dups} seq_jumps={seq_jumps} ids={ids} leftover={len(buf)}")
    # 下行流混合了上行应答帧（PARAM_VALUE=22 / COMMAND_ACK=77 / AUTOPILOT_VERSION=300）
    # 是正常现象：它们与遥测帧(HB/LP/SS)共享 usb0 下行 ring。仅当 HB/LP/SS 自身出现
    # seq_dups>0 或 seq_jumps>0 才表示字节污染；其它 msgid 是合法插入，不是丢帧。
    other = {k: v for k, v in ids.items() if k not in (0, 32, 1)}
    if other:
        print(f"[note] 非遥测帧计数 {other} —— 多为 PARAM_VALUE(22) 等上行应答，属正常混入，"
              f"非字节污染。seq_dups/seq_jumps 仅针对 HB/LP/SS 各自序列。")


if __name__ == "__main__":
    main()
