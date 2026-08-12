#!/usr/bin/env python3
"""USB CDC 上行联调：向板子发 MAVLink v2 指令，验证上行链路 + 命令处理。

两种模式：
  --test   发 PARAM_REQUEST_LIST(21) + COMMAND_LONG(REQUEST_CAPABILITIES=520)，
           期望回 PARAM_VALUE(22) / COMMAND_ACK(77)。
  --arm    （默认）反复发 ARM/DISARM(COMMAND_LONG cmd=400) 切换 armed 状态，
           观察后续心跳(msgid=0) 的 base_mode 是否出现 0x81(armed) vs 0x01(disarmed)。
           base_mode 出现 0x81 即证明上行链路 + ARM 命令处理全通。

复用 tools/mavlink.py。

用法：
  python tools/verify_uplink.py [PORT]                 # 默认 --arm 模式
  python tools/verify_uplink.py COM12 --test
  python tools/verify_uplink.py COM12 --arm
"""
import argparse
import struct
import sys
import time

import serial

import mavlink as ml


def _open(port):
    s = serial.Serial(port, 115200, timeout=0.2)
    s.dtr = False
    s.rts = False
    time.sleep(0.5)
    s.reset_input_buffer()
    return s


def run_test(port):
    s = _open(port)
    seq = 0
    # 1) PARAM_REQUEST_LIST -> 期望 PARAM_VALUE
    seq += 1
    f1 = ml.frame(21, bytes([ml.TGT_SYS, ml.TGT_COMP]), seq)
    print(f"[TX] PARAM_REQUEST_LIST ({len(f1)}B)")
    s.write(f1)
    time.sleep(1.5)
    # 2) COMMAND_LONG REQUEST_AUTOPILOT_CAPABILITIES(520)
    cmd = 520
    pl = bytes([ml.TGT_SYS, ml.TGT_COMP]) + cmd.to_bytes(2, 'little') + bytes(28) + b'\x00'
    seq += 1
    f2 = ml.frame(76, pl, seq)
    print(f"[TX] COMMAND_LONG(REQUEST_CAP, cmd={cmd}) ({len(f2)}B)")
    s.write(f2)
    time.sleep(1.5)

    s.reset_input_buffer()
    buf = bytearray()
    t0 = time.time()
    ids = {}
    print("[RX] capturing 3s ...")
    while time.time() - t0 < 3:
        b = s.read(256)
        if b:
            buf += b
        for d in ml.scan_frames(buf):
            ids[d.msgid] = ids.get(d.msgid, 0) + 1
            if d.msgid in (22, 77):
                print(f"  -> ACK/RESP msgid={d.msgid} crc_ok")
    s.close()
    print(f"[RX] 应答统计 ids={ids}")
    if 22 in ids:
        print("PASS: PARAM_REQUEST_LIST -> 收到 PARAM_VALUE(22)")
    else:
        print("FAIL: 未收到 PARAM_VALUE")
    if 77 in ids:
        print("PASS: COMMAND_LONG -> 收到 COMMAND_ACK(77)")
    else:
        print("FAIL: 未收到 COMMAND_ACK")


def run_arm(port):
    s = _open(port)
    seq = 0
    base_modes = {}
    t0 = time.time()
    toggle = 0
    print("[TX] 每 1s 切换 ARM/DISARM ...")
    while time.time() - t0 < 12:
        now = int(time.time() - t0)
        if now != toggle:
            toggle = now
            arm = (toggle % 2 == 0)
            p1 = 1.0 if arm else 0.0
            pb = struct.pack('<f', p1)
            pl = bytes([ml.TGT_SYS, ml.TGT_COMP]) + (400).to_bytes(2, 'little') \
                 + b'\x00' + pb + bytes(24)
            seq += 1
            s.write(ml.frame(76, pl, seq))
            print(f"[TX] {'ARM' if arm else 'DISARM'} cmd=400 p1={p1}")
        b = s.read(512)
        if b:
            buf = bytearray(b)
            for d in ml.scan_frames(buf):
                if d.msgid == 0 and d.plen >= 9:  # HEARTBEAT, base_mode = payload[2]
                    bm = d.payload[2]
                    base_modes[bm] = base_modes.get(bm, 0) + 1
    s.close()
    print(f"[RX] 心跳 base_mode 统计 = {base_modes}")
    if 0x81 in base_modes:
        print("PASS: 心跳出现 base_mode=0x81 (ARMED) -> 上行链路 + ARM 命令处理全通！")
    else:
        print("FAIL: 心跳始终 base_mode=0x01 (DISARMED) -> ARM 命令未被处理 / 上行未达")


def main():
    ap = argparse.ArgumentParser(description="飞控 USB CDC 上行联调")
    ap.add_argument("port", nargs="?", default=None, help="串口 (默认自动探测)")
    ap.add_argument("--test", action="store_true", help="发 PARAM_REQUEST_LIST + COMMAND_LONG")
    ap.add_argument("--arm", action="store_true", help="反复 ARM/DISARM 测心跳 base_mode")
    args = ap.parse_args()
    port = args.port or (ml.find_cdc() or "COM12")
    if args.test:
        run_test(port)
    else:
        run_arm(port)


if __name__ == "__main__":
    main()
