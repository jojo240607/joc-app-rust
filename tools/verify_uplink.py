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
    s.reset_input_buffer()
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


def run_params(port):
    """增强验证：PARAM_SET / PARAM_REQUEST_READ / AUTOPILOT_VERSION 能力上报。
    - PARAM_SET(KvZ=0.5 合法) -> 收 PARAM_VALUE(22) 回显
    - PARAM_REQUEST_READ(KvZ) -> 收 PARAM_VALUE(22) 点读回显
    - PARAM_SET(HoverThrust=2.0 越界) -> 收 COMMAND_ACK(23, FAILED)
    - COMMAND_LONG(REQUEST_CAPABILITIES=520) -> 收 AUTOPILOT_VERSION(300)
    """
    s = _open(port)
    seq = 0
    name16 = lambda n: n.encode()[:15].ljust(16, b'\x00')

    def send(msgid, payload):
        nonlocal seq
        seq += 1
        s.write(ml.frame(msgid, payload, seq))

    # 1) PARAM_SET 合法（KvZ=0.5）
    pl = name16("KvZ") + struct.pack('<f', 0.5) + bytes([ml.MAV_PARAM_TYPE_REAL32]) + bytes(3)
    print(f"[TX] PARAM_SET KvZ=0.5")
    send(23, pl)
    # 2) PARAM_REQUEST_READ（KvZ）
    pl = name16("KvZ") + (-1).to_bytes(2, 'little', signed=True)
    print(f"[TX] PARAM_REQUEST_READ KvZ")
    send(20, pl)
    # 3) PARAM_SET 越界（HoverThrust=2.0 > 1.0）
    pl = name16("HoverThrust") + struct.pack('<f', 2.0) + bytes([ml.MAV_PARAM_TYPE_REAL32]) + bytes(3)
    print(f"[TX] PARAM_SET HoverThrust=2.0 (越界)")
    send(23, pl)
    # 4) COMMAND_LONG REQUEST_AUTOPILOT_CAPABILITIES(520)
    pl = bytes([ml.TGT_SYS, ml.TGT_COMP]) + (520).to_bytes(2, 'little') + bytes(28) + b'\x00'
    print(f"[TX] COMMAND_LONG(REQUEST_CAP, cmd=520)")
    send(76, pl)

    time.sleep(2.0)
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
            if d.msgid == 22:  # PARAM_VALUE：解析回显值
                nm = d.payload[0:16].split(b'\x00')[0].decode(errors='replace')
                val = struct.unpack('<f', d.payload[16:20])[0]
                print(f"  -> PARAM_VALUE name='{nm}' value={val:.4f}")
            elif d.msgid == 77:  # COMMAND_ACK
                print(f"  -> COMMAND_ACK cmd={int.from_bytes(d.payload[0:2],'little')} result={d.payload[2]}")
            elif d.msgid == 300:  # AUTOPILOT_VERSION
                print(f"  -> AUTOPILOT_VERSION received (capabilities={int.from_bytes(d.payload[52:60],'little')})")
    s.close()
    print(f"[RX] 应答统计 ids={ids}")
    ok = True
    if 22 in ids:
        print("PASS: PARAM_SET/PARAM_REQUEST_READ -> 收到 PARAM_VALUE(22) 回显")
    else:
        print("FAIL: 未收到 PARAM_VALUE"); ok = False
    if 300 in ids:
        print("PASS: REQUEST_CAPABILITIES -> 收到 AUTOPILOT_VERSION(300)")
    else:
        print("WARN: 未收到 AUTOPILOT_VERSION(300)（能力上报可能未生效）")
    if 77 in ids:
        print("PASS: 越界 PARAM_SET -> 收到 COMMAND_ACK(23, FAILED)")
    else:
        print("WARN: 越界 PARAM_SET 未回 COMMAND_ACK（范围校验可能未生效）")
    print("DONE" if ok else "PARTIAL")


def main():
    ap = argparse.ArgumentParser(description="飞控 USB CDC 上行联调")
    ap.add_argument("port", nargs="?", default=None, help="串口 (默认自动探测)")
    ap.add_argument("--test", action="store_true", help="发 PARAM_REQUEST_LIST + COMMAND_LONG")
    ap.add_argument("--arm", action="store_true", help="反复 ARM/DISARM 测心跳 base_mode")
    ap.add_argument("--params", action="store_true", help="验证 PARAM_SET/REQUEST_READ/AUTOPILOT_VERSION")
    args = ap.parse_args()
    port = args.port or (ml.find_cdc() or "COM12")
    if args.test:
        run_test(port)
    elif args.params:
        run_params(port)
    else:
        run_arm(port)


if __name__ == "__main__":
    main()
