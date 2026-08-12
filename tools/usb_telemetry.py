#!/usr/bin/env python3
"""飞控 USB CDC 遥测接收与离线采集（MAVLink v2）。

板端：telemetry 任务经 USB CDC(usb0, CDC-ACM 虚拟串口) 下行标准 MAVLink v2 帧
(HEARTBEAT / LOCAL_POSITION_NED / SYS_STATUS)，电脑免 USB-TTL 转接直接收流。

本脚本：
  1. 自动探测 USB CDC 端口（Win COM* / Linux|Mac /dev/ttyACM*，优先 ST VCP）；
  2. 按 MAVLink v2 帧同步(0xFD) + 反射 CRC-16/ARC(0x8408, 含 CRC_EXTRA) 解析；
  3. 实时打印三类消息，并可选 --csv 导出供仿真分析。

复用 tools/mavlink.py 的 CRC/扫描逻辑（与板端 mavlink.rs 严格一致）。

用法：
  python tools/usb_telemetry.py                 # 自动探测端口，实时打印
  python tools/usb_telemetry.py --port COM12    # 指定端口
  python tools/usb_telemetry.py --csv flight.csv --secs 30   # 采集 30s 到 CSV
"""
import argparse
import csv
import sys
import time

import serial

import mavlink as ml

MSG_NAMES = {0: "HEARTBEAT", 1: "SYS_STATUS", 30: "ATTITUDE",
             32: "LOCAL_POSITION_NED", 76: "COMMAND_LONG", 77: "COMMAND_ACK"}


def parse_heartbeat(pl):
    mav_type, autopilot, base_mode, custom = pl[0], pl[1], pl[2], int.from_bytes(pl[3:5], "little")
    status = pl[5]
    armed = bool(base_mode & 0x80)
    return dict(type=mav_type, autopilot=autopilot, armed=armed,
                custom_mode=custom, status=status)


def parse_local_pos(pl):
    import struct
    t = struct.unpack_from("<i", pl, 0)[0]
    x, y, z, vx, vy, vz = struct.unpack_from("<ffffff", pl, 4)
    return dict(t=t, x=x, y=y, z=z, vx=vx, vy=vy, vz=vz)


def parse_sys_status(pl):
    import struct
    present, enabled, health = struct.unpack_from("<iii", pl, 0)
    load = struct.unpack_from("<H", pl, 12)[0]
    return dict(present=present, enabled=enabled, health=health, load_pct=load / 10.0)


def detect_port():
    return ml.find_cdc()


def main():
    ap = argparse.ArgumentParser(description="飞控 USB CDC 遥测接收 (MAVLink v2)")
    ap.add_argument("--port", default=None, help="串口 (默认自动探测 ST VCP)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--secs", type=float, default=0, help="采集时长(秒)，0=一直")
    ap.add_argument("--csv", default=None, help="导出 CSV 路径")
    args = ap.parse_args()

    port = args.port or detect_port()
    if not port:
        sys.exit("未找到 USB CDC 端口；请用 --port 指定 (如 COM12 / /dev/ttyACM0)，"
                 "并确保板子 USB 已连电脑且枚举")

    print(f"[*] 打开 {port} @ {args.baud}")
    ser = serial.Serial(port, args.baud, timeout=0.2)

    csvf = open(args.csv, "w", newline="") if args.csv else None
    writer = None
    if csvf:
        writer = csv.writer(csvf)
        writer.writerow(["t_sec", "msgid", "msg", "raw_fields"])

    buf = bytearray()
    t_start = time.time()
    frames = 0
    print("[*] 等待首个合法 MAVLink v2 帧(自动同步)... (确保板子已上电并 USB 已枚举)")
    try:
        while True:
            if args.secs and (time.time() - t_start > args.secs):
                break
            b = ser.read(256)
            if b:
                buf += b
            if len(buf) < 12:
                continue
            pending = bytearray()
            for d in ml.scan_frames(buf):
                frames += 1
                ts = time.time() - t_start
                name = MSG_NAMES.get(d.msgid, f"MSG_{d.msgid}")
                if d.msgid == 0:
                    hb = parse_heartbeat(d.payload)
                    print(f"[{ts:6.2f}s] HEARTBEAT type={hb['type']} armed={hb['armed']} "
                          f"custom_mode={hb['custom_mode']} status={hb['status']}")
                    if writer:
                        writer.writerow([f"{ts:.3f}", d.msgid, name,
                                         f"armed={hb['armed']},custom={hb['custom_mode']},status={hb['status']}"])
                elif d.msgid == 32:
                    lp = parse_local_pos(d.payload)
                    print(f"[{ts:6.2f}s] LOCAL_POS_NED x={lp['x']:.3f} y={lp['y']:.3f} z={lp['z']:.3f} "
                          f"vx={lp['vx']:.3f} vy={lp['vy']:.3f} vz={lp['vz']:.3f}")
                    if writer:
                        writer.writerow([f"{ts:.3f}", d.msgid, name,
                                         f"x={lp['x']:.4f},y={lp['y']:.4f},z={lp['z']:.4f},"
                                         f"vx={lp['vx']:.4f},vy={lp['vy']:.4f},vz={lp['vz']:.4f}"])
                elif d.msgid == 1:
                    ss = parse_sys_status(d.payload)
                    print(f"[{ts:6.2f}s] SYS_STATUS health=0x{ss['health']:X} load={ss['load_pct']:.1f}%")
                    if writer:
                        writer.writerow([f"{ts:.3f}", d.msgid, name,
                                         f"health=0x{ss['health']:X},load={ss['load_pct']:.1f}"])
                else:
                    print(f"[{ts:6.2f}s] {name} (len={d.plen})")
                    if writer:
                        writer.writerow([f"{ts:.3f}", d.msgid, name, f"len={d.plen}"])
            # scan_frames 跳过合法帧，但未扫描的尾部（含残缺帧）保留
            buf = bytearray(buf[-512:]) if len(buf) > 1024 else buf
    except KeyboardInterrupt:
        print("\n[*] 用户中断")
    finally:
        ser.close()
        if csvf:
            csvf.close()
        print(f"[*] 共解析 {frames} 帧" + (f"，已导出 {args.csv}" if args.csv else ""))


if __name__ == "__main__":
    main()
