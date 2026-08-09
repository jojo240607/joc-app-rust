#!/usr/bin/env python3
"""飞控 USB CDC 遥测接收与离线仿真采集。

板端：telemetry 任务经 USB CDC(usb0, CDC-ACM 虚拟串口) 下行标准 MAVLink v1 帧
(HEARTBEAT / LOCAL_POSITION_NED / SYS_STATUS)，电脑免 USB-TTL 转接直接收流。

本脚本：
  1. 自动探测 USB CDC 端口（Win COM* / Linux|Mac /dev/ttyACM*）；
  2. 按 MAVLink v1 帧同步(0xFE) + 标准 CRC-16/X25(含 CRC_EXTRA) 解析；
  3. 实时打印三类消息，并可选 --csv 导出供仿真分析。

用法：
  python tools/usb_telemetry.py                 # 自动探测端口，实时打印
  python tools/usb_telemetry.py --port COM9     # 指定端口
  python tools/usb_telemetry.py --csv flight.csv --secs 30   # 采集 30s 到 CSV

注意：板端 CRC_EXTRA 取自标准 common.xml，故本脚本用相同常量，可被标准 pymavlink
地面站(QGC) 同等解析；这里手搓解析器避免额外依赖，便于离线 CSV 采集与仿真。
"""
import argparse
import csv
import platform
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial: pip install pyserial")

# ── MAVLink v1 常量（与板端 mavlink.rs 对齐）─────────────────────
MAVLINK_MAGIC = 0xFE
CRC_EXTRA = [0] * 256
CRC_EXTRA[0] = 50    # HEARTBEAT
CRC_EXTRA[1] = 124   # SYS_STATUS
CRC_EXTRA[30] = 39   # ATTITUDE
CRC_EXTRA[32] = 143  # LOCAL_POSITION_NED

MSG_NAMES = {0: "HEARTBEAT", 1: "SYS_STATUS", 30: "ATTITUDE", 32: "LOCAL_POSITION_NED"}


def crc16_x25(crc, data):
    for b in data:
        x0 = (b ^ (crc & 0xFF)) & 0xFFFF
        x = x0 ^ (x0 << 4)
        x = (x ^ (x << 1) ^ (x << 2) ^ (x << 8) ^ (x << 16) ^ (x >> 4) ^ (x >> 7) ^ (x >> 11)) & 0xFFFF
        crc = ((crc >> 8) ^ x) & 0xFFFF
    return crc


def frame_valid(buf):
    """校验一个完整 MAVLink v1 帧（含 CRC_EXTRA）。返回 (msgid, payload) 或 None。"""
    if len(buf) < 8 or buf[0] != MAVLINK_MAGIC:
        return None
    plen = buf[1]
    if len(buf) < 6 + plen + 2:
        return None
    msgid = buf[5]
    payload = buf[6:6 + plen]
    crc = crc16_x25(0xFFFF, buf[1:6 + plen])
    crc = crc16_x25(crc, bytes([CRC_EXTRA[msgid]]))
    got = (buf[6 + plen + 1] << 8) | buf[6 + plen]
    if crc != got:
        return None
    return msgid, payload


def parse_heartbeat(pl):
    mav_type, autopilot, base_mode, custom = pl[0], pl[1], pl[2], int.from_bytes(pl[3:5], "little")
    status = pl[5]
    armed = bool(base_mode & 0x80)
    return dict(type=mav_type, autopilot=autopilot, armed=armed, custom_mode=custom, status=status)


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
    sysname = platform.system()
    if sysname == "Windows":
        # 枚举所有 COM 端口，优先含 CDC/USB 描述者；找不到则返回 None 交给 argparse
        try:
            import winreg
            ports = []
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM") as k:
                i = 0
                while True:
                    try:
                        name, val, _ = winreg.EnumValue(k, i); i += 1
                        ports.append(val)
                    except OSError:
                        break
            return ports[0] if ports else None
        except Exception:
            return None
    else:
        import glob
        cands = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        return cands[0] if cands else None


def sync_stream(ser, timeout=8.0):
    """从串口流中同步出一个合法 MAVLink v1 帧（含 CRC_EXTRA 校验）。"""
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        b = ser.read(1)
        if not b:
            continue
        buf.append(b[0])
        if buf[0] != MAVLINK_MAGIC:
            del buf[0]
            continue
        if len(buf) < 2:
            continue
        plen = buf[1]
        if len(buf) < 6 + plen + 2:
            continue
        window = bytes(buf[:6 + plen + 2])
        res = frame_valid(window)
        if res is None:
            # 同步失败：丢弃首字节重新同步
            del buf[0]
            continue
        frame = window
        del buf[:6 + plen + 2]
        return frame
    return None


def main():
    ap = argparse.ArgumentParser(description="飞控 USB CDC 遥测接收 (MAVLink v1)")
    ap.add_argument("--port", default=None, help="串口 (默认自动探测)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--secs", type=float, default=0, help="采集时长(秒)，0=一直")
    ap.add_argument("--csv", default=None, help="导出 CSV 路径")
    args = ap.parse_args()

    port = args.port or detect_port()
    if not port:
        sys.exit("未找到 USB CDC 端口；请用 --port 指定 (如 COM9 / /dev/ttyACM0)，并确保板子 USB 已连电脑且枚举")

    print(f"[*] 打开 {port} @ {args.baud}")
    ser = serial.Serial(port, args.baud, timeout=0.2)

    csvf = open(args.csv, "w", newline="") if args.csv else None
    writer = None
    if csvf:
        writer = csv.writer(csvf)
        writer.writerow(["t_sec", "msgid", "msg", "raw_fields"])

    t_start = time.time()
    frames = 0
    print("[*] 等待首个合法 MAVLink 帧(自动同步)... (确保板子已上电并 USB 已枚举)")
    try:
        while True:
            if args.secs and (time.time() - t_start > args.secs):
                break
            frame = sync_stream(ser, timeout=2.0)
            if frame is None:
                continue
            res = frame_valid(frame)
            if res is None:
                continue
            msgid, payload = res
            name = MSG_NAMES.get(msgid, f"MSG_{msgid}")
            frames += 1
            ts = time.time() - t_start
            if msgid == 0:
                d = parse_heartbeat(payload)
                print(f"[{ts:6.2f}s] HEARTBEAT type={d['type']} armed={d['armed']} "
                      f"custom_mode={d['custom_mode']} status={d['status']}")
                if writer:
                    writer.writerow([f"{ts:.3f}", msgid, name,
                                     f"armed={d['armed']},custom={d['custom_mode']},status={d['status']}"])
            elif msgid == 32:
                d = parse_local_pos(payload)
                print(f"[{ts:6.2f}s] LOCAL_POS_NED x={d['x']:.3f} y={d['y']:.3f} z={d['z']:.3f} "
                      f"vx={d['vx']:.3f} vy={d['vy']:.3f} vz={d['vz']:.3f}")
                if writer:
                    writer.writerow([f"{ts:.3f}", msgid, name,
                                     f"x={d['x']:.4f},y={d['y']:.4f},z={d['z']:.4f},"
                                     f"vx={d['vx']:.4f},vy={d['vy']:.4f},vz={d['vz']:.4f}"])
            elif msgid == 1:
                d = parse_sys_status(payload)
                print(f"[{ts:6.2f}s] SYS_STATUS health=0x{d['health']:X} load={d['load_pct']:.1f}%")
                if writer:
                    writer.writerow([f"{ts:.3f}", msgid, name,
                                     f"health=0x{d['health']:X},load={d['load_pct']:.1f}"])
            else:
                print(f"[{ts:6.2f}s] {name} (len={len(payload)})")
                if writer:
                    writer.writerow([f"{ts:.3f}", msgid, name, f"len={len(payload)}"])
    except KeyboardInterrupt:
        print("\n[*] 用户中断")
    finally:
        ser.close()
        if csvf:
            csvf.close()
        print(f"[*] 共解析 {frames} 帧" + (f"，已导出 {args.csv}" if args.csv else ""))


if __name__ == "__main__":
    main()
