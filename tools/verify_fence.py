#!/usr/bin/env python3
"""阶段二：FENCE 围栏上传/下载 验证脚本。

物理链路：板子 USB CDC（ST VCP，VID_0483/PID_5740） <-> 本机 COM 口。
与 verify_mission.py 同款手法：发上行帧 + 抓下行 FENCE_POINT 回包验证。

用法：
  python tools/verify_fence.py COM12

前置条件（与 verify_uplink.py 一致，务必遵守，否则误判）：
  1) 板子须脱离 halt 态：flash_app.py 烧完留 halt 态，需先让板子跑起来
     （串口助手/OpenOCD monitor reset run；本脚本只发帧不复位）。
  2) 端口不被其他进程占用（串口助手开着会读 0）。
  3) 板子侧 FENCE 握手日志经 COM8（115200）输出，本脚本只看 COM12 的下行回包。
"""
import sys
import time
import serial
sys.path.insert(0, 'tools')
from mavlink import (find_cdc, frame, crc16, CRC_EXTRA, MAGIC,
                     enc_fence_point, enc_fence_fetch_point)

PORT_DEFAULT = 'COM12'

# 预设 3 个围栏顶点（lat/lon 用 1e7 整数度，模拟一个三角禁飞区）
FENCE_PTS = [
    (375010000, 1212900000),  # (37.5010000, 121.2900000)
    (375020000, 1212950000),  # (37.5020000, 121.2950000)
    (375015000, 1213000000),  # (37.5015000, 121.3000000)
]


def wait_frame(ser, msgid, timeout=3.0):
    """从串口读字节，滑动扫描直到出现 msgid 且 CRC 正确的帧；返回 payload 或 None。"""
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
    print(f"FENCE verify: opening {port} ...")
    ser = serial.Serial(port, 115200, timeout=0.1, dsrdtr=False)
    ser.dtr = False
    ser.rts = False
    time.sleep(0.3)

    seq = 0
    count = len(FENCE_PTS)
    # ── 上传：逐个 FENCE_POINT ──
    print(f"[1] upload {count} fence points ...")
    for idx, (lat, lon) in enumerate(FENCE_PTS):
        seq += 1
        ser.write(enc_fence_point(idx, count, lat, lon, seq % 256))
        time.sleep(0.05)
    print(f"    sent {count} FENCE_POINT frames (count={count})")

    # ── 下载：逐个 FENCE_FETCH_POINT 并核对回包 ──
    print(f"[2] download + verify ...")
    ok = 0
    for idx, (lat, lon) in enumerate(FENCE_PTS):
        seq += 1
        ser.write(enc_fence_fetch_point(idx, seq % 256))
        pl = wait_frame(ser, 160, timeout=2.0)
        if pl is None:
            print(f"    idx={idx}: NO REPLY (FAIL)")
            continue
        r_lat = int.from_bytes(pl[4:8], 'little', signed=True)
        r_lon = int.from_bytes(pl[8:12], 'little', signed=True)
        r_count = pl[3]
        match = (r_lat == lat and r_lon == lon and r_count == count)
        print(f"    idx={idx}: lat={r_lat} lon={r_lon} count={r_count} -> {'OK' if match else 'MISMATCH'}")
        if match:
            ok += 1
    print(f"[RESULT] FENCE upload/download: {ok}/{count} matched")
    print("OK" if ok == count else "FAIL")
    ser.close()


if __name__ == "__main__":
    main()
