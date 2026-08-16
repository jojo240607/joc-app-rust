#!/usr/bin/env python3
"""MISSION 航点 + RC_OVERRIDE 联调验证（板子 uplink 侧握手实测）。

流程：
  1) 上传 3 个航点（MISSION_COUNT -> 板子回 MISSION_REQUEST(0..2) -> 发 ITEM_INT -> 板子存 -> 回 MISSION_ACK）
  2) 下载航点（MISSION_REQUEST_LIST -> 板子回 MISSION_COUNT + 3x ITEM_INT + MISSION_ACK）
  3) 发 RC_CHANNELS_OVERRIDE 验证无报错（板子存 + 标记 valid）

依赖 tools/mavlink.py。

用法：
  python tools/verify_mission.py [PORT]
"""
import sys, time, serial
sys.path.insert(0, 'tools')
import mavlink as ml

PORT = sys.argv[1] if len(sys.argv) > 1 else (ml.find_cdc() or 'COM12')

# 3 个测试航点（lat/lon 微度，alt 米）
TEST_WPS = [
    {'seq': 0, 'command': 22, 'x': 473602700, 'y': 85150000, 'z': 20.0, 'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0},
    {'seq': 1, 'command': 16, 'x': 473602800, 'y': 85150100, 'z': 30.0, 'param1': 15, 'param2': 0, 'param3': 0, 'param4': 0},
    {'seq': 2, 'command': 21, 'x': 473602900, 'y': 85150200, 'z': 0.0, 'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0},
]

def open_port(port):
    s = serial.Serial(port, 115200, timeout=0.5)
    s.dtr = False; s.rts = False
    time.sleep(0.3)
    s.reset_input_buffer()
    return s

def read_frames(s, dur=2.0):
    """读取 dur 秒，返回 (frames_dict, raw)。"""
    buf = b''
    t0 = time.time()
    while time.time() - t0 < dur:
        try:
            b = s.read(256)
        except Exception:
            break
        if b:
            buf += b
    out = []
    for f in ml.scan_frames(buf):
        out.append(f)
    return out, buf

def main():
    s = open_port(PORT)
    seq = 0
    print(f"[*] port={PORT}, 上传 {len(TEST_WPS)} 个航点")

    # ── 上传 ──
    s.write(ml.enc_mission_count(len(TEST_WPS), seq)); seq += 1
    time.sleep(0.3)
    for wp in TEST_WPS:
        fr, _ = read_frames(s, dur=0.5)  # 板子应回 MISSION_REQUEST(next)
        reqs = [f for f in fr if f.msgid == 40]
        if reqs:
            want = wp['seq']
            got = int.from_bytes(reqs[-1].payload[2:4], 'little')
            print(f"    WP{wp['seq']}: 收到 MISSION_REQUEST seq={got} {'OK' if got == wp['seq'] else 'MISMATCH'}")
        else:
            print(f"    WP{wp['seq']}: 未收到 MISSION_REQUEST！")
        s.write(ml.enc_mission_item_int(wp, seq)); seq += 1
        time.sleep(0.2)
    # 最后应回 MISSION_ACK
    fr, _ = read_frames(s, dur=0.8)
    acks = [f for f in fr if f.msgid == 47]
    if acks:
        t = acks[-1].payload[2]
        print(f"[UPLOAD] MISSION_ACK type={t} {'ACCEPTED' if t == 0 else 'ERROR'}")
    else:
        print("[UPLOAD] 未收到 MISSION_ACK！")

    # ── 下载 ──
    print("[*] 下载航点")
    s.write(ml.enc_mission_request_list(seq)); seq += 1
    fr, _ = read_frames(s, dur=1.5)
    counts = [f for f in fr if f.msgid == 44]
    items = [f for f in fr if f.msgid == 73]
    acks = [f for f in fr if f.msgid == 47]
    cnt = int.from_bytes(counts[-1].payload[2:4], 'little') if counts else -1
    print(f"[DOWNLOAD] MISSION_COUNT={cnt}, ITEM_INT 收到 {len(items)} 个, ACK={'YES' if acks else 'NO'}")
    if cnt == len(TEST_WPS) and len(items) == len(TEST_WPS):
        print("[DOWNLOAD] 航点数一致 OK")
    else:
        print("[DOWNLOAD] 航点数不一致 FAIL")

    # ── RC_OVERRIDE ──
    print("[*] 发 RC_CHANNELS_OVERRIDE (ch3=throttle 1500)")
    ch = [1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000]
    s.write(ml.enc_rc_channels_override(ch, seq)); seq += 1
    time.sleep(0.3)
    fr, _ = read_frames(s, dur=0.5)
    # RC_OVERRIDE 不回 ACK，只验证无异常（板子存）
    print(f"[RC_OVERRIDE] 发送完成，板子侧应已存 8 通道 PWM（COM8 看 'RC_OVERRIDE' 日志）")

    s.close()
    print("[*] done")

if __name__ == '__main__':
    main()
