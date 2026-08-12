#!/usr/bin/env python3
"""字节级结构验证：COM12 流应严格为 [HB21][LP40][SS43]=104B/loop 循环。

App(修复后)每 loop 写 3 帧共享 seq：HB(msgid0,21B) LP(msgid32,40B) SS(msgid1,43B)。
用正确 CRC 在字节流上滑动定位每帧，检查是否严格 104B/loop 无重复/错位。
"""
import sys, time, serial
import mavlink as ml

def crc16(crc, data):
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF

def main():
    port = ml.find_cdc() or "COM12"
    print(f"CDC={port}", flush=True)
    s = serial.Serial(port, 115200, timeout=1)
    s.dtr = False; s.rts = False
    time.sleep(0.5); s.reset_input_buffer()
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < 12:
        b = s.read(256)
        if b:
            buf += b
    s.close()
    data = bytes(buf)
    print(f"TOTAL={len(data)}")

    # 用 CRC 严格解析：从每个 0xFD 扫描完整帧(CRC 通过才算)，记录 (off,msgid,seq,len)
    frames = []
    i = 0
    n = len(data)
    while i < n - 11:
        if data[i] != 0xFD:
            i += 1; continue
        plen = data[i+1]
        total = 10 + plen + 2
        if i + total > n:
            break
        msgid = data[i+7] | (data[i+8] << 8) | (data[i+9] << 16)
        seq = data[i+4]
        c = crc16(0xFFFF, data[i+1:i+10+plen])
        c = crc16(c, [ml.CRC_EXTRA.get(msgid, 0)])
        fc = data[i+total-2] | (data[i+total-1] << 8)
        if c == fc:
            frames.append((i, msgid, seq, total))
        i += 1  # 逐字节滑，找所有合法帧位置
    print(f"CRC_VALID_FRAMES={len(frames)}")
    # 检查每帧之间是否有重叠/间隙，以及 msgid 序列
    # 期望：HB(0) LP(32) SS(1) 严格 21/40/43 连续
    if len(frames) > 3:
        print("first 10 frames (off,msgid,seq,len):")
        for f in frames[:10]:
            print("   ", f)
        # 检查相邻帧边界连续性：下一帧 off 是否 == 上一帧 off+len
        cont = 0
        overlap = 0
        gap = 0
        for k in range(len(frames)-1):
            expected = frames[k][0] + frames[k][3]
            if frames[k+1][0] == expected:
                cont += 1
            elif frames[k+1][0] < expected:
                overlap += 1
            else:
                gap += 1
        print(f"contiguous={cont} overlap={overlap} gap={gap} (of {len(frames)-1})")
        # 检查 msgid 循环模式
        mids = [f[1] for f in frames[:30]]
        print("first 30 msgid:", mids)
        # 检查是否严格 0,32,1 循环
        from collections import Counter
        c3 = Counter()
        for k in range(len(frames)-2):
            t = (frames[k][1], frames[k+1][1], frames[k+2][1])
            c3[t] += 1
        print("msgid triple pattern:", c3.most_common(6))

if __name__ == "__main__":
    main()
