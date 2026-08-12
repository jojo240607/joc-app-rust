import serial, time

def crc16_cont(crc, data):
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF

# 标准 MAVLink v2 CRC_EXTRA（与 flyctrl/core/src/comm/mavlink.rs 对齐）
CRC = {0: 50, 1: 124, 30: 39, 32: 143, 76: 152, 77: 208, 21: 159, 22: 220, 23: 168}

def try_parse(buf, crc_extra):
    """在 buf 中找第一处合法的 MAVLink v2 帧。

    返回 (total, msgid, crcok) 或 None（无完整帧）。
    注意：调用方负责在解析失败时把 buf[0]（疑似错误 magic）丢弃并重新扫描，
    否则一个落单的 0xFD（payload 中巧合出现 / 错位）会卡死整个解析。
    """
    i = 0
    while i < len(buf):
        if buf[i] != 0xFD:
            i += 1
            continue
        if i + 10 > len(buf):
            return None  # 头部尚未收齐，等更多数据
        plen = buf[i + 1]
        # incompat/compat 标志本实现恒为 0；非 0 视作错位，跳过本字节
        if buf[i + 2] != 0 or buf[i + 3] != 0:
            return ("skip", i)
        total = 10 + plen + 2
        if i + total > len(buf):
            return None  # 帧未收齐，等更多数据
        msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        crc = crc16_cont(0xFFFF, buf[i + 1:i + 10 + plen])
        crc = crc16_cont(crc, [crc_extra.get(msgid, 0)])
        frame_crc = buf[i + total - 2] | (buf[i + total - 1] << 8)
        return (total, msgid, crc == frame_crc)
    return None

s = serial.Serial('COM12', 115200, timeout=0.5)
s.dtr = False; s.rts = False
time.sleep(0.3)
s.reset_input_buffer()
buf = bytearray()
frames = 0; ok = 0; bad = 0; ids = {}
t0 = time.time()
while time.time() - t0 < 5:
    b = s.read(256)
    if b:
        buf += b
    while True:
        r = try_parse(buf, CRC)
        if r is None:
            break
        if isinstance(r, tuple) and len(r) == 2 and r[0] == "skip":
            # 错位：丢弃该 0xFD 字节后重扫
            del buf[:1]
            continue
        total, msgid, crcok = r
        frames += 1
        if crcok: ok += 1
        else: bad += 1
        ids[msgid] = ids.get(msgid, 0) + 1
        del buf[:total]
    # 限制缓冲增长（防止错位时无限堆积）
    if len(buf) > 1024:
        del buf[:len(buf) - 512]
s.close()
print(f"frames={frames} crc_ok={ok} crc_bad={bad} ids={ids} leftover={len(buf)}")
