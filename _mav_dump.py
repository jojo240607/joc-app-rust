import serial, time

s = serial.Serial('COM12', 115200, timeout=0.5)
s.dtr = True; s.rts = True
time.sleep(0.3)
s.reset_input_buffer()
buf = bytearray()
shown = 0
t0 = time.time()
while time.time() - t0 < 4 and shown < 4:
    b = s.read(512)
    if b:
        buf += b
    # 找 magic 开头的位置
    idx = buf.find(b'\xfd')
    if idx >= 0 and idx + 10 <= len(buf):
        plen = buf[idx+1]
        total = 10 + plen + 2
        if idx + total <= len(buf):
            frame = bytes(buf[idx:idx+total])
            msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
            print(f"--- frame len={len(frame)} msgid={msgid} plen={plen} seq={frame[4]} sys={frame[5]} comp={frame[6]}")
            print(f"    header+payload crc_bytes={frame[-2]:02x}{frame[-1]:02x}")
            print(f"    hex={frame.hex()}")
            shown += 1
            del buf[:idx+total]
s.close()
