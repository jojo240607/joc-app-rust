#!/usr/bin/env python3
"""抓 COM12 原始字节，dump 前若干 0xFD 帧的 hex，辅助判断是否字节污染 / CRC 表错。"""
import os, socket, subprocess, sys, tempfile, time
import serial

OCD_DIR = "D:/soft/openocd/openocd-4e78563-i686-w64-mingw32"
OCD_BIN = os.path.join(OCD_DIR, "bin", "openocd.exe")
OCD_SCR = os.path.join(OCD_DIR, "share", "openocd", "scripts")
GDB_PORT = 3334
PORT, BAUD, SECS = "COM12", 115200, 6

def start_ocd():
    cfg = tempfile.mktemp(suffix=".cfg", prefix="ocd_d_")
    open(cfg, "w").write(
        "source [find interface/stlink.cfg]\n"
        "transport select swd\n"
        "source [find target/stm32f4x.cfg]\n"
        f"gdb_port {GDB_PORT}\n")
    p = subprocess.Popen([OCD_BIN, "-s", OCD_SCR, "-f", cfg],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(20):
        try:
            s = socket.create_connection(("localhost", GDB_PORT), timeout=1); s.close(); break
        except OSError:
            time.sleep(0.3)
    return p, cfg

def reset_run():
    g = tempfile.mktemp(suffix=".txt", prefix="gdb_d_")
    open(g, "w").write(
        "set pagination off\n"
        f"target remote localhost:{GDB_PORT}\n"
        "monitor reset run\n"
        "detach\nquit\n")
    subprocess.run(["arm-none-eabi-gdb", "-batch", "-x", g], capture_output=True, text=True)

# Do NOT attach OpenOCD — halting the core drops the CDC port. Capture the
# live downlink stream that the already-running board is streaming on COM12.
try:
    s = serial.Serial(PORT, BAUD, timeout=1)
    s.dtr = False; s.rts = False
    time.sleep(0.5)
    s.reset_input_buffer()
    buf = bytearray()
    t0 = time.time()
    # 收集足够字节后 dump 前 6 个 0xFD 出现位置周边的 hex
    while time.time() - t0 < SECS:
        b = s.read(512)
        if b:
            buf += b
        if len(buf) >= 1200:
            break
    s.close()
    # 找前若干个 0xFD 并各 dump 周围 30 字节
    shown = 0
    i = 0
    while i < len(buf) and shown < 8:
        j = buf.find(b'\xfd', i)
        if j < 0:
            break
        seg = bytes(buf[j:j+40])
        print(f"@off={j} hex={seg.hex()}")
        shown += 1
        i = j + 1
    print(f"\n[total_buf={len(buf)}]")
finally:
    try: s.close()
    except Exception: pass
