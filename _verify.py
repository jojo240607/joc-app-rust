#!/usr/bin/env python3
"""临时验证：先 open 串口(监听) -> 1s 后 OpenOCD reset run -> 捕获启动+Rust日志。"""
import os, socket, subprocess, sys, tempfile, time, threading
import serial

OCD_DIR = "D:/soft/openocd/openocd-4e78563-i686-w64-mingw32"
OCD_BIN = os.path.join(OCD_DIR, "bin", "openocd.exe")
OCD_SCR = os.path.join(OCD_DIR, "share", "openocd", "scripts")
GDB_PORT = 3334
PORT, BAUD, SECS = "COM8", 115200, 20

def start_ocd():
    cfg = tempfile.mktemp(suffix=".cfg", prefix="ocd_v_")
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
    g = tempfile.mktemp(suffix=".txt", prefix="gdb_v_")
    open(g, "w").write(
        "set pagination off\n"
        f"target remote localhost:{GDB_PORT}\n"
        "monitor reset run\n"
        "detach\nquit\n")
    subprocess.run(["arm-none-eabi-gdb", "-batch", "-x", g],
                   capture_output=True, text=True)
    try: os.remove(g)
    except OSError: pass

p, cfg = start_ocd()
try:
    s = serial.Serial(PORT, BAUD, timeout=1)
    s.dtr = False; s.rts = False
    time.sleep(0.5)
    print("[ocd] ready, will reset in 1s...", file=sys.stderr)
    threading.Timer(1.0, reset_run).start()
    t = time.time(); chunks = []
    while time.time() - t < SECS:
        d = s.read(400)
        if d:
            chunks.append(d); sys.stdout.buffer.write(d); sys.stdout.flush()
    s.close()
    data = b"".join(chunks)
    print(f"\n[TOTAL={len(data)} HAS_READY={b'ready' in data} "
          f"HAS_APP={b'app mounted' in data} HAS_FC={b'flyctrl' in data} "
          f"HAS_FIRST={b'first loop done' in data} HAS_HB={b'hb ' in data}]", file=sys.stderr)
finally:
    try: p.terminate(); p.wait(timeout=3)
    except Exception:
        try: p.kill()
        except Exception: pass
    try: os.remove(cfg)
    except OSError: pass
