#!/usr/bin/env python3
"""启动 OpenOCD -> GDB 读 g_usb0->dbg_tx_snap 快照 -> 分析下行是否发送侧就丢字节。"""
import os, socket, subprocess, sys, tempfile, time

OCD_DIR = "D:/soft/openocd/openocd-4e78563-i686-w64-mingw32"
OCD_BIN = os.path.join(OCD_DIR, "bin", "openocd.exe")
OCD_SCR = os.path.join(OCD_DIR, "share", "openocd", "scripts")
GDB_PORT = 3334
ELF = "d:/project/mcu/oop/joc-base/build_rel/stm32f407_minimal.elf"

p, cfg = None, None
try:
    cfg = tempfile.mktemp(suffix=".cfg", prefix="ocd_s_")
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
    gdb_cmd = [
        "arm-none-eabi-gdb", "-batch",
        "-ex", f"file {ELF}",
        "-x", "_dump_snap.gdb",
    ]
    r = subprocess.run(gdb_cmd, capture_output=True, encoding="latin-1", errors="replace")
    print(r.stdout)
    if r.stderr:
        print("=== GDB STDERR ===")
        print(r.stderr[-2000:])
finally:
    if p:
        try: p.terminate(); p.wait(timeout=3)
        except Exception:
            try: p.kill()
            except Exception: pass
    if cfg:
        try: os.remove(cfg)
        except OSError: pass
