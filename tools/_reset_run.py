#!/usr/bin/env python3
"""让 ST-Link 连接的板子真正跑起来（reset run，不 halt）。

flash_app.py 末尾 `monitor reset halt` 会停在 halt 态，USB CDC 不枚举、COM9 不出现。
本脚本启动 OpenOCD + gdb，发 `monitor reset run` 后 detach，使板子从 App 分区正常启动。
"""
import subprocess, sys, os, time, signal, socket

BASE = os.path.dirname(os.path.abspath(__file__))
OCD_DIR = "D:/soft/openocd/openocd-4e78563-i686-w64-mingw32"
OCD_BIN = os.path.join(OCD_DIR, "bin", "openocd.exe")
OCD_SCR = os.path.join(OCD_DIR, "share", "openocd", "scripts")
GDB_PORT = 3334

def start_openocd():
    cfg = os.path.join(BASE, "_ocd_run.cfg")
    with open(cfg, "w") as f:
        f.write("source [find interface/stlink.cfg]\n")
        f.write("transport select swd\n")
        f.write("source [find target/stm32f4x.cfg]\n")
        f.write(f"gdb_port {GDB_PORT}\n")
    p = subprocess.Popen(
        [OCD_BIN, "-s", OCD_SCR, "-f", cfg, "-l", os.path.join(BASE, "ocd_run.log")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(20):
        try:
            s = socket.create_connection(("localhost", GDB_PORT), timeout=1)
            s.close(); return p
        except OSError:
            time.sleep(0.3)
    print("[ERR] OpenOCD 未就绪"); sys.exit(1)

def reset_run():
    gdb_cmds = f"""
target extended-remote localhost:{GDB_PORT}
monitor reset run
detach
quit
"""
    tmp = os.path.join(BASE, "_run.gdb")
    open(tmp, "w").write(gdb_cmds)
    r = subprocess.run(["arm-none-eabi-gdb", "-batch", "-x", tmp],
                       cwd=BASE, capture_output=True, text=True)
    print(r.stdout + r.stderr)

p = start_openocd()
try:
    reset_run()
finally:
    p.send_signal(signal.SIGTERM)
    try:
        p.wait(timeout=3)
    except Exception:
        p.kill()
print("[OK] 板子已 reset run，请等待 USB CDC 枚举（COM9 出现后重跑 _mavlink_probe.py）")
