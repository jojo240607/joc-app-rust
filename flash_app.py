#!/usr/bin/env python3
"""joc-app-rust 仅重烧 App 分区（轨 B 快速迭代）。

仅烧录 APP_FLASH 块（0x08060000），不碰系统区，适合 App 代码改动后的
快速验证（系统区已烧过的前提下）。系统区首烧仍用 flash.py。

用法:
    python flash_app.py           # 先 build_app.py 再只烧 app 分区
    python flash_app.py --no-build # 跳过构建，直接烧现有 app.bin
"""
import subprocess, sys, os, time, signal, socket

BASE = os.path.dirname(os.path.abspath(__file__))

OCD_DIR = "D:/soft/openocd/openocd-4e78563-i686-w64-mingw32"
OCD_BIN = os.path.join(OCD_DIR, "bin", "openocd.exe")
OCD_SCR = os.path.join(OCD_DIR, "share", "openocd", "scripts")
APP_BIN = os.path.join(BASE, "app.bin")
APP_ADDR = "0x08060000"
GDB_PORT = 3333


def build_app():
    print("== 生成独立 App 镜像 app.bin ==")
    r = subprocess.run([sys.executable, "build_app.py"], cwd=BASE)
    if r.returncode != 0:
        print("[ERR] build_app.py 失败"); sys.exit(1)


def start_openocd():
    if not os.path.exists(OCD_BIN):
        print(f"[ERR] 找不到 openocd: {OCD_BIN}"); sys.exit(1)
    cfg = os.path.join(BASE, "_ocd_app.cfg")
    with open(cfg, "w") as f:
        f.write("source [find interface/stlink.cfg]\n")
        f.write("transport select swd\n")
        f.write("source [find target/stm32f4x.cfg]\n")
        f.write(f"gdb_port {GDB_PORT}\n")
    p = subprocess.Popen(
        [OCD_BIN, "-s", OCD_SCR, "-f", cfg, "-l", os.path.join(BASE, "ocd_app.log")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(20):
        try:
            s = socket.create_connection(("localhost", GDB_PORT), timeout=1)
            s.close(); return p
        except OSError:
            time.sleep(0.3)
    print("[ERR] OpenOCD 未就绪，看 ocd_app.log"); sys.exit(1)


def flash():
    if not os.path.exists(APP_BIN):
        print(f"[ERR] 缺 app.bin: {APP_BIN}"); sys.exit(1)
    app_g = os.path.abspath(APP_BIN).replace("\\", "/")
    gdb_cmds = f"""
target extended-remote localhost:{GDB_PORT}
monitor reset halt
monitor flash write_image erase {app_g} {APP_ADDR}
monitor reset halt
detach
quit
"""
    tmp = os.path.join(BASE, "_app.gdb")
    open(tmp, "w").write(gdb_cmds)
    print(f"== 仅烧录 App 分区({APP_ADDR}) ==")
    r = subprocess.run(["arm-none-eabi-gdb", "-batch", "-x", tmp],
                       cwd=BASE, capture_output=True, text=True)
    out = r.stdout + r.stderr
    print(out)
    if "wrote" in out.lower() or "verified" in out.lower():
        print("[OK] App 分区烧录完成")
    else:
        print("[WARN] 未检测到 wrote/verified，检查上面输出")


def main():
    if "--no-build" not in sys.argv:
        build_app()
    p = start_openocd()
    try:
        flash()
    finally:
        p.send_signal(signal.SIGTERM)
        try:
            p.wait(timeout=3)
        except Exception:
            p.kill()


if __name__ == "__main__":
    main()
