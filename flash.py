#!/usr/bin/env python3
"""joc-app-rust 独立烧录脚本（轨 B：系统区 + App 分区）。

在本工程目录下即可一条龙烧录，无需切到 joc-base：
  python flash.py            # build_app.py 生成 app.bin + 烧录
  python flash.py --no-build # 跳过 build_app.py，仅烧录

依赖（绝对路径已硬编码，按本机环境）：
  - OpenOCD: D:/soft/openocd/openocd-4e78563-i686-w64-mingw32
  - 系统镜像: ../joc-base/build_stage2/stm32f407_minimal.bin
  - 交叉工具: arm-none-eabi-gdb (需在 PATH)
"""
import subprocess, sys, os, time, signal

BASE = os.path.dirname(os.path.abspath(__file__))

# ---- 路径配置（按需修改）----
OCD_DIR  = "D:/soft/openocd/openocd-4e78563-i686-w64-mingw32"
OCD_BIN  = os.path.join(OCD_DIR, "bin", "openocd.exe")
OCD_SCR  = os.path.join(OCD_DIR, "share", "openocd", "scripts")
SYS_BIN  = os.path.join(BASE, "..", "joc-base", "build_rel", "stm32f407_minimal.bin")
APP_BIN  = os.path.join(BASE, "app.bin")
APP_LDS  = os.path.join(BASE, "app.ld")
GDB_TMP  = os.path.join(BASE, "_flash.gdb")

SYS_ADDR = "0x08000000"
APP_ADDR = "0x08060000"
GDB_PORT = 3333

def run(cmd, **kw):
    print(">>", " ".join(cmd))
    return subprocess.run(cmd, **kw)

def build_app():
    print("== 生成独立 App 镜像 app.bin ==")
    r = run([sys.executable, "build_app.py"], cwd=BASE)
    if r.returncode != 0:
        print("[ERR] build_app.py 失败"); sys.exit(1)

def start_openocd():
    if not os.path.exists(OCD_BIN):
        print(f"[ERR] 找不到 openocd: {OCD_BIN}"); sys.exit(1)
    cfg = os.path.join(BASE, "_ocd_flash.cfg")
    with open(cfg, "w") as f:
        f.write("source [find interface/stlink.cfg]\n")
        f.write("transport select swd\n")
        f.write("source [find target/stm32f4x.cfg]\n")
        f.write(f"gdb_port {GDB_PORT}\n")
    p = subprocess.Popen(
        [OCD_BIN, "-s", OCD_SCR, "-f", cfg, "-l", os.path.join(BASE, "ocd_flash.log")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # 等 GDB server 就绪
    for _ in range(20):
        try:
            import socket
            s = socket.create_connection(("localhost", GDB_PORT), timeout=1)
            s.close(); return p
        except OSError:
            time.sleep(0.3)
    print("[ERR] OpenOCD 未就绪，看 ocd_flash.log"); sys.exit(1)

def flash():
    for path, name in [(SYS_BIN, "系统镜像"), (APP_BIN, "app.bin")]:
        if not os.path.exists(path):
            print(f"[ERR] 缺{name}: {path}"); sys.exit(1)
    sys_g = os.path.abspath(SYS_BIN).replace("\\", "/")
    app_g = os.path.abspath(APP_BIN).replace("\\", "/")
    gdb_cmds = f"""
target extended-remote localhost:{GDB_PORT}
monitor reset halt
monitor flash write_image erase {sys_g} {SYS_ADDR}
monitor flash write_image erase {app_g} {APP_ADDR}
monitor reset halt
detach
quit
"""
    open(GDB_TMP, "w").write(gdb_cmds)
    print(f"== 烧录 系统区({SYS_ADDR}) + App分区({APP_ADDR}) ==")
    r = run(["arm-none-eabi-gdb", "-batch", "-x", GDB_TMP], cwd=BASE,
            capture_output=True, text=True)
    out = r.stdout + r.stderr
    print(out)
    if "wrote" in out.lower() or "written" in out.lower() or "verified" in out.lower():
        print("[OK] 烧录完成")
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
        try: p.wait(timeout=3)
        except Exception: p.kill()

if __name__ == "__main__":
    main()
