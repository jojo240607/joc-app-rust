#!/usr/bin/env python3
"""joc-app-rust 交互式调试脚本（轨 B）。

在本工程目录下运行：python debug.py
  - 自启 OpenOCD (GDB server :3333)
  - GDB 加载系统 ELF + App ELF 符号
  - reset halt 后在 rust_app_start 断住（App 挂载入口）
  - 进入交互式 GDB，方便单步/continue/查变量

依赖（绝对路径已硬编码）：
  - OpenOCD: D:/soft/openocd/openocd-4e78563-i686-w64-mingw32
  - 系统 ELF: ../joc-base/build_rel/stm32f407_minimal.elf
  - 交叉工具: arm-none-eabi-gdb (需在 PATH)
"""
import subprocess, sys, os, time, signal, socket

BASE = os.path.dirname(os.path.abspath(__file__))

OCD_DIR = "D:/soft/openocd/openocd-4e78563-i686-w64-mingw32"
OCD_BIN = os.path.join(OCD_DIR, "bin", "openocd.exe")
OCD_SCR = os.path.join(OCD_DIR, "share", "openocd", "scripts")
SYS_ELF = os.path.join(BASE, "..", "joc-base", "build_rel", "stm32f407_minimal.elf")
APP_ELF = os.path.join(BASE, "app.elf")
APP_LOAD = "0x08060000"   # 与 app.ld APP_FLASH 一致
GDB_PORT = 3333

def need(p, what):
    if not os.path.exists(p):
        print(f"[ERR] 缺少{what}: {p}"); sys.exit(1)

def start_openocd():
    need(OCD_BIN, "openocd")
    cfg = os.path.join(BASE, "_ocd_dbg.cfg")
    open(cfg, "w").write(
        "source [find interface/stlink.cfg]\n"
        "transport select swd\n"
        "source [find target/stm32f4x.cfg]\n"
        f"gdb_port {GDB_PORT}\n")
    p = subprocess.Popen(
        [OCD_BIN, "-s", OCD_SCR, "-f", cfg, "-l", os.path.join(BASE, "ocd_dbg.log")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(20):
        try:
            s = socket.create_connection(("localhost", GDB_PORT), timeout=1); s.close(); return p
        except OSError:
            time.sleep(0.3)
    print("[ERR] OpenOCD 未就绪，看 ocd_dbg.log"); p.kill(); sys.exit(1)

def main():
    need(SYS_ELF, "系统 ELF (joc-base 需 build_rel.bat / -DRTOS_SELFTEST=OFF 构建)")
    need(APP_ELF, "App ELF (先 python build_app.py 或 flash.py)")
    ocd = start_openocd()
    sys_e = os.path.abspath(SYS_ELF).replace("\\", "/")
    app_e = os.path.abspath(APP_ELF).replace("\\", "/")
    gdb_cmd = f"""
set pagination off
target extended-remote localhost:{GDB_PORT}
monitor reset halt
file {sys_e}
add-symbol-file {app_e} {APP_LOAD}
hbreak rust_app_start
echo \\n*** 已在 rust_app_start 前 halt，输入 continue 运行，或 step 单步 ***\n
continue
"""
    gdb_tmp = os.path.join(BASE, "_dbg.gdb")
    open(gdb_tmp, "w").write(gdb_cmd)
    try:
        # 交互式：保留 GDB 终端给用户
        subprocess.run(["arm-none-eabi-gdb", "-x", gdb_tmp], cwd=BASE)
    finally:
        ocd.send_signal(signal.SIGTERM)
        try: ocd.wait(timeout=3)
        except Exception: ocd.kill()
        print("\n[info] OpenOCD 已关闭")

if __name__ == "__main__":
    main()
