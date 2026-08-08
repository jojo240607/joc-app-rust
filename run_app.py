#!/usr/bin/env python3
"""复位并运行板子，同时监听 RTOS 调试控制台（COM8），捕获启动与 Rust 日志。

流程:
    1. 启动 OpenOCD（ST-Link + SWD），gdb_port=3333。
    2. 经 GDB `monitor reset run` 让板子从 Flash 启动运行（系统 + App 分区）。
    3. 监听 COM8 @115200 一段时间，回显所有串口输出。

用法:
    python run_app.py [PORT] [BAUDRATE] [SECONDS]
    python run_app.py                  # COM8 115200 监听 12s
    python run_app.py COM9 115200 20

说明:
    - 板子复位/上电瞬间 CH340 (COM8) 缓冲可能未与 reset run 同步，部分板子
      用 ST-Link `monitor reset run` 抓不到启动打印（但板子确实在跑）。
      若出现 0 字节输出，请用 `listen.py` 并在其 open 端口后手动按复位键。
    - 打开端口时强制 dtr=False / rts=False，避免 CH340 的 DTR 脉冲复位板子。
    - 系统日志前缀 `I/...`（C 侧）；Rust 应用层日志前缀 `R/<L> ...`（App 侧）。

依赖: OpenOCD (openocd.exe) 在硬编码路径；arm-none-eabi-gdb 在 PATH。
"""
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time

import serial

OCD_DIR = "D:/soft/openocd/openocd-4e78563-i686-w64-mingw32"
OCD_BIN = os.path.join(OCD_DIR, "bin", "openocd.exe")
OCD_SCR = os.path.join(OCD_DIR, "share", "openocd", "scripts")
GDB_PORT = 3333


def start_openocd():
    cfg = """
source [find interface/stlink.cfg]
transport select swd
source [find target/stm32f4x.cfg]
gdb_port %d
""" % GDB_PORT
    fd, cfg_path = tempfile.mkstemp(suffix=".cfg", prefix="ocd_run_")
    with os.fdopen(fd, "w") as f:
        f.write(cfg)
    proc = subprocess.Popen(
        [OCD_BIN, "-s", OCD_SCR, "-f", cfg_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    # 等待 GDB server 就绪
    for _ in range(20):
        try:
            s = socket.create_connection(("localhost", GDB_PORT), timeout=1)
            s.close()
            break
        except OSError:
            time.sleep(0.3)
    return proc, cfg_path


def reset_run():
    gdb_txt = (
        "set pagination off\n"
        f"target remote localhost:{GDB_PORT}\n"
        "monitor reset run\n"
        "detach\n"
        "quit\n"
    )
    fd, gdb_path = tempfile.mkstemp(suffix=".txt", prefix="gdb_run_")
    with os.fdopen(fd, "w") as f:
        f.write(gdb_txt)
    res = subprocess.run(
        ["arm-none-eabi-gdb", "-batch", "-x", gdb_path],
        capture_output=True, text=True,
    )
    try:
        os.remove(gdb_path)
    except OSError:
        pass
    return res.stdout.strip() + res.stderr.strip()


def stop_openocd(proc):
    try:
        proc.terminate()
        proc.wait(timeout=3)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def listen(port, baud, secs):
    try:
        s = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print(f"[ERROR] cannot open {port}: {e}")
        return
    s.dtr = False
    s.rts = False
    time.sleep(0.5)
    t = time.time()
    chunks = []
    try:
        while time.time() - t < secs:
            d = s.read(400)
            if d:
                chunks.append(d)
                sys.stdout.buffer.write(d)
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
    data = b"".join(chunks)
    print(f"\n[TOTAL={len(data)} HAS_APP={b'app mounted' in data} "
          f"HAS_FC={b'flyctrl' in data} HAS_HB={b'hb ' in data}]",
          file=sys.stderr)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    secs = int(sys.argv[3]) if len(sys.argv) > 3 else 12

    proc, cfg_path = start_openocd()
    try:
        print("[openocd] started, gdb_port=%d" % GDB_PORT, file=sys.stderr)
        print("[gdb]", reset_run(), file=sys.stderr)
        listen(port, baud, secs)
    finally:
        stop_openocd(proc)
        try:
            os.remove(cfg_path)
        except OSError:
            pass


if __name__ == "__main__":
    main()
