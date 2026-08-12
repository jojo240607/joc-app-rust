#!/usr/bin/env python3
"""UART 控制台捕获：启动 OpenOCD(SWD/ST-Link) -> open 串口监听 -> 1s 后 reset run ->
捕获启动 + Rust App 日志到 stdout。用于抓 boot 日志 / App 启动卡死排查。

注意：COM8 是调试 USART(CH340)，与 USB CDC(COM12) 不同。本脚本针对 COM8。
依赖：openocd（在 PATH）或 OCD_DIR 指向安装目录；arm-none-eabi-gdb 在 PATH。

用法：
  python tools/capture_log.py [PORT] [SECS]
  python tools/capture_log.py COM8 20
"""
import argparse
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time

import serial

GDB_PORT = 3334
OCD_DIR = os.environ.get("OCD_DIR", "D:/soft/openocd/openocd-4e78563-i686-w64-mingw32")
OCD_BIN = os.path.join(OCD_DIR, "bin", "openocd.exe")
OCD_SCR = os.path.join(OCD_DIR, "share", "openocd", "scripts")


def start_ocd():
    cfg = tempfile.mktemp(suffix=".cfg", prefix="ocd_cap_")
    with open(cfg, "w") as fh:
        fh.write(
            "source [find interface/stlink.cfg]\n"
            "transport select swd\n"
            "source [find target/stm32f4x.cfg]\n"
            f"gdb_port {GDB_PORT}\n")
    p = subprocess.Popen([OCD_BIN, "-s", OCD_SCR, "-f", cfg],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(20):
        try:
            s = socket.create_connection(("localhost", GDB_PORT), timeout=1)
            s.close()
            break
        except OSError:
            time.sleep(0.3)
    return p, cfg


def reset_run():
    g = tempfile.mktemp(suffix=".txt", prefix="gdb_cap_")
    with open(g, "w") as fh:
        fh.write(
            "set pagination off\n"
            f"target remote localhost:{GDB_PORT}\n"
            "monitor reset run\n"
            "detach\nquit\n")
    subprocess.run(["arm-none-eabi-gdb", "-batch", "-x", g],
                   capture_output=True, text=True)
    try:
        os.remove(g)
    except OSError:
        pass


def main():
    ap = argparse.ArgumentParser(description="UART 控制台捕获 (COM8, CH340)")
    ap.add_argument("port", nargs="?", default="COM8", help="串口 (默认 COM8)")
    ap.add_argument("secs", nargs="?", type=int, default=20, help="捕获时长秒")
    args = ap.parse_args()
    port, secs = args.port, args.secs
    print(f"[*] port={port} secs={secs}", file=sys.stderr)

    p, cfg = start_ocd()
    try:
        s = serial.Serial(port, 115200, timeout=1)
        s.dtr = False
        s.rts = False
        time.sleep(0.5)
        print("[ocd] ready, will reset in 1s...", file=sys.stderr)
        threading.Timer(1.0, reset_run).start()
        t = time.time()
        chunks = []
        while time.time() - t < secs:
            d = s.read(400)
            if d:
                chunks.append(d)
                sys.stdout.buffer.write(d)
                sys.stdout.flush()
        s.close()
        data = b"".join(chunks)
        print(f"\n[TOTAL={len(data)} HAS_READY={b'ready' in data} "
              f"HAS_APP={b'app mounted' in data} HAS_FC={b'flyctrl' in data} "
              f"HAS_HB={b'hb ' in data}]", file=sys.stderr)
    finally:
        try:
            p.terminate()
            p.wait(timeout=3)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
        try:
            os.remove(cfg)
        except OSError:
            pass


if __name__ == "__main__":
    main()
