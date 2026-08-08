#!/usr/bin/env python3
"""监听 RTOS 调试控制台（默认 COM8 @115200），捕获系统日志与 Rust 应用层日志。

用法:
    python listen.py [PORT] [BAUDRATE] [SECONDS]

示例:
    python listen.py              # COM8 115200 监听 18s
    python listen.py COM9 115200  # 指定端口
    python listen.py COM8 115200 30

注意:
    - 板子复位/上电瞬间 CH340 (COM8) 缓冲可能未同步，最可靠方式是在本脚本
      已 open 端口后，再手动按板子复位键（或断电重上电）触发启动打印。
    - 打开端口时强制 dtr=False / rts=False，避免 CH340 的 DTR 脉冲复位板子。
    - 系统日志前缀 `I/...`（C 侧），Rust 应用层日志前缀 `R/<L> ...`（App 侧），
      二者物理同串口、靠前缀区分。
"""
import serial
import sys
import time


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    secs = int(sys.argv[3]) if len(sys.argv) > 3 else 18

    try:
        s = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print(f"[ERROR] cannot open {port}: {e}")
        return 1
    s.dtr = False
    s.rts = False
    time.sleep(0.3)
    print(f"[listening {port} @{baud} for {secs}s] now press board RESET or power-cycle...",
          file=sys.stderr)
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
    has_app = b"app mounted" in data
    has_fc = b"flyctrl" in data
    has_hb = b"hb " in data
    has_ready = b"ready" in data
    print(f"\n[TOTAL={len(data)} HAS_READY={has_ready} HAS_APP={has_app} "
          f"HAS_FC={has_fc} HAS_HB={has_hb}]", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
