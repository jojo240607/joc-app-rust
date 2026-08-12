#!/usr/bin/env python3
"""USB 下行(COM12 CDC)验证：抓 MAVLink 帧，统计 raw/帧数/CRC 好坏，
并逐 msgid 检查 seq 连续性（暴露字节污染：seq 重复=污染，seq 跳变>1=丢帧）。
使用前需板子 USB 枚举（flash 后先 OpenOCD reset run 救活）。
"""
import os, socket, subprocess, sys, tempfile, time
import serial

OCD_DIR = "D:/soft/openocd/openocd-4e78563-i686-w64-mingw32"
OCD_BIN = os.path.join(OCD_DIR, "bin", "openocd.exe")
OCD_SCR = os.path.join(OCD_DIR, "share", "openocd", "scripts")
GDB_PORT = 3334
PORT, BAUD, SECS = "COM12", 115200, 15


def crc16_cont(crc, data):
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


# CRC_EXTRA 必须与 flyctrl/core/src/comm/mavlink.rs 完全一致：
# LOCAL_POSITION_NED=185（旧误用 143，导致 LP 帧 CRC 永远校验失败）、COMMAND_ACK=143（旧误用 208）。
CRC = {0: 50, 1: 124, 30: 39, 32: 185, 76: 152, 77: 143, 21: 159, 22: 220, 23: 168}


def try_parse(buf):
    i = 0
    while i < len(buf):
        if buf[i] != 0xFD:
            i += 1
            continue
        if i + 10 > len(buf):
            return None
        plen = buf[i + 1]
        if buf[i + 2] != 0 or buf[i + 3] != 0:
            return ("skip", i)
        total = 10 + plen + 2
        if i + total > len(buf):
            return None
        msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        crc = crc16_cont(0xFFFF, buf[i + 1:i + 10 + plen])
        crc = crc16_cont(crc, [CRC.get(msgid, 0)])
        fc = buf[i + total - 2] | (buf[i + total - 1] << 8)
        return (total, msgid, buf[i + 4], crc == fc)
    return None


def start_ocd():
    cfg = tempfile.mktemp(suffix=".cfg", prefix="ocd_u_")
    open(cfg, "w").write(
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
    g = tempfile.mktemp(suffix=".txt", prefix="gdb_u_")
    open(g, "w").write(
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


# NOTE: do NOT attach OpenOCD/gdb here — halting the core stops the USB ISR
# and the CDC port drops (COM12 vanishes). The board is already running and
# streaming telem downlink on COM12, so we just open it live and capture.
try:
    s = serial.Serial(PORT, BAUD, timeout=1)
    s.dtr = False
    s.rts = False
    time.sleep(0.5)
    s.reset_input_buffer()

    buf = bytearray()
    raw = 0
    frames = 0
    ok = 0
    bad = 0
    ids = {}
    seq_last = {}
    seq_dups = 0
    seq_jumps = 0
    t0 = time.time()
    while time.time() - t0 < SECS:
        b = s.read(256)
        if b:
            buf += b
            raw += len(b)
        while True:
            r = try_parse(buf)
            if r is None:
                break
            if isinstance(r, tuple) and len(r) == 2 and r[0] == "skip":
                del buf[:1]
                continue
            total, msgid, seq, crcok = r
            frames += 1
            if crcok:
                ok += 1
            else:
                bad += 1
            ids[msgid] = ids.get(msgid, 0) + 1
            if msgid in seq_last:
                d = (seq - seq_last[msgid]) & 0xFF
                if d == 0:
                    seq_dups += 1
                elif d != 1:
                    seq_jumps += 1
            seq_last[msgid] = seq
            del buf[:total]
        if len(buf) > 1024:
            del buf[:len(buf) - 512]
    s.close()
    print(f"raw={raw} frames={frames} crc_ok={ok} crc_bad={bad} "
          f"seq_dups={seq_dups} seq_jumps={seq_jumps} ids={ids} leftover={len(buf)}")
finally:
    try:
        s.close()
    except Exception:
        pass
