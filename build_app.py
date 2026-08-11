#!/usr/bin/env python3
"""joc-app-rust 阶段 2 独立构建：产出可烧录到 APP_FLASH 块的 app.bin。

流程：
  1) cargo build --target thumbv7em-none-eabihf --release  -> target/.../libapp.a
  2) arm-none-eabi-gcc -nostartfiles -T app.ld 链接 libapp.a -> app.elf
     （app.ld 在头部生成 app_header_t：magic/abi_version/entry）
  3) arm-none-eabi-objcopy -O binary app.elf app.bin

前置：
  - cargo + rustup（stable, thumbv7em-none-eabihf）
  - arm-none-eabi-gcc/objcopy 在 PATH
  - RTOS_ABI_VERSION 须与 joc-base 侧一致（app.ld 里硬编码 = 1，改契约时同步）

用法：
  python build_app.py            # 产出 app.bin
  python build_app.py --check    # 仅校验工具链 + 版本号
"""
import subprocess, sys, os, shutil, argparse

ROOT = os.path.dirname(os.path.abspath(__file__))
TARGET = "thumbv7em-none-eabihf"
RTOS_ABI_VERSION = 1  # MUST == joc-base RTOS_ABI_VERSION（app.ld 头部硬编码）

def run(cmd):
    print("+", " ".join(cmd), flush=True)
    subprocess.check_call(cmd, cwd=ROOT)

def check_tools():
    ok = True
    for t in ("cargo", "arm-none-eabi-gcc", "arm-none-eabi-objcopy"):
        if shutil.which(t) is None:
            print(f"[ERR] 工具未找到: {t}（请加入 PATH）", file=sys.stderr)
            ok = False
    return ok

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="仅校验工具链")
    ap.add_argument("--features", default="", help="cargo 构建 feature（如 usbtest/demo/real-sensors）")
    args = ap.parse_args()

    if not check_tools():
        sys.exit(1)
    if args.check:
        print(f"[OK] 工具链就绪；RTOS_ABI_VERSION={RTOS_ABI_VERSION}")
        return

    # 1) cargo -> libapp.a
    cargo_cmd = ["cargo", "build", "--target", TARGET, "--release"]
    if args.features:
        cargo_cmd += ["--features", args.features]
    run(cargo_cmd)
    libapp = os.path.join(ROOT, "target", TARGET, "release", "libapp.a")
    if not os.path.exists(libapp):
        print(f"[ERR] 未找到 {libapp}", file=sys.stderr); sys.exit(1)

    # 2) 链接（app.ld 生成头部 + 定位到 APP_FLASH/APP_RAM）
    app_elf = os.path.join(ROOT, "app.elf")
    run(["arm-none-eabi-gcc", "-nostartfiles", "-T", "app.ld",
         "-Wl,--gc-sections", "-Wl,--no-warn-rwx-segments",
         "-o", app_elf, libapp, "-lgcc"])

    # 3) objcopy -> app.bin
    app_bin = os.path.join(ROOT, "app.bin")
    run(["arm-none-eabi-objcopy", "-O", "binary", app_elf, app_bin])
    sz = os.path.getsize(app_bin)
    print(f"[OK] app.bin 产出: {app_bin} ({sz} bytes, 须 < {384*1024})")
    if sz > 384 * 1024:
        print("[ERR] App 镜像超过 APP_FLASH 384K 预算", file=sys.stderr); sys.exit(1)

if __name__ == "__main__":
    main()
