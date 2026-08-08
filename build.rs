//! 构建期 ABI 版本校验：确保 Rust 侧镜像的 RTOS_ABI_VERSION 与 joc-rtos
//! 提供的 abi/rtos_abi.h 一致，防止契约漂移导致运行时诡异崩溃。

use std::fs;
use std::path::Path;

/// Rust 侧声明的契约版本（改 C 侧 rtos_abi.h 的 RTOS_ABI_VERSION 时必须同步此处）。
const RUST_ABI_VERSION: u32 = 1;

fn main() {
    // 从 abi/rtos_abi.h 提取 #define RTOS_ABI_VERSION N
    let hdr = Path::new("abi/rtos_abi.h");
    let text = fs::read_to_string(hdr).expect("找不到 abi/rtos_abi.h；请先从 joc-rtos 同步契约头");
    let ver = text
        .lines()
        .find_map(|l| {
            let l = l.trim();
            l.strip_prefix("#define RTOS_ABI_VERSION")
                .map(|v| v.trim().parse::<u32>().ok())
                .flatten()
        })
        .expect("rtos_abi.h 中未找到 RTOS_ABI_VERSION 宏");

    if ver != RUST_ABI_VERSION {
        panic!(
            "RTOS ABI 版本不匹配：rtos_abi.h={ver}, Rust 侧={RUST_ABI_VERSION}。\
             契约已变更，请同步 src/abi.rs 字段并更新 RUST_ABI_VERSION。"
        );
    }
    println!("cargo:rerun-if-changed=abi/rtos_abi.h");
}
