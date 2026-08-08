/* app/rust/rust_app.h — RTOS 侧对 Rust 应用层（joc-app-rust/libapp.a）的契约声明。
 *
 * 设计：RTOS 固件【不包含】任何 demo / 飞控逻辑。用户层任务全部由 Rust 工程
 * 在 rust_app_start() 内自行经 ABI 契约创建（见 rtos_abi.h）。
 *
 * 工程组织：joc-app-rust 与 joc-rtos（本仓库 src/rtos）平级独立 git 仓库，
 * 独立 `cargo build --release` 产出 libapp.a，由 CMake 的 RUST_APP_LIB 注入式链接。
 *
 * RTOS 侧只在 app_main 调用一次 rust_app_start()（受 RUST_APP_LIB 宏门控）。
 *
 * 注意：Rust 工程可直接调用 rtos_abi.h 中声明的内核 API（rtos_task_create 等），
 * 因此大多数符号无需在此重复声明；此处仅声明挂载入口与少量供调试的符号。
 */
#ifndef JOC_RUST_APP_H
#define JOC_RUST_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtos.h"

/* Rust 应用层挂载入口：由 RTOS app_main 在控制台循环前调用。
 * ctx 为 app_ctx_t*（RTOS 私有结构）；Rust 侧可直接忽略或按 ABI 使用。 */
void rust_app_start(app_ctx_t *ctx);

/* 调试辅助符号（可选，供控制台/诊断读取，Rust 工程实现） */
uint32_t rust_ticks(void);
void     rust_wait_once(void);

#ifdef __cplusplus
}
#endif

#endif /* JOC_RUST_APP_H */
