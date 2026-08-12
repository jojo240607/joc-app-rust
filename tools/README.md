# 联调工具 (tools/)

飞控 `joc-app-rust` 的 PC 侧无头联调脚本，配合 USB CDC（ST VCP，VID_0483/PID_5740）
与地面站做双向 MAVLink v2 联调。全部复用 `mavlink.py` 的 CRC(0x8408 反射) 与帧解析，
与板端 `flyctrl/core/src/comm/mavlink.rs` 严格对齐。

## 运行前
- 依赖：`pip install pyserial`
- 板子已上电 + USB 已枚举（**不要挂 OpenOCD/gdb**，halt 会让 CDC 端口掉线）
- 端口缺省自动探测 ST VCP；也可 `--port COM12` / `/dev/ttyACM0` 显式指定

## 工具清单
| 脚本 | 用途 |
|---|---|
| `mavlink.py` | 共享库：CRC-16/ARC(0x8408) + 组帧 `frame()` + 滑动扫描 `scan_frames()` + 端口探测 `find_cdc()`。勿直接跑（自带自检）。 |
| `verify_downlink.py` | **下行严格校验**：抓 COM 原始流，只接受 CRC 通过的完整帧，统计帧数/各 msgid/seq 连续度（重复=字节污染，跳变=丢帧）。`python tools/verify_downlink.py [PORT] [SECS]` |
| `verify_uplink.py` | **上行联调**：`--test` 发 PARAM_REQUEST_LIST+COMMAND_LONG 看应答；默认 `--arm` 反复 ARM/DISARM 并通过心跳 base_mode(0x81) 证明上行全通。`python tools/verify_uplink.py [PORT] [--test\|--arm]` |
| `dump_usb.py` | 原始字节诊断：打印前若干 0xFD 帧的 hex + 解析，辅助查字节污染 / CRC 表错误 / 上行 ACK 是否到达。`python tools/dump_usb.py [PORT] [SECS] [MAXFRAMES]` |
| `usb_telemetry.py` | 实时遥测接收 + 可选 `--csv` 导出 HEARTBEAT/LOCAL_POSITION_NED/SYS_STATUS。`python tools/usb_telemetry.py [--port P] [--csv f.csv] [--secs N]` |
| `capture_log.py` | UART 控制台捕获（COM8，CH340）：启动 OpenOCD → open 串口 → 1s 后 reset run → 抓启动/App 日志。用于 boot 日志 / 卡死排查。`python tools/capture_log.py [PORT] [SECS]` |

## 关键约定（与固件一致，错则 CRC 全失败）
- 反射 CRC-16/ARC 多项式 `0x8408`，初值 `0xFFFF`。
- CRC 计算范围：从 `byte[1]`(len) 起到 payload 末尾，再追加 `CRC_EXTRA[msgid]`。
- `CRC_EXTRA`：HEARTBEAT=50, SYS_STATUS=124, PARAM_REQUEST_LIST=159, PARAM_VALUE=220,
  PARAM_SET=168, ATTITUDE=39, LOCAL_POSITION_NED=185, COMMAND_LONG=152, COMMAND_ACK=143。
- 下行 telemetry 持续写会占满 USB TX ring，上行 ACK 非阻塞 write 可能被丢帧——
  验证上行时优先用 `verify_uplink.py --arm`（看心跳 base_mode 变化），比收显式 ACK 更稳。
