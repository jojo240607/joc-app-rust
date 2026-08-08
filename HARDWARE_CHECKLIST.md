# 硬件联调 Checklist — joc-app-rust 飞控 (STM32F407 Discovery)

> 目标平台：STM32F407 Discovery + joc-base (C RTOS device 体系) + joc-app-rust (Rust 应用层)
> 用途：静态代码适配已闭环，本表仅覆盖**运行时 / 硬件联调**待验证项。
> 关联提交：joc-base `a7a518e`、joc-app-rust `ebf40e1`

---

## 0. 烧录与启动自检
- [ ] OpenOCD 烧录两仓库固件（joc-base 主体 + joc-app-rust `app.bin`/`app.elf`）。
- [ ] 上电后 uart0（调试 UART，CH340 COM8）看到 RTOS 启动横幅。
- [ ] 敲 `PING` → 收到 `PONG`。
- [ ] 敲 `RTOSALL`（或控制台对应的全量自测命令）→ 所有子项 PASS（内核/IPC/FPU/MPU/BH…）。
- [ ] 敲 `SELF-TEST` / `BIST` → 25+ 子项全绿（含 USB 控制自测、各类驱动 BIST）。

## 1. 遥测链路 uart3 / USART6 / PC6–PC7
- [ ] uart3 初始化成功（已解除与 PWM PC6 的复用冲突：pwm1 已挪到 `TIM1_CH2_PA9`）。
- [ ] 地面站 / 串口助手在 USART6 上收到心跳 / 遥测帧。
- [ ] 波特率与协议（MAVLink / 自定义）两端一致。

## 2. RC 接收 — SBUS on uart2 / USART3 / PD9
- [ ] ⚠ **必须外加 SBUS 反相器**：STM32F4 USART 无硬件 RXINV/TXINV 位，`uart_hal_set_inverted` 已是 no-op，反相只能靠外部电路。
- [ ] 100k / 8E2 / 偶校验参数生效。
- [ ] 收到 0x0F 帧头 + 25 字节完整帧，16 通道解析正确。
- [ ] 偶发丢帧时可把 uart 引擎从 POLL 切到 IRQ（`uart_ioctl SET_MODE IRQ`）。
- [ ] 解锁 / 失控保护（F/S 通道丢失）逻辑触发正常。

## 3. GPS — uart1 / USART2 / PA2–PA3
- [ ] 波特自适应探测（9600 → 38400 → 57600）成功并锁定。
- [ ] 解析出 GGA 语句，校验和通过。
- [ ] 定位数据（经纬度 / 卫星数）合理更新。

## 4. I2C0 / I2C1 — PB6 / PB7 传感器
- [ ] ⚠ **必须外加上拉电阻 4.7k**：Discovery 板载未接 I2C 上拉，否则总线拉不高、读写失败。
- [ ] MPU6050 (0x68) 加速度 / 陀螺仪数据正常、零偏合理。
- [ ] BMP280 (0x76) 气压 / 温度读取正常，海拔高度随时间变化合理。
- [ ] QMC5883L (0x0D) 磁力计读取正常。
- [ ] 任一传感器缺失时驱动正确降级、不卡死启动（看门狗 / 超时保护）。

## 5. 电机 PWM — pwm0..3
- [ ] 引脚确认：`pwm0=TIM3_CH1_PA6`、`pwm1=TIM1_CH2_PA9`、`pwm2=TIM1_CH1_PA8`、`pwm3=TIM4_CH1_PD12`。
- [ ] ⚠ pwm1/pwm2 同属 TIM1，确认占空比写入相互独立、无相位串扰。
- [ ] 示波器量脉宽：解锁脉宽映射为 1000–2000 us（1–2ms），频率 400Hz。
- [ ] 控制律输出限制在 40%–80% 安全区间（1000–2000us 对应范围）。
- [ ] 上电默认输出安全值（不上电即转）。

## 6. 控制闭环飞行前确认
- [ ] FC 主环 `FC_LOOP_MS=4`（250Hz）稳定运行、无周期抖动。
- [ ] EKF → PID → PWM 数据通路闭环（IMU/Baro/Mag 融合 → 姿态 → 电机）。
- [ ] FDIR（故障检测与重构）/ 失控保护（RC 丢失 / 低电压）逻辑验证。
- [ ] 高度保持 / 定点模式在地面系绳或测试架上先行验证，再实飞。

---

## 已知硬件坑（务必先处理）
1. **SBUS 反相器**：软件无法反相，PD9 前端必须加反相电路（如 NPN + 上拉 / 专用电平反相 IC）。
2. **I2C 上拉电阻**：PB6/PB7 必须外接 4.7k 上拉到 3.3V，Discovery 板载未提供。
3. **PWM 引脚复用**：pwm1 已从 PC6 挪到 PA9，避免与 uart3(TX) 冲突；TIM1 双通道共享，注意写入隔离。

## 验证完成标记
- [ ] 全部 0–6 项通过 → 静态 + 运行时适配闭环，可进入系绳 / 测试架试飞阶段。
- [ ] 发现新问题 → 记录现象、寄存器 / 引脚证据，回填两仓库并追加提交。
