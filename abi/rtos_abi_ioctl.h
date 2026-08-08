#ifndef RTOS_ABI_IOCTL_H
#define RTOS_ABI_IOCTL_H

/* ===========================================================================
 * 驱动私有 ioctl 命令常量（集中抽取，供 Rust 侧镜像）
 * 命令值与原驱动头严格一致；新增/修改驱动 ioctl 必须同步更新本文件 + RTOS_ABI_VERSION。
 * 用法：Rust 侧 device.ioctl(dev, ADC_IOCTL_SET_CHANNEL, &ch) 等。
 * ========================================================================= */

/* ---- ADC (drv/adc.h) ---- */
#define ADC_IOCTL_SET_CHANNEL    0x01   /* arg: const uint32_t* channel */
#define ADC_IOCTL_GET_CHANNEL    0x02   /* arg: uint32_t* channel */
#define ADC_IOCTL_SET_VREF_MV    0x03   /* arg: const uint32_t* vdda_mv */

/* ---- Stream engine (iface/stream_device.h) ---- */
#define STREAM_IOCTL_SET_MODE    0xF0   /* arg: const stream_xfer_mode_t* (POLL/IRQ/DMA) */

/* ---- UART (drv/uart.h) ---- */
#define UART_IOCTL_SET_BAUDRATE  0x01   /* arg: const uint32_t* baud */
#define UART_IOCTL_GET_BAUDRATE  0x02   /* arg: uint32_t* baud */
#define UART_IOCTL_GET_BRR       0x03   /* arg: uint32_t* BRR */
#define UART_IOCTL_SET_FRAMING   0x05   /* arg: const uart_frame_t* (NONE/IDLE) */

/* ---- GPIO pin (drv/gpio_pin.h) ---- */
#define GPIO_IOCTL_TOGGLE        0x01   /* arg: NULL */

/* ---- PWM (drv/pwm.h) ---- */
#define PWM_IOCTL_SET_DUTY_PERCENT  0x20   /* arg = int* (0..100) */
#define PWM_IOCTL_SET_DUTY_TICKS    0x21   /* arg = uint32_t* (0..period_ticks) */
#define PWM_IOCTL_SET_FREQ          0x22   /* arg = uint32_t* (Hz; independent mode only) */
#define PWM_IOCTL_GET_PERIOD_TICKS  0x23   /* arg = uint32_t* (ARR+1) */
#define PWM_IOCTL_GET_DUTY_TICKS    0x24   /* arg = uint32_t* (current CCR) */
#define PWM_IOCTL_ENABLE_CHANNEL    0x25   /* arg = NULL (CCER.CCxE = 1) */
#define PWM_IOCTL_DISABLE_CHANNEL   0x26   /* arg = NULL (CCER.CCxE = 0) */
#define PWM_IOCTL_GET_BDTR          0x27   /* arg = uint32_t* (raw BDTR; adv TIM) */

/* ---- SPI (drv/spi.h) ---- */
#define SPI_IOCTL_XFER        0x40   /* arg = spi_xfer_t* (full-duplex) */
#define SPI_IOCTL_GET_CR1     0x41   /* arg = uint32_t* (raw CR1) */

/* ---- I2C (drv/i2c.h) ---- */
#define I2C_IOCTL_MASTER_WRITE  0x30   /* arg = i2c_xfer_t* */
#define I2C_IOCTL_MASTER_READ   0x31   /* arg = i2c_xfer_t* */
#define I2C_IOCTL_BUS_SCAN      0x32   /* arg = i2c_scan_t* */
#define I2C_IOCTL_SET_SPEED     0x33   /* arg = uint32_t* */
#define I2C_IOCTL_GET_CCR       0x37   /* arg = uint32_t* */
#define I2C_IOCTL_GET_CR2_FREQ  0x38   /* arg = uint32_t* */
#define I2C_IOCTL_GET_CR1       0x35   /* arg = uint32_t* */
#define I2C_IOCTL_GET_BUSY      0x36   /* arg = uint32_t* */
#define I2C_IOCTL_SET_ADDR      0x39   /* arg = uint16_t* (current slave addr) */

/* ---- Temperature sensor (drv/temp_sensor.h) ---- */
#define TEMP_IOCTL_READ_X10      0x01   /* arg: int32_t* t_x10 (1 decimal, *10) */
#define TEMP_IOCTL_SET_VREF_MV   0x02   /* arg: const uint32_t* vdda_mv */
#define TEMP_IOCTL_GET_CAL1      0x03   /* arg: uint16_t* factory calib @30C */

/* ---- Timer (drv/timer.h) ---- */
#define TIMER_IOCTL_GET_OVERFLOWS  0x01   /* arg = uint32_t* : overflow count */
#define TIMER_IOCTL_GET_COUNTER    0x02   /* arg = uint32_t* : current CNT */
#define TIMER_IOCTL_SET_REPETITION 0x03   /* arg = uint32_t* : RCR 0..255 (adv TIM) */

/* ---- EXTI (drv/exti.h) ---- */
#define EXTI_IOCTL_GET_COUNT   0x30   /* arg = uint32_t* : interrupt count */

/* ---- USB CDC (drv/usb.h) ---- */
#define USB_IOCTL_GET_GINTSTS        0xD0
#define USB_IOCTL_GET_GCCFG          0xD1
#define USB_IOCTL_GET_DSTS           0xD2
#define USB_IOCTL_GET_ADDRESS        0xD3
#define USB_IOCTL_CONNECTED          0xD4
#define USB_IOCTL_SET_LINE_CODING    0xD5   /* arg: uint8_t[7] */
#define USB_IOCTL_GET_LINE_CODING    0xD6   /* arg: uint8_t[7] */
#define USB_IOCTL_RUN_CTRL_SELFTEST  0xD7   /* arg: NULL; ret 0=pass, -1=fail */
#define USB_IOCTL_DBG_DUMP           0xD8   /* arg: NULL; print ISR/enum counters */
#define USB_IOCTL_DBG_SET            0xD9   /* arg: int* (0/1); toggle ISR trace */
#define USB_IOCTL_SET_DAD_TEST       0xDA   /* arg: uint8_t* (addr) */
#define USB_IOCTL_TX_FREE            0xDB   /* arg: size_t*; bytes free in TX ring */
#define USB_IOCTL_TX_PUMP            0xDC   /* arg: NULL; drain TX ring -> arm bulk-IN */

/* ---- Clock (drv/clock.h) ---- */
#define CLK_IOCTL_GET_SYSCLK_HZ  0x01   /* arg: uint32_t* hz */

#endif /* RTOS_ABI_IOCTL_H */
