//! 手写镜像 rtos_abi_ioctl.h（驱动私有 ioctl 命令常量）。值与 C 侧严格一致。

/* ---- ADC ---- */
pub const ADC_IOCTL_SET_CHANNEL: i32 = 0x01;
pub const ADC_IOCTL_GET_CHANNEL: i32 = 0x02;
pub const ADC_IOCTL_SET_VREF_MV: i32 = 0x03;

/* ---- Stream engine ---- */
pub const STREAM_IOCTL_SET_MODE: i32 = 0xF0;

/* ---- UART ---- */
pub const UART_IOCTL_SET_BAUDRATE: i32 = 0x01;
pub const UART_IOCTL_GET_BAUDRATE: i32 = 0x02;
pub const UART_IOCTL_GET_BRR: i32 = 0x03;
pub const UART_IOCTL_SET_FRAMING: i32 = 0x05;

/* ---- GPIO pin ---- */
pub const GPIO_IOCTL_TOGGLE: i32 = 0x01;

/* ---- PWM ---- */
pub const PWM_IOCTL_SET_DUTY_PERCENT: i32 = 0x20;
pub const PWM_IOCTL_SET_DUTY_TICKS: i32 = 0x21;
pub const PWM_IOCTL_SET_FREQ: i32 = 0x22;
pub const PWM_IOCTL_GET_PERIOD_TICKS: i32 = 0x23;
pub const PWM_IOCTL_GET_DUTY_TICKS: i32 = 0x24;
pub const PWM_IOCTL_ENABLE_CHANNEL: i32 = 0x25;
pub const PWM_IOCTL_DISABLE_CHANNEL: i32 = 0x26;
pub const PWM_IOCTL_GET_BDTR: i32 = 0x27;

/* ---- SPI ---- */
pub const SPI_IOCTL_XFER: i32 = 0x40;
pub const SPI_IOCTL_GET_CR1: i32 = 0x41;

/* ---- I2C ---- */
pub const I2C_IOCTL_MASTER_WRITE: i32 = 0x30;
pub const I2C_IOCTL_MASTER_READ: i32 = 0x31;
pub const I2C_IOCTL_BUS_SCAN: i32 = 0x32;
pub const I2C_IOCTL_SET_SPEED: i32 = 0x33;
pub const I2C_IOCTL_GET_CCR: i32 = 0x37;
pub const I2C_IOCTL_GET_CR2_FREQ: i32 = 0x38;
pub const I2C_IOCTL_GET_CR1: i32 = 0x35;
pub const I2C_IOCTL_GET_BUSY: i32 = 0x36;
pub const I2C_IOCTL_SET_ADDR: i32 = 0x39;

/* ---- Temperature sensor ---- */
pub const TEMP_IOCTL_READ_X10: i32 = 0x01;
pub const TEMP_IOCTL_SET_VREF_MV: i32 = 0x02;
pub const TEMP_IOCTL_GET_CAL1: i32 = 0x03;

/* ---- Timer ---- */
pub const TIMER_IOCTL_GET_OVERFLOWS: i32 = 0x01;
pub const TIMER_IOCTL_GET_COUNTER: i32 = 0x02;
pub const TIMER_IOCTL_SET_REPETITION: i32 = 0x03;
pub const TIMER_IOCTL_ENABLE: i32 = 0x05;  /* start counting + arm update IRQ */
pub const TIMER_IOCTL_DISABLE: i32 = 0x06; /* stop counting + mask update IRQ */

/* ---- EXTI ---- */
pub const EXTI_IOCTL_GET_COUNT: i32 = 0x30;

/* ---- USB CDC ---- */
pub const USB_IOCTL_GET_GINTSTS: i32 = 0xD0;
pub const USB_IOCTL_GET_GCCFG: i32 = 0xD1;
pub const USB_IOCTL_GET_DSTS: i32 = 0xD2;
pub const USB_IOCTL_GET_ADDRESS: i32 = 0xD3;
pub const USB_IOCTL_CONNECTED: i32 = 0xD4;
pub const USB_IOCTL_SET_LINE_CODING: i32 = 0xD5;
pub const USB_IOCTL_GET_LINE_CODING: i32 = 0xD6;
pub const USB_IOCTL_RUN_CTRL_SELFTEST: i32 = 0xD7;
pub const USB_IOCTL_DBG_DUMP: i32 = 0xD8;
pub const USB_IOCTL_DBG_SET: i32 = 0xD9;
pub const USB_IOCTL_SET_DAD_TEST: i32 = 0xDA;
pub const USB_IOCTL_TX_FREE: i32 = 0xDB;
pub const USB_IOCTL_TX_PUMP: i32 = 0xDC;

/* ---- Clock ---- */
pub const CLK_IOCTL_GET_SYSCLK_HZ: i32 = 0x01;
