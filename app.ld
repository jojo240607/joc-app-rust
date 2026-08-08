/* joc-app-rust 独立链接脚本（阶段 2 应用分区）。
 * 把 cargo 产出的 libapp.a 链接成可直接烧录到 APP_FLASH 块的 app.bin。
 * 契约（须与 joc-base 系统侧严格一致）：
 *   - APP_FLASH ORIGIN = 0x08060000（sector 7），header 必须在此起点。
 *   - 头部 app_header_t 由本脚本用 LONG() 直接生成：
 *       magic=0x41504800("APH\0") / abi_version=RTOS_ABI_VERSION /
 *       entry=ABSOLUTE(rust_app_start) / app_size=0(占位) / reserved=0
 *   - App 代码 XIP 运行（.app_code 在 APP_FLASH，绝对地址），无需重定位。
 *   - App 运行期 .bss 在 APP_RAM（0x2001E000，8K），由系统加载器清零。
 *   - App 不使用 .data（无需要 LMA 拷贝的初始化数据），全部 .bss/.rodata。 */

MEMORY
{
  APP_FLASH (rx) : ORIGIN = 0x08060000, LENGTH = 384K
  APP_RAM   (xrw): ORIGIN = 0x2001E000, LENGTH = 8K
}

/* 入口固定为 Rust 挂载点；header.entry 由链接器填其绝对地址 */
ENTRY(rust_app_start)

SECTIONS
{
  /* g_app_slot 是系统侧固定地址(0x2001DC00)的服务表，App 只引用不定义。
   * App 独立链接时由 PROVIDE 提供该符号地址，运行时系统已在那里填好函数指针。
   * 该地址必须 == joc-base 链接脚本 APP_SLOT_RAM ORIGIN。 */
  PROVIDE(g_app_slot = 0x2001DC00);

  /* ---- 应用头部（固定 32 字节，App 镜像起点）---- */
  .app_header :
  {
    LONG(0x41504800)                    /* magic: "APH\0" */
    LONG(1)                             /* abi_version: MUST == RTOS_ABI_VERSION (joc-base) */
    LONG(ABSOLUTE(rust_app_start))      /* entry: App 入口绝对地址 */
    LONG(0)                             /* app_size: 占位（系统侧仅作日志，不强制校验） */
    LONG(0)                             /* reserved */
    LONG(0)                             /* reserved */
    LONG(0)                             /* reserved */
    LONG(0)                             /* reserved */
    . = ALIGN(16);
  } >APP_FLASH

  /* ---- App 代码 + 只读数据（XIP）---- */
  .app_code :
  {
    . = ALIGN(4);
    *(.text)
    *(.text.*)
    *(.rodata)
    *(.rodata.*)
    *(.glue_7)
    *(.glue_7t)
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >APP_FLASH

  /* ---- App 运行期 .bss（APP_RAM，系统加载器清零）---- */
  .app_bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sappbss = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    _eappbss = .;
  } >APP_RAM

  /DISCARD/ :
  {
    *(.note .note.*)
    *(.eh_frame)
  }
}
