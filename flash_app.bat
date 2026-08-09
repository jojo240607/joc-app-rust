@echo off
REM Flash APP partition only (stage-2 standalone app image -> APP_FLASH block, sector 7+).
REM Prereq: build app in joc-app-rust (python build_app.py -> app.bin), copy app.bin here.
REM Does NOT touch system area (sector 0~6). App image must be < 384K.
if not exist app.bin (
  echo [ERR] app.bin missing. Build it in joc-app-rust first: python build_app.py
  exit /b 1
)
"D:\soft\openocd\openocd-4e78563-i686-w64-mingw32\bin\openocd.exe" -s "D:\soft\openocd\openocd-4e78563-i686-w64-mingw32\share\openocd\scripts" -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program app.bin 0x08060000 verify reset exit"
