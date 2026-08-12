@echo off
start "ocd" openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "gdb_port 3334" -c "telnet_port 4444"
