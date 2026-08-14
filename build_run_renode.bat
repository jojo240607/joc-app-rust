@echo off
rem ============================================================
rem  一键：编译 + Renode 运行 jOS RTOS + Rust App + 自动验证
rem  等价于: python build_run_renode.py %*
rem  常用: build_run_renode.bat           全量运行
rem        build_run_renode.bat --quick  快速验证
rem ============================================================
cd /d %~dp0
python build_run_renode.py %*
pause
