@echo off
REM ========================================
REM  ESP32-S3 CSI人体检测 - 构建烧录脚本
REM  用法: build.bat tx flash COM5
REM        build.bat rx monitor COM6
REM ========================================

set IDF_PATH=D:\esp32\v5.5.1\esp-idf
call D:\esp32\v5.5.1\esp-idf\export.bat

if "%1"=="" goto :help
if "%1"=="help" goto :help

set PROJECT=%1
set ACTION=%2
set PORT=%3

cd /d D:\esp32c5\see\%PROJECT%

if "%ACTION%"=="build" (
    echo === 构建 %PROJECT% ===
    idf.py set-target esp32s3
    idf.py build
    goto :done
)

if "%ACTION%"=="flash" (
    if "%PORT%"=="" (
        echo 错误: 请指定COM端口，例如: build.bat %PROJECT% flash COM5
        goto :done
    )
    echo === 烧录 %PROJECT% 到 %PORT% ===
    idf.py -p %PORT% flash
    goto :done
)

if "%ACTION%"=="monitor" (
    if "%PORT%"=="" (
        echo 错误: 请指定COM端口，例如: build.bat %PROJECT% monitor COM6
        goto :done
    )
    echo === 烧录并监控 %PROJECT% @ %PORT% ===
    idf.py -p %PORT% flash monitor
    goto :done
)

if "%ACTION%"=="clean" (
    echo === 清理 %PROJECT% ===
    idf.py fullclean
    goto :done
)

echo 未知操作: %ACTION%
goto :help

:help
echo.
echo ESP32-S3 CSI人体检测 构建脚本
echo.
echo 用法: build.bat [tx^|rx] [build^|flash^|monitor^|clean] [COM端口]
echo.
echo 示例:
echo   build.bat tx build              - 构建TX固件
echo   build.bat tx flash COM5         - 烧录TX到COM5
echo   build.bat rx build              - 构建RX固件
echo   build.bat rx flash COM6         - 烧录RX到COM6
echo   build.bat rx monitor COM6       - 烧录RX并打开串口监视器
echo   build.bat rx clean              - 清理RX构建产物
echo.
echo 注意: 请从ESP-IDF 5.5 PowerShell或本脚本运行
echo.

:done
