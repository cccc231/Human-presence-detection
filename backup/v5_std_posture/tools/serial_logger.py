#!/usr/bin/env python3
"""
ESP32-S3 CSI Human Presence Detection - Serial Logger
Reads detection results from RX board via serial and saves to CSV.
"""

import argparse
import csv
import re
import sys
import os
import time
import threading
from datetime import datetime

try:
    import serial
except ImportError:
    print("错误: 请先安装 pyserial: pip install pyserial")
    sys.exit(1)

CSI_LINE_PATTERN = re.compile(
    r"((?:\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})|SYNCING_\d+s),\s*(有人|没人),\s*([\d.]+)(?:,\s*(\d+)/(\d+))?"
)

# ANSI color codes
GREEN = "\033[92m"
RED = "\033[91m"
GRAY = "\033[90m"
RESET = "\033[0m"
BOLD = "\033[1m"


def colorize_status(status: str) -> str:
    if status == "有人":
        return f"{RED}{BOLD}有人{RESET}"
    return f"{GRAY}没人{RESET}"


def main():
    parser = argparse.ArgumentParser(
        description="ESP32-S3 CSI 人体检测串口日志工具"
    )
    parser.add_argument("--port", required=True, help="串口号，如 COM3")
    parser.add_argument("--baud", type=int, default=115200, help="波特率 (默认 115200)")
    parser.add_argument("--output", default="csi_log.csv", help="输出CSV文件名 (默认 csi_log.csv)")
    parser.add_argument("--duration", type=int, default=0, help="采集时长（秒），0表示手动停止 (默认 0)")
    parser.add_argument("--delay", type=int, default=10, help="开始采集前等待秒数 (默认 10)")
    args = parser.parse_args()

    print(f"打开串口 {args.port} @ {args.baud} baud...")
    print(f"输出文件: {args.output}")
    if args.duration > 0:
        print(f"采集时长: {args.duration} 秒 (到时自动停止)")
    else:
        print("采集时长: 手动停止 (Ctrl+C)")
    print("-" * 60)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {args.port}: {e}")
        sys.exit(1)

    if args.delay > 0:
        print(f"等待 {args.delay} 秒后开始采集 (请离开房间)...")
        for i in range(args.delay, 0, -1):
            sys.stdout.write(f"\r  倒计时: {i} 秒 ")
            sys.stdout.flush()
            time.sleep(1)
        print(f"\r  开始采集!            ")
        print("-" * 60)

    file_exists = os.path.exists(args.output)
    csv_file = open(args.output, "a", newline="", encoding="utf-8-sig")
    writer = csv.writer(csv_file)

    if not file_exists:
        writer.writerow(["timestamp", "status", "metric", "presence_count"])
        csv_file.flush()

    stop_event = threading.Event()

    # Timer thread for auto-stop
    if args.duration > 0:
        def timer_stop():
            time.sleep(args.duration)
            print(f"\n{'=' * 60}")
            print(f"采集时间到 ({args.duration}秒)，自动停止...")
            stop_event.set()
        threading.Thread(target=timer_stop, daemon=True).start()

    start_time = time.time()
    line_count = 0
    try:
        while not stop_event.is_set():
            raw_line = ser.readline()
            if not raw_line:
                continue

            try:
                line = raw_line.decode("utf-8", errors="replace").strip()
            except UnicodeDecodeError:
                continue

            if not line:
                continue

            # Print all serial output (including ESP-IDF log messages)
            if not CSI_LINE_PATTERN.match(line):
                print(f"{GRAY}[LOG] {line}{RESET}")
                continue

            match = CSI_LINE_PATTERN.match(line)
            status = match.group(2)
            metric = float(match.group(3))
            present_count = match.group(4)
            total_count = match.group(5)
            timestamp = datetime.now().strftime("%H:%M:%S")

            count_str = f"{present_count}/{total_count}" if present_count else ""
            writer.writerow([timestamp, status, f"{metric:.4f}", count_str])
            csv_file.flush()
            line_count += 1

            colored_status = colorize_status(status)
            ratio_info = f"  [{present_count}/{total_count}]" if present_count else ""
            print(f"  [{timestamp}] {colored_status}  (metric={metric:.4f}){ratio_info}  [#{line_count}]")

    except KeyboardInterrupt:
        print(f"\n{'=' * 60}")
    finally:
        elapsed = time.time() - start_time
        print(f"停止记录。共记录 {line_count} 条检测数据到 {args.output} (耗时 {elapsed:.0f} 秒)")
        csv_file.close()
        ser.close()
        print("串口已关闭。")


if __name__ == "__main__":
    main()
