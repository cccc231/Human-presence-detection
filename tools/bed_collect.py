#!/usr/bin/env python3
"""
CSI 数据采集脚本（床边检测用）
从 ESP32 RX 串口读取原始 I/Q 数据，计算振幅并保存到 CSV。
用于采集空床、躺下、起床等场景的数据样本。

采集时长包含：动作前静止 → 动作过程 → 动作后静止，共约 15-20 秒。

用法:
  python tools/bed_collect.py --port COM28 --label empty --duration 20
  python tools/bed_collect.py --port COM28 --label lying --duration 20
  python tools/bed_collect.py --port COM28 --label getup --duration 20
"""

import argparse
import csv
import sys
import time
import os
import numpy as np
from datetime import datetime

try:
    import serial
except ImportError:
    print("错误: pip install pyserial")
    sys.exit(1)


def parse_csi_line(line):
    """解析: CSI,<timestamp_us>,<rssi>,<num_sub>,<I0>,<Q0>,..."""
    if not line.startswith("CSI,"):
        return None
    parts = line.strip().split(",")
    if len(parts) < 4:
        return None
    try:
        ts_us = int(parts[1])
        rssi = int(parts[2])
        num_sub = int(parts[3])
        expected = 4 + num_sub * 2
        if len(parts) < expected:
            return None
        iq = np.array([int(x) for x in parts[4:expected]], dtype=np.int8)
        iq_pairs = iq.reshape(-1, 2)
        amp = np.sqrt(iq_pairs[:, 0].astype(float)**2 + iq_pairs[:, 1].astype(float)**2)
        return ts_us, rssi, num_sub, amp
    except (ValueError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description="CSI 数据采集（床边检测）")
    parser.add_argument("--port", required=True, help="串口号 (如 COM28)")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--label", required=True, choices=["empty", "lying", "getup"],
                        help="场景标签: empty=空床, lying=躺下过程, getup=起床过程")
    parser.add_argument("--duration", type=int, default=20, help="采集时长(秒), 默认20")
    parser.add_argument("--delay", type=int, default=5, help="开始前等待(秒)")
    parser.add_argument("--output", default=None, help="输出文件名 (默认: data/bed_<label>.csv)")
    args = parser.parse_args()

    # 输出目录
    data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
    os.makedirs(data_dir, exist_ok=True)

    if args.output is None:
        args.output = os.path.join(data_dir, f"bed_{args.label}.csv")

    label_desc = {"empty": "空床", "lying": "躺下过程", "getup": "起床过程"}
    print("=" * 60)
    print(f"CSI 数据采集 - 床边检测")
    print(f"场景: {label_desc[args.label]}")
    print(f"串口: {args.port} @ {args.baud}")
    print(f"采集时长: {args.duration}秒")
    print(f"输出: {args.output}")
    print("=" * 60)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {args.port}: {e}")
        sys.exit(1)

    if args.delay > 0:
        hint = {
            "empty": "请确保床上无人，保持空床",
            "lying": "先保持站立/坐着，倒计时结束后再躺下",
            "getup": "先躺在床上，倒计时结束后再起来",
        }
        print(f"\n等待 {args.delay} 秒 ({hint[args.label]})...")
        for i in range(args.delay, 0, -1):
            sys.stdout.write(f"\r  倒计时: {i} 秒 ")
            sys.stdout.flush()
            time.sleep(1)
        print("\r  开始采集!            ")

    # CSV 表头：timestamp_us, rssi, num_sub, mean_amp, amp_0, amp_1, ...
    # 先采集第一包确定子载波数
    print("\n等待第一包数据...")
    n_sub = None
    while n_sub is None:
        raw = ser.readline()
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="replace").strip()
        except Exception:
            continue
        parsed = parse_csi_line(line)
        if parsed:
            _, _, n_sub, _ = parsed

    print(f"子载波数: {n_sub}")
    print(f"采集中...\n")

    csv_file = open(args.output, "w", newline="", encoding="utf-8-sig")
    writer = csv.writer(csv_file)
    header = ["timestamp_us", "rssi", "num_sub", "mean_amp"] + [f"amp_{i}" for i in range(n_sub)]
    writer.writerow(header)
    csv_file.flush()

    start_time = time.time()
    count = 0
    ema_mean = None
    ema_alpha = 0.02  # 用于实时显示

    try:
        while time.time() - start_time < args.duration:
            raw = ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue

            # 打印非CSI日志
            if line and not line.startswith("CSI,"):
                print(f"  [LOG] {line}")
                continue

            parsed = parse_csi_line(line)
            if parsed is None:
                continue

            ts_us, rssi, num_sub, amp = parsed
            mean_amp = np.mean(amp)

            # EMA 用于实时显示
            if ema_mean is None:
                ema_mean = mean_amp
            else:
                ema_mean = ema_alpha * mean_amp + (1 - ema_alpha) * ema_mean

            row = [ts_us, rssi, num_sub, f"{mean_amp:.4f}"] + [f"{a:.4f}" for a in amp]
            writer.writerow(row)
            count += 1

            if count % 120 == 0:
                elapsed = time.time() - start_time
                rate = count / elapsed
                print(f"  {count} 包 ({elapsed:.0f}s), {rate:.0f} pkt/s, "
                      f"mean_amp={mean_amp:.2f}, EMA={ema_mean:.2f}")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        elapsed = time.time() - start_time
        csv_file.close()
        ser.close()

        print(f"\n{'='*60}")
        print(f"采集完成: {count} 包, {elapsed:.1f}秒, {count/max(elapsed,1):.0f} pkt/s")
        print(f"数据保存到: {args.output}")

        if count > 0:
            print(f"mean_amp 范围: 实时值最后={mean_amp:.2f}, EMA={ema_mean:.2f}")
        print("=" * 60)


if __name__ == "__main__":
    main()
