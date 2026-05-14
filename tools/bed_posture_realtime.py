#!/usr/bin/env python3
"""
CSI 体位变化实时检测（per-subcarrier STD 方法）

从 ESP32 RX 串口实时读取原始 I/Q 数据，计算 per-subcarrier STD，
与空床基线比较，实时输出躺下/坐起事件。

算法同 bed_posture_detect.py，但改为在线版本。

用法:
  python tools/bed_posture_realtime.py --port COM28

流程:
  1. 校准阶段: 采集 N 秒空床数据 → 建立 baseline_sub_std
  2. 监测阶段: 每 T 秒对最近 W 秒做 STD 检测 → 输出事件
"""

import argparse
import sys
import time
import csv
import os
import numpy as np
from collections import deque
from datetime import datetime

try:
    import serial
except ImportError:
    print("错误: pip install pyserial")
    sys.exit(1)


# ============================================================
# 默认参数（与 bed_posture_detect.py 一致）
# ============================================================

CALIB_DURATION = 10       # 校准阶段时长（秒）
WINDOW_SEC = 5            # 滑动窗口（秒）
STEP_SEC = 1              # 检测步长（秒）
RATIO_LYING = 1.5         # lying 触发比值
SUB_FRAC = 0.4            # lying 所需高比值子载波比例
RATIO_SITTING = 1.1       # sitting 触发下限

# 使用 --scan 分析出的参数（可通过命令行覆盖）
# python tools/bed_posture_detect.py --scan 输出的最优参数


# ============================================================
# 解析 CSI 行
# ============================================================

def parse_csi_line(line):
    """CSI,<ts_us>,<rssi>,<num_sub>,<I0>,<Q0>,..."""
    if not line.startswith("CSI,"):
        return None
    parts = line.strip().split(",")
    if len(parts) < 4:
        return None
    try:
        ts_us = int(parts[1])
        num_sub = int(parts[3])
        expected = 4 + num_sub * 2
        if len(parts) < expected:
            return None
        iq = np.array([int(x) for x in parts[4:expected]], dtype=np.int8)
        iq_pairs = iq.reshape(-1, 2)
        amp = np.sqrt(iq_pairs[:, 0].astype(float)**2
                      + iq_pairs[:, 1].astype(float)**2)
        return ts_us, num_sub, amp
    except (ValueError, IndexError):
        return None


# ============================================================
# 核心：事件检测（与 bed_posture_detect.py 相同逻辑）
# ============================================================

def detect_from_window(amps, baseline_sub_std,
                       ratio_th, sub_frac, sit_th):
    """
    对一段窗口内的子载波振幅做检测。
    amps: (n_packets, n_sub)
    返回: 'lying', 'sitting', 或 'quiet'
    """
    n_sub = amps.shape[1]
    if len(baseline_sub_std) != n_sub:
        return 'quiet'

    seg_std = np.std(amps, axis=0)
    ratio = seg_std / (baseline_sub_std + 0.01)
    high_count = int(np.sum(ratio > ratio_th))

    if high_count > n_sub * sub_frac:
        return 'lying'
    elif np.mean(ratio) > sit_th:
        return 'sitting'
    else:
        return 'quiet'


# ============================================================
# 校准阶段
# ============================================================

def calibrate(ser, duration, verbose=True):
    """
    采集空床数据，建立 per-subcarrier STD 基线。
    返回: baseline_sub_std (n_sub,), n_sub
    """
    if verbose:
        print(f"\n{'='*60}")
        print(f"校准阶段: 采集 {duration} 秒空床数据")
        print(f"请确保床上无人!")
        print(f"{'='*60}")

        for i in range(5, 0, -1):
            sys.stdout.write(f"\r  倒计时: {i} 秒 ")
            sys.stdout.flush()
            time.sleep(1)
        print("\r  开始校准!            ")

    # 清空串口缓冲
    ser.reset_input_buffer()

    all_amps = []
    start = time.time()
    count = 0

    while time.time() - start < duration:
        raw = ser.readline()
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="replace").strip()
        except Exception:
            continue

        parsed = parse_csi_line(line)
        if parsed is None:
            if line and not line.startswith("CSI,"):
                if verbose:
                    print(f"  [LOG] {line}")
            continue

        _, n_sub, amp = parsed
        all_amps.append(amp)
        count += 1

        if verbose and count % 120 == 0:
            elapsed = time.time() - start
            sys.stdout.write(f"\r  已采集 {count} 包 ({elapsed:.0f}s)")
            sys.stdout.flush()

    if count == 0:
        print("\n错误: 未收到任何 CSI 数据")
        sys.exit(1)

    amps_array = np.array(all_amps)
    # 用全部校准数据的 per-subcarrier STD 作为基线
    baseline_sub_std = np.std(amps_array, axis=0)

    if verbose:
        elapsed = time.time() - start
        print(f"\r  校准完成: {count} 包, {elapsed:.0f}s, "
              f"速率 {count/elapsed:.0f} pkt/s")
        print(f"  baseline_sub_std: mean={np.mean(baseline_sub_std):.3f}, "
              f"range=[{np.min(baseline_sub_std):.3f}, {np.max(baseline_sub_std):.3f}]")

    return baseline_sub_std, n_sub


# ============================================================
# 实时监测
# ============================================================

def monitor(ser, baseline_sub_std, n_sub,
            window_sec, step_sec, ratio_th, sub_frac, sit_th,
            output_csv=None):
    """
    实时监测体位变化。

    维持一个时间窗口缓冲区，每 step_sec 秒做一次检测。
    """
    print(f"\n{'='*60}")
    print("监测阶段: 实时体位检测")
    print(f"  窗口: {window_sec}s, 步长: {step_sec}s")
    print(f"  lying阈值: >{sub_frac*100:.0f}%子载波 >{ratio_th}x")
    print(f"  sitting阈值: ratio_mean >{sit_th}")
    print(f"{'='*60}\n")

    # CSV
    csv_file = None
    writer = None
    if output_csv:
        csv_file = open(output_csv, "w", newline="", encoding="utf-8-sig")
        writer = csv.writer(csv_file)
        writer.writerow(["timestamp", "direction", "ratio_mean", "high_sub_count"])
        csv_file.flush()

    # 缓冲区: 存储 (timestamp, amp_vector)
    buffer = deque()

    last_event = None
    last_event_time = 0
    cooldown_sec = 5

    last_check = time.time()
    running = True
    count = 0
    last_status = time.time()

    try:
        while running:
            raw = ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue

            parsed = parse_csi_line(line)
            if parsed is None:
                if line and not line.startswith("CSI,"):
                    print(f"  [LOG] {line}")
                continue

            ts_us, _, amp = parsed
            ts_sec = ts_us / 1e6
            buffer.append((ts_sec, amp))
            count += 1

            # 移除过期数据
            cutoff = ts_sec - window_sec
            while buffer and buffer[0][0] < cutoff:
                buffer.popleft()

            # 按 step_sec 间隔做检测
            now = time.time()
            if now - last_check >= step_sec and len(buffer) > 10:
                last_check = now

                # 取窗口内振幅
                window_amps = np.array([a for _, a in buffer])
                direction = detect_from_window(
                    window_amps, baseline_sub_std,
                    ratio_th, sub_frac, sit_th)

                ratio = np.mean(np.std(window_amps, axis=0)
                                / (baseline_sub_std + 0.01)) if len(window_amps) > 0 else 0

                # 每 3 秒打印状态（便于调试）
                if now - last_status >= 3:
                    last_status = now
                    sys.stdout.write(f"\r  [{datetime.now().strftime('%H:%M:%S')}] "
                                     f"包数={count}, 窗口={len(buffer)}, "
                                     f"方向={direction}, ratio={ratio:.2f}    ")
                    sys.stdout.flush()

                if direction != 'quiet':
                    now_ts = datetime.now().strftime("%H:%M:%S")

                    # 冷却检查
                    if direction != last_event or now - last_event_time > cooldown_sec:
                        print(f"\n  [{now_ts}] *** {direction.upper()} *** "
                              f"(ratio={ratio:.2f}, 窗口={len(buffer)}包)")
                        if writer:
                            writer.writerow([now_ts, direction,
                                             f"{ratio:.2f}",
                                             int(np.sum(np.std(window_amps, axis=0)
                                                        / (baseline_sub_std + 0.01)
                                                        > ratio_th))])
                            csv_file.flush()
                        last_event = direction
                        last_event_time = now

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        if csv_file:
            csv_file.close()
        print(f"共处理 {count} 包")
        print(f"串口已关闭")


# ============================================================
# 主函数
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="CSI 体位实时检测（per-subcarrier STD 方法）")
    parser.add_argument("--port", required=True, help="串口号 (如 COM28)")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--calib", type=int, default=CALIB_DURATION,
                        help="校准阶段时长(秒) (默认 10)")
    parser.add_argument("--window", type=float, default=WINDOW_SEC,
                        help="滑动窗口(秒) (默认 5)")
    parser.add_argument("--step", type=float, default=STEP_SEC,
                        help="检测步长(秒) (默认 1)")
    parser.add_argument("--ratio-lying", type=float, default=RATIO_LYING,
                        help="lying 触发比值 (默认 1.5)")
    parser.add_argument("--sub-frac", type=float, default=SUB_FRAC,
                        help="lying 子载波比例 (默认 0.4)")
    parser.add_argument("--ratio-sitting", type=float, default=RATIO_SITTING,
                        help="sitting 触发下限 (默认 1.1)")
    parser.add_argument("--output", default=None, help="输出 CSV 文件")
    args = parser.parse_args()

    print("=" * 60)
    print("CSI 体位实时检测 (per-subcarrier STD)")
    print(f"串口: {args.port} @ {args.baud}")
    print("=" * 60)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"错误: {e}")
        sys.exit(1)

    # 1. 校准
    baseline_sub_std, n_sub = calibrate(ser, args.calib)

    # 2. 监测
    try:
        monitor(ser, baseline_sub_std, n_sub,
                args.window, args.step,
                args.ratio_lying, args.sub_frac, args.ratio_sitting,
                args.output)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
