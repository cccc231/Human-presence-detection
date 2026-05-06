#!/usr/bin/env python3
"""
WiFi CSI Vital Signs Monitor
Based on: "Design and Evaluation of Volunteer User Trials of Unobtrusive
Vital Signs Monitoring for Older People in Care Using Wi-Fi CSI Sensing"
(IEEE JTEHM 2025)

Signal processing pipeline:
  1. CSI I/Q raw data collection from ESP32
  2. Amplitude extraction: amp = sqrt(I^2 + Q^2)
  3. Interpolation & resampling (uniform time spacing)
  4. DWT db4 wavelet filtering (RR: level 4-6, HR: level 2-4)
  5. PCA dimensionality reduction (52 subcarriers -> 10 PCs)
  6. PC-SampEn selection (lowest entropy = most regular)
  7. CWT Morlet wavelet -> average wavelet energy peaks -> RR/HR
"""

import argparse
import sys
import time
import csv
import re
import numpy as np
from datetime import datetime

try:
    import serial
except ImportError:
    print("错误: pip install pyserial")
    sys.exit(1)

try:
    import pywt
except ImportError:
    print("错误: pip install PyWavelets")
    sys.exit(1)

try:
    from scipy import signal as scipy_signal
    from scipy.interpolate import interp1d
except ImportError:
    print("错误: pip install scipy")
    sys.exit(1)

try:
    from sklearn.decomposition import PCA
except ImportError:
    print("错误: pip install scikit-learn")
    sys.exit(1)

try:
    import nolds
except ImportError:
    print("错误: pip install nolds")
    sys.exit(1)

# --- Constants from paper ---
DWT_WAVELET = 'db4'
DWT_LEVEL = 6
PCA_COMPONENTS = 10
SAMPEN_EMBED_DIM = 3
SAMPEN_TOLERANCE_FACTOR = 0.1  # r = 0.1 * std(PC)
CWT_SCALES = np.arange(1, 29)  # 28 scales
MORLET_W = 6  # Morlet wavelet parameter omega0

# Resampling rates (from paper)
FS_RR = 40.0  # Hz, for respiration rate
FS_HR = 60.0  # Hz, for heart rate

# Frequency bands (Hz)
RR_BAND = (0.1, 0.5)   # 6-30 breaths/min
HR_BAND = (0.8, 2.0)   # 48-120 bpm


def parse_csi_line(line):
    """Parse: CSI,<timestamp_us>,<rssi>,<num_sub>,<I0>,<Q0>,..."""
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
        iq_pairs = iq.reshape(-1, 2)  # (num_sub, 2) -> [real, imag]
        return ts_us, rssi, num_sub, iq_pairs
    except (ValueError, IndexError):
        return None


def collect_data(ser, duration_sec):
    """Collect CSI packets from serial port."""
    timestamps_us = []
    amplitudes = []
    print(f"采集数据中 ({duration_sec}秒)...")
    start = time.time()
    count = 0

    while time.time() - start < duration_sec:
        raw = ser.readline()
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="replace").strip()
        except Exception:
            continue

        parsed = parse_csi_line(line)
        if parsed is None:
            # Print non-CSI lines (ESP-IDF log messages)
            if line and not line.startswith("CSI"):
                print(f"  [LOG] {line}")
            continue

        ts_us, rssi, num_sub, iq_pairs = parsed
        amp = np.sqrt(iq_pairs[:, 0].astype(float)**2 + iq_pairs[:, 1].astype(float)**2)
        timestamps_us.append(ts_us)
        amplitudes.append(amp)
        count += 1

        if count % 120 == 0:
            elapsed = time.time() - start
            print(f"  已采集 {count} 包 ({elapsed:.0f}s), 实际速率: {count/elapsed:.0f} pkt/s")

    if count == 0:
        print("错误: 未采集到任何 CSI 数据")
        sys.exit(1)

    timestamps_us = np.array(timestamps_us)
    amplitudes = np.array(amplitudes)  # (N, num_sub)
    actual_rate = count / (time.time() - start)
    print(f"采集完成: {count} 包, 实际速率 {actual_rate:.0f} pkt/s")
    return timestamps_us, amplitudes


def resample_uniform(timestamps_us, signal_2d, target_fs):
    """Resample non-uniform CSI data to uniform time spacing."""
    t_sec = (timestamps_us - timestamps_us[0]) / 1e6
    duration = t_sec[-1]
    n_out = int(duration * target_fs)
    if n_out < 10:
        return None, None
    t_uniform = np.linspace(0, duration, n_out)

    n_sub = signal_2d.shape[1]
    resampled = np.zeros((n_out, n_sub))
    for k in range(n_sub):
        f = interp1d(t_sec, signal_2d[:, k], kind='linear', fill_value='extrapolate')
        resampled[:, k] = f(t_uniform)

    return t_uniform, resampled


def dwt_filter_bank(data_2d, wavelet, level, keep_levels):
    """
    Apply DWT filter bank to each subcarrier.
    keep_levels: list of decomposition levels to keep (1-indexed from finest)
    Returns filtered signal for each subcarrier.
    """
    n_samples, n_sub = data_2d.shape
    filtered = np.zeros_like(data_2d)

    for k in range(n_sub):
        coeffs = pywt.wavedec(data_2d[:, k], wavelet, level=level)
        # coeffs = [cA_level, cD_level, cD_level-1, ..., cD_1]
        # keep_levels are 1-indexed: 1=finest detail, level=coarsest
        # coeffs index: 0=cA, 1=cD_level, 2=cD_level-1, ..., i=cD_level+1-i
        new_coeffs = [np.zeros_like(c) for c in coeffs]
        for lvl in keep_levels:
            idx = level + 1 - lvl  # convert level to coeffs index
            if 0 < idx < len(coeffs):
                new_coeffs[idx] = coeffs[idx]
        filtered[:, k] = pywt.waverec(new_coeffs, wavelet)[:n_samples]

    return filtered


def apply_pca(data_2d, n_components):
    """Apply PCA to reduce subcarrier dimensionality."""
    pca = PCA(n_components=min(n_components, data_2d.shape[1]))
    components = pca.fit_transform(data_2d)
    return components, pca


def compute_sampen(signal_1d, m, r_factor):
    """Compute Sample Entropy using nolds library."""
    if len(signal_1d) < 20:
        return float('inf')
    r = r_factor * np.std(signal_1d)
    if r == 0:
        return float('inf')
    try:
        return nolds.sampen(signal_1d, emb_dim=m, tolerance=r)
    except Exception:
        return float('inf')


def select_best_pc(components):
    """Select PC with lowest Sample Entropy (most regular signal)."""
    n_pc = components.shape[1]
    sampen_values = []
    for i in range(n_pc):
        se = compute_sampen(components[:, i], SAMPEN_EMBED_DIM, SAMPEN_TOLERANCE_FACTOR)
        sampen_values.append(se)
    best_pc = np.argmin(sampen_values)
    return best_pc, sampen_values


def extract_rate_cwt(signal_1d, fs, freq_band):
    """
    Extract rate using CWT with Morlet wavelet.
    Returns rate in breaths/min or bpm.
    """
    # Compute CWT
    widths = CWT_SCALES
    # scipy.signal.cwt with morlet2
    try:
        coefficients = scipy_signal.cwt(signal_1d, scipy_signal.morlet2, widths, w=MORLET_W)
    except Exception:
        return None, None, None

    # Average wavelet energy per scale
    energy = np.mean(np.abs(coefficients)**2, axis=1)

    # Convert scales to frequencies
    # For Morlet: f = (w + sqrt(2 + w^2)) / (4*pi*scale) * fs
    # Simplified: f ≈ central_freq * fs / scale
    central_freq = (MORLET_W + np.sqrt(2 + MORLET_W**2)) / (4 * np.pi)
    frequencies = central_freq * fs / widths

    # Find peaks in frequency band
    mask = (frequencies >= freq_band[0]) & (frequencies <= freq_band[1])
    if not np.any(mask):
        return None, energy, frequencies

    band_energy = energy[mask]
    band_freqs = frequencies[mask]

    # Find the dominant frequency (peak energy)
    peak_idx = np.argmax(band_energy)
    peak_freq = band_freqs[peak_idx]
    rate = peak_freq * 60  # convert Hz to per-minute

    return rate, energy, frequencies


def process_vital_signs(timestamps_us, amplitudes):
    """
    Full signal processing pipeline.
    Returns (rr_bpm, hr_bpm, details_dict)
    """
    n_packets, n_sub = amplitudes.shape
    duration = (timestamps_us[-1] - timestamps_us[0]) / 1e6

    if duration < 10:
        print(f"数据太短 ({duration:.1f}秒), 至少需要10秒")
        return None, None, {}

    print(f"\n{'='*50}")
    print(f"信号处理: {n_packets} 包, {n_sub} 子载波, {duration:.1f}秒")
    print(f"{'='*50}")

    # Step 1: Remove DC offset per subcarrier
    amplitudes_centered = amplitudes - np.mean(amplitudes, axis=0)

    # Step 2: Process for RR
    print("\n[RR] 插值重采样到 40 Hz...")
    t_rr, amp_rr = resample_uniform(timestamps_us, amplitudes_centered, FS_RR)
    if t_rr is None:
        print("[RR] 重采样失败")
        rr_bpm = None
    else:
        print(f"[RR] 重采样后: {len(t_rr)} 样本")

        print("[RR] DWT db4 小波滤波 (level 4-6, 0.1-0.5 Hz)...")
        rr_filtered = dwt_filter_bank(amp_rr, DWT_WAVELET, DWT_LEVEL, keep_levels=[4, 5, 6])

        print("[RR] PCA 降维...")
        rr_pca, rr_pca_model = apply_pca(rr_filtered, PCA_COMPONENTS)

        print("[RR] PC-SampEn 选择最佳主成分...")
        rr_best_pc, rr_sampen = select_best_pc(rr_pca)
        print(f"[RR] 最佳 PC: {rr_best_pc+1}, SampEn: {rr_sampen[rr_best_pc]:.4f}")

        print("[RR] CWT Morlet 小波提取呼吸率...")
        rr_bpm, rr_energy, rr_freqs = extract_rate_cwt(
            rr_pca[:, rr_best_pc], FS_RR, RR_BAND)

        if rr_bpm is not None:
            print(f"[RR] 呼吸率: {rr_bpm:.1f} BrPM")
        else:
            print("[RR] 呼吸率提取失败")

    # Step 3: Process for HR
    print("\n[HR] 插值重采样到 60 Hz...")
    t_hr, amp_hr = resample_uniform(timestamps_us, amplitudes_centered, FS_HR)
    if t_hr is None:
        print("[HR] 重采样失败")
        hr_bpm = None
    else:
        print(f"[HR] 重采样后: {len(t_hr)} 样本")

        print("[HR] DWT db4 小波滤波 (level 2-4, 0.8-2.0 Hz)...")
        hr_filtered = dwt_filter_bank(amp_hr, DWT_WAVELET, DWT_LEVEL, keep_levels=[2, 3, 4])

        print("[HR] PCA 降维...")
        hr_pca, hr_pca_model = apply_pca(hr_filtered, PCA_COMPONENTS)

        print("[HR] PC-SampEn 选择最佳主成分...")
        hr_best_pc, hr_sampen = select_best_pc(hr_pca)
        print(f"[HR] 最佳 PC: {hr_best_pc+1}, SampEn: {hr_sampen[hr_best_pc]:.4f}")

        print("[HR] CWT Morlet 小波提取心率...")
        hr_bpm, hr_energy, hr_freqs = extract_rate_cwt(
            hr_pca[:, hr_best_pc], FS_HR, HR_BAND)

        if hr_bpm is not None:
            print(f"[HR] 心率: {hr_bpm:.1f} BPM")
        else:
            print("[HR] 心率提取失败")

    details = {
        'n_packets': n_packets,
        'n_sub': n_sub,
        'duration': duration,
        'rr_best_pc': rr_best_pc + 1 if rr_bpm is not None else None,
        'rr_sampen': rr_sampen[rr_best_pc] if rr_bpm is not None else None,
        'hr_best_pc': hr_best_pc + 1 if hr_bpm is not None else None,
        'hr_sampen': hr_sampen[hr_best_pc] if hr_bpm is not None else None,
    }

    return rr_bpm, hr_bpm, details


def main():
    parser = argparse.ArgumentParser(description="WiFi CSI 生命体征监测 (RR/HR)")
    parser.add_argument("--port", required=True, help="串口号 (如 COM28)")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--duration", type=int, default=120, help="采集时长(秒), 默认120")
    parser.add_argument("--output", default="vital_signs.csv", help="输出CSV文件")
    parser.add_argument("--delay", type=int, default=5, help="开始前等待(秒)")
    parser.add_argument("--interval", type=int, default=30, help="分析间隔(秒), 默认30")
    args = parser.parse_args()

    print("=" * 60)
    print("WiFi CSI 生命体征监测系统")
    print("论文: IEEE JTEHM 2025 - Alzaabi et al.")
    print("=" * 60)
    print(f"串口: {args.port} @ {args.baud}")
    print(f"采集时长: {args.duration}秒, 分析间隔: {args.interval}秒")
    print(f"输出: {args.output}")
    print()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {args.port}: {e}")
        sys.exit(1)

    if args.delay > 0:
        print(f"等待 {args.delay} 秒 (请保持静止，面对天线)...")
        for i in range(args.delay, 0, -1):
            sys.stdout.write(f"\r  倒计时: {i} 秒 ")
            sys.stdout.flush()
            time.sleep(1)
        print("\r  开始采集!            ")

    # Open CSV output
    csv_file = open(args.output, "w", newline="", encoding="utf-8-sig")
    writer = csv.writer(csv_file)
    writer.writerow(["timestamp", "rr_bpm", "hr_bpm", "n_packets", "duration_sec"])
    csv_file.flush()

    try:
        # Collect data for the specified duration
        timestamps_us, amplitudes = collect_data(ser, args.duration)

        # Process vital signs
        rr_bpm, hr_bpm, details = process_vital_signs(timestamps_us, amplitudes)

        # Output results
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        rr_str = f"{rr_bpm:.1f}" if rr_bpm is not None else "N/A"
        hr_str = f"{hr_bpm:.1f}" if hr_bpm is not None else "N/A"

        writer.writerow([timestamp, rr_str, hr_str,
                        details.get('n_packets', 0),
                        f"{details.get('duration', 0):.1f}"])
        csv_file.flush()

        print(f"\n{'='*60}")
        print(f"结果已保存到 {args.output}")
        print(f"呼吸率 (RR): {rr_str} BrPM")
        print(f"心率   (HR): {hr_str} BPM")
        print(f"{'='*60}")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        csv_file.close()
        ser.close()
        print("串口已关闭")


if __name__ == "__main__":
    main()
