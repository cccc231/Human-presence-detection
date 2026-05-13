#!/usr/bin/env python3
"""
CSI 体位变化检测脚本（论文方法扩展）

基于论文 "Design and Evaluation of Volunteer User Trials of Unobtrusive
Vital Signs Monitoring for Older People in Care Using Wi-Fi CSI Sensing"
(IEEE JTEHM 2025) 的信号处理工具链扩展，用于检测躺下/坐起动作。

算法流程:
  CSI I/Q 原始数据
        ↓
  振幅提取: amp = sqrt(I² + Q²)
        ↓
  插值重采样到 40 Hz
        ↓
  DWT db4 level 6 → cA6 (近似系数, <0.3Hz)
        ↓
  PCA 降维 (52子载波 → 5主成分)
        ↓
  滑动窗口 SampEn (每个 PC 独立计算)
        ↓
  SampEn 突变检测 (CUSUM)
        ↓
  mean_amp 方向判断 → 躺下 / 坐起

用法:
  python tools/bed_posture_detect.py --data-dir data
  python tools/bed_posture_detect.py --file data/bed_lying.csv
"""

import argparse
import csv
import sys
import os
import numpy as np
from datetime import datetime

try:
    import pywt
except ImportError:
    print("错误: pip install PyWavelets")
    sys.exit(1)

try:
    from scipy.interpolate import interp1d
    from scipy.signal import resample
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

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    print("错误: pip install matplotlib")
    sys.exit(1)


# ============================================================
# 数据加载与预处理
# ============================================================

def load_csv(filepath):
    """加载 CSV，返回 (timestamps_us, mean_amp, amp_matrix)"""
    timestamps, mean_amps, amp_rows = [], [], []
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        header = next(reader)
        n_amp = len([c for c in header if c.startswith('amp_')])
        for row in reader:
            if len(row) < 4 + n_amp:
                continue
            try:
                timestamps.append(int(row[0]))
                mean_amps.append(float(row[3]))
                amp_rows.append([float(row[4+i]) for i in range(n_amp)])
            except (ValueError, IndexError):
                continue
    return np.array(timestamps), np.array(mean_amps), np.array(amp_rows)


def resample_to_uniform(timestamps_us, signal_2d, target_fs=40.0):
    """将非均匀采样数据重采样到均匀时间间隔"""
    t_sec = (timestamps_us - timestamps_us[0]) / 1e6
    duration = t_sec[-1]
    n_out = int(duration * target_fs)
    if n_out < 10:
        return None, None, None
    t_uniform = np.linspace(0, duration, n_out)
    n_sub = signal_2d.shape[1]
    resampled = np.zeros((n_out, n_sub))
    for k in range(n_sub):
        f = interp1d(t_sec, signal_2d[:, k], kind='linear',
                     fill_value='extrapolate')
        resampled[:, k] = f(t_uniform)
    return t_uniform, resampled, target_fs


# ============================================================
# DWT 滤波
# ============================================================

def dwt_lowpass(data_2d, wavelet='db4', level=6):
    """
    DWT 低通滤波：只保留 cA6（近似系数）。
    cA6 对应频率 < fs/2^(level+1)，在 40Hz 采样下约 <0.3Hz。
    这个频段包含了体位变化的缓慢趋势。
    """
    n_samples, n_sub = data_2d.shape
    filtered = np.zeros_like(data_2d)

    for k in range(n_sub):
        coeffs = pywt.wavedec(data_2d[:, k], wavelet, level=level)
        # 只保留 cA6，其余系数置零
        new_coeffs = [coeffs[0]] + [np.zeros_like(c) for c in coeffs[1:]]
        reconstructed = pywt.waverec(new_coeffs, wavelet)
        # 裁剪到原始长度
        filtered[:, k] = reconstructed[:n_samples]

    return filtered


# ============================================================
# PCA 降维
# ============================================================

def apply_pca(data_2d, n_components=5):
    """PCA 降维，返回 (components, pca_model)"""
    pca = PCA(n_components=min(n_components, data_2d.shape[1]))
    components = pca.fit_transform(data_2d)
    return components, pca


# ============================================================
# 滑动窗口 SampEn
# ============================================================

def compute_sliding_sampen(signal_1d, window=300, step=10,
                           emb_dim=3, tolerance_factor=0.1):
    """
    滑动窗口计算 SampEn。

    参数:
      signal_1d: 输入信号
      window: 窗口大小（样本数）
      step: 步长（每隔多少样本计算一次）
      emb_dim: 嵌入维度（论文用 3）
      tolerance_factor: 容差因子（r = factor × std）

    返回:
      indices: SampEn 对应的样本索引
      sampen_values: SampEn 值序列
    """
    n = len(signal_1d)
    indices = []
    sampen_values = []

    for i in range(window, n, step):
        segment = signal_1d[i-window:i]
        std = np.std(segment)
        r = tolerance_factor * std
        if r == 0:
            indices.append(i)
            sampen_values.append(0)
            continue
        try:
            se = nolds.sampen(segment, emb_dim=emb_dim, tolerance=r)
            indices.append(i)
            sampen_values.append(se)
        except Exception:
            indices.append(i)
            sampen_values.append(0)

    return np.array(indices), np.array(sampen_values)


# ============================================================
# SampEn 突变检测 (CUSUM)
# ============================================================

def detect_sampen_transitions(sampen_values, delta_factor=0.5, h_factor=3.0):
    """
    用 CUSUM 检测 SampEn 的突增（体位变化信号）。

    SampEn 在体位稳定时较低（信号规则），体位变化时突然升高。
    只检测 SampEn 的上升突变。

    参数:
      delta_factor: delta = factor × std(SampEn)
      h_factor: h = factor × std(SampEn)

    返回:
      cusum: CUSUM 累积和序列
      triggers: [(index, sampen_value), ...] 触发位置
    """
    if len(sampen_values) < 10:
        return np.array([]), []

    baseline_se = np.median(sampen_values)
    std_se = np.std(sampen_values)
    delta = delta_factor * std_se
    h = h_factor * std_se

    n = len(sampen_values)
    cusum = np.zeros(n)
    triggers = []

    for i in range(1, n):
        # 只检测上升突变
        cusum[i] = max(0, cusum[i-1] + (sampen_values[i] - baseline_se) - delta)
        if cusum[i] > h:
            triggers.append((i, sampen_values[i]))
            cusum[i] = 0  # 重置

    return cusum, triggers


# ============================================================
# 方向判断
# ============================================================

def determine_direction(mean_amps, trigger_idx, window_after=120):
    """
    在 SampEn 触发位置，检查 mean_amp 的变化方向来判断是躺下还是坐起。

    返回: 'lying' (躺下，振幅通常下降) 或 'sitting' (坐起，振幅通常上升)
    """
    n = len(mean_amps)
    if trigger_idx >= n - 10:
        return 'unknown'

    # 取触发前 60 包的均值作为 before
    before_start = max(0, trigger_idx - 60)
    before_mean = np.mean(mean_amps[before_start:trigger_idx])

    # 取触发后 window_after 包的均值作为 after
    after_end = min(n, trigger_idx + window_after)
    after_mean = np.mean(mean_amps[trigger_idx:after_end])

    change = after_mean - before_mean

    # 下降 → 躺下，上升 → 坐起
    if change < 0:
        return 'lying'
    else:
        return 'sitting'


# ============================================================
# 完整分析流水线
# ============================================================

def analyze_posture(filepath, target_fs=40.0, dwt_level=6,
                    n_pca=5, sampen_window=300, sampen_step=10,
                    cusum_delta=0.5, cusum_h=3.0):
    """
    对单个 CSV 文件执行完整的体位变化检测分析。

    返回分析结果 dict。
    """
    # 加载数据
    ts, ma, amp = load_csv(filepath)
    if len(ts) < 100:
        return {'error': '数据太短'}

    duration = (ts[-1] - ts[0]) / 1e6
    pkt_rate = len(ts) / duration

    # 重采样到均匀时间间隔
    t_uniform, amp_resampled, fs = resample_to_uniform(ts, amp, target_fs)
    if t_uniform is None:
        return {'error': '重采样失败'}

    # mean_amp 也重采样
    f_ma = interp1d((ts - ts[0]) / 1e6, ma, kind='linear',
                    fill_value='extrapolate')
    ma_resampled = f_ma(t_uniform)

    # DWT 低通滤波
    amp_filtered = dwt_lowpass(amp_resampled, 'db4', dwt_level)

    # PCA 降维
    components, pca_model = apply_pca(amp_filtered, n_pca)

    # 对每个 PC 计算滑动窗口 SampEn
    pc_sampen = {}
    for i in range(n_pca):
        indices, se_values = compute_sliding_sampen(
            components[:, i], sampen_window, sampen_step)
        pc_sampen[i] = (indices, se_values)

    # 选择 SampEn 方差最大的 PC（体位变化会引起 SampEn 显著波动）
    best_pc = 0
    best_var = 0
    for i in range(n_pca):
        _, se_vals = pc_sampen[i]
        if len(se_vals) > 0:
            var = np.var(se_vals)
            if var > best_var:
                best_var = var
                best_pc = i

    # 对最佳 PC 做突变检测
    se_indices, se_values = pc_sampen[best_pc]
    cusum, triggers = detect_sampen_transitions(
        se_values, cusum_delta, cusum_h)

    # 对每个触发点判断方向
    events = []
    for idx_in_se, se_val in triggers:
        # 转换回原始时间轴索引
        original_idx = se_indices[idx_in_se]
        # 转换为重采样后的 mean_amp 索引
        amp_idx = int(original_idx)
        if amp_idx >= len(ma_resampled):
            continue

        direction = determine_direction(ma_resampled, amp_idx)
        event_time = t_uniform[amp_idx]
        events.append({
            'time_sec': event_time,
            'sampen': se_val,
            'direction': direction,
            'amp_idx': amp_idx,
        })

    return {
        'file': filepath,
        'n_packets': len(ts),
        'duration': duration,
        'pkt_rate': pkt_rate,
        'n_subcarriers': amp.shape[1],
        'fs': fs,
        'n_resampled': len(t_uniform),
        't_uniform': t_uniform,
        'ma_resampled': ma_resampled,
        'components': components,
        'pca_explained_var': pca_model.explained_variance_ratio_,
        'best_pc': best_pc,
        'pc_sampen': pc_sampen,
        'se_indices': se_indices,
        'se_values': se_values,
        'cusum': cusum,
        'triggers': triggers,
        'events': events,
    }


# ============================================================
# 可视化
# ============================================================

def generate_plots(results, output_dir):
    """生成分析图表"""
    fig, axes = plt.subplots(4, 1, figsize=(16, 20))

    labels = {'empty': 'Empty Bed', 'lying': 'Lying Down', 'getup': 'Getting Up'}
    colors = {'empty': 'green', 'lying': 'red', 'getup': 'blue'}

    # --- 图1: mean_amp 对比 ---
    ax = axes[0]
    for label, result in results.items():
        if 'error' in result:
            continue
        ax.plot(result['t_uniform'], result['ma_resampled'],
                color=colors[label], linewidth=1, alpha=0.7,
                label=labels[label])
        for ev in result['events']:
            marker = 'v' if ev['direction'] == 'lying' else '^'
            ax.plot(ev['time_sec'], result['ma_resampled'][ev['amp_idx']],
                    marker=marker, color='black', markersize=12)
    ax.set_title('Mean Amplitude (resampled)')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Amplitude')
    ax.legend(); ax.grid(True, alpha=0.3)

    # --- 图2: PCA 主成分 ---
    ax = axes[1]
    for label, result in results.items():
        if 'error' in result:
            continue
        best = result['best_pc']
        ax.plot(result['t_uniform'], result['components'][:, best],
                color=colors[label], linewidth=1, alpha=0.7,
                label=f"{labels[label]} PC{best+1}")
    ax.set_title('Best Principal Component')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('PC Value')
    ax.legend(); ax.grid(True, alpha=0.3)

    # --- 图3: SampEn 变化 ---
    ax = axes[2]
    for label, result in results.items():
        if 'error' in result:
            continue
        se_idx = result['t_uniform'][result['se_indices'].astype(int)]
        ax.plot(se_idx, result['se_values'],
                color=colors[label], linewidth=1.5, alpha=0.7,
                label=labels[label])
    ax.set_title(f'SampEn (best PC, window={300})')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Sample Entropy')
    ax.legend(); ax.grid(True, alpha=0.3)

    # --- 图4: CUSUM ---
    ax = axes[3]
    for label, result in results.items():
        if 'error' in result:
            continue
        se_idx = result['t_uniform'][result['se_indices'].astype(int)]
        ax.plot(se_idx, result['cusum'],
                color=colors[label], linewidth=1.5, alpha=0.7,
                label=labels[label])
        for ev in result['events']:
            marker = 'v' if ev['direction'] == 'lying' else '^'
            ax.plot(ev['time_sec'], max(result['cusum']) * 0.9,
                    marker=marker, color='black', markersize=12)
            ax.annotate(f"{ev['direction']}\nSE={ev['sampen']:.3f}",
                        (ev['time_sec'], max(result['cusum']) * 0.8),
                        fontsize=8, ha='center')
    ax.set_title('CUSUM on SampEn (body position change detector)')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('CUSUM Value')
    ax.legend(); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(output_dir, "posture_analysis.png")
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"图表已保存: {path}")
    return path


# ============================================================
# 主函数
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="CSI 体位变化检测（论文方法扩展）")
    parser.add_argument("--data-dir", default="data",
                        help="数据目录")
    parser.add_argument("--output-dir", default="analysis",
                        help="输出目录")
    parser.add_argument("--file", default=None,
                        help="单文件分析模式")
    parser.add_argument("--sampen-window", type=int, default=300,
                        help="SampEn 窗口大小（默认 300，@40Hz 约 7.5 秒）")
    parser.add_argument("--sampen-step", type=int, default=10,
                        help="SampEn 步长（默认 10）")
    parser.add_argument("--n-pca", type=int, default=5,
                        help="PCA 成分数（默认 5）")
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(base_dir, args.data_dir)
    output_dir = os.path.join(base_dir, args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 60)
    print("CSI 体位变化检测（论文方法扩展）")
    print("  DWT db4 → PCA → SampEn → CUSUM")
    print("=" * 60)

    if args.file:
        # 单文件分析
        filepath = args.file
        if not os.path.isabs(filepath):
            filepath = os.path.join(base_dir, filepath)
        if not os.path.exists(filepath):
            print(f"错误: 找不到 {filepath}")
            sys.exit(1)

        print(f"\n分析: {filepath}")
        result = analyze_posture(
            filepath, n_pca=args.n_pca,
            sampen_window=args.sampen_window,
            sampen_step=args.sampen_step)

        if 'error' in result:
            print(f"错误: {result['error']}")
            sys.exit(1)

        print(f"  包数: {result['n_packets']}, 时长: {result['duration']:.1f}s, "
              f"速率: {result['pkt_rate']:.0f} pkt/s")
        print(f"  重采样: {result['n_resampled']} 样本 @ {result['fs']} Hz")
        print(f"  PCA 解释方差比: {result['pca_explained_var']}")
        print(f"  最佳 PC: {result['best_pc']+1}")

        print(f"\n  检测到 {len(result['events'])} 个体位变化事件:")
        for ev in result['events']:
            print(f"    t={ev['time_sec']:.1f}s, "
                  f"SampEn={ev['sampen']:.4f}, "
                  f"方向={ev['direction']}")

    else:
        # 三组数据对比分析
        results = {}
        for label in ['empty', 'lying', 'getup']:
            filepath = os.path.join(data_dir, f"bed_{label}.csv")
            if not os.path.exists(filepath):
                print(f"跳过 {label}: 文件不存在 ({filepath})")
                continue

            print(f"\n分析 {label}: {filepath}")
            result = analyze_posture(
                filepath, n_pca=args.n_pca,
                sampen_window=args.sampen_window,
                sampen_step=args.sampen_step)

            if 'error' in result:
                print(f"  错误: {result['error']}")
                continue

            results[label] = result

            print(f"  包数: {result['n_packets']}, 时长: {result['duration']:.1f}s")
            print(f"  PCA 方差比: {result['pca_explained_var']}")
            print(f"  最佳 PC: {result['best_pc']+1}")
            print(f"  检测到 {len(result['events'])} 个事件:")
            for ev in result['events']:
                print(f"    t={ev['time_sec']:.1f}s, SampEn={ev['sampen']:.4f}, "
                      f"方向={ev['direction']}")

        if not results:
            print("\n没有有效数据。请先采集:")
            print("  python tools/bed_collect.py --port COM28 --label empty")
            print("  python tools/bed_collect.py --port COM28 --label lying")
            print("  python tools/bed_collect.py --port COM28 --label getup")
            sys.exit(1)

        # 生成图表
        print(f"\n生成图表...")
        generate_plots(results, output_dir)

        # 汇总报告
        print(f"\n{'='*60}")
        print("分析汇总:")
        print(f"{'='*60}")
        for label, result in results.items():
            ev_count = len(result['events'])
            lying_count = sum(1 for e in result['events']
                              if e['direction'] == 'lying')
            sitting_count = sum(1 for e in result['events']
                                if e['direction'] == 'sitting')
            print(f"  [{label}] {ev_count} 事件 "
                  f"(躺下={lying_count}, 坐起={sitting_count})")

        # 空床误报检查
        if 'empty' in results:
            fa = len(results['empty']['events'])
            if fa > 0:
                print(f"\n  WARNING: 空床数据有 {fa} 次误报!")
                print(f"  可能需要调整 --sampen-window 或 CUSUM 阈值")
            else:
                print(f"\n  空床 0 误报 ✓")

        print(f"\n输出文件: {output_dir}/posture_analysis.png")


if __name__ == "__main__":
    main()
