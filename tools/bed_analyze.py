#!/usr/bin/env python3
"""
CSI 床边数据分析脚本
读取三个场景的 CSV 数据（empty/lying/getup），分析振幅特征，推荐检测参数。

用法:
  python tools/bed_analyze.py
  python tools/bed_analyze.py --data-dir data --output-dir analysis

输入文件（默认在 data/ 目录下）:
  bed_empty.csv  - 空床数据
  bed_lying.csv  - 躺下过程数据
  bed_getup.csv  - 起床过程数据

输出:
  analysis/bed_analysis.png  - 可视化图表
  analysis/bed_params.txt    - 推荐参数
"""

import argparse
import csv
import sys
import os
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')  # 无显示器模式
    import matplotlib.pyplot as plt
except ImportError:
    print("错误: pip install matplotlib")
    sys.exit(1)


def load_csv(filepath):
    """加载 CSV 数据，返回 (timestamps_us, mean_amp, amp_matrix)"""
    timestamps = []
    mean_amps = []
    amp_rows = []

    with open(filepath, 'r', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        header = next(reader)  # 跳过表头
        n_amp_cols = len([c for c in header if c.startswith('amp_')])

        for row in reader:
            if len(row) < 4 + n_amp_cols:
                continue
            try:
                ts = int(row[0])
                mean_amp = float(row[3])
                amps = [float(row[4 + i]) for i in range(n_amp_cols)]
                timestamps.append(ts)
                mean_amps.append(mean_amp)
                amp_rows.append(amps)
            except (ValueError, IndexError):
                continue

    return (np.array(timestamps), np.array(mean_amps), np.array(amp_rows))


def compute_ema(data, alpha):
    """计算指数移动平均"""
    ema = np.zeros_like(data)
    ema[0] = data[0]
    for i in range(1, len(data)):
        ema[i] = alpha * data[i] + (1 - alpha) * ema[i-1]
    return ema


def compute_variance_per_subcarrier(amp_matrix, window=100):
    """计算每个子载波的滑动窗口方差，返回平均方差"""
    n_samples, n_sub = amp_matrix.shape
    var_metric = np.zeros(n_samples)
    for i in range(window, n_samples):
        window_data = amp_matrix[i-window:i, :]
        var_per_sub = np.var(window_data, axis=0)
        var_metric[i] = np.mean(var_per_sub)
    return var_metric


def compute_amp_deviation(amp_matrix, baseline):
    """计算振幅与 baseline 的偏差"""
    dev = np.mean(np.abs(amp_matrix - baseline), axis=1)
    return dev


def analyze_scenario(name, timestamps, mean_amps, amp_matrix):
    """分析单个场景的特征"""
    duration = (timestamps[-1] - timestamps[0]) / 1e6
    rate = len(timestamps) / duration

    stats = {
        'name': name,
        'n_packets': len(timestamps),
        'duration': duration,
        'rate': rate,
        'mean_amp_avg': np.mean(mean_amps),
        'mean_amp_std': np.std(mean_amps),
        'mean_amp_min': np.min(mean_amps),
        'mean_amp_max': np.max(mean_amps),
        'mean_amp_median': np.median(mean_amps),
    }

    # 不同 EMA alpha 的结果
    for alpha in [0.005, 0.01, 0.02, 0.05]:
        ema = compute_ema(mean_amps, alpha)
        stats[f'ema_{alpha}_final'] = ema[-1]
        stats[f'ema_{alpha}_std'] = np.std(ema)

    return stats


def find_thresholds(empty_amps, lying_amps, getup_amps, ema_alpha=0.01):
    """
    基于三个场景数据推荐阈值。

    逻辑:
    - 空床 EMA 作为 baseline
    - 躺下后 EMA 应低于 LOW_THRESHOLD → 设为 baseline × ratio_low
    - 起床后 EMA 应回升到 HIGH_THRESHOLD 以上 → 设为 baseline × ratio_high
    - 扫描不同 ratio 组合，找最优分离
    """
    empty_ema = compute_ema(empty_amps, ema_alpha)
    lying_ema = compute_ema(lying_amps, ema_alpha)
    getup_ema = compute_ema(getup_amps, ema_alpha)

    baseline = np.mean(empty_amps)  # 用原始均值作 baseline

    # 躺下数据的后半段（已躺下状态）的 EMA 水平
    lying_stable_start = len(lying_ema) * 2 // 3
    lying_stable_level = np.mean(lying_ema[lying_stable_start:])

    # 起床数据的后半段（已起床状态）的 EMA 水平
    getup_stable_start = len(getup_ema) * 2 // 3
    getup_stable_level = np.mean(getup_ema[getup_stable_start:])

    # 躺下后振幅下降比例
    lying_drop_ratio = lying_stable_level / baseline if baseline > 0 else 0.5

    # 起床后振幅恢复比例
    getup_recover_ratio = getup_stable_level / baseline if baseline > 0 else 0.8

    # LOW_THRESHOLD: 应低于 lying_stable_level，高于 empty 噪声下限
    # 推荐: baseline * (lying_drop_ratio + 0.05)，留 5% 余量
    threshold_low_ratio = lying_drop_ratio + 0.05

    # HIGH_THRESHOLD: 应低于 empty_stable_level，高于 getup_stable_level
    # 推荐: baseline * (getup_recover_ratio - 0.05)，留 5% 余量
    threshold_high_ratio = getup_recover_ratio - 0.05

    # 确保 LOW < HIGH
    if threshold_low_ratio >= threshold_high_ratio:
        mid = (threshold_low_ratio + threshold_high_ratio) / 2
        threshold_low_ratio = mid - 0.05
        threshold_high_ratio = mid + 0.05

    return {
        'baseline': baseline,
        'ema_alpha': ema_alpha,
        'lying_stable_level': lying_stable_level,
        'getup_stable_level': getup_stable_level,
        'lying_drop_ratio': lying_drop_ratio,
        'getup_recover_ratio': getup_recover_ratio,
        'threshold_low_ratio': threshold_low_ratio,
        'threshold_high_ratio': threshold_high_ratio,
        'threshold_low_abs': baseline * threshold_low_ratio,
        'threshold_high_abs': baseline * threshold_high_ratio,
        'empty_ema': empty_ema,
        'lying_ema': lying_ema,
        'getup_ema': getup_ema,
    }


def generate_plots(empty_data, lying_data, getup_data, thresholds, output_dir):
    """生成可视化图表"""
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle('CSI Bed Activity Detection - Data Analysis', fontsize=16)

    ts_e, ma_e, amp_e = empty_data
    ts_l, ma_l, amp_l = lying_data
    ts_g, ma_g, amp_g = getup_data

    # 归一化时间到秒
    t_e = (ts_e - ts_e[0]) / 1e6
    t_l = (ts_l - ts_l[0]) / 1e6
    t_g = (ts_g - ts_g[0]) / 1e6

    alpha = thresholds['ema_alpha']
    ema_e = compute_ema(ma_e, alpha)
    ema_l = compute_ema(ma_l, alpha)
    ema_g = compute_ema(ma_g, alpha)

    baseline = thresholds['baseline']
    low_thresh = thresholds['threshold_low_abs']
    high_thresh = thresholds['threshold_high_abs']

    # --- 图1: 空床 mean_amp ---
    ax = axes[0, 0]
    ax.plot(t_e, ma_e, alpha=0.3, color='blue', linewidth=0.5, label='Raw')
    ax.plot(t_e, ema_e, color='blue', linewidth=2, label=f'EMA (α={alpha})')
    ax.axhline(baseline, color='green', linestyle='--', linewidth=1.5, label=f'Baseline={baseline:.1f}')
    ax.axhline(low_thresh, color='red', linestyle=':', linewidth=1.5, label=f'Low={low_thresh:.1f}')
    ax.axhline(high_thresh, color='orange', linestyle=':', linewidth=1.5, label=f'High={high_thresh:.1f}')
    ax.set_title('Empty Bed')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Mean Amplitude')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- 图2: 躺下过程 ---
    ax = axes[0, 1]
    ax.plot(t_l, ma_l, alpha=0.3, color='blue', linewidth=0.5, label='Raw')
    ax.plot(t_l, ema_l, color='blue', linewidth=2, label=f'EMA (α={alpha})')
    ax.axhline(baseline, color='green', linestyle='--', linewidth=1.5, label='Baseline')
    ax.axhline(low_thresh, color='red', linestyle=':', linewidth=1.5, label=f'Low Thresh')
    ax.axhline(high_thresh, color='orange', linestyle=':', linewidth=1.5, label=f'High Thresh')
    ax.set_title('Lying Down Process')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Mean Amplitude')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- 图3: 起床过程 ---
    ax = axes[1, 0]
    ax.plot(t_g, ma_g, alpha=0.3, color='blue', linewidth=0.5, label='Raw')
    ax.plot(t_g, ema_g, color='blue', linewidth=2, label=f'EMA (α={alpha})')
    ax.axhline(baseline, color='green', linestyle='--', linewidth=1.5, label='Baseline')
    ax.axhline(low_thresh, color='red', linestyle=':', linewidth=1.5, label='Low Thresh')
    ax.axhline(high_thresh, color='orange', linestyle=':', linewidth=1.5, label='High Thresh')
    ax.set_title('Getting Up Process')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Mean Amplitude')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- 图4: 三场景 EMA 对比 ---
    ax = axes[1, 1]
    ax.plot(t_e, ema_e, color='green', linewidth=2, label='Empty')
    ax.plot(t_l, ema_l, color='red', linewidth=2, label='Lying')
    ax.plot(t_g, ema_g, color='blue', linewidth=2, label='Getup')
    ax.axhline(baseline, color='green', linestyle='--', alpha=0.5)
    ax.axhline(low_thresh, color='red', linestyle=':', linewidth=1.5)
    ax.axhline(high_thresh, color='orange', linestyle=':', linewidth=1.5)
    ax.set_title('EMA Comparison (all scenarios)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('EMA Amplitude')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- 图5: 子载波振幅分布对比 (热力图) ---
    ax = axes[2, 0]
    # 取每个场景的平均子载波振幅
    mean_per_sub_empty = np.mean(amp_e, axis=0)
    mean_per_sub_lying = np.mean(amp_l, axis=0)
    mean_per_sub_getup = np.mean(amp_g, axis=0)
    n_sub = len(mean_per_sub_empty)
    x = np.arange(n_sub)
    width = 0.25
    ax.bar(x - width, mean_per_sub_empty, width, label='Empty', alpha=0.7, color='green')
    ax.bar(x, mean_per_sub_lying, width, label='Lying', alpha=0.7, color='red')
    ax.bar(x + width, mean_per_sub_getup, width, label='Getup', alpha=0.7, color='blue')
    ax.set_title('Mean Amplitude per Subcarrier')
    ax.set_xlabel('Subcarrier Index')
    ax.set_ylabel('Amplitude')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- 图6: 滑动窗口方差对比 ---
    ax = axes[2, 1]
    window = 100
    var_e = compute_variance_per_subcarrier(amp_e, window)
    var_l = compute_variance_per_subcarrier(amp_l, window)
    var_g = compute_variance_per_subcarrier(amp_g, window)
    ax.plot(t_e, var_e, color='green', linewidth=1.5, label='Empty', alpha=0.7)
    ax.plot(t_l, var_l, color='red', linewidth=1.5, label='Lying', alpha=0.7)
    ax.plot(t_g, var_g, color='blue', linewidth=1.5, label='Getup', alpha=0.7)
    ax.set_title(f'Variance Metric (window={window})')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Mean Variance')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(output_dir, "bed_analysis.png")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"图表已保存: {plot_path}")
    return plot_path


def main():
    parser = argparse.ArgumentParser(description="CSI 床边数据分析")
    parser.add_argument("--data-dir", default="data", help="数据目录 (默认: data)")
    parser.add_argument("--output-dir", default="analysis", help="输出目录 (默认: analysis)")
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(base_dir, args.data_dir)
    output_dir = os.path.join(base_dir, args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # 检查输入文件
    files = {}
    for label in ["empty", "lying", "getup"]:
        path = os.path.join(data_dir, f"bed_{label}.csv")
        if not os.path.exists(path):
            print(f"错误: 找不到 {path}")
            print(f"请先运行数据采集:")
            print(f"  python tools/bed_collect.py --port COM28 --label {label} --duration 30")
            sys.exit(1)
        files[label] = path

    print("=" * 60)
    print("CSI 床边数据分析")
    print("=" * 60)

    # 加载数据
    print("\n加载数据...")
    empty_data = load_csv(files['empty'])
    lying_data = load_csv(files['lying'])
    getup_data = load_csv(files['getup'])

    ts_e, ma_e, amp_e = empty_data
    ts_l, ma_l, amp_l = lying_data
    ts_g, ma_g, amp_g = getup_data

    print(f"  空床: {len(ts_e)} 包, {len(amp_e[0])} 子载波, "
          f"{(ts_e[-1]-ts_e[0])/1e6:.1f}秒")
    print(f"  躺下: {len(ts_l)} 包, {len(amp_l[0])} 子载波, "
          f"{(ts_l[-1]-ts_l[0])/1e6:.1f}秒")
    print(f"  起床: {len(ts_g)} 包, {len(amp_g[0])} 子载波, "
          f"{(ts_g[-1]-ts_g[0])/1e6:.1f}秒")

    # 分析各场景
    print("\n场景统计:")
    for name, (ts, ma, amp) in [("空床", empty_data), ("躺下", lying_data),
                                 ("起床", getup_data)]:
        s = analyze_scenario(name, ts, ma, amp)
        print(f"  [{name}]")
        print(f"    包数: {s['n_packets']}, 时长: {s['duration']:.1f}s, "
              f"速率: {s['rate']:.0f} pkt/s")
        print(f"    mean_amp: avg={s['mean_amp_avg']:.2f}, "
              f"std={s['mean_amp_std']:.2f}, "
              f"range=[{s['mean_amp_min']:.2f}, {s['mean_amp_max']:.2f}]")
        for alpha in [0.005, 0.01, 0.02, 0.05]:
            print(f"    EMA(α={alpha}): final={s[f'ema_{alpha}_final']:.2f}, "
                  f"std={s[f'ema_{alpha}_std']:.2f}")

    # 扫描多个 EMA alpha，找最优
    print("\n阈值分析:")
    best_alpha = None
    best_margin = 0

    for alpha in [0.005, 0.01, 0.02, 0.05]:
        t = find_thresholds(ma_e, ma_l, ma_g, alpha)
        # 分离度 = (high_thresh - low_thresh) / baseline
        margin = (t['threshold_high_ratio'] - t['threshold_low_ratio'])
        print(f"  α={alpha}: baseline={t['baseline']:.2f}, "
              f"lying_level={t['lying_stable_level']:.2f} "
              f"({t['lying_drop_ratio']:.2f}x), "
              f"getup_level={t['getup_stable_level']:.2f} "
              f"({t['getup_recover_ratio']:.2f}x)")
        print(f"    LOW={t['threshold_low_ratio']:.3f}x "
              f"({t['threshold_low_abs']:.1f}), "
              f"HIGH={t['threshold_high_ratio']:.3f}x "
              f"({t['threshold_high_abs']:.1f}), "
              f"margin={margin:.3f}")

        if margin > best_margin:
            best_margin = margin
            best_alpha = alpha

    # 使用最优 alpha 生成最终结果
    print(f"\n最优 EMA alpha: {best_alpha}")
    thresholds = find_thresholds(ma_e, ma_l, ma_g, best_alpha)

    # 推荐 STABLE_COUNT
    # 躺下过程：从开始到稳定的大致包数
    lying_ema = thresholds['lying_ema']
    low_abs = thresholds['threshold_low_abs']
    # 找到 EMA 首次穿越 LOW_THRESHOLD 的位置
    cross_idx = np.where(lying_ema < low_abs)[0]
    if len(cross_idx) > 0:
        cross_time_l = cross_idx[0] / thresholds.get('ema_alpha', 0.01)
        # 实际时间
        lying_rate = len(ts_l) / ((ts_l[-1] - ts_l[0]) / 1e6)
        cross_sec_l = cross_idx[0] / lying_rate
    else:
        cross_sec_l = 5.0

    # 起床过程：从开始到 EMA 穿越 HIGH_THRESHOLD
    getup_ema = thresholds['getup_ema']
    high_abs = thresholds['threshold_high_abs']
    cross_idx_g = np.where(getup_ema > high_abs)[0]
    if len(cross_idx_g) > 0:
        getup_rate = len(ts_g) / ((ts_g[-1] - ts_g[0]) / 1e6)
        cross_sec_g = cross_idx_g[0] / getup_rate
    else:
        cross_sec_g = 5.0

    # 推荐 STABLE_COUNT = max(穿越时间 * 1.5, 120)
    lying_rate = len(ts_l) / ((ts_l[-1] - ts_l[0]) / 1e6)
    recommended_stable = max(int(max(cross_sec_l, cross_sec_g) * lying_rate * 1.5), 120)
    recommended_stable = min(recommended_stable, 600)  # 上限 5秒

    # 生成图表
    print("\n生成图表...")
    plot_path = generate_plots(empty_data, lying_data, getup_data, thresholds, output_dir)

    # 保存推荐参数
    params_path = os.path.join(output_dir, "bed_params.txt")
    with open(params_path, 'w', encoding='utf-8') as f:
        f.write("CSI 床边检测 - 推荐参数\n")
        f.write("=" * 50 + "\n\n")
        f.write(f"数据来源:\n")
        f.write(f"  空床: {files['empty']}\n")
        f.write(f"  躺下: {files['lying']}\n")
        f.write(f"  起床: {files['getup']}\n\n")
        f.write(f"Baseline (空床平均振幅): {thresholds['baseline']:.4f}\n")
        f.write(f"躺下稳定水平: {thresholds['lying_stable_level']:.4f} "
                f"({thresholds['lying_drop_ratio']:.3f}x baseline)\n")
        f.write(f"起床稳定水平: {thresholds['getup_stable_level']:.4f} "
                f"({thresholds['getup_recover_ratio']:.3f}x baseline)\n\n")
        f.write("推荐 ESP32 固件参数:\n")
        f.write("-" * 50 + "\n")
        f.write(f"#define BED_EMA_ALPHA         {best_alpha}f\n")
        f.write(f"#define BED_THRESH_LOW_RATIO  {thresholds['threshold_low_ratio']:.3f}f\n")
        f.write(f"#define BED_THRESH_HIGH_RATIO {thresholds['threshold_high_ratio']:.3f}f\n")
        f.write(f"#define BED_STABLE_COUNT      {recommended_stable}\n")
        f.write(f"#define BED_INIT_SAMPLES      500\n")
        f.write(f"\n阈值绝对值:\n")
        f.write(f"  LOW  = {thresholds['threshold_low_abs']:.2f} "
                f"(振幅低于此 → 躺下)\n")
        f.write(f"  HIGH = {thresholds['threshold_high_abs']:.2f} "
                f"(振幅高于此 → 起床)\n")
        f.write(f"  滞回区间: {thresholds['threshold_high_abs'] - thresholds['threshold_low_abs']:.2f}\n")

    print(f"参数文件已保存: {params_path}")

    # 打印最终推荐
    print(f"\n{'='*60}")
    print("推荐 ESP32 固件参数:")
    print(f"{'='*60}")
    print(f"  BED_EMA_ALPHA         = {best_alpha}")
    print(f"  BED_THRESH_LOW_RATIO  = {thresholds['threshold_low_ratio']:.3f}")
    print(f"  BED_THRESH_HIGH_RATIO = {thresholds['threshold_high_ratio']:.3f}")
    print(f"  BED_STABLE_COUNT      = {recommended_stable}")
    print(f"  BED_INIT_SAMPLES      = 500")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
