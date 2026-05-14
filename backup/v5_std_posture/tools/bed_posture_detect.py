#!/usr/bin/env python3
"""
CSI 体位变化检测脚本（per-subcarrier STD 方法）

原理：躺下时身体进入信号路径 → 大量子载波波动增强 → per-subcarrier STD 升高
      起床时身体离开信号路径 → 子载波波动变化小 → per-subcarrier STD 接近空床

算法流程:
  原始 CSI → 重采样 40Hz → 滑动窗口 per-subcarrier STD → 与空床基线比较
                                                              ↓
                                          > 50% 子载波 > 1.8x → lying
                                          否则但 ratio_mean > 1.2 → sitting
                                          否则 → quiet

用法:
  python tools/bed_posture_detect.py                     # 三组数据分析
  python tools/bed_posture_detect.py --scan              # 参数扫描
  python tools/bed_posture_detect.py --file data/bed_lying.csv
"""

import argparse, csv, sys, os, numpy as np
from scipy.interpolate import interp1d

try:
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    print("错误: pip install matplotlib"); sys.exit(1)


# ============================================================
# 数据加载
# ============================================================

def load_csv(fp):
    ts, ma, rows = [], [], []
    with open(fp, encoding='utf-8-sig') as f:
        r = csv.reader(f); h = next(r)
        n = len([c for c in h if c.startswith('amp_')])
        for row in r:
            if len(row) >= 4 + n:
                try:
                    ts.append(int(row[0])); ma.append(float(row[3]))
                    rows.append([float(row[4 + i]) for i in range(n)])
                except (ValueError, IndexError): pass
    return np.array(ts), np.array(ma), np.array(rows)


def resample(ts, amp_2d, fs=40):
    t = (ts - ts[0]) / 1e6; dur = t[-1]; n_out = int(dur * fs)
    if n_out < 10: return None, None
    t_u = np.linspace(0, dur, n_out); out = np.zeros((n_out, amp_2d.shape[1]))
    for k in range(amp_2d.shape[1]):
        out[:, k] = interp1d(t, amp_2d[:, k], kind='linear',
                             fill_value='extrapolate')(t_u)
    return t_u, out


# ============================================================
# 核心：滑动窗口 per-subcarrier STD 检测
# ============================================================

def detect_posture_events(amp_data, t_u, baseline_sub_std,
                          win=200, step=40, ratio_th=1.8, sub_frac=0.5,
                          sit_th=1.2, cooldown_win=3):
    """
    滑动窗口 per-subcarrier STD 检测体位变化事件。

    参数:
      amp_data: 重采样后的子载波振幅 (n_samples, n_sub)
      t_u: 均匀时间轴
      baseline_sub_std: 空床每子载波 STD 基线 (n_sub,)
      win: 滑动窗口大小（样本，默认 200 @40Hz = 5s）
      step: 窗口步长（样本，默认 40 @40Hz = 1s）
      ratio_th: lying 判定的 per-subcarrier STD 比值阈值（默认 1.8）
      sub_frac: lying 需要的高比值子载波占比（默认 0.5 = 50%）
      sit_th: sitting 判定的 ratio_mean 下限（默认 1.2）
      cooldown_win: 同类事件合并的时间窗口数

    返回: events 列表
    """
    n_samples, n_sub = amp_data.shape
    windows = []

    for i in range(win, n_samples, step):
        seg = amp_data[i - win:i]
        seg_std = np.std(seg, axis=0)
        ratio = seg_std / (baseline_sub_std + 0.01)
        high_count = int(np.sum(ratio > ratio_th))

        if high_count > n_sub * sub_frac:
            direction = 'lying'
        elif np.mean(ratio) > sit_th:
            direction = 'sitting'
        else:
            direction = 'quiet'

        windows.append({
            't_start': t_u[i - win],
            't_end': t_u[i],
            'high_count': high_count,
            'ratio_mean': np.mean(ratio),
            'direction': direction,
        })

    # 合并连续的同类事件，lying 优先
    events = []
    buf = []
    buf_dir = None

    for w in windows:
        if w['direction'] in ('lying', 'sitting'):
            if buf_dir is None:
                # 第一个事件
                buf = [w]; buf_dir = w['direction']
            elif buf_dir == w['direction']:
                # 同类，检查连续性
                if w['t_start'] - buf[-1]['t_end'] < cooldown_win:
                    buf.append(w)
                else:
                    # 间隔过大，输出前一组
                    events.append({
                        't_start': buf[0]['t_start'],
                        't_end': buf[-1]['t_end'],
                        'direction': buf_dir,
                        'ratio_mean': np.mean([b['ratio_mean'] for b in buf]),
                        'high_count_max': max(b['high_count'] for b in buf),
                    })
                    buf = [w]; buf_dir = w['direction']
            elif buf_dir == 'lying' and w['direction'] == 'sitting':
                # Lying 优先: sitting 可能是 lying 衰减尾部
                if w['t_start'] - buf[-1]['t_end'] < 3:
                    buf.append(w)  # 合并入 lying
                else:
                    events.append({
                        't_start': buf[0]['t_start'],
                        't_end': buf[-1]['t_end'],
                        'direction': buf_dir,
                        'ratio_mean': np.mean([b['ratio_mean'] for b in buf]),
                        'high_count_max': max(b['high_count'] for b in buf),
                    })
                    buf = [w]; buf_dir = w['direction']
            else:
                # 不同方向，输出前一组
                events.append({
                    't_start': buf[0]['t_start'],
                    't_end': buf[-1]['t_end'],
                    'direction': buf_dir,
                    'ratio_mean': np.mean([b['ratio_mean'] for b in buf]),
                    'high_count_max': max(b['high_count'] for b in buf),
                })
                buf = [w]; buf_dir = w['direction']
        else:
            # 安静窗口，输出缓冲
            if buf and buf_dir is not None:
                events.append({
                    't_start': buf[0]['t_start'],
                    't_end': buf[-1]['t_end'],
                    'direction': buf_dir,
                    'ratio_mean': np.mean([b['ratio_mean'] for b in buf]),
                    'high_count_max': max(b['high_count'] for b in buf),
                })
                buf = []; buf_dir = None

    # 刷新最后缓冲
    if buf and buf_dir is not None:
        events.append({
            't_start': buf[0]['t_start'],
            't_end': buf[-1]['t_end'],
            'direction': buf_dir,
            'ratio_mean': np.mean([b['ratio_mean'] for b in buf]),
            'high_count_max': max(b['high_count'] for b in buf),
        })

    return events


# ============================================================
# 参数扫描
# ============================================================

def scan_params(amp_empty, amp_lying, amp_getup, t_e, t_l, t_g,
                baseline_sub_std):
    """扫描 win/step/ratio_th/sub_frac/sit_th 找最优参数"""
    win_list = [150, 200, 250, 300]
    step_list = [20, 40, 60]
    ratio_list = [1.5, 1.6, 1.8, 2.0]
    sub_frac_list = [0.4, 0.5, 0.6]
    sit_th_list = [1.1, 1.2, 1.3]

    results = []
    n_sub = amp_empty.shape[1]
    total = len(win_list) * len(step_list) * len(ratio_list) * len(sub_frac_list) * len(sit_th_list)
    count = 0

    for win in win_list:
        for step in step_list:
            for rt in ratio_list:
                for sf in sub_frac_list:
                    for st in sit_th_list:
                        count += 1
                        ev_e = detect_posture_events(
                            amp_empty, t_e, baseline_sub_std, win, step, rt, sf, st)
                        ev_l = detect_posture_events(
                            amp_lying, t_l, baseline_sub_std, win, step, rt, sf, st)
                        ev_g = detect_posture_events(
                            amp_getup, t_g, baseline_sub_std, win, step, rt, sf, st)

                        # 评分
                        score = 0
                        # 空床: 0 事件
                        if len(ev_e) == 0:
                            score += 1000
                        else:
                            score -= 10000 * len(ev_e)
                            continue

                        # 躺下: 恰好 1 次 lying
                        lying_ev = [e for e in ev_l if e['direction'] == 'lying']
                        if len(lying_ev) == 1:
                            score += 500
                            score -= lying_ev[0]['t_start'] * 10  # 延迟惩罚
                        else:
                            score -= 1000 * abs(len(lying_ev) - 1)

                        # 起床: 恰好 1 次 sitting
                        sitting_ev = [e for e in ev_g if e['direction'] == 'sitting']
                        if len(sitting_ev) == 1:
                            score += 500
                            score -= sitting_ev[0]['t_start'] * 10
                        else:
                            score -= 1000 * abs(len(sitting_ev) - 1)

                        # 额外事件惩罚
                        extra_l = len(ev_l) - len(lying_ev)
                        extra_g = len(ev_g) - len(sitting_ev)
                        score -= (extra_l + extra_g) * 50

                        if score > 0:
                            results.append({
                                'score': score, 'win': win, 'step': step,
                                'ratio_th': rt, 'sub_frac': sf, 'sit_th': st,
                                'n_empty': len(ev_e),
                                'n_lying': len(lying_ev),
                                'n_sitting': len(sitting_ev),
                            })

    results.sort(key=lambda x: x['score'], reverse=True)
    return results


# ============================================================
# 可视化
# ============================================================

def generate_plots(results, baseline_sub_std, params, output_dir):
    """生成分析图表"""
    fig, axes = plt.subplots(3, 1, figsize=(16, 16))
    labels = {'empty': 'Empty Bed', 'lying': 'Lying Down', 'getup': 'Getting Up'}
    colors = {'empty': 'green', 'lying': 'red', 'getup': 'blue'}

    for ax_i, label in enumerate(['empty', 'lying', 'getup']):
        ax = axes[ax_i]
        if label not in results:
            continue
        r = results[label]
        t = r['t_u']; amp = r['amp_r']; ma = r['ma']
        events = r['events']

        ax.plot(t, ma, color=colors[label], linewidth=0.8, alpha=0.7)
        ax.set_title(labels[label])
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Mean Amplitude')
        ax.grid(True, alpha=0.3)

        for ev in events:
            c = 'purple' if ev['direction'] == 'lying' else 'orange'
            ax.axvspan(ev['t_start'], ev['t_end'], alpha=0.2, color=c,
                       label=ev['direction'])
            ax.annotate(ev['direction'],
                        ((ev['t_start'] + ev['t_end']) / 2, np.max(ma)),
                        fontsize=10, fontweight='bold',
                        ha='center', va='bottom', color=c)
        if events:
            ax.legend(fontsize=8)

    plt.suptitle(f'CSI Posture Detection (win={params["win"]}({params["win"]/40:.0f}s), '
                 f'ratio_th={params["ratio_th"]}, sub_frac={params["sub_frac"]})',
                 fontsize=14)
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
        description="CSI 体位变化检测（per-subcarrier STD 方法）")
    parser.add_argument("--data-dir", default="data")
    parser.add_argument("--output-dir", default="analysis")
    parser.add_argument("--file", default=None, help="单文件分析")
    parser.add_argument("--scan", action="store_true", help="参数扫描模式")
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(base_dir, args.data_dir)
    output_dir = os.path.join(base_dir, args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 60)
    print("CSI 体位变化检测（per-subcarrier STD 方法）")
    print("=" * 60)

    labels = ['empty', 'lying', 'getup']
    files = {}
    for lb in labels:
        fp = os.path.join(data_dir, f"bed_{lb}.csv")
        if not os.path.exists(fp):
            print(f"错误: 找不到 {fp}")
            sys.exit(1)
        files[lb] = fp

    # 加载 + 重采样
    print("\n加载并重采样...")
    raw = {}
    for lb in labels:
        ts, ma, amp = load_csv(files[lb])
        t_u, amp_r = resample(ts, amp)
        if amp_r is None:
            print(f"  {lb}: 重采样失败")
            sys.exit(1)
        f_ma = interp1d((ts - ts[0]) / 1e6, ma, kind='linear',
                        fill_value='extrapolate')
        raw[lb] = {'t_u': t_u, 'amp_r': amp_r, 'ma': f_ma(t_u),
                   'n_sub': amp_r.shape[1], 'n_samples': len(t_u)}
        print(f"  {lb}: {len(ts)}包 -> {len(t_u)}样本 @40Hz, "
              f"{amp_r.shape[1]}子载波")

    # 空床基线
    amp_e = raw['empty']['amp_r']
    baseline_sub_std = np.std(amp_e[:len(amp_e)//2], axis=0)

    if args.scan:
        # 参数扫描
        print(f"\n参数扫描...")
        scan_results = scan_params(
            raw['empty']['amp_r'], raw['lying']['amp_r'],
            raw['getup']['amp_r'],
            raw['empty']['t_u'], raw['lying']['t_u'],
            raw['getup']['t_u'], baseline_sub_std)

        if not scan_results:
            print("  未找到有效参数")
            sys.exit(1)

        best = scan_results[0]
        print(f"\n最优参数 (score={best['score']}):")
        for k in ['win', 'step', 'ratio_th', 'sub_frac', 'sit_th',
                  'n_empty', 'n_lying', 'n_sitting']:
            print(f"  {k} = {best[k]}")

        # Top 5
        print(f"\n{'win':>5s} {'step':>5s} {'ratio':>6s} {'sub':>6s} "
              f"{'sit':>5s} {'FA':>4s} {'ly':>4s} {'st':>4s} {'score':>8s}")
        for r in scan_results[:5]:
            print(f"{r['win']:5d} {r['step']:5d} {r['ratio_th']:6.2f} "
                  f"{r['sub_frac']:6.2f} {r['sit_th']:5.2f} "
                  f"{r['n_empty']:4d} {r['n_lying']:4d} {r['n_sitting']:4d} "
                  f"{r['score']:8.0f}")

        # 用最优参数跑最终结果
        params = {k: best[k] for k in ['win', 'step', 'ratio_th',
                                         'sub_frac', 'sit_th']}
        final = {}
        for lb in labels:
            events = detect_posture_events(
                raw[lb]['amp_r'], raw[lb]['t_u'], baseline_sub_std,
                **{k: params[k] for k in ['win', 'step',
                                           'ratio_th', 'sub_frac', 'sit_th']})
            final[lb] = {'t_u': raw[lb]['t_u'], 'amp_r': raw[lb]['amp_r'],
                         'ma': raw[lb]['ma'], 'events': events}
            print(f"\n[{lb}] {len(events)} events:")
            for ev in events:
                print(f"  {ev['direction']} [{ev['t_start']:.0f}-{ev['t_end']:.0f}s] "
                      f"ratio_mean={ev['ratio_mean']:.2f}")

        generate_plots(final, baseline_sub_std, params, output_dir)

        # 生成 bed_params.h
        hp = os.path.join(output_dir, "bed_posture_params.h")
        with open(hp, 'w', encoding='utf-8') as f:
            f.write("/* CSI 体位检测参数 - per-subcarrier STD 方法 */\n\n"
                    "#pragma once\n\n"
                    f"#define STD_WIN_SAMPLES    {params['win']}\n"
                    f"#define STD_STEP_SAMPLES  {params['step']}\n"
                    f"#define STD_RATIO_LYING   {params['ratio_th']:.2f}f\n"
                    f"#define STD_SUB_FRAC      {params['sub_frac']:.2f}f\n"
                    f"#define STD_RATIO_SITTING {params['sit_th']:.2f}f\n")
        print(f"\n参数已保存: {hp}")

    else:
        # 默认参数模式
        params = {'win': 200, 'step': 40, 'ratio_th': 1.8,
                  'sub_frac': 0.5, 'sit_th': 1.2}

        results = {}
        for lb in labels:
            events = detect_posture_events(
                raw[lb]['amp_r'], raw[lb]['t_u'], baseline_sub_std,
                **params)
            results[lb] = {'t_u': raw[lb]['t_u'], 'amp_r': raw[lb]['amp_r'],
                           'ma': raw[lb]['ma'], 'events': events}
            print(f"\n[{lb}] {len(events)} events:")
            for ev in events:
                print(f"  {ev['direction']} [{ev['t_start']:.0f}-{ev['t_end']:.0f}s] "
                      f"ratio_mean={ev['ratio_mean']:.2f}")

        generate_plots(results, baseline_sub_std, params, output_dir)


if __name__ == "__main__":
    main()
