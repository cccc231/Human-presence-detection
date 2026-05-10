#!/usr/bin/env python3
"""
CSI 床边数据分析脚本 (v4 - 完整状态机 + 多特征融合)

架构:
  occupancy_score 做方向无关的状态判断
  EMA 滞回阈值做最终确认
  CUSUM / fast EMA 做候选触发
  MAYBE 状态 + 稳定计数 + 超时 + 冷却防误报
  只在稳定空床时慢速更新 baseline
  离线状态机模拟选参数

用法:
  python tools/bed_analyze.py
  python tools/bed_analyze.py --data-dir data --output-dir analysis
"""

import argparse
import csv
import sys
import os
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    print("错误: pip install matplotlib")
    sys.exit(1)


# ============================================================
# 数据加载
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


# ============================================================
# Baseline 估计
# ============================================================

def estimate_baseline(ma_empty, amp_empty, n_init=500):
    """从空床数据估算 baseline 和噪声统计"""
    n = min(n_init, len(ma_empty))
    baseline_mean = np.mean(ma_empty[:n])
    baseline_sub = np.mean(amp_empty[:n], axis=0)
    empty_std = np.std(ma_empty[:n])
    empty_var = np.var(ma_empty[:n])
    return {
        'baseline_mean': baseline_mean,
        'baseline_sub': baseline_sub,
        'empty_std': empty_std,
        'empty_var': empty_var,
        'n_used': n,
    }


# ============================================================
# 特征计算（对整个序列一次性计算）
# ============================================================

def compute_features(mean_amps, amp_matrix, baseline, alpha_slow, alpha_fast):
    """
    计算全部特征序列。

    返回 dict:
      slow_ema, fast_ema, occupancy_score, change_score,
      motion_score, deviation_score, innovation,
      signed_innovation, cusum_to_lying, cusum_to_empty
    """
    n = len(mean_amps)
    baseline_mean = baseline['baseline_mean']
    baseline_sub = baseline['baseline_sub']

    # 判断 direction: 躺下后振幅是升还是降
    # 先用整体趋势估算，后面 state machine 会用实际数据
    # 这里先假设 direction=-1 (下降)，state machine 会自动修正
    direction = -1

    slow_ema = np.zeros(n)
    fast_ema = np.zeros(n)
    slow_ema[0] = mean_amps[0]
    fast_ema[0] = mean_amps[0]

    for i in range(1, n):
        slow_ema[i] = alpha_slow * mean_amps[i] + (1 - alpha_slow) * slow_ema[i-1]
        fast_ema[i] = alpha_fast * mean_amps[i] + (1 - alpha_fast) * fast_ema[i-1]

    # occupancy_score: 方向无关，越大越像有人
    if baseline_mean > 0:
        occupancy_score_raw = (slow_ema - baseline_mean) / baseline_mean
    else:
        occupancy_score_raw = np.zeros(n)

    # direction 会在 state machine 中根据实际数据确定
    # 这里先输出原始值，state machine 会乘以 direction
    occupancy_score = occupancy_score_raw

    # fast_slow_diff
    fast_slow_diff = fast_ema - slow_ema

    # change_score (先用 raw，direction 由 state machine 处理)
    if baseline_mean > 0:
        change_score = fast_slow_diff / baseline_mean
    else:
        change_score = np.zeros(n)

    # motion_score: 滑动窗口方差 (窗口=100包)
    window = 100
    motion_score = np.zeros(n)
    for i in range(window, n):
        motion_score[i] = np.var(mean_amps[i-window:i])

    # deviation_score: 子载波级别的偏离
    deviation_score = np.zeros(n)
    for i in range(n):
        if np.all(baseline_sub > 0):
            deviation_score[i] = np.mean(
                np.abs(amp_matrix[i] - baseline_sub) / baseline_sub)

    # innovation: mean_amp - slow_ema
    innovation = mean_amps - slow_ema

    # signed_innovation (direction 由 state machine 修正)
    signed_innovation = innovation.copy()

    return {
        'slow_ema': slow_ema,
        'fast_ema': fast_ema,
        'occupancy_score': occupancy_score,
        'change_score': change_score,
        'motion_score': motion_score,
        'deviation_score': deviation_score,
        'innovation': innovation,
        'signed_innovation': signed_innovation,
        'fast_slow_diff': fast_slow_diff,
    }


# ============================================================
# 状态机定义
# ============================================================

@dataclass
class BedState:
    """床检测状态"""
    INIT_CALIBRATING = 0
    EMPTY = 1
    MAYBE_LYING = 2
    LYING = 3
    MAYBE_EMPTY = 4
    COOLDOWN = 5

STATE_NAMES = {
    0: 'INIT', 1: 'EMPTY', 2: 'MAYBE_LYING',
    3: 'LYING', 4: 'MAYBE_EMPTY', 5: 'COOLDOWN'
}


@dataclass
class BedParams:
    """检测参数"""
    alpha_slow: float = 0.01
    alpha_fast: float = 0.1
    baseline_alpha: float = 0.0005
    occ_enter_th: float = 0.18
    occ_exit_th: float = 0.10
    change_th: float = 0.03
    motion_high_th: float = 12.0
    motion_quiet_th: float = 5.0
    dev_enter_th: float = 0.08
    cusum_delta: float = 1.5
    cusum_h: float = 12.0
    enter_stable_count: int = 240
    exit_stable_count: int = 180
    candidate_timeout: int = 720
    cooldown_count: int = 240
    init_samples: int = 500


@dataclass
class SimResult:
    """模拟结果"""
    confirmed_events: List[Tuple[int, str]]  # (index, event_type)
    state_log: np.ndarray
    occ_log: np.ndarray
    cusum_lying_log: np.ndarray
    cusum_empty_log: np.ndarray
    enter_counter_log: np.ndarray
    exit_counter_log: np.ndarray
    lying_confirm_count: int = 0
    getup_confirm_count: int = 0
    lying_delay: float = 0.0
    getup_delay: float = 0.0
    false_confirms: int = 0
    maybe_lying_count: int = 0
    maybe_empty_count: int = 0
    state_flip_count: int = 0


# ============================================================
# 状态机模拟
# ============================================================

def simulate_state_machine(mean_amps, amp_matrix, baseline,
                           params: BedParams, pkt_rate=120.0) -> SimResult:
    """
    用给定参数模拟完整状态机。

    输入:
      mean_amps: 振幅序列
      amp_matrix: 子载波矩阵
      baseline: baseline dict
      params: 检测参数
      pkt_rate: 包率

    返回:
      SimResult
    """
    n = len(mean_amps)
    base = baseline['baseline_mean']
    base_sub = baseline['baseline_sub']

    # 状态
    state = BedState.INIT_CALIBRATING
    confirmed_state = BedState.EMPTY
    confirmed_events = []

    # 计数器
    enter_counter = 0
    exit_counter = 0
    candidate_timer = 0
    cooldown_counter = 0
    cooldown_target = BedState.EMPTY
    init_count = 0

    # Baseline（可更新）
    bl = base

    # CUSUM
    cusum_lying = 0.0
    cusum_empty = 0.0

    # EMA 状态
    slow_ema = mean_amps[0]
    fast_ema = mean_amps[0]

    # 日志
    state_log = np.zeros(n, dtype=int)
    occ_log = np.zeros(n)
    cusum_lying_log = np.zeros(n)
    cusum_empty_log = np.zeros(n)
    enter_log = np.zeros(n, dtype=int)
    exit_log = np.zeros(n, dtype=int)

    # direction: 由实际数据确定
    direction = 0  # 未确定

    # 统计
    maybe_lying_count = 0
    maybe_empty_count = 0
    state_flip_count = 0
    last_confirmed = BedState.EMPTY

    for i in range(n):
        amp = mean_amps[i]

        # --- 滤波 ---
        slow_ema = params.alpha_slow * amp + (1 - params.alpha_slow) * slow_ema
        fast_ema = params.alpha_fast * amp + (1 - params.alpha_fast) * fast_ema

        # --- 特征 ---
        # occupancy_score: |slow_ema - baseline| / baseline
        if bl > 0:
            raw_occ = (slow_ema - bl) / bl
        else:
            raw_occ = 0.0

        # direction 确定: 首次进入 LYING 时确定
        if direction == 0 and state == BedState.INIT_CALIBRATING and init_count > params.init_samples:
            direction = -1  # 默认下降，后续可修正

        # 根据 direction 修正 occupancy_score
        if direction != 0:
            occupancy_score = direction * raw_occ
            # 如果 direction=-1（躺下下降），raw_occ 为负时 occupancy_score 为正
            # 即 slow_ema < baseline → occupancy_score > 0 → 像有人
        else:
            occupancy_score = abs(raw_occ)

        # innovation
        innovation = amp - slow_ema
        signed_inn = direction * innovation if direction != 0 else innovation

        # CUSUM
        cusum_lying = max(0, cusum_lying + signed_inn - params.cusum_delta)
        cusum_empty = max(0, cusum_empty - signed_inn - params.cusum_delta)

        # motion_score (简化: 用最近窗口方差)
        window = 100
        if i >= window:
            motion_score = np.var(mean_amps[i-window:i])
        else:
            motion_score = 0.0

        # deviation_score
        if np.all(base_sub > 0):
            dev_score = np.mean(np.abs(amp_matrix[i] - base_sub) / base_sub)
        else:
            dev_score = 0.0

        # --- 候选条件 ---
        lying_candidate = (
            occupancy_score > params.occ_enter_th
            or cusum_lying > params.cusum_h
            or (direction != 0 and direction * (fast_ema - slow_ema) / bl > params.change_th)
        )
        empty_candidate = (
            occupancy_score < params.occ_exit_th
            or cusum_empty > params.cusum_h
            or (direction != 0 and direction * (fast_ema - slow_ema) / bl < -params.change_th)
        )

        # --- 状态机 ---
        if state == BedState.INIT_CALIBRATING:
            init_count += 1
            if init_count >= params.init_samples:
                # 用初始化段重新估算 baseline
                bl = np.mean(mean_amps[:params.init_samples])
                # 用初始化后的数据确定 direction (需要后续数据)
                state = BedState.EMPTY

        elif state == BedState.EMPTY:
            # 慢速更新 baseline (仅在安静空床时)
            if (occupancy_score < params.occ_exit_th
                and motion_score < params.motion_quiet_th
                and cusum_lying < params.cusum_h * 0.5):
                bl = bl * (1 - params.baseline_alpha) + amp * params.baseline_alpha

            if lying_candidate:
                state = BedState.MAYBE_LYING
                enter_counter = 0
                candidate_timer = 0
                maybe_lying_count += 1

        elif state == BedState.MAYBE_LYING:
            candidate_timer += 1
            if occupancy_score > params.occ_enter_th:
                enter_counter += 1
            else:
                enter_counter = max(0, enter_counter - 1)

            if enter_counter >= params.enter_stable_count:
                # 确认躺下
                confirmed_state = BedState.LYING
                confirmed_events.append((i, 'LYING_CONFIRMED'))
                state = BedState.COOLDOWN
                cooldown_target = BedState.LYING
                cooldown_counter = params.cooldown_count
                cusum_lying = 0
                if last_confirmed != BedState.LYING:
                    state_flip_count += 1
                last_confirmed = BedState.LYING
                # 确定 direction
                if direction == 0:
                    direction = -1 if slow_ema < bl else 1

            elif occupancy_score < params.occ_exit_th:
                state = BedState.EMPTY
                enter_counter = 0

            elif candidate_timer > params.candidate_timeout:
                state = BedState.EMPTY
                enter_counter = 0

        elif state == BedState.LYING:
            if empty_candidate:
                state = BedState.MAYBE_EMPTY
                exit_counter = 0
                candidate_timer = 0
                maybe_empty_count += 1

        elif state == BedState.MAYBE_EMPTY:
            candidate_timer += 1
            if occupancy_score < params.occ_exit_th:
                exit_counter += 1
            else:
                exit_counter = max(0, exit_counter - 1)

            if exit_counter >= params.exit_stable_count:
                confirmed_state = BedState.EMPTY
                confirmed_events.append((i, 'GETUP_CONFIRMED'))
                state = BedState.COOLDOWN
                cooldown_target = BedState.EMPTY
                cooldown_counter = params.cooldown_count
                cusum_empty = 0
                if last_confirmed != BedState.EMPTY:
                    state_flip_count += 1
                last_confirmed = BedState.EMPTY

            elif occupancy_score > params.occ_enter_th:
                state = BedState.LYING
                exit_counter = 0

            elif candidate_timer > params.candidate_timeout:
                state = BedState.LYING
                exit_counter = 0

        elif state == BedState.COOLDOWN:
            cooldown_counter -= 1
            if cooldown_counter <= 0:
                state = cooldown_target

        # 记录
        state_log[i] = state
        occ_log[i] = occupancy_score
        cusum_lying_log[i] = cusum_lying
        cusum_empty_log[i] = cusum_empty
        enter_log[i] = enter_counter
        exit_log[i] = exit_counter

    # 统计
    lying_events = [e for e in confirmed_events if e[1] == 'LYING_CONFIRMED']
    getup_events = [e for e in confirmed_events if e[1] == 'GETUP_CONFIRMED']
    lying_delay = lying_events[0][0] / pkt_rate if lying_events else 0
    getup_delay = getup_events[0][0] / pkt_rate if getup_events else 0

    return SimResult(
        confirmed_events=confirmed_events,
        state_log=state_log, occ_log=occ_log,
        cusum_lying_log=cusum_lying_log, cusum_empty_log=cusum_empty_log,
        enter_counter_log=enter_log, exit_counter_log=exit_log,
        lying_confirm_count=len(lying_events),
        getup_confirm_count=len(getup_events),
        lying_delay=lying_delay, getup_delay=getup_delay,
        false_confirms=0,  # 由调用方计算
        maybe_lying_count=maybe_lying_count,
        maybe_empty_count=maybe_empty_count,
        state_flip_count=state_flip_count,
    )


# ============================================================
# 参数扫描与评分
# ============================================================

def score_result(empty_sim, lying_sim, getup_sim):
    """
    评分: 空床无误报 + 躺下/起床各确认1次 + 延迟短 + 误触发少。

    评分标准:
      +1000  空床 0 误报
      -10000 每次空床误报
      +500   躺下确认恰好 1 次
      -1000  偏离 1 次
      +500   起床确认恰好 1 次
      -1000  偏离 1 次
      -延迟秒 × 20
      -maybe 误触发次数 × 5
      -状态跳变次数 × 100
    """
    score = 0

    # 空床: 必须 0 误报
    empty_events = len(empty_sim.confirmed_events)
    if empty_events == 0:
        score += 1000
    else:
        score -= 10000 * empty_events
        return score  # 有误报直接淘汰

    # 躺下: 必须恰好 1 次
    if lying_sim.lying_confirm_count == 1:
        score += 500
    else:
        score -= 1000 * abs(lying_sim.lying_confirm_count - 1)

    # 起床: 必须恰好 1 次
    if getup_sim.getup_confirm_count == 1:
        score += 500
    else:
        score -= 1000 * abs(getup_sim.getup_confirm_count - 1)

    # 延迟惩罚
    score -= lying_sim.lying_delay * 20
    score -= getup_sim.getup_delay * 20

    # maybe 误触发惩罚
    score -= lying_sim.maybe_lying_count * 5
    score -= lying_sim.maybe_empty_count * 5
    score -= getup_sim.maybe_lying_count * 5
    score -= getup_sim.maybe_empty_count * 5

    # 状态跳变惩罚
    score -= lying_sim.state_flip_count * 100
    score -= getup_sim.state_flip_count * 100

    return score


def scan_parameters(ma_empty, amp_empty, ma_lying, amp_lying,
                    ma_getup, amp_getup, baseline, pkt_rate=120.0):
    """
    扫描参数，返回按得分排序的结果列表。
    """
    empty_std = baseline['empty_std']

    # 参数候选
    alpha_slow_list = [0.005, 0.01, 0.02]
    alpha_fast_list = [0.05, 0.1, 0.2]
    enter_stable_list = [120, 180, 240, 360]
    exit_stable_list = [60, 120, 180, 240]
    timeout_list = [360, 600, 960]
    cooldown_list = [120, 240, 360]
    cusum_delta_list = [0.5, 1.0, 1.5]
    cusum_h_list = [5, 8, 10, 15, 20]

    # 基于 baseline 估算 occupancy 阈值
    # 先用 alpha_slow=0.01 跑一遍 lying 数据，看稳定段的 occupancy
    # 这里用粗略估计
    lying_est_level = np.mean(ma_lying[len(ma_lying)*2//3:])
    getup_est_level = np.mean(ma_getup[len(ma_getup)*2//3:])
    bl = baseline['baseline_mean']

    lying_delta = abs(lying_est_level - bl) / bl
    getup_delta = abs(getup_est_level - bl) / bl

    # occupancy 阈值候选
    occ_enter_list = [lying_delta * f for f in [0.5, 0.6, 0.7, 0.8]]
    occ_exit_list = [lying_delta * f for f in [0.2, 0.3, 0.4]]
    # 确保 enter > exit
    occ_enter_list = [max(v, 0.05) for v in occ_enter_list]
    occ_exit_list = [max(v, 0.02) for v in occ_exit_list]

    change_th_list = [0.01, 0.02, 0.03, 0.05]

    results = []
    total = (len(alpha_slow_list) * len(alpha_fast_list) *
             len(enter_stable_list) * len(exit_stable_list) *
             len(timeout_list) * len(cooldown_list))

    print(f"  参数组合总数: {total}")
    print(f"  lying_delta估计: {lying_delta:.4f}")
    print(f"  occ_enter范围: [{occ_enter_list[0]:.4f}, {occ_enter_list[-1]:.4f}]")
    print(f"  occ_exit范围: [{occ_exit_list[0]:.4f}, {occ_exit_list[-1]:.4f}]")

    count = 0
    for a_slow in alpha_slow_list:
        for a_fast in alpha_fast_list:
            for enter_cnt in enter_stable_list:
                for exit_cnt in exit_stable_list:
                    for timeout in timeout_list:
                        for cooldown in cooldown_list:
                            for occ_enter in occ_enter_list:
                                for occ_exit in occ_exit_list:
                                    if occ_enter <= occ_exit:
                                        continue
                                    for change_th in change_th_list:
                                        for cd in cusum_delta_list:
                                            for ch in cusum_h_list:
                                                count += 1
                                                if count % 5000 == 0:
                                                    print(f"  已扫描 {count} 组...")

                                                p = BedParams(
                                                    alpha_slow=a_slow,
                                                    alpha_fast=a_fast,
                                                    occ_enter_th=occ_enter,
                                                    occ_exit_th=occ_exit,
                                                    change_th=change_th,
                                                    cusum_delta=cd * empty_std,
                                                    cusum_h=ch * empty_std,
                                                    enter_stable_count=enter_cnt,
                                                    exit_stable_count=exit_cnt,
                                                    candidate_timeout=timeout,
                                                    cooldown_count=cooldown,
                                                )

                                                sim_e = simulate_state_machine(
                                                    ma_empty, amp_empty, baseline, p, pkt_rate)
                                                sim_l = simulate_state_machine(
                                                    ma_lying, amp_lying, baseline, p, pkt_rate)
                                                sim_g = simulate_state_machine(
                                                    ma_getup, amp_getup, baseline, p, pkt_rate)

                                                sc = score_result(sim_e, sim_l, sim_g)

                                                if sc > 0:
                                                    results.append({
                                                        'score': sc,
                                                        'params': p,
                                                        'sim_empty': sim_e,
                                                        'sim_lying': sim_l,
                                                        'sim_getup': sim_g,
                                                    })

    results.sort(key=lambda x: x['score'], reverse=True)
    print(f"\n  总扫描: {count} 组, 有效: {len(results)} 组")
    return results


# ============================================================
# 快速扫描（减少参数空间）
# ============================================================

def scan_parameters_fast(ma_empty, amp_empty, ma_lying, amp_lying,
                         ma_getup, amp_getup, baseline, pkt_rate=120.0):
    """精简参数空间的快速扫描版本"""
    empty_std = baseline['empty_std']
    bl = baseline['baseline_mean']

    lying_est = np.mean(ma_lying[len(ma_lying)*2//3:])
    lying_delta = abs(lying_est - bl) / bl

    alpha_slow_list = [0.005, 0.01, 0.02]
    alpha_fast_list = [0.05, 0.1]
    enter_list = [180, 240, 360]
    exit_list = [120, 180]
    timeout_list = [600, 960]
    cooldown_list = [180, 240]
    occ_enter_list = [lying_delta * f for f in [0.5, 0.7]]
    occ_exit_list = [lying_delta * f for f in [0.2, 0.35]]
    change_list = [0.02, 0.03]
    cd_list = [0.5, 1.0, 1.5]
    ch_list = [8, 10, 15]

    results = []
    count = 0

    for a_slow in alpha_slow_list:
        for a_fast in alpha_fast_list:
            for enter_cnt in enter_list:
                for exit_cnt in exit_list:
                    for timeout in timeout_list:
                        for cooldown in cooldown_list:
                            for occ_enter in occ_enter_list:
                                for occ_exit in occ_exit_list:
                                    if occ_enter <= occ_exit:
                                        continue
                                    for ct in change_list:
                                        for cd in cd_list:
                                            for ch in ch_list:
                                                count += 1
                                                p = BedParams(
                                                    alpha_slow=a_slow,
                                                    alpha_fast=a_fast,
                                                    occ_enter_th=occ_enter,
                                                    occ_exit_th=occ_exit,
                                                    change_th=ct,
                                                    cusum_delta=cd * empty_std,
                                                    cusum_h=ch * empty_std,
                                                    enter_stable_count=enter_cnt,
                                                    exit_stable_count=exit_cnt,
                                                    candidate_timeout=timeout,
                                                    cooldown_count=cooldown,
                                                )
                                                sim_e = simulate_state_machine(
                                                    ma_empty, amp_empty, baseline, p, pkt_rate)
                                                sim_l = simulate_state_machine(
                                                    ma_lying, amp_lying, baseline, p, pkt_rate)
                                                sim_g = simulate_state_machine(
                                                    ma_getup, amp_getup, baseline, p, pkt_rate)
                                                sc = score_result(sim_e, sim_l, sim_g)
                                                if sc > 0:
                                                    results.append({
                                                        'score': sc,
                                                        'params': p,
                                                        'sim_empty': sim_e,
                                                        'sim_lying': sim_l,
                                                        'sim_getup': sim_g,
                                                    })

    results.sort(key=lambda x: x['score'], reverse=True)
    print(f"  快速扫描: {count} 组, 有效: {len(results)} 组")
    return results


# ============================================================
# 可视化
# ============================================================

def generate_plots(empty_data, lying_data, getup_data,
                   best_result, baseline, output_dir):
    """生成 3x2 可视化图"""
    fig, axes = plt.subplots(3, 2, figsize=(18, 16))
    fig.suptitle('CSI Bed Detection v4 - State Machine Simulation', fontsize=16)

    ts_e, ma_e, _ = empty_data
    ts_l, ma_l, _ = lying_data
    ts_g, ma_g, _ = getup_data

    t_e = (ts_e - ts_e[0]) / 1e6
    t_l = (ts_l - ts_l[0]) / 1e6
    t_g = (ts_g - ts_g[0]) / 1e6

    p = best_result['params']
    sim_e = best_result['sim_empty']
    sim_l = best_result['sim_lying']
    sim_g = best_result['sim_getup']
    bl = baseline['baseline_mean']

    # 颜色映射: 状态 → 颜色
    state_colors = {0: 'gray', 1: 'green', 2: 'orange',
                    3: 'red', 4: 'yellow', 5: 'lightblue'}

    def plot_state_bg(ax, t, state_log):
        for s_val, color in state_colors.items():
            mask = state_log == s_val
            if np.any(mask):
                ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1],
                                where=mask, alpha=0.1, color=color, label=STATE_NAMES[s_val])

    # --- 图1: 躺下 - 振幅 + 状态背景 ---
    ax = axes[0, 0]
    ax.plot(t_l, ma_l, alpha=0.3, color='blue', linewidth=0.5)
    ax.axhline(bl, color='green', linestyle='--', label=f'Baseline={bl:.1f}')
    for idx, typ in sim_l.confirmed_events:
        ax.axvline(t_l[idx], color='red', linestyle='--', linewidth=2)
        ax.annotate(typ, (t_l[idx], np.max(ma_l)*0.9), fontsize=8, fontweight='bold')
    ax.set_title('Lying Down: Amplitude + State')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Mean Amplitude')
    ax.grid(True, alpha=0.3)

    # --- 图2: 起床 - 振幅 + 状态背景 ---
    ax = axes[0, 1]
    ax.plot(t_g, ma_g, alpha=0.3, color='blue', linewidth=0.5)
    ax.axhline(bl, color='green', linestyle='--', label=f'Baseline={bl:.1f}')
    for idx, typ in sim_g.confirmed_events:
        ax.axvline(t_g[idx], color='red', linestyle='--', linewidth=2)
        ax.annotate(typ, (t_g[idx], np.max(ma_g)*0.9), fontsize=8, fontweight='bold')
    ax.set_title('Getting Up: Amplitude + State')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Mean Amplitude')
    ax.grid(True, alpha=0.3)

    # --- 图3: 三场景 occupancy_score ---
    ax = axes[1, 0]
    ax.plot(t_e, sim_e.occ_log, color='green', linewidth=1, alpha=0.7, label='Empty')
    ax.plot(t_l, sim_l.occ_log, color='red', linewidth=1, alpha=0.7, label='Lying')
    ax.plot(t_g, sim_g.occ_log, color='blue', linewidth=1, alpha=0.7, label='Getup')
    ax.axhline(p.occ_enter_th, color='red', linestyle=':', linewidth=1.5,
               label=f'Enter={p.occ_enter_th:.3f}')
    ax.axhline(p.occ_exit_th, color='orange', linestyle=':', linewidth=1.5,
               label=f'Exit={p.occ_exit_th:.3f}')
    ax.set_title('Occupancy Score (direction-independent)')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Score')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # --- 图4: 躺下 CUSUM ---
    ax = axes[1, 1]
    ax.plot(t_l, sim_l.cusum_lying_log, color='red', linewidth=1.5, label='CUSUM lying')
    ax.plot(t_l, sim_l.cusum_empty_log, color='blue', linewidth=1.5, label='CUSUM empty')
    ax.axhline(p.cusum_h, color='black', linestyle='--', linewidth=1.5,
               label=f'H={p.cusum_h:.1f}')
    ax.set_title('Lying Down: CUSUM')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('CUSUM Value')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # --- 图5: 躺下 - 状态机 + 计数器 ---
    ax = axes[2, 0]
    ax.plot(t_l, sim_l.state_log, color='purple', linewidth=1.5, label='State')
    ax.plot(t_l, sim_l.enter_counter_log / max(p.enter_stable_count, 1),
            color='orange', linewidth=1, alpha=0.7,
            label=f'Enter cnt / {p.enter_stable_count}')
    ax.plot(t_l, sim_l.exit_counter_log / max(p.exit_stable_count, 1),
            color='cyan', linewidth=1, alpha=0.7,
            label=f'Exit cnt / {p.exit_stable_count}')
    ax.set_yticks(list(STATE_NAMES.keys()))
    ax.set_yticklabels([STATE_NAMES[k] for k in sorted(STATE_NAMES.keys())])
    ax.set_title('Lying Down: State Machine')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('State')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # --- 图6: 起床 - 状态机 + 计数器 ---
    ax = axes[2, 1]
    ax.plot(t_g, sim_g.state_log, color='purple', linewidth=1.5, label='State')
    ax.plot(t_g, sim_g.enter_counter_log / max(p.enter_stable_count, 1),
            color='orange', linewidth=1, alpha=0.7, label='Enter cnt')
    ax.plot(t_g, sim_g.exit_counter_log / max(p.exit_stable_count, 1),
            color='cyan', linewidth=1, alpha=0.7, label='Exit cnt')
    ax.set_yticks(list(STATE_NAMES.keys()))
    ax.set_yticklabels([STATE_NAMES[k] for k in sorted(STATE_NAMES.keys())])
    ax.set_title('Getting Up: State Machine')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('State')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(output_dir, "bed_analysis.png")
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"图表已保存: {path}")
    return path


# ============================================================
# 分析报告
# ============================================================

def generate_report(results, baseline, ma_empty, ma_lying, ma_getup,
                    files, output_dir):
    """生成参数文件 + 分析报告"""
    if not results:
        print("无有效结果，跳过报告生成")
        return

    best = results[0]
    p = best['params']
    sim_e = best['sim_empty']
    sim_l = best['sim_lying']
    sim_g = best['sim_getup']
    bl = baseline['baseline_mean']

    # direction
    lying_avg = np.mean(ma_lying[len(ma_lying)*2//3:])
    direction = -1 if lying_avg < bl else 1

    # occupancy_score 分离度分析
    bl_sub = baseline['baseline_sub']

    report_path = os.path.join(output_dir, "bed_analysis_report.txt")
    params_h_path = os.path.join(output_dir, "bed_params.h")

    with open(report_path, 'w', encoding='utf-8') as f:
        f.write("CSI 床边检测分析报告 (v4)\n")
        f.write("=" * 60 + "\n\n")

        f.write("数据来源:\n")
        for label, path in files.items():
            f.write(f"  {label}: {path}\n")
        f.write("\n")

        f.write("Baseline 统计:\n")
        f.write(f"  baseline_mean = {bl:.4f}\n")
        f.write(f"  empty_std     = {baseline['empty_std']:.4f}\n")
        f.write(f"  direction     = {direction}\n")
        f.write(f"  lying_avg     = {lying_avg:.4f}\n")
        f.write(f"  lying_delta   = {abs(lying_avg - bl) / bl:.4f}\n\n")

        f.write("检测性能:\n")
        f.write(f"  空床误报:     {len(sim_e.confirmed_events)}\n")
        f.write(f"  躺下确认:     {sim_l.lying_confirm_count} 次\n")
        f.write(f"  起床确认:     {sim_g.getup_confirm_count} 次\n")
        f.write(f"  躺下延迟:     {sim_l.lying_delay:.2f}s\n")
        f.write(f"  起床延迟:     {sim_g.getup_delay:.2f}s\n")
        f.write(f"  躺下MAYBE触发: {sim_l.maybe_lying_count}\n")
        f.write(f"  起床MAYBE触发: {sim_g.maybe_empty_count}\n")
        f.write(f"  状态跳变:     lying={sim_l.state_flip_count}, "
                f"getup={sim_g.state_flip_count}\n\n")

        # 警告
        f.write("风险评估:\n")
        if len(sim_e.confirmed_events) > 0:
            f.write("  WARNING 1: 空床有误报!\n")
        if abs(lying_avg - bl) / bl < 0.1:
            f.write("  WARNING 2: lying_delta < 10%，分离度弱\n")
        if sim_l.maybe_lying_count > 3:
            f.write("  WARNING 3: MAYBE_LYING 触发频繁，环境扰动大\n")
        f.write("\n")

        f.write("最优参数 (Top 5):\n")
        f.write("-" * 60 + "\n")
        for i, r in enumerate(results[:5]):
            pp = r['params']
            f.write(f"\n  Rank {i+1}: score={r['score']}\n")
            f.write(f"    alpha_slow       = {pp.alpha_slow}\n")
            f.write(f"    alpha_fast       = {pp.alpha_fast}\n")
            f.write(f"    occ_enter_th     = {pp.occ_enter_th:.4f}\n")
            f.write(f"    occ_exit_th      = {pp.occ_exit_th:.4f}\n")
            f.write(f"    change_th        = {pp.change_th:.4f}\n")
            f.write(f"    cusum_delta      = {pp.cusum_delta:.4f}\n")
            f.write(f"    cusum_h          = {pp.cusum_h:.1f}\n")
            f.write(f"    enter_stable     = {pp.enter_stable_count}\n")
            f.write(f"    exit_stable      = {pp.exit_stable_count}\n")
            f.write(f"    timeout          = {pp.candidate_timeout}\n")
            f.write(f"    cooldown         = {pp.cooldown_count}\n")
            se = r['sim_empty']
            sl = r['sim_lying']
            sg = r['sim_getup']
            f.write(f"    empty_FA={len(se.confirmed_events)}, "
                    f"lying={sl.lying_confirm_count}({sl.lying_delay:.1f}s), "
                    f"getup={sg.getup_confirm_count}({sg.getup_delay:.1f}s)\n")

    # 生成 C 头文件
    with open(params_h_path, 'w', encoding='utf-8') as f:
        f.write("/* CSI 床边检测参数 (v4) - 由 bed_analyze.py 自动生成 */\n")
        f.write("/* 不要手动修改，重新运行分析脚本覆盖 */\n\n")
        f.write("#pragma once\n\n")
        f.write(f"#define BED_DIRECTION              {direction}\n")
        f.write(f"#define BED_INIT_SAMPLES           500\n\n")
        f.write(f"#define BED_EMA_ALPHA_SLOW         {p.alpha_slow:.4f}f\n")
        f.write(f"#define BED_EMA_ALPHA_FAST         {p.alpha_fast:.4f}f\n")
        f.write(f"#define BED_BASELINE_ALPHA         0.0005f\n\n")
        f.write(f"#define BED_OCC_ENTER_TH           {p.occ_enter_th:.4f}f\n")
        f.write(f"#define BED_OCC_EXIT_TH            {p.occ_exit_th:.4f}f\n\n")
        f.write(f"#define BED_CHANGE_TH              {p.change_th:.4f}f\n\n")
        f.write(f"#define BED_MOTION_HIGH_TH         12.0f\n")
        f.write(f"#define BED_MOTION_QUIET_TH        5.0f\n\n")
        f.write(f"#define BED_DEV_ENTER_TH           0.080f\n\n")
        f.write(f"#define BED_CUSUM_DELTA            {p.cusum_delta:.4f}f\n")
        f.write(f"#define BED_CUSUM_H                {p.cusum_h:.1f}f\n\n")
        f.write(f"#define BED_ENTER_STABLE_COUNT     {p.enter_stable_count}\n")
        f.write(f"#define BED_EXIT_STABLE_COUNT      {p.exit_stable_count}\n")
        f.write(f"#define BED_CANDIDATE_TIMEOUT      {p.candidate_timeout}\n")
        f.write(f"#define BED_COOLDOWN_COUNT         {p.cooldown_count}\n\n")
        f.write(f"#define BED_MIN_VALID_RATE         80\n")

    print(f"报告已保存: {report_path}")
    print(f"参数头文件已保存: {params_h_path}")


# ============================================================
# 基础统计
# ============================================================

def analyze_scenario(name, timestamps, mean_amps):
    duration = (timestamps[-1] - timestamps[0]) / 1e6
    return {
        'name': name, 'n': len(timestamps), 'dur': duration,
        'rate': len(timestamps) / duration,
        'avg': np.mean(mean_amps), 'std': np.std(mean_amps),
        'min': np.min(mean_amps), 'max': np.max(mean_amps),
    }


# ============================================================
# 主函数
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="CSI 床边数据分析 (v4 - 完整状态机)")
    parser.add_argument("--data-dir", default="data")
    parser.add_argument("--output-dir", default="analysis")
    parser.add_argument("--fast", action="store_true",
                        help="快速扫描模式（参数空间精简）")
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(base_dir, args.data_dir)
    output_dir = os.path.join(base_dir, args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    files = {}
    for label in ["empty", "lying", "getup"]:
        path = os.path.join(data_dir, f"bed_{label}.csv")
        if not os.path.exists(path):
            print(f"错误: 找不到 {path}")
            print(f"请先采集: python tools/bed_collect.py --port COM28 "
                  f"--label {label}")
            sys.exit(1)
        files[label] = path

    print("=" * 60)
    print("CSI 床边数据分析 (v4 - 完整状态机 + 多特征融合)")
    print("=" * 60)

    # [1] 加载
    print("\n[1/5] 加载数据...")
    empty_data = load_csv(files['empty'])
    lying_data = load_csv(files['lying'])
    getup_data = load_csv(files['getup'])
    ts_e, ma_e, amp_e = empty_data
    ts_l, ma_l, amp_l = lying_data
    ts_g, ma_g, amp_g = getup_data
    pkt_rate = len(ts_e) / ((ts_e[-1] - ts_e[0]) / 1e6)
    print(f"  空床: {len(ts_e)} 包, {amp_e.shape[1]} 子载波, {pkt_rate:.0f} pkt/s")
    print(f"  躺下: {len(ts_l)} 包")
    print(f"  起床: {len(ts_g)} 包")

    # [2] Baseline
    print("\n[2/5] 估算 baseline...")
    baseline = estimate_baseline(ma_e, amp_e)
    lying_avg = np.mean(ma_l[len(ma_l)*2//3:])
    direction = -1 if lying_avg < baseline['baseline_mean'] else 1
    lying_delta = abs(lying_avg - baseline['baseline_mean']) / baseline['baseline_mean']
    print(f"  baseline_mean = {baseline['baseline_mean']:.4f}")
    print(f"  empty_std     = {baseline['empty_std']:.4f}")
    print(f"  direction     = {direction} ({'躺下振幅下降' if direction == -1 else '躺下振幅上升'})")
    print(f"  lying_delta   = {lying_delta:.4f} ({lying_delta*100:.1f}%)")

    if lying_delta < 0.05:
        print("\n  WARNING: lying_delta < 5%，分离度极弱，检测可能不可靠")

    # [3] 场景统计
    print("\n[3/5] 场景统计:")
    for name, (ts, ma) in [("空床", (ts_e, ma_e)),
                             ("躺下", (ts_l, ma_l)),
                             ("起床", (ts_g, ma_g))]:
        s = analyze_scenario(name, ts, ma)
        print(f"  [{name}] avg={s['avg']:.2f}, std={s['std']:.2f}, "
              f"range=[{s['min']:.2f}, {s['max']:.2f}]")

    # [4] 参数扫描
    print(f"\n[4/5] 参数扫描 ({'快速' if args.fast else '完整'}模式)...")
    scan_fn = scan_parameters_fast if args.fast else scan_parameters
    results = scan_fn(ma_e, amp_e, ma_l, amp_l, ma_g, amp_g, baseline, pkt_rate)

    if not results:
        print("  未找到有效参数组合。")
        print("  可能原因: 数据分离度不足或动作不够明显。")
        sys.exit(1)

    best = results[0]
    p = best['params']

    print(f"\n  最优参数 (score={best['score']}):")
    print(f"    alpha_slow   = {p.alpha_slow}")
    print(f"    alpha_fast   = {p.alpha_fast}")
    print(f"    occ_enter_th = {p.occ_enter_th:.4f}")
    print(f"    occ_exit_th  = {p.occ_exit_th:.4f}")
    print(f"    cusum_delta  = {p.cusum_delta:.4f}")
    print(f"    cusum_h      = {p.cusum_h:.1f}")
    print(f"    enter_stable = {p.enter_stable_count}")
    print(f"    exit_stable  = {p.exit_stable_count}")
    print(f"    timeout      = {p.candidate_timeout}")
    print(f"    cooldown     = {p.cooldown_count}")
    print(f"    空床FA={len(best['sim_empty'].confirmed_events)}, "
          f"躺下={best['sim_lying'].lying_confirm_count}, "
          f"起床={best['sim_getup'].getup_confirm_count}")

    # [5] 生成输出
    print(f"\n[5/5] 生成图表和报告...")
    generate_plots(empty_data, lying_data, getup_data, best, baseline, output_dir)
    generate_report(results, baseline, ma_e, ma_l, ma_g, files, output_dir)

    # 最终输出
    print(f"\n{'='*60}")
    print("完成! 输出文件:")
    print(f"  {output_dir}/bed_analysis.png")
    print(f"  {output_dir}/bed_analysis_report.txt")
    print(f"  {output_dir}/bed_params.h  (ESP32 固件参数)")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
