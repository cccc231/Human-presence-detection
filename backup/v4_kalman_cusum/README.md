# v4 - 床边检测 (Kalman + CUSUM + 完整状态机)

## 概述
基于 v3 的 CSI 数据流，新增床边起床/躺下实时检测。融合 EMA 滞回阈值（主确认）+ CUSUM 快速变点检测（候选触发）+ 6 状态状态机（防抖/冷却/超时）。

## 架构
```
Core 0                              Core 1
┌──────────────────────┐           ┌──────────────────────┐
│ csi_processor_task   │           │ bed_detector_task    │
│                      │           │                      │
│ 从 csi_raw_queue     │           │ 从 bed_snapshot_queue│
│ 取原始 I/Q 数据       │           │ 取振幅数据            │
│       │              │           │       │              │
│       ├→ printf 串口  │           │  slow/fast EMA       │
│       │  (生命体征)    │           │  occupancy_score     │
│       │              │           │  CUSUM 变点检测       │
│       └→ 计算振幅     │           │  6 状态状态机         │
│          sqrt(I²+Q²)  │           │       │              │
│              │        │           │  ESP_LOGI 输出事件   │
│              ▼        │           └──────────────────────┘
│       bed_snapshot    │
│       _queue ─────────┼──→ Core 1
└──────────────────────┘
```

## 状态机
```
INIT_CALIBRATING → EMPTY ⇄ MAYBE_LYING → LYING ⇄ MAYBE_EMPTY → COOLDOWN
```

- **CUSUM 触发** → 进入 MAYBE 状态（候选）
- **EMA 稳定确认** → 进入正式状态
- **超时/拒绝** → 回退
- **冷却** → 防重复触发

## 特征层（每包实时计算）

| 特征 | 计算方式 | 作用 |
|------|---------|------|
| slow_ema | α_slow × amp + (1-α) × prev | 判断稳定状态 |
| fast_ema | α_fast × amp + (1-α) × prev | 捕捉短期变化 |
| occupancy_score | direction × (slow_ema - baseline) / baseline | 方向无关的"有人程度" |
| change_score | direction × (fast_ema - slow_ema) / baseline | 快慢EMA差异→正在动作 |
| motion_score | 最近100包方差 | 动作强度 |
| innovation | amp - slow_ema | Kalman新息近似 |
| CUSUM | 累积 signed_innovation | 持续偏移检测 |

`direction` 由系统自动确定：首次确认躺下时比较 slow_ema 和 baseline，下降则 -1，上升则 +1。

## 文件清单

| 文件 | 说明 |
|------|------|
| `rx/main/bed_detector.h` | 状态定义、快照结构、API |
| `rx/main/bed_detector.c` | 6 状态状态机实现 |
| `rx/main/csi_processor.c` | CSI 处理 + 振幅计算（增加 bed_snapshot_queue） |
| `rx/main/csi_processor.h` | CSI 数据结构 |
| `rx/main/rx_main.c` | RX 入口（增加 bed_detector_init） |
| `rx/main/CMakeLists.txt` | 构建配置 |
| `tx/main/tx_main.c` | TX 固件（120Hz） |
| `tools/bed_analyze.py` | 离线分析：完整状态机模拟 + 参数扫描评分 + bed_params.h 生成 |
| `tools/bed_collect.py` | 数据采集脚本 |

## 数据采集与分析流程

```bash
# 1. 采集三组数据
python tools/bed_collect.py --port COM28 --label empty
python tools/bed_collect.py --port COM28 --label lying
python tools/bed_collect.py --port COM28 --label getup

# 2. 离线分析，扫描最优参数
python tools/bed_analyze.py          # 完整扫描
python tools/bed_analyze.py --fast   # 快速扫描

# 3. 输出文件
#   analysis/bed_analysis.png        可视化图表
#   analysis/bed_analysis_report.txt 分析报告
#   analysis/bed_params.h            ESP32 固件参数（C宏定义）
```

## 分析脚本评分标准

```
+1000   空床 0 误报
-10000  每次空床误报（直接淘汰）
+500    躺下确认恰好 1 次
+500    起床确认恰好 1 次
-延迟×20
-误触发×5
-状态跳变×100
```

## 默认参数（仅供开发调试）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| EMA_ALPHA_SLOW | 0.01 | 慢速 EMA |
| EMA_ALPHA_FAST | 0.1 | 快速 EMA |
| OCC_ENTER_TH | 0.18 | 进入有人阈值 |
| OCC_EXIT_TH | 0.10 | 退出有人阈值 |
| CUSUM_DELTA | 1.5 | CUSUM 最小偏移 |
| CUSUM_H | 12.0 | CUSUM 判定阈值 |
| ENTER_STABLE_COUNT | 240 | 躺下确认包数 |
| EXIT_STABLE_COUNT | 180 | 起床确认包数 |
| CANDIDATE_TIMEOUT | 720 | 候选超时包数 |
| COOLDOWN_COUNT | 240 | 冷却包数 |
| INIT_SAMPLES | 500 | 初始化采样数 |

> 实际参数由 bed_analyze.py 扫描确定，输出到 bed_params.h。

## 备份时间
2026-05-07
