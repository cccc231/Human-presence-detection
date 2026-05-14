# v5 - 体位变化检测 (per-subcarrier STD 方法)

## 概述
基于 per-subcarrier STD（逐子载波标准差）滑动窗口检测躺下/坐起动作。无需 DWT、PCA、SampEn 等复杂信号处理，仅依赖空床基线子载波 STD 作为参照。

## 核心原理
```
躺下: 身体进入信号路径 → 大量子载波波动增强 → per-subcarrier STD 大幅升高
起床: 身体离开信号路径 → 子载波波动变化小 → per-subcarrier STD 接近空床水平
```

## 算法
```
CSI 原始数据 (120Hz 非均匀)
    ↓ 重采样到 40Hz
滑动窗口(5秒, 步长1秒) per-subcarrier STD
    ↓
每个子载波的 STD / 空床基线 STD = 比值
    ↓
>40% 子载波 > 1.5x 基线 → lying (躺下)
否则 ratio_mean > 1.1      → sitting (坐起)
否则                        → quiet (空床)
```

## 文件清单

| 文件 | 说明 |
|------|------|
| `tools/bed_posture_detect.py` | **核心**：per-subcarrier STD 检测 + 参数扫描 |
| `tools/bed_collect.py` | 数据采集脚本 |
| `tools/bed_analyze.py` | EMA + 阈值分析（v3 版本） |
| `tools/vital_signs.py` | 生命体征检测（v3 版本） |
| `tools/serial_logger.py` | 存在检测日志（v2 版本） |
| `rx/main/csi_processor.c` | CSI 原始 I/Q 输出 |
| `rx/main/csi_processor.h` | CSI 数据结构 |
| `rx/main/rx_main.c` | RX 主入口 |
| `rx/main/CMakeLists.txt` | 构建配置 |
| `tx/main/tx_main.c` | TX 固件（120Hz） |

## 依赖
```
pip install numpy scipy matplotlib pyserial
```

## 使用方法

```bash
# 1. 采集数据
python tools/bed_collect.py --port COM28 --label empty
python tools/bed_collect.py --port COM28 --label lying
python tools/bed_collect.py --port COM28 --label getup

# 2. 参数扫描
python tools/bed_posture_detect.py --scan

# 3. 默认参数分析
python tools/bed_posture_detect.py
```

## 验证结果（data/ 目录下数据）

| 场景 | 期望 | 实际 | 结果 |
|------|------|------|------|
| 空床 | 0 事件 | 0 | ✓ |
| 躺下 | 1 lying | 1 lying [5-19s] | ✓ |
| 起床 | 1 sitting | 1 sitting [10-17s] | ✓ |

## 相比 v4 的改动

| | v4 | v5 |
|------|-----|-----|
| 检测器 | DWT+PCA+SampEn+CUSUM | **滑动窗口 per-subcarrier STD** |
| 方向判断 | mean_amp 方向 | per-subcarrier STD 比值 |
| 依赖 | pywt, sklearn, nolds | **仅 scipy, numpy** |
| 行数 | ~1000 | **~427** |
| getup 检出 | 0（失败） | **1（成功）** |

## 参数（默认值）

| 参数 | 值 | 说明 |
|------|-----|------|
| win | 200 | 窗口大小（@40Hz = 5秒） |
| step | 40 | 步长（@40Hz = 1秒） |
| ratio_th | 1.5 | lying 触发比值 |
| sub_frac | 0.4 | lying 所需高比值子载波比例 |
| sit_th | 1.1 | sitting 的 ratio_mean 下限 |

## 备份时间
2026-05-14
