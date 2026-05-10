# v4 - 床边检测 (Kalman + CUSUM + 完整状态机)

## 概述
基于 v3 的 CSI 数据流，新增床边起床/躺下实时检测。融合 EMA 滞回阈值（主确认）+ CUSUM 快速变点检测（候选触发）+ 6 状态状态机（防抖/冷却/超时）。

## 架构
```
CSI 包 → csi_processor (Core 0)
           ├→ printf 原始 I/Q（串口输出）
           └→ 计算振幅 → bed_snapshot_queue → bed_detector (Core 1)
                                                ├→ slow/fast EMA
                                                ├→ occupancy_score
                                                ├→ CUSUM 变点检测
                                                ├→ 6 状态状态机
                                                └→ ESP_LOGI 输出事件
```

## 状态机
```
INIT_CALIBRATING → EMPTY ⇄ MAYBE_LYING → LYING ⇄ MAYBE_EMPTY → COOLDOWN
```
- CUSUM 触发 → 进入 MAYBE 状态（候选）
- EMA 稳定确认 → 进入正式状态
- 超时/拒绝 → 回退
- 冷却 → 防重复触发

## 文件清单

| 文件 | 说明 |
|------|------|
| `rx/main/bed_detector.h` | 状态定义、数据结构、API |
| `rx/main/bed_detector.c` | 6 状态状态机实现 |
| `rx/main/csi_processor.c` | CSI 处理 + 振幅计算（增加 bed_snapshot_queue） |
| `rx/main/rx_main.c` | RX 入口（增加 bed_detector_init） |
| `rx/main/CMakeLists.txt` | 构建配置 |
| `tools/bed_analyze.py` | 离线分析：状态机模拟 + 参数扫描评分 |
| `tools/bed_collect.py` | 数据采集脚本 |
| `tx/main/tx_main.c` | TX 固件（120Hz） |

## 参数来源
参数由 `bed_analyze.py` 离线扫描确定，自动生成 `bed_params.h`。默认参数仅供开发调试。

## 使用方法

### 采集数据
```bash
python tools/bed_collect.py --port COM28 --label empty
python tools/bed_collect.py --port COM28 --label lying
python tools/bed_collect.py --port COM28 --label getup
```

### 分析参数
```bash
python tools/bed_analyze.py          # 完整扫描
python tools/bed_analyze.py --fast   # 快速扫描
```

### 编译烧录
```bash
cd rx && idf.py build && idf.py -p COM28 flash
```

## 备份时间
2026-05-07
