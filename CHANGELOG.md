# 操作记录 (CHANGELOG)

## 2026-05-06 - v2: 双指标检测 + 1秒聚合

### 背景
v1（固定阈值1.5 + 逐包防抖）测试发现：人静坐不动时系统输出"没人"，只有姿态变化时才输出"有人"。原因是方差指标只衡量"变化量"，静止时方差和空房间一样低。

### 改动内容

#### 1. presence_detector.h
- 新增 `INIT_SAMPLES = 500`：初始化阶段采集空房间振幅基准（约4秒）
- 新增 `AGG_WINDOW_MS = 1000`：1秒聚合窗口
- 新增 `PRESENCE_RATIO = 0.4f`：有人比例阈值（40%）
- 移除 `DEBOUNCE_COUNT`（被1秒聚合替代）
- 保留 `THRESHOLD_HIGH = 1.5`、`THRESHOLD_LOW = 1.2`、`WINDOW_SIZE = 100`

#### 2. presence_detector.c - 核心算法重写
- **初始化阶段**：采集500包空房间数据，计算每个子载波的平均振幅作为 `baseline[]`
- **新增 `compute_amp_dev_metric()`**：计算当前窗口振幅与baseline的偏差（`mean|current[k] - baseline[k]|`），用于检测静止人体
- **双指标**：最终 metric = max(variance_metric, amp_dev_metric)
- **1秒聚合**：每包计算原始判定并存入缓冲区，每秒统计"有人"比例，≥40%输出"有人"
- **baseline自适应**："没人"时以0.1%/包速率缓慢更新，"有人"时冻结
- 输出格式：`HH:MM:SS, 有人, 2.1477, 45/80`（新增 有人次数/总包数）

#### 3. serial_logger.py
- 正则更新：匹配新增的 `45/80` 计数字段
- CSV 新增 `presence_count` 列
- 控制台显示比例信息 `[45/80]`

### 使用注意事项
- **必须在空房间上电启动**，等初始化完成（~4秒）+ 预热（~1.25秒）后再进入
- CSV 时间戳由 Python 脚本使用 PC 本地时间（北京时间），不依赖 ESP32 的 NTP

### 未改动文件
- `rx/main/rx_main.c`：保持原样（已恢复为无热点连接的简洁版）
- `rx/main/csi_processor.c/h`：无变化
- `rx/main/time_utils.c/h`：无变化
- `tx/main/tx_main.c`：无变化

---

## 2026-05-06 - v1: 固定阈值（已备份到 backup/v1_fixed_threshold/）

### 改动内容
- 移除自适应校准逻辑（baseline、calibration_sum、CALIBRATION_SAMPLES）
- 改为固定阈值：`THRESHOLD_HIGH = 1.5`，`THRESHOLD_LOW = 1.2`
- 依据实测数据：空房间稳定段 metric ≈ 1.1~1.3，有人稳定段 ≈ 1.8~2.0

### 备份文件
`backup/v1_fixed_threshold/` 包含所有 rx/main/ 和 tx/main/ 的源文件

---

## 2026-05-06 - 初始版本：ESP-NOW 重写 + 数据采集

### 从 softAP 信标帧模式改为 ESP-NOW 广播
- TX：80Hz ESP-NOW 广播（原 softAP 仅 0.6Hz）
- RX：ESP-NOW 接收 + CSI 提取 + 滑动窗口方差检测
- 协议：802.11n HT20

### 数据采集
- `empty.csv`：空房间数据（ESP-NOW版）
- `person.csv`：有人数据（ESP-NOW版，含 SYNCING 和 NTP 校准后时段）

### 解决的问题
| 问题 | 解决方案 |
|------|---------|
| AP 信标帧发包率太低（0.6Hz） | 改用 ESP-NOW，80Hz |
| 方差阈值 0.035 完全错误 | 实际 metric 在 1.0~7.0，改为固定阈值 1.5 |
| CSV 中文在 Excel 乱码 | 改用 utf-8-sig 编码（带 BOM） |
| VS Code IntelliSense 红色报错 | 配置 compileCommands，config 名改为 "Win32" |
| Python regex 不匹配 SYNCING 格式 | 更新正则支持 `SYNCING_\d+s` |
| idf.py 在 PowerShell 找不到 | export.bat 是 cmd 脚本，需用 cmd.exe |

---

## 硬件信息
- ESP-IDF v5.5.1：`D:\esp32\v5.5.1\esp-idf`
- ESP-IDF tools：`D:\esp32\.espressif`
- TX 串口：未记录
- RX 串口：COM28
