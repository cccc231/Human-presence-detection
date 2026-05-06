# v3 - WiFi CSI 生命体征监测 (Vital Signs Monitoring)

## 概述

复现论文 IEEE JTEHM 2025 - Alzaabi et al. 的 WiFi CSI 生命体征检测算法，从 WiFi 子载波信号中提取呼吸率 (RR) 和心率 (HR)。

**核心原理**：人体呼吸（胸腔位移 4-12mm）和心跳（胸腔位移 0.2-0.5mm）会引起 WiFi 信号的微小多径变化，这些变化反映在 CSI 子载波的 I/Q 数据中。通过小波变换 + PCA + 熵分析从 52 个子载波中分离出生命体征信号。

---

## 系统架构

```
┌──────────┐   ESP-NOW    ┌──────────┐   USB 串口    ┌──────────────────┐
│ ESP32 TX │ ──────────> │ ESP32 RX │ ──────────> │ PC (Python)      │
│ 120 Hz   │  2.4GHz HT20 │ 原始I/Q  │  115200 baud │ vital_signs.py   │
│ 广播发送  │              │ 串口输出  │              │ 信号处理流水线    │
└──────────┘              └──────────┘              └──────────────────┘
```

### ESP32 端
- **TX**：ESP-NOW 广播，120 包/秒，每包含 8 字节（seq + timestamp）
- **RX**：接收 ESP-NOW 包触发 CSI 回调，提取 52 个子载波的 I/Q 原始数据，通过串口输出

### PC 端
- **vital_signs.py**：串口读取 → 信号处理 → 输出呼吸率/心率

---

## 信号处理流水线

```
CSI I/Q 原始数据 (120Hz, 52子载波)
        │
        ▼
振幅提取: amp = sqrt(I² + Q²)
        │
        ├──────────────────────────┐
        ▼ 呼吸率路径               ▼ 心率路径
插值重采样 → 40 Hz          插值重采样 → 60 Hz
        │                          │
DWT db4 小波 (level 4-6)   DWT db4 小波 (level 2-4)
  通带: 0.1-0.5 Hz           通带: 0.8-2.0 Hz
        │                          │
PCA 降维 (52 → 10主成分)    PCA 降维 (52 → 10主成分)
        │                          │
PC-SampEn 选择最规则主成分   PC-SampEn 选择最规则主成分
        │                          │
CWT Morlet 小波              CWT Morlet 小波
  28 个尺度                    28 个尺度
        │                          │
平均小波能量峰值检测          平均小波能量峰值检测
        │                          │
        ▼                          ▼
   呼吸率 (BrPM)              心率 (BPM)
```

### 各步骤说明

| 步骤 | 算法 | 作用 |
|------|------|------|
| 振幅提取 | `sqrt(I² + Q²)` | 将复数 CSI 转为实数振幅 |
| 插值重采样 | scipy `interp1d` (linear) | 非均匀时间戳 → 均匀采样 |
| DWT 滤波 | pywt `wavedec` db4 level=6 | 按频段分离呼吸/心跳信号 |
| PCA 降维 | sklearn `PCA(n=10)` | 52 子载波 → 10 个主成分 |
| PC-SampEn | nolds `sampen(m=3, r=0.1σ)` | 选最规则（最低熵）的主成分 |
| CWT 提取 | scipy `cwt(morlet2)` 28 scales | 频率分析 → 峰值 = RR/HR |

---

## 文件清单

### 备份目录 `backup/v3_vital_signs/`

```
backup/v3_vital_signs/
├── README.md                 ← 本文件
├── tx/
│   └── main/
│       └── tx_main.c         TX 固件（120Hz ESP-NOW 广播）
├── rx/
│   └── main/
│       ├── CMakeLists.txt     RX 构建配置
│       ├── rx_main.c          RX 主入口
│       ├── csi_processor.c    CSI 数据处理（原始 I/Q 输出）
│       └── csi_processor.h    CSI 数据结构定义
└── tools/
    └── vital_signs.py         PC 端生命体征信号处理脚本
```

### 各文件详细说明

#### `tx/main/tx_main.c`
- ESP-NOW 广播发送端
- `CONFIG_SEND_FREQUENCY = 120`：120 包/秒
- 发送 8 字节数据包：`{seq(uint32), tx_us(int64)}`
- WiFi: STA 模式, 2.4GHz, HT20, MCS0, 广播地址 ff:ff:ff:ff:ff:ff
- 主循环：`esp_now_send()` + `usleep(1000000/120)`

#### `rx/main/csi_processor.h`
- `MAX_CSI_LEN = 256`：CSI 缓冲区最大长度
- `MAX_SUBCARRIERS = 64`：最大子载波数
- `CSI_QUEUE_SIZE = 40`：FreeRTOS 队列深度（约 333ms 缓冲 @120Hz）
- `csi_raw_t` 结构体：`buf[256](I/Q) + len + rssi + timestamp + first_word_invalid`

#### `rx/main/csi_processor.c`
- `csi_rx_cb()`：WiFi CSI 回调，将原始数据入队（在 WiFi 任务上下文执行，不做计算）
- `csi_processor_task()`：Core 0 运行，从队列取数据，处理 `first_word_invalid` 偏移，输出原始 I/Q 到串口
- 输出格式：`CSI,<timestamp_us>,<rssi>,<num_sub>,<I0>,<Q0>,<I1>,<Q1>,...`

#### `rx/main/rx_main.c`
- `app_main()` 初始化顺序：NVS → CSI processor → WiFi STA → CSI config → ESP-NOW
- WiFi 配置：`csi_enable=1`, `WIFI_PS_NONE`(关闭省电), HT20, 2.4GHz only
- CSI 配置：`lltf_en=true`, `htltf_en=true`, `channel_filter_en=true`
- 不含 presence_detector（生命体征模式不需要存在检测）

#### `rx/main/CMakeLists.txt`
```cmake
idf_component_register(SRCS "rx_main.c" "csi_processor.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash esp_wifi esp_netif esp_event esp_timer)
```

#### `tools/vital_signs.py`
- 完整论文信号处理流水线的 Python 实现
- 主要函数：
  - `parse_csi_line()`：解析串口 CSI 数据行
  - `collect_data()`：从串口采集指定时长的数据
  - `resample_uniform()`：插值重采样到均匀时间间隔
  - `dwt_filter_bank()`：DWT db4 小波滤波（按 level 选择频段）
  - `apply_pca()`：PCA 降维
  - `compute_sampen()`：计算样本熵
  - `select_best_pc()`：选择最低 SampEn 的主成分
  - `extract_rate_cwt()`：CWT Morlet 小波提取频率
  - `process_vital_signs()`：完整处理流水线
- 输出 CSV：`timestamp, rr_bpm, hr_bpm, n_packets, duration_sec`

---

## 串口输出格式

RX 固件每收到一个 CSI 包输出一行：

```
CSI,1715001234567890,-45,52,10,-5,8,3,-2,7,1,...
│   │                  │   │  │  │  │ │ │ │ │
│   timestamp_us       │   │  I0 Q0 I1 Q1 ...
│   (微秒时间戳)        │   num_sub (子载波数)
│                      rssi (信号强度)
前缀 (标识 CSI 数据)
```

- **I/Q 顺序**：每对 `[real, imag]`，`int8_t` 范围 -128~127
- **子载波数**：通常 52（ESP32 HT20 LLTF 模式）
- **非 CSI 行**：ESP-IDF 日志信息（如 `I (1234) csi_rx: CSI initialized`），Python 脚本会跳过

---

## 使用方法

### 1. 环境准备

```bash
# ESP-IDF 环境
# Windows: D:\esp32\v5.5.1\esp-idf
# 每次打开新终端需先初始化:
D:\esp32\v5.5.1\esp-idf\export.bat

# Python 依赖
pip install numpy scipy PyWavelets scikit-learn nolds pyserial
```

### 2. 编译烧录 TX

```bash
cd D:\esp32c5\see\tx
idf.py build
idf.py -p COMxx flash    # 替换为 TX 串口号
```

### 3. 编译烧录 RX

```bash
cd D:\esp32c5\see\rx
idf.py build
idf.py -p COM28 flash    # 替换为 RX 串口号
```

### 4. 运行生命体征检测

```bash
cd D:\esp32c5\see
python tools/vital_signs.py --port COM28 --duration 120
```

### 测试条件

| 条件 | 要求 |
|------|------|
| 被测者姿势 | 静坐，面对天线 |
| 距离 | 1-3 米 |
| 采集时长 | 建议 120 秒（论文验证用） |
| 环境 | 尽量安静，减少人员走动 |
| 天线 | 使用 ESP32 板载 PCB 天线 |

### 命令行参数

```
vital_signs.py:
  --port PORT       串口号（如 COM28）[必填]
  --baud BAUD       波特率（默认 115200）
  --duration SEC    采集时长秒数（默认 120）
  --output FILE     输出 CSV 文件（默认 vital_signs.csv）
  --delay SEC       开始前等待秒数（默认 5）
  --interval SEC    分析间隔秒数（默认 30）
```

---

## 关键参数

### ESP32 固件参数

| 参数 | 值 | 文件 | 说明 |
|------|----|------|------|
| TX 发包率 | 120 PPS | `tx_main.c:23` | 论文设定，满足心率采样要求 |
| WiFi 协议 | 802.11n HT20 | `tx_main.c` / `rx_main.c` | 2.4GHz, 20MHz 带宽 |
| WiFi 信道 | 0 (自动) | `tx_main.c:16` | TX/RX 需在同一信道 |
| CSI 队列深度 | 40 | `csi_processor.h:10` | 约 333ms 缓冲 |
| 处理任务优先级 | 5 | `csi_processor.c:83` | FreeRTOS 任务优先级 |
| 处理任务栈 | 8192 | `csi_processor.c:83` | Core 0 运行 |
| LLTF | 启用 | `rx_main.c:59` | Legacy Long Training Field |
| HT-LTF | 启用 | `rx_main.c:60` | HT Long Training Field |
| channel_filter | 启用 | `rx_main.c:63` | 信道滤波 |

### Python 信号处理参数

| 参数 | 值 | 来源 | 说明 |
|------|----|------|------|
| DWT 母小波 | db4 | 论文 | Daubechies-4 小波 |
| DWT 分解层数 | 6 | 论文 | 6 层分解 |
| RR 重采样率 | 40 Hz | 论文 | 呼吸率分析 |
| HR 重采样率 | 60 Hz | 论文 | 心率分析 |
| RR 频段 | 0.1-0.5 Hz | 论文 | 6-30 BrPM, DWT level 4-6 |
| HR 频段 | 0.8-2.0 Hz | 论文 | 48-120 BPM, DWT level 2-4 |
| PCA 成分数 | 10 | 论文 | 前 10 个主成分 |
| SampEn 嵌入维度 | m=3 | 论文 | 样本熵参数 |
| SampEn 容差 | r=0.1×σ | 论文 | σ 为每个 PC 的标准差 |
| CWT 母小波 | Morlet | 论文 | omega0=6 |
| CWT 尺度 | 1-28 | 论文 | 28 个尺度 |

---

## 相比 v2 的改动

| 项目 | v2 (存在检测) | v3 (生命体征) |
|------|--------------|--------------|
| TX 发包率 | 80 Hz | **120 Hz** |
| RX 输出 | 振幅计算 → 存在判定 | **原始 I/Q 数据** |
| 存在检测 | presence_detector (双指标) | **移除** |
| 信号处理 | ESP32 实时 | **PC 端 Python 离线** |
| 检测目标 | 有人/没人 | **呼吸率 + 心率** |
| 串口格式 | `HH:MM:SS, 有人, metric` | **`CSI,ts,rssi,nsub,I,Q,...`** |

---

## 硬件信息

| 项目 | 值 |
|------|----|
| ESP-IDF 版本 | v5.5.1 |
| ESP-IDF 路径 | `D:\esp32\v5.5.1\esp-idf` |
| ESP-IDF tools | `D:\esp32\.espressif` |
| 开发板 | ESP32-S3 DevKitC-VE |
| 天线 | 板载 PCB 天线 |
| TX 串口 | 未记录 |
| RX 串口 | COM28 |
| 串口波特率 | 115200 |

---

## 论文参考

- **标题**: Design and Evaluation of Volunteer User Trials of Unobtrusive Vital Signs Monitoring for Older People in Care Using Wi-Fi CSI Sensing
- **期刊**: IEEE Journal of Translational Engineering in Health and Medicine (JTEHM), 2025
- **DOI**: 10.1109/JTEHM.2025.3624469
- **作者**: Aaesha Alzaabi, Imran Saied, Tughrul Arslan (University of Edinburgh)
- **报告精度**: RR ~89%, HR ~85%（家庭环境）

---

## 备份时间
2026-05-06
