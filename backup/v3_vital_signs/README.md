# v3 - 生命体征监测 (Vital Signs Monitoring)

复现论文 IEEE JTEHM 2025 - Alzaabi et al. 的 WiFi CSI 生命体征检测算法。

## 功能
- 呼吸率 (RR) 检测：6-30 BrPM
- 心率 (HR) 检测：48-120 BPM

## 架构
```
ESP32 TX (120Hz) ──ESP-NOW──> ESP32 RX ──串口──> PC (vital_signs.py)
                                         I/Q原始数据      ↓
                                                   DWT db4 小波滤波
                                                           ↓
                                                    PCA 降维 (52→10)
                                                           ↓
                                                   PC-SampEn 选最优主成分
                                                           ↓
                                                   CWT Morlet 提取频率
                                                           ↓
                                                      RR / HR
```

## 文件清单

| 文件 | 说明 |
|------|------|
| `rx/main/csi_processor.c` | CSI 处理：输出原始 I/Q 数据到串口 |
| `rx/main/csi_processor.h` | CSI 数据结构定义 |
| `rx/main/rx_main.c` | RX 主入口（移除 presence_detector） |
| `rx/main/CMakeLists.txt` | 构建配置 |
| `tx/main/tx_main.c` | TX 固件（120Hz 发包率） |
| `tools/vital_signs.py` | PC 端信号处理脚本 |

## 相比 v2 的改动
- TX 发包率：80Hz → 120Hz
- RX 输出：振幅计算 → 原始 I/Q 数据
- 移除 presence_detector（生命体征不需要存在检测）
- 新增 vital_signs.py：完整的信号处理流水线

## 使用方法

```bash
# 安装依赖
pip install numpy scipy PyWavelets scikit-learn nolds pyserial

# 烧录固件
cd tx && idf.py build && idf.py -p COMxx flash
cd rx && idf.py build && idf.py -p COM28 flash

# 运行（被测者需静坐面对天线，距离1-3米）
python tools/vital_signs.py --port COM28 --duration 120
```

## 关键参数
| 参数 | 值 | 来源 |
|------|----|------|
| TX 发包率 | 120 PPS | 论文设定 |
| 子载波数 | 52 | ESP32 HT20 LLTF |
| DWT 小波 | db4, 6层分解 | 论文设定 |
| RR 频段 | 0.1-0.5 Hz | level 4-6 |
| HR 频段 | 0.8-2.0 Hz | level 2-4 |
| PCA 成分数 | 10 | 前10个主成分 |
| SampEn 嵌入维度 | m=3 | 论文设定 |
| SampEn 容差 | r=0.1*σ | 论文设定 |
| CWT 母小波 | Morlet, 28 scales | 论文设定 |

## 备份时间
2026-05-06
