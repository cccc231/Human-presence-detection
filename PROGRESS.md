# ESP32-S3 CSI 生命体征监测系统 - 项目进度

## 当前状态：v3 生命体征监测代码已完成，待测试

---

## 版本历史

### v3 - 生命体征监测 (当前版本)
复现 IEEE JTEHM 2025 论文，从 WiFi CSI 提取呼吸率(RR)和心率(HR)。
- 状态：**代码完成，未测试**
- 备份：`backup/v3_vital_signs/`

### v2 - 双指标存在检测
方差 + 振幅偏差双指标，1秒聚合，支持静态人体检测。
- 状态：已完成
- 备份：`backup/v2_dual_metric/`

### v1 - 固定阈值存在检测
固定阈值 1.5 + 逐包防抖。
- 状态：已完成
- 备份：`backup/v1_fixed_threshold/`

---

## v3 已完成工作

### ESP32 固件
- [x] TX 发包率改为 120Hz (`tx/main/tx_main.c`)
- [x] RX 输出原始 I/Q 数据 (`rx/main/csi_processor.c`)
- [x] RX 简化入口，移除 presence_detector (`rx/main/rx_main.c`)
- [x] RX CMakeLists 更新

### Python 信号处理
- [x] `tools/vital_signs.py` 完整实现
  - 数据采集（串口读取）
  - 振幅提取
  - 插值重采样（RR: 40Hz, HR: 60Hz）
  - DWT db4 小波滤波
  - PCA 降维
  - PC-SampEn 选择最优主成分
  - CWT Morlet 提取频率

---

## 待办事项

### 测试验证
- [ ] 安装 Python 依赖：`pip install numpy scipy PyWavelets scikit-learn nolds pyserial`
- [ ] 重新编译并烧录 TX 固件（120Hz）
- [ ] 重新编译并烧录 RX 固件（原始 I/Q 输出）
- [ ] 运行 `python tools/vital_signs.py --port COM28 --duration 120`
- [ ] 验证呼吸率和心率输出
- [ ] 对比论文精度：RR ~89%，HR ~85%（家庭环境）

---

## 关键参数（v3）

### ESP32 固件
| 参数 | 值 | 文件 |
|------|----|------|
| TX 发包率 | 120 Hz | `tx/main/tx_main.c` |
| CSI 输出格式 | 原始 I/Q | `rx/main/csi_processor.c` |

### Python 信号处理
| 参数 | 值 | 说明 |
|------|----|------|
| DWT 小波 | db4, 6层 | 论文设定 |
| RR 频段 | 0.1-0.5 Hz | level 4-6 |
| HR 频段 | 0.8-2.0 Hz | level 2-4 |
| PCA 成分数 | 10 | 前10个主成分 |
| SampEn 嵌入维度 | m=3 | 论文设定 |
| SampEn 容差 | r=0.1*σ | 每个PC标准差的10% |
| CWT 母小波 | Morlet, 28 scales | 论文设定 |

---

## 使用方法（v3）

### 1. 安装 Python 依赖
```bash
pip install numpy scipy PyWavelets scikit-learn nolds pyserial
```

### 2. 编译烧录固件
```bash
# TX
cd D:\esp32c5\see\tx
idf.py build
idf.py -p COMxx flash  # 替换为 TX 串口号

# RX
cd D:\esp32c5\see\rx
idf.py build
idf.py -p COM28 flash
```

### 3. 运行生命体征检测
```bash
cd D:\esp32c5\see
python tools/vital_signs.py --port COM28 --duration 120
```

**测试条件**：
- 被测者静坐，面对天线
- 距离 1-3 米
- 保持静止（可正常呼吸）
- 采集时长建议 120 秒

---

## 信号处理流程（论文算法）

```
CSI I/Q 原始数据 (120Hz, 52子载波)
        ↓
振幅提取: amp = sqrt(I² + Q²)
        ↓
插值重采样 (非均匀 → 均匀)
   RR: 40 Hz    HR: 60 Hz
        ↓
DWT db4 小波滤波 (6层分解)
   RR: level 4-6 (0.1-0.5 Hz)
   HR: level 2-4 (0.8-2.0 Hz)
        ↓
PCA 降维 (52子载波 → 10主成分)
        ↓
PC-SampEn 选择最规则主成分
        ↓
CWT Morlet 小波 → 平均小波能量
        ↓
峰值检测 → 频率 → 呼吸率/心率
```

---

## 关键文件速查

| 文件 | 作用 |
|------|------|
| `tools/vital_signs.py` | PC 端生命体征信号处理 |
| `rx/main/csi_processor.c` | CSI 原始 I/Q 数据输出 |
| `rx/main/rx_main.c` | RX 主入口 |
| `tx/main/tx_main.c` | TX 固件（120Hz） |
| `CHANGELOG.md` | 详细改动记录 |

---

## 硬件信息
- ESP-IDF v5.5.1：`D:\esp32\v5.5.1\esp-idf`
- ESP-IDF tools：`D:\esp32\.espressif`
- TX 串口：未记录
- RX 串口：COM28
- WiFi 信道：自动（channel 0）
- 协议：802.11n HT20

---

## 论文参考
- 标题：Design and Evaluation of Volunteer User Trials of Unobtrusive Vital Signs Monitoring for Older People in Care Using Wi-Fi CSI Sensing
- 期刊：IEEE Journal of Translational Engineering in Health and Medicine (JTEHM), 2025
- 作者：Alzaabi et al.
- 精度：RR ~89%，HR ~85%（家庭环境）

---

## 更新记录
- 2026-05-06：完成 v3 代码，备份到 backup/v3_vital_signs/
- 2026-05-06：完成 v2 双指标算法，备份到 backup/v2_dual_metric/
- 2026-05-06：完成 v1 固定阈值，备份到 backup/v1_fixed_threshold/
