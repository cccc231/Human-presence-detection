# ESP32-S3 CSI 人体存在检测 - 项目进度

## 当前状态：已采集数据，需改进算法以支持静态人体检测

---

## 已完成的工作

### 1. 项目代码（全部完成）
- `tx/main/tx_main.c` - 发射端固件（ESP-NOW 广播，80Hz发包率，HT20 802.11n）
- `rx/main/rx_main.c` - 接收端主入口（WiFi STA + CSI + ESP-NOW + NTP）
- `rx/main/csi_processor.c/h` - CSI数据提取（回调→队列→振幅计算）
- `rx/main/presence_detector.c/h` - 人体检测算法（滑动窗口方差 + 固定阈值 + 防抖）
- `rx/main/time_utils.c/h` - NTP时间同步（北京时间 UTC+8，ntp.aliyun.com）
- `tools/serial_logger.py` - PC端CSV记录脚本（支持--duration自动停止、--delay倒计时、utf-8-sig编码）

### 2. 构建烧录（已完成）
- ESP-IDF v5.5.1 路径：`D:\esp32\v5.5.1\esp-idf`
- ESP-IDF tools 路径：`D:\esp32\.espressif`
- TX和RX工程均已成功编译烧录

### 3. 数据采集（已完成）
- `empty.csv` - 空房间数据（ESP-NOW版，SYNCING时段）
- `person.csv` - 有人数据（ESP-NOW版，含SYNCING + NTP校准后时段）
- VS Code IntelliSense 已配置（`.vscode/c_cpp_properties.json`）

---

## 已解决的问题
| 问题 | 解决方案 |
|------|---------|
| AP信标帧发包率太低（0.6Hz） | 改用ESP-NOW，80Hz发包 |
| 方差阈值0.035完全错误 | 实际metric在1.0-7.0范围，改为固定阈值1.5 |
| CSV中文在Excel乱码 | 改用utf-8-sig编码（带BOM） |
| 自适应校准不可靠 | 去掉校准，使用固定阈值 |
| VS Code红色报错 | 配置compileCommands，config名改为"Win32" |

---

## 待解决：静态人体检测

### 问题描述
当前算法使用**方差**作为唯一指标。方差衡量的是"变化量"：
- 人走动时 → 子载波振幅变化大 → 方差高 → ✅ 检测到
- 人静坐不动时 → 振幅稳定 → 方差低 → ❌ 误判为"没人"
- 空房间 → 振幅稳定 → 方差低 → ✅ 正确

### 数据对比（person.csv）
| 场景 | metric（方差） |
|------|--------------|
| 人静坐不动（16:47:52~16:48:41） | 1.86 ~ 1.98 |
| 空房间（16:58:20+，稳定后） | 1.0 ~ 1.3 |

**说明**：最新固件（固定阈值1.5）下，静坐时metric=1.86理论上应能判"有人"。
但用户实测反馈：静坐时判"没人"，只有姿态变化时才判"有人"。
**需确认**：RX板是否已重新烧录最新固件。

### 改进方案：方差 + 振幅偏差双指标
在现有方差基础上，增加**绝对振幅检测**：
1. 初始化阶段采集空房间每个子载波的振幅作为 `baseline[]`
2. 运行时计算当前振幅与baseline的偏差（`amp_dev = mean|current[k] - baseline[k]|`）
3. 最终指标 = max(variance_metric, amp_dev_metric)
4. 人体即使静止也会改变子载波振幅分布 → amp_dev升高 → 检测到
5. baseline在"没人"状态时缓慢自适应，"有人"时冻结

---

## 关键参数（当前值）
| 参数 | 值 | 文件 |
|------|-----|------|
| WINDOW_SIZE | 100 | presence_detector.h |
| THRESHOLD_HIGH | 1.5 | presence_detector.h |
| THRESHOLD_LOW | 1.2 | presence_detector.h |
| DEBOUNCE_COUNT | 3 | presence_detector.h |
| CONFIG_SEND_FREQUENCY | 80 | tx_main.c |

## 关键文件速查
| 文件 | 作用 |
|------|------|
| `rx/main/presence_detector.h` | 修改阈值、窗口大小、防抖次数 |
| `rx/main/presence_detector.c` | 检测算法核心逻辑 |
| `rx/main/csi_processor.c` | CSI数据处理管道 |
| `tools/serial_logger.py` | PC端CSV记录 |
| `build.bat` | Windows一键构建脚本（需从cmd运行） |

## 硬件信息
- TX串口：未记录（请确认COM口号）
- RX串口：COM28
- ESP-NOW广播地址：ff:ff:ff:ff:ff:ff
- WiFi信道：自动（channel 0）
- 协议：802.11n HT20
