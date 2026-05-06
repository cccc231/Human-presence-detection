# v1: 固定阈值检测

## 版本概述
使用滑动窗口方差算法 + 固定阈值判断人体存在。去除自适应校准，直接根据实测数据设定阈值。

## 算法原理
1. RX 端每收到一个 CSI 包，提取所有子载波振幅（`amp = sqrt(I² + Q²)`）
2. 维护一个 100 包的滑动窗口（约 1.25 秒）
3. 对每个子载波计算窗口内方差：`var_k = E[x²] - E[x]²`
4. 最终指标 = 所有子载波方差的均值
5. 逐包判断：metric > 1.5 → 有人，metric < 1.2 → 没人（迟滞防抖）
6. 连续 3 次相同判断才切换状态（防抖）

## 关键参数
| 参数 | 值 | 说明 |
|------|-----|------|
| WINDOW_SIZE | 100 | 滑动窗口大小 |
| THRESHOLD_HIGH | 1.5 | 有人判定阈值 |
| THRESHOLD_LOW | 1.2 | 没人判定阈值（迟滞） |
| DEBOUNCE_COUNT | 3 | 防抖计数 |
| CONFIG_SEND_FREQUENCY | 80 | TX 发包率 (pkt/s) |

## 已知问题
- **静止人体无法检测**：方差只衡量变化量，人静坐不动时方差与空房间相近，会误判为"没人"
- 只有姿态变化时才会触发"有人"

## 文件列表
| 文件 | 说明 |
|------|------|
| presence_detector.h | 阈值和参数定义 |
| presence_detector.c | 检测算法核心逻辑 |
| csi_processor.c/h | CSI 数据提取（回调→队列→振幅计算） |
| rx_main.c | RX 端入口（WiFi STA + CSI + ESP-NOW） |
| time_utils.c/h | NTP 时间同步（北京时间） |
| tx_main.c | TX 端（ESP-NOW 80Hz 广播） |
| serial_logger.py | PC 端 CSV 记录脚本 |
