# UWB跟随+避障小车 系统技术文档

---

## 一、模块功能综述

本系统由 **4 块硬件板** 协作完成 UWB 跟随 + TOF 激光避障功能：

| 模块 | MCU/芯片 | 核心职责 | 代码目录 |
|------|----------|----------|----------|
| **BU03+蓝牙模块** | UWB+BLE | UWB 标签（被跟踪目标），蓝牙按键发送 START/STOP | `Ai-UWB-SDK/` |
| **BU04模块** | UWB | UWB 锚点，检测 BU03 位置，输出 `$FLW` 帧 | `BU04程序/` |
| **信号板** | STM32F103C8T6 | 中枢决策：蓝牙门控 + UWB 转发 + TOF CAN 避障融合 | `signal_board_c8t6/` |
| **电机驱动板** | STM32F103RC | 接收 `$FLW` 帧，P 控制器解析，驱动左右 BLDC 电机 | `Src/` + `Inc/` |

**数据流总览：**
```
[BU03 标签] ──BLE──> [信号板 USART1]
[BU04 锚点] ──UART──> [信号板 USART2]  ──融合决策──> [信号板 USART3] ──$FLW──> [电机驱动板 USART3]
[TOFSense-M] ──CAN──> [信号板 CAN1]                                              ↓
                                                                          BLDC FOC → 左右电机
```

---

## 二、各模块具体职责

### 2.1 BU03+蓝牙模块（UWB标签端）

**职责：**
- 作为 UWB 标签被 BU04 定位
- 板上按键通过 HC-04 蓝牙发送 `START\n` / `STOP\n` 给信号板

**判断逻辑：** 无复杂逻辑，按键触发即发送字符串命令。

**性能指标：**
- UWB 测距精度：±10cm（BU03/BU04 规格）
- 蓝牙传输延迟：<20ms

---

### 2.2 BU04模块（UWB锚点/检测端）

**职责：**
- 实时测距 + 测角 BU03 标签
- 通过 UART 输出标准 `$FLW` 帧给信号板

**协议格式：**
```
$FLW,<dist_cm>,<angle_x10>,<x_cm>,<y_cm>,<status>\r\n
```

| 字段 | 含义 |
|------|------|
| `dist_cm` | 标签距离(cm) |
| `angle_x10` | 角度×10 (正=右偏) |
| `x_cm`, `y_cm` | 平面坐标(cm) |
| `status` | 0=太近, 1=跟随, 2=太远, 3=丢失 |

**性能指标：**
- 刷新率：~10-20Hz
- 有效测距范围：0.3m ~ 15m

---

### 2.3 信号板（中枢决策板）

**代码文件：** `signal_board_c8t6/src/main.c` + `tofsense_can.h/.c`

**职责：**
1. **蓝牙门控** — USART1 接收 HC-04 的 `START`/`STOP`，切换 `bt_armed` 标志
2. **UWB 数据接收** — USART2 收 BU04 `$FLW` 帧，缓存最新距离/角度
3. **TOF 避障融合** — CAN1 收 TOFSense-M 64 像素深度图，分为左/中/右三区域
4. **综合决策+转发** — 20Hz 周期向电机驱动板 USART3 发送融合后的 `$FLW` 帧

**接口分配：**

| 接口 | 引脚 | 波特率/速率 | 连接对象 |
|------|------|------------|----------|
| USART1 | PA9/PA10 | 115200 | HC-04 蓝牙 |
| USART2 | PA2/PA3 | 115200 | BU04 UWB |
| USART3 | PB10/PB11 | 115200 | 电机驱动板 |
| CAN1 | PA11/PA12 | 1Mbps | TOFSense-M |

**避障判断逻辑（优先级从高到低）：**

```
if (!bt_armed)
    → 发送停车帧 $FLW,0,0,0,0,3

else if (三面围堵: 左中右均<danger)
    → ROTATE_CW/CCW: 大角度原地转向脱困
       steer=±1500, dist=130

else if (单侧/双侧<danger)
    → SHIFT_LEFT/RIGHT: 向空旷侧转向闪避
       steer=±1100, dist=180

else if (任一方向<slow_down_distance)
    → SLOW_DOWN: 保持UWB角度，限制dist≤200

else if (UWB在线)
    → NONE: 正常转发 UWB 帧（角度取反×5增益）

else (UWB超时>500ms)
    → 发送停车帧
```

**避障参数（差速车）：**

| 参数 | 值 | 含义 |
|------|----|------|
| `danger_distance` | 0.60m | 触发转向避障阈值 |
| `slow_down_distance` | 1.0m | 触发减速阈值 |
| `shift_steer` | 1100 | 闪避转向角(≈110°) |
| `shift_dist` | 180 | 闪避时前进速度 |
| `rotate_steer` | 1500 | 脱困转向角(≈150°) |
| `rotate_dist` | 130 | 脱困极慢前进 |
| `slow_dist` | 200 | 减速区最大dist |

**性能指标：**
- 决策周期：50ms（20Hz）
- BU04 超时阈值：500ms
- CAN 接收：1Mbps，64像素/轮

---

### 2.4 电机驱动板（执行端）

**代码文件：** `Src/main.c` + `Src/flw_follow.c` + `Src/util.c`

**职责：**
1. **$FLW 解析** (`flw_follow.c`) — DMA 接收 USART3 数据，行缓冲解析 `$FLW` 帧
2. **P 控制器** (`flw_follow.c`) — 将距离/角度转换为 speed/steer 指令
3. **滤波+混合** (`main.c`) — 速率限幅 + 低通滤波 + 差速混合
4. **BLDC FOC** (`BLDC_controller.c`) — Matlab 自动生成的正弦波换相控制器
5. **安全监控** (`main.c`) — 电压/温度/超时保护

**P 控制器逻辑（`flw_apply`）：**

```
if (status != 1) → 停车
if (dist < TARGET=150cm) → 停车（太近）
if (dist < TARGET+HYST=165cm) → 蠕行 (speed=MIN_SPEED=80)
if (dist >= 165cm) → speed = 80 + (dist-165) × KP=3, 上限100
steer = angle_deg × ANGLE_KP=4, 死区±30°, 上限±350
```

**滤波管线（每2ms主循环）：**
```
uart_speed_cmd → rateLimiter16(rate=480) → filtLowPass32(coef=6553) → speed
uart_steer_cmd → rateLimiter16(rate=480) → filtLowPass32(coef=6553) → steer
speed + steer → mixerFcn() → cmdL, cmdR → PWM驱动
```

**关键参数（config.h VARIANT_BBCAR）：**

| 参数 | 值 | 含义 |
|------|----|------|
| `FLW_TARGET_DIST_CM` | 150 | 目标跟随距离 1.5m |
| `FLW_HYSTERESIS_CM` | 15 | 迟滞区防抖 |
| `FLW_DIST_KP` | 3 | 速度P增益 |
| `FLW_ANGLE_KP` | 4 | 转向P增益 |
| `FLW_ANGLE_DEADZONE_DEG` | 30 | 角度死区 |
| `FLW_MAX_SPEED` | 100 | 最大前进cmd |
| `FLW_MAX_STEER` | 350 | 最大转向cmd |
| `FLW_MIN_SPEED` | 80 | 最小启动cmd |
| `FLW_TIMEOUT_MS` | 1000 | 通信超时 |
| `PWM_FREQ` | 16kHz | PWM频率 |
| `I_MOT_MAX` | 15A | 单电机电流限制 |
| `N_MOT_MAX` | 1500rpm | 转速上限 |

**性能指标：**
- 主循环周期：2ms（500Hz）
- 调试串口输出：125ms/次
- RAM 使用：6.1%（2992/49152 bytes）
- Flash 使用：17.5%（45800/262144 bytes）

---

## 三、系统架构

### 3.1 硬件拓扑

```
┌─────────────────┐          ┌─────────────────────────────────┐
│  BU03 + HC-04   │──BLE────→│  信号板 (STM32F103C8T6)         │
│  (UWB 标签)      │          │                                 │
└─────────────────┘          │  USART1 ← 蓝牙 START/STOP      │
                             │  USART2 ← BU04 $FLW (UWB)      │
┌─────────────────┐          │  CAN1   ← TOFSense-M (64像素)  │
│  BU04           │──UART───→│                                 │
│  (UWB 锚点)     │          │  融合决策引擎                    │
└─────────────────┘          │       ↓                         │
                             │  USART3 → 融合后$FLW帧          │
┌─────────────────┐          └────────────┬────────────────────┘
│ TOFSense-M      │──CAN1──→             │
│ (8×8 激光雷达)   │                      │ UART 115200
└─────────────────┘                      ↓
                             ┌─────────────────────────────────┐
                             │  电机驱动板 (STM32F103RC)        │
                             │                                 │
                             │  USART3 DMA ← $FLW              │
                             │  flw_follow.c → P控制器          │
                             │  滤波 + 差速混合                  │
                             │  BLDC FOC → 左/右电机 PWM        │
                             └─────────────────────────────────┘
```

### 3.2 软件分层（电机驱动板）

```
┌───────────────────────────────────────────┐
│           应用层 (main.c)                  │
│  主循环: 读输入 → 安全检查 → 滤波 → 混合   │
├───────────────────────────────────────────┤
│       通信层                               │
│  flw_follow.c  ← $FLW 解析 + P控制器      │
│  comms.c       ← 串口调试协议              │
│  util.c        ← USART DMA收发            │
├───────────────────────────────────────────┤
│       控制层                               │
│  BLDC_controller.c ← Matlab FOC           │
│  util.c (mixerFcn, rateLimiter, filtLP)   │
├───────────────────────────────────────────┤
│       驱动层                               │
│  setup.c ← GPIO/TIM/ADC/UART/DMA 初始化   │
│  bldc.c  ← PWM输出 + Hall中断             │
├───────────────────────────────────────────┤
│       HAL层 (STM32CubeF1)                 │
└───────────────────────────────────────────┘
```

---

## 四、模块配合时序

### 4.1 正常跟随时序（一个完整周期）

```
时间轴 →

BU04:     ──测距──[100ms]──测距──
               ↓                ↓
          $FLW帧(UART)     $FLW帧(UART)
               ↓                ↓
信号板:   收$FLW → 缓存 ─[50ms周期]─ 读TOF → 融合决策 → 发$FLW → ...
               │                                           ↓
               │                                    USART3 TX
               │                                           ↓
电机驱动: ─[2ms]─ DMA收 → parse → P控 → 限幅 → 滤波 → 混合 → PWM
```

### 4.2 启动序列

```
1. 全系统上电
2. BU04 开始测距 → 发 $FLW 帧
3. 信号板初始化: UART1/2/3 + CAN1 + TOFSense → LED闪烁6次 → "Signal+TOF Ready"
4. 电机驱动板初始化: GPIO + TIM + ADC + BLDC → 开机旋律 → 检测驾驶模式 → 等待按钮释放
5. 信号板 bt_armed=0: 持续发 STOP_FRAME ($FLW,0,0,0,0,3)
6. 用户按 BU03 按键 → BLE发"START\n"
7. 信号板收到 → bt_armed=1 → LED常亮 → 回复"ACK_START"
8. 信号板开始转发融合后的 $FLW 帧
9. 电机驱动板解析 → 电机启动
```

### 4.3 避障介入时序

```
正常跟随中:
  信号板 20Hz 轮询 TOFSense 64像素 CAN 帧
    ↓
  tofsense_get_zones() → 左/中/右最小距离
    ↓
  tofsense_avoid_danger() → avoid_result_t
    ↓
  [优先级判断]:
    ROTATE > SHIFT > SLOW_DOWN > NONE
    ↓
  build_avoid_frame() → 修改/覆盖 dist 和 angle
    ↓
  uart3_send_str() → 电机驱动板立即响应
```

### 4.4 停止序列

```
1. 用户按 BU03 按键 → BLE发"STOP\n"
2. 信号板收到 → bt_armed=0 → LED灭 → 回复"ACK_STOP"
3. 信号板发 STOP_FRAME ($FLW,0,0,0,0,3)
4. 电机驱动板: status=3 → speed=0, steer=0 → 清滤波器 → 电机停止
```

---

## 五、保护机制

### 5.1 信号板保护

| 保护项 | 实现方式 | 触发条件 |
|--------|----------|----------|
| **蓝牙门控** | `bt_armed` 标志 | 未收到 START → 永远发停车帧 |
| **BU04 超时** | `flw_last_ms` 时间戳比较 | >500ms 无 $FLW → 发停车帧 |
| **UART 溢出** | ORE 标志清除 + DR 读取 | USART1/2 中断中处理 |
| **行缓冲溢出** | `flw_idx >= LINE_MAX-1 → reset` | 异常长帧/乱码 |
| **CAN 自动离线恢复** | `AutoBusOff = ENABLE` | CAN 总线错误 |
| **避障优先级** | 避障决策覆盖 UWB 数据 | TOF 检测到障碍物时跟随让位 |

### 5.2 电机驱动板保护

| 保护项 | 实现方式 | 触发条件 | 响应 |
|--------|----------|----------|------|
| **$FLW 通信超时** | `flw_check_timeout()` | >1000ms 无帧 | speed=0, steer=0 |
| **过温保护** | `TEMP_POWEROFF_ENABLE` | ≥65°C | 自动关机 |
| **过温警告** | `TEMP_WARNING_ENABLE` | ≥60°C | 5声蜂鸣 |
| **低电压保护** | `BAT_DEAD_ENABLE` | <`BAT_DEAD` | 自动关机 |
| **低电量警告** | `BAT_LVL1/2_ENABLE` | 两级阈值 | 蜂鸣提醒 |
| **电流限制** | `I_MOT_MAX=15A`, `I_DC_MAX=17A` | FOC 内部限流 | 斩波限流 |
| **转速限制** | `N_MOT_MAX=1500rpm` | FOC 内部限速 | 限幅 |
| **无操作超时** | `INACTIVITY_TIMEOUT=8min` | 8分钟无动作 | 自动关机 |
| **速率限幅** | `rateLimiter16(rate=480)` | 每个主循环周期 | 平滑加速，防突变 |
| **FOC 故障码** | `rtY_Left/Right.z_errCode` | 电机/霍尔异常 | 禁用电机+蜂鸣 |
| **急停清零** | `speed=0 && steer=0` 时清滤波器状态 | 停车命令 | 立即归零，无残余运动 |

### 5.3 行缓冲保护（两板通用）

```c
// 信号板
if (flw_idx >= LINE_MAX - 1) { flw_idx = 0; }      // 128字节溢出重置
if (bt_idx >= sizeof(bt_line) - 1) { bt_idx = 0; }  // 16字节溢出重置

// 电机驱动板
if (flw_idx >= sizeof(flw_line) - 1) { flw_idx = 0; }  // 80字节溢出重置
```

---

## 六、异常处理方式

### 6.1 通信异常

| 异常场景 | 检测方 | 处理方式 |
|----------|--------|----------|
| **BU04 掉线** | 信号板 (`flw_last_ms` 超时) | 发停车帧，电机停止 |
| **蓝牙断连** | 信号板 (无 START/STOP) | `bt_armed` 保持当前值；若已 armed 则 UWB 正常转发 |
| **信号板→电机板 UART 断线** | 电机驱动板 (`flw_check_timeout`) | 1000ms 无帧 → speed/steer 归零 |
| **TOFSense CAN 初始化失败** | 信号板 (`tofsense_can_init`) | 串口输出 "ERR:CAN init fail"，避障失效但跟随正常 |
| **TOFSense 帧不完整** | 信号板 (`frame_ready` 标志) | 等待 64 像素收齐才更新避障决策 |
| **$FLW 帧格式错误** | 两板 (`sscanf` 返回值≠5) | 丢弃该帧，保持上一帧数据 |
| **UART 溢出 (ORE)** | 信号板 (USART1/2 IRQ) | 清 ORE 标志 + 读 DR 寄存器丢弃 |

### 6.2 传感器异常

| 异常场景 | 检测方 | 处理方式 |
|----------|--------|----------|
| **TOF 单像素无效** | 信号板 (`dis_status=255`) | 该像素不参与最小距离计算 |
| **TOF 信号强度过低** | 信号板 (`signal_strength < thr=2`) | 视为无效数据忽略 |
| **UWB 目标丢失** | BU04 (`status=3`) | 信号板发停车帧 |
| **UWB 太近** | BU04 (`status=0`) | 电机驱动板 P 控制器输出 0 |

### 6.3 电气异常

| 异常场景 | 检测方 | 处理方式 |
|----------|--------|----------|
| **电机堵转** | 电机驱动板 (FOC errCode) | 禁用电机 + 1声蜂鸣 |
| **霍尔传感器异常** | 电机驱动板 (FOC errCode) | 同上 |
| **主板过温** | 电机驱动板 (ADC温度采集) | ≥65°C 自动关机 |
| **电池低压** | 电机驱动板 (ADC电压采集) | 两级警告 → 最终关机 |
| **CAN 总线错误** | 信号板 (AutoBusOff) | 硬件自动恢复 |

### 6.4 异常恢复策略

```
通信恢复: 任一超时后，一旦收到有效帧，立即恢复正常运行（无需重启）
避障恢复: 障碍物移除后，下一个 TOF 轮询周期 (50ms) 自动回到正常跟随
电气恢复: FOC 错误需用户重启；温度/电压关机需充电或冷却后重新开机
```

---

> **文档版本**: v1.0  
> **对应代码**: 重构后的 `可以躲避垃圾桶V1.0` 工程  
> **四板系统**: BU03(标签) + BU04(锚点) + 信号板(C8T6) + 电机驱动板(F103RC)
