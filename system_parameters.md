# 系统运行说明 & 参数一览

---

## 一、上电后各板工作流程

### 信号板 (STM32F103C8T6)

```
上电 → LED闪3次 → 蓝牙输出 "Signal+TOF Ready"
     → 等待蓝牙收到 "START" 命令
     → bt_armed=1 后开始工作:
         每50ms (20Hz) 向电机板发送一帧 $FLW
         同时 CAN 中断接收 TOFSense-M 像素数据
```

| 状态 | 信号板行为 |
|------|-----------|
| bt_armed=0 | 发停车帧 `$FLW,0,0,0,0,3` |
| bt_armed=1 + UWB在线 + 无障碍 | 转发UWB帧 (角度取反×3) |
| bt_armed=1 + UWB在线 + 有障碍 | 用避障结果覆盖角度/距离 |
| bt_armed=1 + UWB超时(>500ms) | 发停车帧 |

### 电机驱动板 (STM32F103RCT6)

```
上电 → BLDC初始化 + PID初始化
     → UART3 DMA 接收 $FLW 帧
     → 解析帧 → 计算 speed/steer → 驱动电机
     → 超过1000ms没收到帧 → 自动停车
```

---

## 二、$FLW 协议格式

```
$FLW,<dist_cm>,<angle_x10>,<x_cm>,<y_cm>,<status>\r\n
```

| 字段 | 含义 | 范围 |
|------|------|------|
| dist_cm | 目标距离 (厘米) | 0~999 |
| angle_x10 | 目标角度×10 (正=右,负=左) | -1800~+1800 |
| x_cm | X坐标 (暂未用) | - |
| y_cm | Y坐标 (暂未用) | - |
| status | 1=跟随, 3=停车 | 0/1/2/3 |

---

## 三、跟随参数 (电机驱动板 util.c)

| 参数 | 值 | 说明 |
|------|-----|------|
| FLW_TARGET_DIST_CM | **120 cm** | 目标跟随距离 |
| FLW_HYSTERESIS_CM | **15 cm** | 死区: 120~135cm 内缓速前进 |
| FLW_DIST_KP | **5** | 距离→速度增益 |
| FLW_ANGLE_KP | **8** | 角度→转向增益 |
| FLW_ANGLE_DEADZONE_DEG | **±10°** | 角度死区 (±10°内不转) |
| FLW_MAX_SPEED | **400** | 最大前进速度 |
| FLW_MIN_SPEED | **60** | 最低前进速度 (缓行) |
| FLW_MAX_STEER | **500** | 最大转向量 |
| FLW_TIMEOUT_MS | **1000 ms** | 无帧超时→停车 |

**速度计算公式:**
```
if dist < 120:        stop (不动)
if 120 ≤ dist < 135:  speed = 60 (缓行)
if dist ≥ 135:        speed = 60 + (dist - 135) × 5, 上限400
```

**转向计算公式:**
```
angle_deg = angle_x10 / 10
if |angle_deg| < 10:  steer = 0 (死区)
else:                 steer = angle_deg × 8, 限幅 ±500
```

**信号板角度处理:**
```
发送角度 = -(UWB原始角度) × 3
效果: 死区从30°缩小到10°, 转向增益放大3倍
```

---

## 四、避障参数 (信号板 main.c + tofsense_can.c)

### TOFSense-M 传感器

| 项目 | 值 |
|------|-----|
| 接口 | CAN 1Mbps (PA11/PA12) |
| 像素 | 8×8 = 64点, 分左/中/右三区各16点 |
| 刷新率 | ~30Hz (64帧CAN全收齐为一次) |
| 有效判据 | dis_status==0 且 signal_strength > 2 |

### 避障阈值

| 参数 | 值 | 说明 |
|------|-----|------|
| danger_distance | **0.37 m** | <37cm 触发避障动作 |
| slow_down_distance | **1.0 m** | <1m 开始减速 |
| signal_strength_thr | **2** | 信号强度过滤阈值 |
| shift_speed | **100** | 避障横移强度 (0~127) |
| slow_down_speed | **35** | 减速模式最低速度 |

### 避障动作 → $FLW 映射

| 触发条件 | 动作 | 发给电机的帧 |
|----------|------|-------------|
| 三面全堵 (L<0.37 M<0.37 R<0.37) | 原地旋转脱困 | dist=130, angle=±900 |
| 右+中堵 | 向左闪 | dist=180, angle=-600 |
| 中+左堵 | 向右闪 | dist=180, angle=+600 |
| 右+左堵 | 向左闪 | dist=180, angle=-600 |
| 仅右堵 | 向左闪 | dist=180, angle=-600 |
| 仅中堵 | 向左闪 | dist=180, angle=-600 |
| 仅左堵 | 向右闪 | dist=180, angle=+600 |
| 无障碍但<1m | 减速 | dist≤145, 保持UWB角度 |
| 无障碍 | 正常跟随 | 转发UWB帧 |

### 避障时电机实际响应

```
SHIFT (dist=180, angle=±600):
  speed = 60 + (180-135)×5 = 285  (中速前进)
  steer = ±60×8 = ±480            (接近满转向)

ROTATE (dist=130, angle=±900):
  speed = 60 (缓行, 130<135在缓行区)
  steer = ±90×8 = ±500 (满转向, 被限幅)

SLOW_DOWN (dist=145):
  speed = 60 + (145-135)×5 = 110  (低速)
  steer = 正常UWB角度
```

---

## 五、接线总览

### 信号板 C8T6

| 引脚 | 功能 | 连接 |
|------|------|------|
| PA9/PA10 | USART1 115200 | HC-04 蓝牙 (收START/STOP) |
| PA2/PA3 | USART2 115200 | BU04 UWB模组 (收$FLW) |
| PB10/PB11 | USART3 115200 | 电机驱动板 (发$FLW) |
| PA11/PA12 | CAN1 1Mbps | TOFSense-M (经CAN收发器) |
| PC13 | LED | 亮=已启动, 灭=待机 |

### 电机驱动板 RCT6

| 引脚 | 功能 | 连接 |
|------|------|------|
| PB10/PB11 | USART3 115200 DMA | 信号板 (收$FLW) |
| 电机PWM | TIM | 左右BLDC电机 |

---

## 六、调参指南

| 想调什么 | 改哪里 | 当前值 → 建议范围 |
|---------|--------|-------------------|
| 跟随距离 | 电机板 FLW_TARGET_DIST_CM | 120 → 80~200 cm |
| 转向灵敏度 | 信号板 angle×3 的倍数 | 3 → 2~5 |
| 转向死区 | 电机板 FLW_ANGLE_DEADZONE_DEG | 10 → 5~15° |
| 最大速度 | 电机板 FLW_MAX_SPEED | 400 → 200~600 |
| 避障触发距离 | 信号板 danger_distance | 0.37 → 0.3~0.6 m |
| 减速开始距离 | 信号板 slow_down_distance | 1.0 → 0.5~1.5 m |
| 避障转向力度 | 信号板 OA_FLW_ANGLE_SHIFT | 600 → 400~900 |
| 避障时前进速度 | 信号板 OA_FLW_DIST_SHIFT | 180 → 140~220 |
