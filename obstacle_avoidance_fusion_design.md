# 避障融合方案设计文档

## 1. 系统架构

```
TOFSense x3 (前左/前中/前右)
     │ UART 921600 (查询模式, 5ms/传感器)
     ▼
 避障处理板 (CAN节点)
     │ CAN Bus
     ▼
 信号板 C8T6 ──UART3──> 电机驱动板 RCT6
     ▲
     │ UART2 ($FLW)
 BU04 UWB模组
```

**CAN 帧定义 (避障板 → 信号板):**

| CAN ID | Byte 0-1 | Byte 2-3 | Byte 4-5 | Byte 6 | Byte 7 |
|--------|----------|----------|----------|--------|--------|
| 0x100  | FR距离mm | FM距离mm | FL距离mm | 状态位 | 保留   |

- 距离: uint16_t, 单位 mm, 0xFFFF = 无效
- 状态位: bit0=FR有效, bit1=FM有效, bit2=FL有效
- 帧率: 30-50Hz (与TOF查询周期匹配)

---

## 2. 避障策略: 两级阈值 + 预测 + 跟随融合

### 2.1 核心参数

```c
#define TOF_SLOW_MM       1000    // 减速区阈值 (1.0m)
#define TOF_DANGER_MM      400    // 危险区阈值 (0.4m)
#define TOF_PREDICT_MS     200    // 预测窗口 (200ms后的位置)
#define TOF_STEER_MAX     1500    // 最大避障转向叠加量 (angle_x10 单位)
#define TOF_SPEED_SLOW_PCT   50   // 减速区速度百分比 (50%)
#define TOF_SPEED_MIN_PCT    30   // 危险区最低速度百分比 (30%, 不停车)
#define TOF_CAN_TIMEOUT_MS  200   // CAN数据超时 (超时=传感器离线, 不避障)
```

### 2.2 状态定义

```c
typedef enum {
    OA_IDLE = 0,     // 无障碍, 正常跟随
    OA_SLOWDOWN,     // 减速区: 降速但不改方向
    OA_DODGE,        // 危险区: 降速 + 偏转方向
} oa_state_t;
```

### 2.3 预测距离计算

```c
// 每次收到新 CAN 帧时更新
typedef struct {
    uint16_t dist_mm;           // 当前距离
    uint16_t prev_dist_mm;      // 上一帧距离
    uint32_t last_update_ms;    // 上次更新时间戳
    int16_t  velocity_mm_s;     // 接近速度 (负=靠近)
} tof_sensor_t;

tof_sensor_t tof_fr, tof_fm, tof_fl;

// 预测距离 = 当前距离 + 速度 × 预测窗口
static uint16_t tof_predicted_mm(tof_sensor_t *s) {
    int32_t pred = (int32_t)s->dist_mm + 
                   (int32_t)s->velocity_mm_s * TOF_PREDICT_MS / 1000;
    if (pred < 0) pred = 0;
    if (pred > 0xFFFE) pred = 0xFFFE;
    return (uint16_t)pred;
}
```

### 2.4 避障决策逻辑 (每帧调用一次)

```c
// 输入: 解析后的 $FLW 帧 (dist, angle_x10, status)
// 输出: 修改后的 angle_x10 和 speed_scale (0-100%)
// 原则: 只修改角度和速度比例, 不改变跟随逻辑本身

void oa_process(int *angle_x10, int *speed_pct) {
    // 超时检查: CAN 数据过期则退出避障
    if ((HAL_GetTick() - tof_fm.last_update_ms) > TOF_CAN_TIMEOUT_MS) {
        *speed_pct = 100;
        return;  // 传感器离线, 不干预
    }

    // 取预测距离
    uint16_t pred_fr = tof_predicted_mm(&tof_fr);
    uint16_t pred_fm = tof_predicted_mm(&tof_fm);
    uint16_t pred_fl = tof_predicted_mm(&tof_fl);
    uint16_t pred_min = MIN3(pred_fr, pred_fm, pred_fl);

    // ===== 阶段判断 =====
    if (pred_min >= TOF_SLOW_MM) {
        // 无障碍
        *speed_pct = 100;
        return;
    }

    if (pred_min >= TOF_DANGER_MM) {
        // === 减速区: 只降速, 不改方向 ===
        // 线性插值: SLOW_MM→100%, DANGER_MM→SLOW_PCT
        int32_t range = TOF_SLOW_MM - TOF_DANGER_MM;
        int32_t over  = pred_min - TOF_DANGER_MM;
        *speed_pct = TOF_SPEED_SLOW_PCT + 
                     (100 - TOF_SPEED_SLOW_PCT) * over / range;
        return;
    }

    // === 危险区: 降速 + 偏转 ===
    // 速度: 线性从 SLOW_PCT 降到 MIN_PCT (但不降到0!)
    int32_t danger_ratio = (int32_t)pred_min * 100 / TOF_DANGER_MM;
    *speed_pct = TOF_SPEED_MIN_PCT + 
                 (TOF_SPEED_SLOW_PCT - TOF_SPEED_MIN_PCT) * danger_ratio / 100;

    // 方向偏转: 根据左/右哪边更近来决定偏转方向
    // 偏转量与最近距离成反比
    int16_t steer_bias = 0;

    if (pred_fr < TOF_DANGER_MM && pred_fl >= TOF_DANGER_MM) {
        // 右前有障碍, 左前无 → 向左偏
        steer_bias = -(int16_t)(TOF_STEER_MAX * 
                     (TOF_DANGER_MM - pred_fr) / TOF_DANGER_MM);
    } 
    else if (pred_fl < TOF_DANGER_MM && pred_fr >= TOF_DANGER_MM) {
        // 左前有障碍, 右前无 → 向右偏
        steer_bias = +(int16_t)(TOF_STEER_MAX * 
                     (TOF_DANGER_MM - pred_fl) / TOF_DANGER_MM);
    }
    else if (pred_fr < TOF_DANGER_MM && pred_fl < TOF_DANGER_MM) {
        // 两侧都有障碍 → 选更远的一侧偏
        if (pred_fr > pred_fl) {
            steer_bias = +(int16_t)(TOF_STEER_MAX * 
                         (TOF_DANGER_MM - pred_fl) / TOF_DANGER_MM);
        } else {
            steer_bias = -(int16_t)(TOF_STEER_MAX * 
                         (TOF_DANGER_MM - pred_fr) / TOF_DANGER_MM);
        }
    }
    // 前中触发但两侧没触发: 选跟随角度的反方向偏 (绕过障碍后继续跟)
    else if (pred_fm < TOF_DANGER_MM) {
        if (*angle_x10 >= 0) {
            steer_bias = +(int16_t)(TOF_STEER_MAX / 2);  // 目标在右→往右绕
        } else {
            steer_bias = -(int16_t)(TOF_STEER_MAX / 2);  // 目标在左→往左绕
        }
    }

    // 叠加到跟随角度上 (融合, 非替代)
    *angle_x10 += steer_bias;
}
```

### 2.5 在信号板主循环中的调用位置

```c
// signal_board_c8t6/src/main.c 中修改:
// 在 flw_feed_byte() 解析出 dist, angle_x10 之后:

int speed_pct = 100;
oa_process(&angle_x10, &speed_pct);

// 应用速度缩放 (dist 不变, 让电机板自己算速度比例)
// 方法: 如果 speed_pct < 100, 缩短 dist 使电机板减速
int effective_dist = dist;
if (speed_pct < 100) {
    // 电机板速度正比于 (dist - target), 缩 dist 等效减速
    effective_dist = FLW_TARGET_DIST_CM + 
                     (dist - FLW_TARGET_DIST_CM) * speed_pct / 100;
    if (effective_dist < FLW_TARGET_DIST_CM) 
        effective_dist = FLW_TARGET_DIST_CM + 1;  // 保证不停车
}

snprintf(flw_latest, LINE_MAX, "$FLW,%d,%d,%d,%d,%d\r\n",
         effective_dist, -angle_x10 * 3, x, y, status);
```

---

## 3. 与 AutoRobo A 策略的关键区别

| 设计点 | AutoRobo A | 本方案 |
|--------|-----------|--------|
| 避障时电机控制 | 完全接管 Move_Control | 修改 angle/speed 叠加到跟随 |
| 是否停车 | 三面围堵→原地转 | **永不停车**, 最低30%速度 |
| 方向选择 | 固定组合逻辑 | 结合跟随方向选最优绕行路径 |
| 预测 | 无 | 200ms 窗口预测 |
| 速度控制 | 固定档位 (35/100/127) | 连续线性插值 |
| 传感器失效 | 无处理 | 超时200ms→退出避障, 不影响跟随 |

---

## 4. CAN 总线硬件方案

### 4.1 避障板选型建议
- **MCU**: STM32F103C8T6 (带CAN) 或 STM32G431 (性价比高)
- **CAN收发器**: TJA1050 / SN65HVD230
- **TOFSense接口**: UART (921600baud, 查询模式)
- 3个TOFSense用同一路UART级联 (ID 0/1/2, 5ms查询周期)

### 4.2 信号板 C8T6 的 CAN 接口
- STM32F103C8T6 自带 CAN 控制器 (PA11=CAN_RX, PA12=CAN_TX)
- 加 TJA1050 收发器即可
- CAN 不占用 UART, 三路串口保持不变

### 4.3 CAN 总线配置
- 波特率: 500kbps (足够 50Hz × 8字节)
- 仅需接收避障板发的 0x100 帧
- 中断接收, 更新 tof_fr/tof_fm/tof_fl 结构体

---

## 5. 实施步骤

1. **硬件准备**: 避障板 PCB (MCU + CAN收发器 + TOFSense接口)
2. **避障板固件**: 查询TOFSense → 打包CAN帧 → 30Hz发送
3. **信号板改造**: 加CAN收发器, PA11/PA12接CAN总线
4. **信号板固件**: CAN接收中断 → 更新距离 → oa_process() 修改$FLW帧
5. **调参**: 现场调整 TOF_SLOW_MM / TOF_DANGER_MM / TOF_STEER_MAX
