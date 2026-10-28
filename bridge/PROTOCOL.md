# UDP 协议

两端都用本机回环, **全部小端**。正向(传感器)和反向(遥控)各占用一个端口:

```
                 ┌───────────────────────────────┐
   /scan  ──────►│                               │──► 9100  MSG_SCAN / MSG_TELEMETRY / MSG_STATUS
                 │      lidar_bridge.py          │
   /cmd_vel ◄────│                               │◄── 9101  CTRL (遥控包)
                 └───────────────────────────────┘
```

为什么分两个端口: 遥控是 Godot → bridge 的反方向流, 共用一个 socket 会互相抢读,
而且两边的节奏完全独立(扫描 10Hz、遥测 5Hz、遥控 20Hz)。分开之后两边的缓冲区
互不影响, 加超时/丢弃策略也不会牵连对方。

---

# 一、bridge → Godot (端口 9100)

## 头(固定 40 字节)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | `4s` | `magic` | 固定 `"WLDR"` |
| 4 | `u8` | `version` | 目前为 `1` |
| 5 | `u8` | `msg_type` | `1`=扫描 `2`=状态 `3`=图像(预留) `4`=遥测 |
| 6 | `u16` | `point_count` | 该消息的点/元素个数(遥测与状态为 0) |
| 8 | `u32` | `seq` | 递增序号,用于检测丢包 |
| 12 | `u32` | `stamp_sec` | ROS 时间戳(秒) |
| 16 | `u32` | `stamp_nsec` | ROS 时间戳(纳秒) |
| 20 | `f32` | `angle_min` | 第一个点的角度(弧度) |
| 24 | `f32` | `angle_increment` | 角度步长(弧度) |
| 28 | `f32` | `range_min` | 有效距离下限(米) |
| 32 | `f32` | `range_max` | 有效距离上限(米) |
| 36 | `u16` | `flags` | 位 0: 负载为 u16 毫米 |
| 38 | `u16` | `reserved` | 置 0 |

## type = 1 扫描

负载: `u16[point_count]`,单位**毫米**,`0` 表示无效(无回波 / 超量程)。

第 i 个点的角度 = `angle_min + i * angle_increment`,距离 = `mm / 1000`。

坐标换算(在 Godot 侧做):

```
ROS:   x 朝前, y 朝左, 角度 0 = +x, 逆时针为正
点   = (r*cos(a), r*sin(a))

Godot: -Z 朝前, +X 朝右, Y 朝上
点   = (-r*sin(a), 0, -r*cos(a))
```

这样车头永远朝屏幕上方,左边就是左边,和雷达自身坐标系一致。

540 点共 40 + 1080 = **1120 字节**, 塞得进一个 MTU。

## type = 4 遥测(底盘状态)

负载: 固定 **44 字节**, `struct '<10fHH'`。

| 偏移 | 类型 | 字段 | 单位 / 来源 |
| ---: | --- | --- | --- |
| 0 | `f32` | `pos_x` | m,`/odom` |
| 4 | `f32` | `pos_y` | m,`/odom` |
| 8 | `f32` | `yaw` | rad,`/odom` 四元数解出的偏航角 |
| 12 | `f32` | `vel_x` | m/s,`/odom` 线速度 |
| 16 | `f32` | `vel_y` | m/s,轮式底盘通常恒为 0 |
| 20 | `f32` | `omega` | rad/s,`/odom` 角速度 |
| 24 | `f32` | `acc_x` | m/s²,`/imu/data_raw` |
| 28 | `f32` | `acc_y` | m/s²,`/imu/data_raw` |
| 32 | `f32` | `acc_z` | m/s²,`/imu/data_raw`(静止时 ≈ 9.8) |
| 36 | `f32` | `voltage` | V,`/PowerVoltage` |
| 40 | `u16` | `flags` | 位 0 `TEL_ODOM`、位 1 `TEL_IMU`、位 2 `TEL_BATTERY` |
| 42 | `u16` | `reserved` | 置 0 |

**`flags` 是必须看的东西**: 字段是定长结构,不会缺项,所以"这个字段有没有意义"
全靠 flags 说明。位没亮时 Godot 侧显示 `—` 而不是 0,避免把"没这个传感器"
误读成"读数是 0"。

发送频率 **5 Hz**(它比激光慢得多, 没必要每帧都发)。

## type = 2 状态

负载: UTF-8 JSON。每秒一次; **遥控状态一变会立刻补发一包**, 这样拨开关、
按键的手感不用等一秒。例:

```json
{"source":"demo","hz":9.8,"frames":314,"points":540,"port":9100,"uptime":31.7,
 "control":{"backend":"ros2:/cmd_vel","enabled":true,"fresh":true,"stale":false,
            "active":true,"state":"sending","linear":0.15,"angular":0.6,
            "subs":1,"published":88,"packets":91,"bad":0,"gaps":0,
            "watchdog":0.5,"port":9101}}
```

`control` 段是**遥控回执**, 只有 bridge 知道, 所以由它回传给 HUD:

| 字段 | 含义 |
| --- | --- |
| `backend` | 真正的出口。`ros2:/cmd_vel` 表示会驱动底盘; `模拟底盘(无 ROS)` 是 demo/bag; `disabled` 是 `--no-control` |
| `enabled` | Godot 送来的开关位 |
| `fresh` / `stale` | 最近 0.5 秒内还有包 / 曾经有过但现在断了 |
| `active` | 最终是否在发令(`enabled && fresh`) |
| `state` | `sending` / `tail`(正在补零速度) / `idle` |
| `linear` / `angular` | 当前实际下发的速度(已限幅) |
| `subs` | 底盘对 `/cmd_vel` 的订阅数。**只有 `ros2:` 出口这个数才有意义** |
| `published` | bridge 累计发令次数 |
| `packets` / `bad` / `gaps` | 收到的遥控包数 / 坏包数 / 序号跳变累计 |

## type = 3 图像(预留,尚未实现)

规划:头后接 `u16 width, u16 height, u8 format(1=jpeg)`,再接 JPEG 字节。
超过 MTU 时按 `u16 chunk_index, u16 chunk_total` 分片。Godot 侧按 `seq` 重组。

---

# 二、Godot → bridge (端口 9101)

## 遥控包(固定 24 字节)

```
magic ver flags rsvd  linear   angular    seq   stamp_ms
<'4s  B   B     H      f        f         I       I '>
```

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | `4s` | `magic` | 固定 `"WCTL"` |
| 4 | `u8` | `version` | 目前为 `1` |
| 5 | `u8` | `flags` | **位 0 = 遥控开关**。不亮则接收端一律当 0 速度 |
| 6 | `u16` | `reserved` | 置 0 |
| 8 | `f32` | `linear` | 线速度 m/s |
| 12 | `f32` | `angular` | 角速度 rad/s |
| 16 | `u32` | `seq` | 递增序号,只用于统计丢包 |
| 20 | `u32` | `stamp_ms` | 发送端的毫秒计时(不是 epoch, 接收端不解析) |

频率 **20 Hz**(和 `wheeltec-control` 的 `publish_rate_hz` 一致)。

## 为什么用"开关位"而不是"停发"

如果关开关就单纯停发, 底盘会一直保留最后一条命令, 直到它自己的看门狗超时。
所以开关关闭时 **仍然继续发几帧零速度**(`ZERO_BURST = 5` 帧 = 0.25 秒)再停,
把"关掉"变成确定性的停车动作, 而不是"等对面超时"。

## 三层保护

| 位置 | 条件 | 结果 |
| --- | --- | --- |
| Godot | 开关关闭 | 立刻补 5 帧零速度, 之后不再发包 |
| Godot | 窗口失去焦点 / 被后台 | 主循环停下 → 不再发包 → 走下一行 |
| bridge | 0.5 秒收不到包, 或 flags 的开关位不亮 | 命令归零, 再补 5 帧零速度, 然后静默 |

所以任何一端崩掉, 车最多再走 0.5 秒。**这是设计值, 不是巧合。**
想更保守就调小 `--watchdog`(例如 0.3), 代价是网络抖一下就会提前停车。

## 限幅在两侧各做一次

Godot 按档位限(0.08~0.35 m/s, 0.30~1.20 rad/s), bridge 收到后再按
`CTRL_MAX_LINEAR=0.35` / `CTRL_MAX_ANGULAR=1.20` 硬限一次。
后者是兜底: 万一 Godot 侧被改坏(或有人手搓包), 也不会给底盘灌一个离谱的速度。

---

# 三、扩展方式

加新消息类型只需要:

1. `lidar_bridge.py` 里定义新的 `MSG_*` 常量和编码函数;
2. Godot 的 `scripts/lidar_receiver.gd` 里 `match msg_type:` 加一个分支;
3. 传输层(UDP、端口、`PacketPeerUDP`)完全不用动。

加新的**上行控制**(比如云台角度、灯效), 照抄 `ControlLink` 的做法即可:
在 9101 上再定义一个 `magic` 不同的包类型, 用第二个 `msg_type` 字节区分。
