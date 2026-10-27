# UDP 协议

中间件(`bridge/lidar_bridge.py`)发给 Godot 的包格式。**全部小端**。

设计取舍:只传极坐标的 `u16` 毫米距离,不传 `x/y` 浮点。540 点从 4320 字节降到
1080 字节,一包就能塞进 1500 MTU,以后跨机部署也不会分片。

## 头(固定 40 字节)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | `4s` | `magic` | 固定 `"WLDR"` |
| 4 | `u8` | `version` | 目前为 `1` |
| 5 | `u8` | `msg_type` | `1`=扫描 `2`=状态 `3`=图像(预留) |
| 6 | `u16` | `point_count` | 该消息的点/元素个数 |
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

第 i 个点的角度 = `angle_min + i * angle_increment`,
距离 = `mm / 1000`。

坐标换算(在 Godot 侧做):

```
ROS:   x 朝前, y 朝左, 角度 0 = +x, 逆时针为正
点   = (r*cos(a), r*sin(a))

Godot: -Z 朝前, +X 朝右, Y 朝上
点   = (-r*sin(a), 0, -r*cos(a))
```

这样车头永远朝屏幕上方,左边就是左边,和雷达自身坐标系一致。

## type = 2 状态

负载: UTF-8 JSON,例如:

```json
{"source":"bag:wheeltec_20261013_185317_0.db3//scan","hz":10.0,
 "frames":1820,"points":540,"port":9100,"uptime":182.0}
```

每秒发一次,用于 HUD 显示数据源名称。

## type = 3 图像(预留,尚未实现)

规划:头后接 `u16 width, u16 height, u8 format(1=jpeg)`,再接 JPEG 字节。
超过 MTU 时按 `u16 chunk_index, u16 chunk_total` 分片。Godot 侧按 `seq` 重组。

## 扩展方式

加新消息类型只需要:

1. `lidar_bridge.py` 里定义新的 `MSG_*` 常量和编码函数;
2. Godot 的 `scripts/lidar_receiver.gd` 里 `match msg_type:` 加一个分支;
3. 传输层(UDP、端口、`PacketPeerUDP`)完全不用动。
