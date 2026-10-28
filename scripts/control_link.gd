extends Node
class_name WheeltecControl
## Godot 侧遥控出口: 只负责发 UDP, 真正 publish /cmd_vel 的是 bridge。
##
##     Godot(本脚本) --UDP 127.0.0.1:9101--> lidar_bridge.py --ROS2/DDS--> 小车底盘
##
## 为什么不在 Godot 里直接连 ROS2: Godot 没有 ROS2 绑定, 而 bridge 已经是一个
## 常驻的 rclpy 节点了。把它扩成"上行收数据 + 下行收命令", 就多出一条完整的
## 控制回路, 不用再引入第二个进程。
##
## 安全约束(和 bridge 一一对应, 两层都做):
##   1. 只有 enabled 才发命令。开关一关, 立刻停止发令。
##   2. 关闭的瞬间补发几帧零速度 —— 关开关就等于停车, 不等底盘自己的看门狗。
##   3. bridge 侧还有一层: 0.5 秒收不到包也当 0 速度。所以就算 Godot 崩了,
##      车最多再走 0.5 秒。
##
## 按键是每帧直接查物理键位, 不靠动作(action)也不靠按键事件回调:
## 这样长按不会受"终端式自动重复"影响, 鼠标被第一人称视角捕获时也照常工作。

signal enabled_changed(on: bool)

const MAGIC := "WCTL"
const VERSION := 1
const FLAG_ENABLED := 0x0001
const PACKET_SIZE := 24
## 发令频率, 和 wheeltec-control 的 publish_rate_hz 保持一致
const SEND_RATE := 20.0
## 关闭遥控后补几帧零速度: 20Hz 下 5 帧 = 0.25 秒
const ZERO_BURST := 5

@export var host := "127.0.0.1"
@export var port := 9101
## 4 档 [线速度 m/s, 角速度 rad/s], 与 wheeltec-control/config.json 的默认档位一致
@export var gears: Array[Vector2] = [
	Vector2(0.08, 0.30),
	Vector2(0.15, 0.60),
	Vector2(0.25, 0.90),
	Vector2(0.35, 1.20),
]
@export var start_gear := 1

var enabled := false
var gear_index := 1
var linear := 0.0
var angular := 0.0
var sent := 0
var send_errors := 0
## bridge 回执里的 control 段(enabled/fresh/subs/published/…), 由主控转交进来
var bridge: Dictionary = {}

var _udp: PacketPeerUDP
var _seq := 0
var _last_send := 0.0
var _zero_left := 0
var _keys := {"w": false, "s": false, "a": false, "d": false, "estop": false}


func _ready() -> void:
	if gears.is_empty():
		gears = [Vector2(0.15, 0.60)]
	gear_index = clampi(start_gear, 0, gears.size() - 1)
	_udp = PacketPeerUDP.new()
	# 只发不收, 所以不 bind: 用系统临时端口把包发给本机的 bridge
	_udp.set_dest_address(host, port)


func _process(_delta: float) -> void:
	var now := Time.get_ticks_msec() / 1000.0
	_read_keys()

	var cmd := command_for(_keys)
	linear = cmd.x
	angular = cmd.y

	if enabled:
		_zero_left = 0
		if now - _last_send >= 1.0 / SEND_RATE:
			_send(linear, angular, true)
			_last_send = now
	elif _zero_left > 0:
		# 刚关掉: 补几帧零速度, 让底盘确定收到停车命令
		if now - _last_send >= 1.0 / SEND_RATE:
			_send(0.0, 0.0, false)
			_last_send = now
			_zero_left -= 1


## 纯函数: 按键状态 → 命令 (线速度, 角速度)。
## 抽出来是为了能脱离 Input 和场景树单独验证 —— 真键盘只在 _read_keys() 里读。
## 前后互斥(W+S、A+D)相消为 0, 斜按(W+A)是边走边转, 和 wheeltec-control 一致。
func command_for(keys: Dictionary) -> Vector2:
	if not enabled or keys.get("estop", false):
		return Vector2.ZERO
	var gear: Vector2 = gears[gear_index]
	var cmd := Vector2.ZERO
	if keys.get("w", false):
		cmd.x += gear.x
	if keys.get("s", false):
		cmd.x -= gear.x
	if keys.get("a", false):
		cmd.y += gear.y
	if keys.get("d", false):
		cmd.y -= gear.y
	return cmd


func _read_keys() -> void:
	_keys["w"] = Input.is_physical_key_pressed(KEY_W)
	_keys["s"] = Input.is_physical_key_pressed(KEY_S)
	_keys["a"] = Input.is_physical_key_pressed(KEY_A)
	_keys["d"] = Input.is_physical_key_pressed(KEY_D)
	_keys["estop"] = Input.is_physical_key_pressed(KEY_SPACE)


# ---------------------------------------------------------------------------
# 对外
# ---------------------------------------------------------------------------
func set_enabled(on: bool) -> void:
	if enabled == on:
		return
	enabled = on
	if on:
		_zero_left = 0
	else:
		linear = 0.0
		angular = 0.0
		_zero_left = ZERO_BURST
	enabled_changed.emit(enabled)


func change_gear(step: int) -> void:
	gear_index = clampi(gear_index + step, 0, gears.size() - 1)


func apply_bridge_status(info: Dictionary) -> void:
	bridge = info


func status() -> Dictionary:
	return {
		"enabled": enabled,
		"linear": linear,
		"angular": angular,
		"gear": gear_index + 1,
		"gear_count": gears.size(),
		"keys": _keys.duplicate(),
		"estop": _keys["estop"],
		"sent": sent,
		"errors": send_errors,
		"bridge": bridge,
	}


# ---------------------------------------------------------------------------
# 打包
# ---------------------------------------------------------------------------
func _send(lin: float, ang: float, on: bool) -> void:
	# 24 字节定长, 和 bridge/lidar_bridge.py 里的 CTRL_HEADER 一一对应
	var packet := PackedByteArray()
	packet.resize(PACKET_SIZE)
	for i in 4:
		packet.encode_u8(i, MAGIC.unicode_at(i))
	packet.encode_u8(4, VERSION)
	packet.encode_u8(5, FLAG_ENABLED if on else 0)
	packet.encode_u16(6, 0)
	packet.encode_float(8, lin)
	packet.encode_float(12, ang)
	packet.encode_u32(16, _seq & 0xFFFFFFFF)
	packet.encode_u32(20, Time.get_ticks_msec() & 0xFFFFFFFF)
	_seq += 1

	if _udp.put_packet(packet) == OK:
		sent += 1
	else:
		send_errors += 1
