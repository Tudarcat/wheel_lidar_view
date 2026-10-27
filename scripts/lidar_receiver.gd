extends Node
class_name LidarReceiver
## 接收 bridge/lidar_bridge.py 发来的 UDP 包并解析。
##
## 协议 (小端):
##   头 40 字节:
##     4s magic "WLDR" | u8 ver | u8 type | u16 count
##     u32 seq | u32 sec | u32 nsec
##     f32 angle_min | f32 angle_increment | f32 range_min | f32 range_max
##     u16 flags | u16 reserved
##   负载:
##     type=1 扫描: u16[count] 距离(毫米, 0=无效)
##     type=2 状态: UTF-8 JSON
##     type=3 图像: 预留
##
## 加新的消息类型只要在 _decode 里加一个分支, 不用动传输层。

signal scan_received(scan: Dictionary)
signal status_received(info: Dictionary)
signal telemetry_received(tel: Dictionary)

const MAGIC := "WLDR"
const MSG_SCAN := 1
const MSG_STATUS := 2
const MSG_TELEMETRY := 4
const FLAG_RANGE_MM := 0x0001

const TEL_ODOM := 0x0001
const TEL_IMU := 0x0002
const TEL_BATTERY := 0x0004

@export var port: int = 9100
@export var bind_address: String = "127.0.0.1"

var packets := 0
var scans := 0
var bad := 0
var dropped := 0
var source := ""
var last_scan_msec := 0
var hz := 0.0
var last_seq := -1

var _udp: PacketPeerUDP
var _arrivals: Array[int] = []
var _bind_error: String = ""


func _ready() -> void:
	_udp = PacketPeerUDP.new()
	# 只有本机的桥接会发数据, 绑回环地址就够, 也不怕外部乱发包
	var err := _udp.bind(port, bind_address)
	if err != OK:
		_bind_error = error_string(err)
		push_error("UDP 绑定 %s:%d 失败: %s" % [bind_address, port, _bind_error])


func is_listening() -> bool:
	return _bind_error.is_empty()


func bind_error() -> String:
	return _bind_error


func _process(_delta: float) -> void:
	if not is_listening():
		return
	while _udp.get_available_packet_count() > 0:
		var packet := _udp.get_packet()
		packets += 1
		_decode(packet)

	# 用最近 1 秒的到达时间算帧率(只记真正的扫描帧, 不记 _process 调用)
	var now := Time.get_ticks_msec()
	while _arrivals.size() > 0 and now - _arrivals[0] > 1000:
		_arrivals.pop_front()
	hz = float(_arrivals.size())


func _decode(packet: PackedByteArray) -> void:
	if packet.size() < 40:
		bad += 1
		return
	var sp := StreamPeerBuffer.new()
	sp.data_array = packet
	sp.big_endian = false

	if sp.get_string(4) != MAGIC:
		bad += 1
		return
	var version := sp.get_u8()
	if version != 1:
		bad += 1
		return
	var msg_type := sp.get_u8()
	var count := sp.get_u16()
	var seq := sp.get_u32()
	var sec := sp.get_u32()
	var nsec := sp.get_u32()
	var angle_min := sp.get_float()
	var angle_increment := sp.get_float()
	var range_min := sp.get_float()
	var range_max := sp.get_float()
	var flags := sp.get_u16()
	sp.get_u16()   # reserved

	match msg_type:
		MSG_SCAN:
			_decode_scan(sp, count, seq, sec, nsec, angle_min, angle_increment,
				range_min, range_max, flags)
		MSG_STATUS:
			_decode_status(sp)
		MSG_TELEMETRY:
			_decode_telemetry(sp)
		_:
			bad += 1


func _decode_scan(sp: StreamPeerBuffer, count: int, seq: int, sec: int, nsec: int,
		angle_min: float, angle_increment: float, range_min: float, range_max: float,
		flags: int) -> void:
	var need := count * 2
	if sp.get_available_bytes() < need:
		bad += 1
		return
	var ranges := PackedFloat32Array()
	ranges.resize(count)
	if flags & FLAG_RANGE_MM:
		for i in count:
			var mm := sp.get_u16()
			ranges[i] = 0.0 if mm == 0 else float(mm) * 0.001
	else:
		for i in count:
			ranges[i] = sp.get_float()

	# 序号不连续说明中间 UDP 包丢了
	if last_seq >= 0 and seq > last_seq + 1:
		# 上限 1000: 换数据源时序号可能跳变, 不要让它污染统计
		dropped += mini(seq - last_seq - 1, 1000)
	last_seq = seq

	scans += 1
	last_scan_msec = Time.get_ticks_msec()
	_arrivals.append(last_scan_msec)
	scan_received.emit({
		"seq": seq,
		"stamp_sec": sec,
		"stamp_nsec": nsec,
		"angle_min": angle_min,
		"angle_increment": angle_increment,
		"range_min": range_min,
		"range_max": range_max,
		"ranges": ranges,
	})


func _decode_telemetry(sp: StreamPeerBuffer) -> void:
	if sp.get_available_bytes() < 44:
		bad += 1
		return
	var pos_x := sp.get_float()
	var pos_y := sp.get_float()
	var yaw := sp.get_float()
	var vel_x := sp.get_float()
	var vel_y := sp.get_float()
	var omega := sp.get_float()
	var acc_x := sp.get_float()
	var acc_y := sp.get_float()
	var acc_z := sp.get_float()
	var voltage := sp.get_float()
	var flags := sp.get_u16()
	sp.get_u16()
	telemetry_received.emit({
		"pos_x": pos_x, "pos_y": pos_y, "yaw": yaw,
		"vel_x": vel_x, "vel_y": vel_y, "omega": omega,
		"acc_x": acc_x, "acc_y": acc_y, "acc_z": acc_z,
		"voltage": voltage, "flags": flags,
	})


func _decode_status(sp: StreamPeerBuffer) -> void:
	var text := sp.get_utf8_string(sp.get_available_bytes())
	var parsed = JSON.parse_string(text)
	if typeof(parsed) == TYPE_DICTIONARY:
		source = String(parsed.get("source", ""))
		status_received.emit(parsed)
	else:
		bad += 1


## 距上一帧扫描过去了多少秒, 用于判断"断流"
func seconds_since_scan() -> float:
	if last_scan_msec == 0:
		return INF
	return float(Time.get_ticks_msec() - last_scan_msec) / 1000.0
