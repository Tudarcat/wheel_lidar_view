extends Node3D
## 雷达 3D 视图主控: 收包 → 建点云 → 摆相机 → 刷 HUD。
##
## 坐标约定: 雷达数据是传感器自身坐标系 (/scan 的 frame_id = laser),
## 所以把车模型放在原点、点云绕着它摆, 天然就是"车辆居中"。
## ROS 的 x 朝前 / y 朝左, 映射到 Godot 的 -Z 朝前 / +X 朝右:
##     点位置 = (-r*sin(a), 0, -r*cos(a))
## 这样车头永远朝屏幕上方, 左就是左。
##
## 三种视角:
##   TOP          正俯视, 看全局
##   ANGLED       斜俯视, 能看清方块侧面
##   FIRST_PERSON 第一人称, 坐在车上看向墙壁; 准星带测距

enum View { TOP, ANGLED, FIRST_PERSON }

const VIEW_NAMES := ["俯视", "斜俯视", "第一人称"]

@export var view_radius: float = 11.0
@export var min_radius: float = 3.0
@export var max_radius: float = 16.0
@export var camera_fov: float = 45.0
@export var stale_seconds: float = 1.5
## 前几帧按实际最远点自动定视野, 用户滚轮后锁定
@export var auto_fit_frames: int = 5
## 第一人称的测距锥角(度)。激光点比较稀, 太小会经常测不到
@export var cone_degrees: float = 2.5
## 第一人称下墙要像墙; 俯视时矮一些更像"扫描"
@export var fp_wall_height: float = 1.35
@export var top_wall_height: float = 0.30
@export var look_sensitivity: float = 0.0022

var view: View = View.TOP

@onready var _receiver: LidarReceiver = $LidarReceiver
@onready var _cloud: LidarPointCloud = $LidarPoints
@onready var _camera: Camera3D = $Camera3D
@onready var _fp_camera: Camera3D = $Robot/FPCamera
@onready var _hud: LidarHud = $HUD
@onready var _floor: MeshInstance3D = $Floor
@onready var _hit: MeshInstance3D = $HitMarker
@onready var _control: WheeltecControl = $ControlLink

var _last_scan: Dictionary = {}
var _has_scan := false
var _auto_fit_left := 5
var _user_zoomed := false
var _yaw := 0.0
var _pitch := 0.0
var _last_tel: Dictionary = {}
var _tel_msec := 0


func _ready() -> void:
	_auto_fit_left = auto_fit_frames
	_receiver.scan_received.connect(_on_scan)
	_receiver.telemetry_received.connect(_on_telemetry)
	_receiver.status_received.connect(_on_status)
	_hud.view_requested.connect(func(v: int): _set_view(v as View))
	_hud.control_toggled.connect(_control.set_enabled)
	_apply_view()


func _on_scan(scan: Dictionary) -> void:
	_last_scan = scan
	_has_scan = true
	_cloud.apply_scan(scan)
	if not _user_zoomed and _auto_fit_left > 0:
		_auto_fit_left -= 1
		var reference := _frame_distance(scan)
		if reference > 0.5:
			view_radius = clampf(reference * 1.5, min_radius, max_radius)
			_apply_camera()


func _process(_delta: float) -> void:
	_update_hud()
	_update_telemetry_pane()
	_update_control_pane()
	_update_rangefinder()


## bridge 每秒(以及遥控状态一变)会回一包状态, 里面的 control 段就是遥控回执:
## 底盘订阅者数、已发令次数、是否超时归零 —— 这些只有 bridge 知道。
func _on_status(info: Dictionary) -> void:
	var ctrl = info.get("control", {})
	if typeof(ctrl) == TYPE_DICTIONARY:
		_control.apply_bridge_status(ctrl)


func _update_control_pane() -> void:
	_hud.update_control(_control.status())


func _on_telemetry(tel: Dictionary) -> void:
	_last_tel = tel
	_tel_msec = Time.get_ticks_msec()


func _update_telemetry_pane() -> void:
	# 3 秒没新遥测就算中断, 和激光的判定分开(底盘可能比雷达停得早)
	var fresh := _tel_msec > 0 and (Time.get_ticks_msec() - _tel_msec) < 3000
	_last_tel["fresh"] = fresh
	_hud.update_telemetry(_last_tel)


## 视野参考距离: 取有效测距的 95 分位, 而不是最远点。
##
## 用最远点会踩坑: 一两个孤立的远点(玻璃反射、地面回波、对面走廊尽头)就能把
## 视野半径顶到十几米, 结果整面近墙被压成一条细线, 车小得看不见。
## 取分位数则代表"大部分观察落在这个距离内", 对稀疏异常点不敏感。
func _frame_distance(scan: Dictionary) -> float:
	var ranges: PackedFloat32Array = scan.ranges
	var rmax: float = min(scan.range_max, 16.0)
	var valid := PackedFloat32Array()
	for i in ranges.size():
		var r := ranges[i]
		if r > 0.0 and r <= rmax:
			valid.append(r)
	if valid.is_empty():
		return 0.0
	valid.sort()
	var index := int(valid.size() * 0.95)
	return valid[mini(index, valid.size() - 1)]


# ---------------------------------------------------------------------------
# HUD
# ---------------------------------------------------------------------------
func _update_hud() -> void:
	var stale: float = _receiver.seconds_since_scan()
	var connected := _has_scan and stale < stale_seconds
	var status := ""
	if not _receiver.is_listening():
		status = "UDP 绑定失败"
	elif not _has_scan:
		status = "等待 bridge 发数据"
	elif stale >= stale_seconds:
		status = "数据中断 %.1fs" % stale
	else:
		status = "实时 · %s" % _receiver.source if not _receiver.source.is_empty() else "实时"

	_hud.update_state({
		"connected": connected,
		"status_text": status,
		"valid": _cloud.stats["valid"],
		"instances": _cloud.stats["instances"],
		"hz": _receiver.hz,
		"mode": _cloud.mode_name(),
		"range": view_radius,
		"view": VIEW_NAMES[view],
		"source": _receiver.source,
	})


# ---------------------------------------------------------------------------
# 第一人称准星测距
#
# 不做物理射线: 点云没有碰撞体。这里用"视线锥"扫一遍雷达点:
# 把每个点投影到水平面, 取与视线夹角小于 cone_degrees、且距离最近的那个。
# 激光是 2D 的, 所以忽略高度差, 报的是水平距离 —— 和"到那面墙有多远"一致。
# ---------------------------------------------------------------------------
func _update_rangefinder() -> void:
	if view != View.FIRST_PERSON or not _has_scan:
		return
	var forward3 := -_fp_camera.global_transform.basis.z
	var forward := Vector3(forward3.x, 0.0, forward3.z)
	if forward.length() < 0.001:
		_show_no_hit()
		return
	forward = forward.normalized()
	# 视线的方位角, 和点云用同一套约定: 角度 0 = -Z, 逆时针为正
	var view_angle := atan2(-forward.x, -forward.z)

	var ranges: PackedFloat32Array = _last_scan.ranges
	var inc: float = _last_scan.angle_increment
	var amin: float = _last_scan.angle_min
	var rmax: float = min(_last_scan.range_max, 16.0)
	var max_offset := deg_to_rad(cone_degrees)

	# 取"角度上最接近视线"的那条射线。
	# 为什么不取锥内最近点: 稀疏激光一旦正前方有空洞, 最近点会跳到更远的那面墙,
	# 于是会出现"看着不到 1m 却报 2m+"的假读数。按射线取则和雷达自己的测量一一对应。
	var best_offset := INF
	var best_index := -1
	for i in ranges.size():
		var rr := ranges[i]
		if rr <= 0.0 or rr > rmax:
			continue
		var aa := amin + i * inc
		var off := absf(wrapf(aa - view_angle, -PI, PI))
		if off < best_offset:
			best_offset = off
			best_index = i

	if best_index < 0 or best_offset > max_offset:
		_show_no_hit()
		return

	# 报的是雷达原始测距(从车体中心/激光原点算), 可以直接和 ros2 topic echo 对账
	var r := ranges[best_index]
	var a := amin + best_index * inc
	var point := Vector3(-sin(a) * r, 0.0, -cos(a) * r)
	_hit.visible = true
	_hit.global_position = Vector3(point.x, 0.55, point.z)
	_hud.set_distance(r, true, rad_to_deg(best_offset))


func _show_no_hit() -> void:
	_hit.visible = false
	_hud.set_distance(-1.0, false, 0.0)


# ---------------------------------------------------------------------------
# 输入
# ---------------------------------------------------------------------------
func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_M:
				_cloud.set_mode((_cloud.mode + 1) % 3)
				if _has_scan:
					_cloud.apply_scan(_last_scan)
			KEY_V:
				_set_view(((view + 1) % 3) as View)
			KEY_TAB:
				_hud.toggle_manager()
			KEY_ESCAPE:
				if view == View.FIRST_PERSON:
					_set_view(View.TOP)
			KEY_R:
				view_radius = 11.0
				_user_zoomed = false
				_auto_fit_left = auto_fit_frames
				_set_view(View.TOP)
			# 档位和 wheeltec-control 用同一组键, 免得两个入口手感不一致
			KEY_EQUAL, KEY_KP_ADD:
				_control.change_gear(1)
			KEY_MINUS, KEY_KP_SUBTRACT:
				_control.change_gear(-1)
	elif event is InputEventMouseMotion and view == View.FIRST_PERSON:
		_yaw = wrapf(_yaw - event.relative.x * look_sensitivity, -PI, PI)
		_pitch = clampf(_pitch - event.relative.y * look_sensitivity, -1.2, 1.2)
		_fp_camera.rotation = Vector3(_pitch, _yaw, 0.0)
	elif event is InputEventMouseButton and event.pressed:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_user_zoomed = true
			view_radius = clampf(view_radius * 0.88, min_radius, max_radius)
			_apply_camera()
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_user_zoomed = true
			view_radius = clampf(view_radius * 1.14, min_radius, max_radius)
			_apply_camera()


# ---------------------------------------------------------------------------
# 视角
# ---------------------------------------------------------------------------
func _set_view(new_view: View) -> void:
	if view == new_view:
		return
	view = new_view
	if view == View.FIRST_PERSON:
		_yaw = 0.0
		_pitch = 0.0
		_fp_camera.rotation = Vector3.ZERO
		Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	else:
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
	_apply_view()


func _apply_view() -> void:
	# 第一人称需要"墙"的感觉, 俯视时矮一点更像扫描线
	var want := fp_wall_height if view == View.FIRST_PERSON else top_wall_height
	if not is_equal_approx(_cloud.wall_height, want):
		_cloud.wall_height = want
		if _has_scan:
			_cloud.apply_scan(_last_scan)

	var fp := view == View.FIRST_PERSON
	_camera.current = not fp
	_fp_camera.current = fp
	_hud.set_fp(fp)
	# 第一人称像 FPS 一样把自车模型藏起来, 否则镜头会被自己车顶挡住
	for part in [$Robot/Van, $Robot/LidarBase, $Robot/LidarPuck]:
		(part as Node3D).visible = not fp
	if not fp:
		_hit.visible = false
		_apply_camera()


func _apply_camera() -> void:
	# 让半径为 view_radius 的地面圆正好铺满画面高度:
	#   可见高度 = 2 * h * tan(fov/2)  =>  h = R / tan(fov/2)
	var half := deg_to_rad(camera_fov) * 0.5
	var height := view_radius / tan(half)
	_camera.fov = camera_fov
	if view == View.ANGLED:
		_camera.position = Vector3(0.0, height * 0.94, height * 0.34)
	else:
		_camera.position = Vector3(0.0, height, 0.0)
	_camera.look_at(Vector3.ZERO, Vector3(0.0, 0.0, -1.0))

	# 地面的网格/测距环随缩放一起淡出
	if _floor and _floor.material_override is ShaderMaterial:
		(_floor.material_override as ShaderMaterial).set_shader_parameter(
			"fade_radius", view_radius * 2.2)
