extends CanvasLayer
class_name LidarHud
## 三个浮窗的调度器:
##   雷达 3D 视图  —— 点云/帧率/合并方式/量程/视角/数据源
##   基本信息      —— 线速度/角速度/加速度/位姿/电压
##   管理          —— 面板显隐开关 + 视角按钮(Tab 呼出)
##
## 窗口外壳(标题栏、拖动、折叠、关闭)由 scripts/ui_window.gd 提供。
##
## 注意: 这里用 find_child() 而不是 %UniqueName。原因是被 main.tscn 实例化的
## 场景根节点 owner 为 null, "%" 是按 owner 链查找的, 在这个位置找不到。

signal view_requested(view: int)

const C_OK := Color(0.55, 0.9, 0.6)
const C_BAD := Color(0.95, 0.45, 0.45)
const C_WARN := Color(0.94, 0.78, 0.45)
const C_WAIT := Color(0.65, 0.65, 0.65)
const C_DIST_HIT := Color(0.62, 0.88, 1.0)
const C_DIST_MISS := Color(0.65, 0.65, 0.65)

@onready var _lidar_pane: UiWindow = $LidarPane
@onready var _chassis_pane: UiWindow = $ChassisPane
@onready var _manager: UiWindow = $ManagerPane

# ---- 雷达 ----
@onready var _lidar_dot: Label = _find(_lidar_pane, "LidarDot")
@onready var _lidar_text: Label = _find(_lidar_pane, "LidarText")
@onready var _value_points: Label = _find(_lidar_pane, "ValuePoints")
@onready var _value_hz: Label = _find(_lidar_pane, "ValueHz")
@onready var _value_mode: Label = _find(_lidar_pane, "ValueMode")
@onready var _value_range: Label = _find(_lidar_pane, "ValueRange")
@onready var _value_view: Label = _find(_lidar_pane, "ValueView")
@onready var _value_source: Label = _find(_lidar_pane, "ValueSource")

# ---- 底盘 ----
@onready var _chassis_dot: Label = _find(_chassis_pane, "ChassisDot")
@onready var _chassis_text: Label = _find(_chassis_pane, "ChassisText")
@onready var _big_speed: Label = _find(_chassis_pane, "BigSpeed")
@onready var _big_omega: Label = _find(_chassis_pane, "BigOmega")
@onready var _value_accel: Label = _find(_chassis_pane, "ValueAccel")
@onready var _value_pos: Label = _find(_chassis_pane, "ValuePos")
@onready var _value_yaw: Label = _find(_chassis_pane, "ValueYaw")
@onready var _value_volt: Label = _find(_chassis_pane, "ValueVolt")
@onready var _value_imu: Label = _find(_chassis_pane, "ValueImu")

# ---- 管理 ----
@onready var _toggle_lidar: CheckButton = _find(_manager, "ToggleLidar")
@onready var _toggle_chassis: CheckButton = _find(_manager, "ToggleChassis")
@onready var _btn_top: Button = _find(_manager, "BtnTop")
@onready var _btn_angled: Button = _find(_manager, "BtnAngled")
@onready var _btn_fp: Button = _find(_manager, "BtnFp")

# ---- 准星 ----
@onready var _crosshair: Control = $Crosshair
@onready var _hbar: ColorRect = $Crosshair/HBar
@onready var _vbar: ColorRect = $Crosshair/VBar
@onready var _cross_dot: ColorRect = $Crosshair/CrossDot
@onready var _dist: Label = $Crosshair/Dist
@onready var _tip: Label = $Crosshair/Tip


func _find(root: Node, name_: String) -> Node:
	return root.find_child(name_, true, false)


func _ready() -> void:
	layer = 10
	_toggle_lidar.toggled.connect(func(on: bool): _lidar_pane.visible = on)
	_toggle_chassis.toggled.connect(func(on: bool): _chassis_pane.visible = on)
	_lidar_pane.closed.connect(func(): _toggle_lidar.set_pressed_no_signal(false))
	_chassis_pane.closed.connect(func(): _toggle_chassis.set_pressed_no_signal(false))

	_btn_top.pressed.connect(func(): view_requested.emit(0))
	_btn_angled.pressed.connect(func(): view_requested.emit(1))
	_btn_fp.pressed.connect(func(): view_requested.emit(2))

	_dist.add_theme_font_size_override("font_size", 17)
	_tip.add_theme_font_size_override("font_size", 11)
	_tip.add_theme_color_override("font_color", C_DIST_MISS)
	set_fp(false)
	update_telemetry({})


# ---------------------------------------------------------------------------
func toggle_manager() -> void:
	_manager.visible = not _manager.visible


func is_manager_visible() -> bool:
	return _manager.visible


# ---------------------------------------------------------------------------
# 雷达窗格
# ---------------------------------------------------------------------------
func update_state(state: Dictionary) -> void:
	var connected: bool = state.get("connected", false)
	_lidar_dot.add_theme_color_override("font_color", C_OK if connected else C_BAD)
	_lidar_text.text = state.get("status_text", "")
	_lidar_text.add_theme_color_override("font_color", C_OK if connected else C_WARN)

	_value_points.text = "%d / %d" % [state.get("instances", 0), state.get("valid", 0)]
	_value_hz.text = "%.1f Hz" % state.get("hz", 0.0)
	_value_mode.text = state.get("mode", "—")
	_value_range.text = "%.0f m" % state.get("range", 0.0)
	_value_view.text = state.get("view", "—")
	_value_source.text = _shorten(state.get("source", ""))


# ---------------------------------------------------------------------------
# 底盘窗格
# ---------------------------------------------------------------------------
func update_telemetry(tel: Dictionary) -> void:
	var flags: int = tel.get("flags", 0)
	var has_odom := (flags & 1) != 0
	var has_imu := (flags & 2) != 0
	var has_batt := (flags & 4) != 0
	var fresh: bool = tel.get("fresh", false)

	if not has_odom and not has_imu and not has_batt:
		_chassis_dot.add_theme_color_override("font_color", C_WAIT)
		_chassis_text.text = "无遥测数据"
		_chassis_text.add_theme_color_override("font_color", C_WAIT)
	elif not fresh:
		_chassis_dot.add_theme_color_override("font_color", C_WARN)
		_chassis_text.text = "遥测中断"
		_chassis_text.add_theme_color_override("font_color", C_WARN)
	else:
		_chassis_dot.add_theme_color_override("font_color", C_OK)
		var parts := []
		if has_odom:
			parts.append("里程计")
		if has_imu:
			parts.append("IMU")
		if has_batt:
			parts.append("电压")
		_chassis_text.text = "在线 · " + "、".join(parts)
		_chassis_text.add_theme_color_override("font_color", C_OK)

	_big_speed.text = "%.2f" % tel.get("vel_x", 0.0) if has_odom else "—"
	_big_omega.text = "%.2f" % tel.get("omega", 0.0) if has_odom else "—"

	if has_imu:
		var ax: float = tel.get("acc_x", 0.0)
		var ay: float = tel.get("acc_y", 0.0)
		_value_accel.text = "%.2f" % sqrt(ax * ax + ay * ay)
		_value_imu.text = "%.2f" % tel.get("acc_z", 0.0)
	else:
		_value_accel.text = "—"
		_value_imu.text = "—"

	if has_odom:
		_value_pos.text = "%.2f, %.2f" % [tel.get("pos_x", 0.0), tel.get("pos_y", 0.0)]
		_value_yaw.text = "%.1f°" % rad_to_deg(tel.get("yaw", 0.0))
	else:
		_value_pos.text = "—"
		_value_yaw.text = "—"

	_value_volt.text = "%.2f V" % tel.get("voltage", 0.0) if has_batt else "—"


# ---------------------------------------------------------------------------
# 准星
# ---------------------------------------------------------------------------
func set_fp(on: bool) -> void:
	_crosshair.visible = on
	if on:
		set_distance(-1.0, false)


func set_distance(distance: float, hit: bool, offset_deg: float = 0.0) -> void:
	var color := C_DIST_HIT if hit else C_DIST_MISS
	if hit:
		_dist.text = "%.2f m" % distance
		_tip.text = "偏角 %+.1f° · 到雷达" % offset_deg
	else:
		_dist.text = "— —"
		_tip.text = "对准墙壁测距"
	_dist.add_theme_color_override("font_color", color)
	_tip.add_theme_color_override("font_color", color)
	for bar in [_hbar, _vbar, _cross_dot]:
		(bar as ColorRect).color = color


func _shorten(text: String) -> String:
	return text if text.length() <= 30 else text.substr(0, 29) + "…"
