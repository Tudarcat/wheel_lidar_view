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
## 遥控开关被拨动(或遥控窗被关掉)时通知主控, 由主控去使能 ControlLink
signal control_toggled(on: bool)

const C_OK := Color(0.55, 0.9, 0.6)
const C_BAD := Color(0.95, 0.45, 0.45)
const C_WARN := Color(0.94, 0.78, 0.45)
const C_WAIT := Color(0.65, 0.65, 0.65)
const C_DIST_HIT := Color(0.62, 0.88, 1.0)
const C_DIST_MISS := Color(0.65, 0.65, 0.65)
const C_INFO := Color(0.62, 0.88, 1.0)
## 按键灯: 没按 / 按着但没发令 / 按着且真的在发令
const C_KEY_IDLE := Color(1, 1, 1, 0.06)
const C_KEY_DOWN := Color(0.29, 0.55, 0.81, 0.95)
const C_KEY_LIVE := Color(1, 0.75, 0.27, 0.95)

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
@onready var _toggle_ctrl_pane: CheckButton = _find(_manager, "ToggleCtrl")

# ---- 遥控 ----
@onready var _ctrl_pane: UiWindow = $ControlPane
@onready var _ctrl_dot: Label = _find(_ctrl_pane, "CtrlDot")
@onready var _ctrl_text: Label = _find(_ctrl_pane, "CtrlText")
@onready var _switch_ctrl: CheckButton = _find(_ctrl_pane, "SwitchControl")
@onready var _value_lin: Label = _find(_ctrl_pane, "ValueLin")
@onready var _value_ang: Label = _find(_ctrl_pane, "ValueAng")
@onready var _value_gear: Label = _find(_ctrl_pane, "ValueGear")
@onready var _value_backend: Label = _find(_ctrl_pane, "ValueBackend")
@onready var _value_subs: Label = _find(_ctrl_pane, "ValueSubs")
@onready var _value_sent: Label = _find(_ctrl_pane, "ValueSent")
@onready var _keys := {
	"w": _find(_ctrl_pane, "KeyW"),
	"s": _find(_ctrl_pane, "KeyS"),
	"a": _find(_ctrl_pane, "KeyA"),
	"d": _find(_ctrl_pane, "KeyD"),
}

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

	_switch_ctrl.toggled.connect(func(on: bool): control_toggled.emit(on))
	_toggle_ctrl_pane.toggled.connect(func(on: bool): _ctrl_pane.visible = on)
	# 关掉遥控窗必须同时断开遥控: 否则开关被藏起来了, 车却还在动
	_ctrl_pane.closed.connect(func():
		_switch_ctrl.set_pressed_no_signal(false)
		_toggle_ctrl_pane.set_pressed_no_signal(false)
		control_toggled.emit(false))

	_dist.add_theme_font_size_override("font_size", 17)
	_tip.add_theme_font_size_override("font_size", 11)
	_tip.add_theme_color_override("font_color", C_DIST_MISS)
	set_fp(false)
	update_telemetry({})
	update_control({})


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
	# 数据源名可能很长(bag:xxx.db3//scan), 不截短会把窗格顶宽
	_lidar_text.text = _shorten(state.get("status_text", ""), 24)
	_lidar_text.add_theme_color_override("font_color", C_OK if connected else C_WARN)

	_value_points.text = "%d / %d" % [state.get("instances", 0), state.get("valid", 0)]
	_value_hz.text = "%.1f Hz" % state.get("hz", 0.0)
	_value_mode.text = state.get("mode", "—")
	_value_range.text = "%.0f m" % state.get("range", 0.0)
	_value_view.text = state.get("view", "—")
	_value_source.text = _shorten(state.get("source", ""), 26)


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


func _shorten(text: String, limit: int = 30) -> String:
	return text if text.length() <= limit else text.substr(0, limit - 1) + "…"


# ---------------------------------------------------------------------------
# 遥控窗格
# ---------------------------------------------------------------------------
## st 来自 WheeltecControl.status(): 本机状态 + bridge 回执(bridge 段)
func update_control(st: Dictionary) -> void:
	var on: bool = st.get("enabled", false)
	var bridge: Dictionary = st.get("bridge", {})
	var keys: Dictionary = st.get("keys", {})
	var estop: bool = st.get("estop", false)
	# "真的在发令"= 开关开着 + bridge 收到了包 + 命令非零
	var live: bool = on and bridge.get("active", false)

	for key in ["w", "s", "a", "d"]:
		_style_key(_keys[key], keys.get(key, false), live)

	var subs: int = bridge.get("subs", 0)
	var backend := String(bridge.get("backend", ""))
	var has_ros := backend.begins_with("ros2:")
	if not on:
		_set_status_row(C_WAIT, "遥控关闭 · 不发令")
	elif estop:
		_set_status_row(C_BAD, "急停中 · 松开空格恢复")
	elif bridge.is_empty():
		_set_status_row(C_WARN, "已开启 · 等 bridge 回执")
	elif bridge.get("stale", false):
		_set_status_row(C_WARN, "已开启 · 正在归零")
	elif not bridge.get("fresh", false):
		_set_status_row(C_BAD, "已开启 · bridge 没收到包")
	elif not has_ros:
		# demo / bag 没有 ROS 出口, "订阅者" 这个概念不存在, 不能报成故障
		_set_status_row(C_INFO, "发令中 · 模拟底盘")
	elif subs > 0:
		_set_status_row(C_OK, "发令中 · 底盘已订阅")
	else:
		_set_status_row(C_WARN, "发令中 · 无订阅者")

	_value_lin.text = "%+.2f m/s" % st.get("linear", 0.0)
	_value_ang.text = "%+.2f rad/s" % st.get("angular", 0.0)
	_value_gear.text = "%d / %d" % [st.get("gear", 1), st.get("gear_count", 4)]
	_value_backend.text = _shorten(String(bridge.get("backend", "—")), 18)
	_value_subs.text = "%d" % subs if has_ros else "—"
	_value_sent.text = "—" if bridge.is_empty() else "%d / %d" % [
		st.get("sent", 0), bridge.get("published", 0)]


func _set_status_row(color: Color, text: String) -> void:
	_ctrl_dot.add_theme_color_override("font_color", color)
	_ctrl_text.text = text
	_ctrl_text.add_theme_color_override("font_color", color)


## 按键灯。颜色编码: 没按=浅灰底, 按着但不会发令=蓝, 按着且正在发令=琥珀。
## 用 meta 缓存上一次的颜色, 否则每帧都新建 StyleBoxFlat。
func _style_key(panel: PanelContainer, pressed: bool, live: bool) -> void:
	var color := C_KEY_IDLE
	if pressed:
		color = C_KEY_LIVE if live else C_KEY_DOWN
	if panel.has_meta("key_color") and panel.get_meta("key_color") == color:
		return
	panel.set_meta("key_color", color)

	var sb := StyleBoxFlat.new()
	sb.bg_color = color
	sb.set_corner_radius_all(5)
	sb.content_margin_left = 4
	sb.content_margin_right = 4
	sb.content_margin_top = 2
	sb.content_margin_bottom = 2
	panel.add_theme_stylebox_override("panel", sb)

	var label := panel.get_child(0) as Label
	if label:
		label.add_theme_font_size_override("font_size", 17)
		label.add_theme_color_override(
			"font_color", Color(1, 1, 1, 0.95) if pressed else Color(0.86, 0.89, 0.95))
