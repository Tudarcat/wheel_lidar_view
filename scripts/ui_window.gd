extends PanelContainer
class_name UiWindow
## 通用浮窗组件: 蓝色标题栏 + 深色面板 + 可拖拽 + 可折叠 + 可关闭。
##
## 样式照搬参考工程的 Solver 面板(assets/styles/titlebar_blue.tres 等):
##   标题栏  Color(0.22, 0.36, 0.52)   圆角 6/6/0/0
##   面板    Color(0.09, 0.09, 0.11, 0.88) + 阴影
##   分区    白色 4% 叠一层, 圆角 4
##   标签    近白 / 数值 亮蓝 / 小标题 灰
##
## 用法: 场景根节点用 PanelContainer 挂这个脚本, 里面放一个名为 "Content" 的
## VBoxContainer 装内容; 命名约定决定上色:
##   Hdr*   分区小标题(灰, 12)
##   K*     键名(近白, 13)
##   V*     数值(亮蓝, 13)
##   Big*   大号数值(亮蓝, 20)
##   Cap*   大号数值上方的小标签(灰, 11)

signal closed

const C_PANEL_BG := Color(0.09, 0.09, 0.11, 0.88)
const C_TITLE_BG := Color(0.22, 0.36, 0.52, 1.0)
const C_TITLE_TEXT := Color(0.92, 0.95, 1.0)
const C_SECTION_BG := Color(1, 1, 1, 0.04)
const C_LABEL := Color(0.86, 0.89, 0.95)
const C_VALUE := Color(0.62, 0.88, 1.0)
const C_MUTED := Color(0.72, 0.76, 0.84)
const C_OK := Color(0.55, 0.9, 0.6)
const C_BAD := Color(0.95, 0.45, 0.45)
const C_WARN := Color(0.94, 0.78, 0.45)

@export var title := "面板"        : set = _set_title
@export var closable := false
@export var collapsible := true
@export var start_collapsed := false

var body: VBoxContainer
var title_label: Label

var _content: Control
var _holder: MarginContainer
var _collapse_btn: Button
var _dragging := false
var _collapsed := false


func _ready() -> void:
	var panel := StyleBoxFlat.new()
	panel.bg_color = C_PANEL_BG
	panel.set_corner_radius_all(6)
	panel.shadow_color = Color(0, 0, 0, 0.4)
	panel.shadow_size = 4
	panel.shadow_offset = Vector2(0, 2)
	add_theme_stylebox_override("panel", panel)

	_content = get_node_or_null("Content")
	if _content:
		remove_child(_content)

	var root := VBoxContainer.new()
	root.add_theme_constant_override("separation", 0)
	add_child(root)
	root.add_child(_build_title_bar())

	_holder = MarginContainer.new()
	for side in ["left", "right"]:
		_holder.add_theme_constant_override("margin_" + side, 8)
	_holder.add_theme_constant_override("margin_top", 6)
	_holder.add_theme_constant_override("margin_bottom", 8)
	root.add_child(_holder)

	if _content:
		_holder.add_child(_content)
		body = _content as VBoxContainer
	else:
		body = VBoxContainer.new()
		body.add_theme_constant_override("separation", 6)
		_holder.add_child(body)

	_style_tree(self)
	if start_collapsed:
		_toggle_collapse()


func _set_title(value: String) -> void:
	title = value
	if title_label:
		title_label.text = value


func _build_title_bar() -> Control:
	var bar := PanelContainer.new()
	var sb := StyleBoxFlat.new()
	sb.bg_color = C_TITLE_BG
	sb.set_corner_radius_all(6)
	sb.corner_radius_bottom_left = 0
	sb.corner_radius_bottom_right = 0
	sb.content_margin_left = 10
	sb.content_margin_top = 4
	sb.content_margin_right = 6
	sb.content_margin_bottom = 4
	bar.add_theme_stylebox_override("panel", sb)

	var header := HBoxContainer.new()
	header.add_theme_constant_override("separation", 4)
	header.mouse_filter = Control.MOUSE_FILTER_STOP
	bar.add_child(header)

	title_label = Label.new()
	title_label.text = title
	title_label.add_theme_font_size_override("font_size", 16)
	title_label.add_theme_color_override("font_color", C_TITLE_TEXT)
	title_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	title_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	header.add_child(title_label)

	if collapsible:
		_collapse_btn = _icon_button("−", 16)
		_collapse_btn.pressed.connect(_toggle_collapse)
		header.add_child(_collapse_btn)

	if closable:
		var close := _icon_button("✕", 14)
		close.pressed.connect(func():
			visible = false
			closed.emit())
		header.add_child(close)

	header.gui_input.connect(_on_header_input)
	return bar


func _icon_button(text: String, size: int) -> Button:
	var btn := Button.new()
	btn.text = text
	btn.flat = true
	btn.focus_mode = Control.FOCUS_NONE
	btn.custom_minimum_size = Vector2(22, 20)
	btn.add_theme_font_size_override("font_size", size)
	btn.add_theme_color_override("font_color", Color(0.85, 0.9, 1.0, 0.75))
	btn.add_theme_color_override("font_hover_color", Color(1, 1, 1, 1))
	return btn


# ---------------------------------------------------------------------------
func _on_header_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		_dragging = event.pressed
	elif event is InputEventMouseMotion and _dragging:
		var limit := get_viewport().get_visible_rect().size - size
		position = (position + event.relative).clamp(Vector2.ZERO, limit)


func _toggle_collapse() -> void:
	if not collapsible:
		return
	_collapsed = not _collapsed
	_holder.visible = not _collapsed
	if _collapse_btn:
		_collapse_btn.text = "+" if _collapsed else "−"
	reset_size()


func is_collapsed() -> bool:
	return _collapsed


## 按命名约定给整棵子树配色
func _style_tree(node: Node) -> void:
	for child in node.get_children():
		if child is PanelContainer and String(child.name).begins_with("Section"):
			var sb := StyleBoxFlat.new()
			sb.bg_color = C_SECTION_BG
			sb.set_corner_radius_all(4)
			sb.content_margin_left = 8
			sb.content_margin_top = 5
			sb.content_margin_right = 8
			sb.content_margin_bottom = 5
			(child as PanelContainer).add_theme_stylebox_override("panel", sb)
		elif child is Label:
			_style_label(child as Label)
		elif child is Button and not (child is CheckButton) and not (child as Button).flat:
			_style_button(child as Button)
		if child.get_child_count() > 0:
			_style_tree(child)


func _style_button(btn: Button) -> void:
	for state in ["normal", "hover", "pressed"]:
		var sb := StyleBoxFlat.new()
		match state:
			"normal":
				sb.bg_color = Color(1, 1, 1, 0.06)
			"hover":
				sb.bg_color = Color(1, 1, 1, 0.13)
			_:
				sb.bg_color = Color(0.22, 0.36, 0.52, 0.9)
		sb.set_corner_radius_all(4)
		sb.content_margin_left = 6
		sb.content_margin_top = 3
		sb.content_margin_right = 6
		sb.content_margin_bottom = 3
		btn.add_theme_stylebox_override(state, sb)
	btn.add_theme_font_size_override("font_size", 12)
	btn.add_theme_color_override("font_color", C_LABEL)
	btn.add_theme_color_override("font_hover_color", Color(1, 1, 1, 1))
	btn.add_theme_color_override("font_pressed_color", Color(1, 1, 1, 1))


func _style_label(label: Label) -> void:
	var name_ := String(label.name)
	if name_ == "Hdr":
		label.add_theme_font_size_override("font_size", 12)
		label.add_theme_color_override("font_color", C_MUTED)
	elif name_.begins_with("Big"):
		label.add_theme_font_size_override("font_size", 20)
		label.add_theme_color_override("font_color", C_VALUE)
	elif name_.begins_with("Cap"):
		label.add_theme_font_size_override("font_size", 11)
		label.add_theme_color_override("font_color", C_MUTED)
	elif name_.begins_with("K"):
		label.add_theme_font_size_override("font_size", 13)
		label.add_theme_color_override("font_color", C_LABEL)
	elif name_.begins_with("V"):
		label.add_theme_font_size_override("font_size", 13)
		label.add_theme_color_override("font_color", C_VALUE)
	elif name_.begins_with("Hint"):
		label.add_theme_font_size_override("font_size", 11)
		label.add_theme_color_override("font_color", C_MUTED)
