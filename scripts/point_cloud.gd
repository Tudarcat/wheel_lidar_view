extends MultiMeshInstance3D
class_name LidarPointCloud
## 雷达点云渲染。
##
## 激光是"每度一个测距值"的散点, 直接一帧画 540 个点会又碎又乱。这里提供三种
## 处理方式, 按 M 切换:
##   RAW    每个有效点一个小方块 (最忠实, 但看起来是散点)
##   VOXEL  按网格合并, 同一格里只留最近的点 (消掉重叠/抖动, 密度均匀)
##   WALLS  沿扫描顺序把相邻点拟合成线段, 再用长条方块表示 (最像墙)
##
## MultiMesh 一次 draw call 画完所有方块, 540~1000 个实例毫无压力。

enum Mode { RAW, VOXEL, WALLS }

@export var capacity: int = 2048
@export var point_size: float = 0.075
@export var wall_height: float = 0.30
@export var voxel_size: float = 0.14
@export var wall_max_gap: float = 0.40   # 相邻点超过这个距离就断开
@export var wall_max_len: float = 1.8    # 单段最长
@export var wall_tolerance: float = 0.07 # 拟合成直线允许的最大偏离
@export var max_range: float = 16.0      # 超过这个距离的点不画

# Kenney Prototype 调色板里挑的三个主色
const COLOR_NEAR := Color("ff7e44")   # 近: 橙红
const COLOR_MID := Color("ffd565")    # 中: 琥珀黄
const COLOR_FAR := Color("6794d9")    # 远: 冷蓝

var mode: Mode = Mode.WALLS
var stats := {"raw": 0, "valid": 0, "instances": 0}

var _mm: MultiMesh
var _cursor: int = 0


func _ready() -> void:
	var box := BoxMesh.new()
	box.size = Vector3.ONE

	_mm = MultiMesh.new()
	_mm.transform_format = MultiMesh.TRANSFORM_3D
	_mm.use_colors = true
	_mm.mesh = box
	_mm.instance_count = capacity
	_mm.visible_instance_count = 0
	multimesh = _mm

	# 无光照 + 按法线做一点点明暗, 保持 Kenney 那种平涂质感
	var mat := ShaderMaterial.new()
	mat.shader = load("res://shaders/box.gdshader")
	material_override = mat


func set_mode(new_mode: Mode) -> void:
	if mode != new_mode:
		mode = new_mode


func mode_name() -> String:
	return ["散点", "网格合并", "墙面合并"][mode]


## 主入口: 用一帧解析好的扫描数据重建全部实例。
## scan 是 lidar_receiver 解出来的字典:
##   { angle_min, angle_increment, range_min, range_max, ranges: PackedFloat32Array }
func apply_scan(scan: Dictionary) -> void:
	var ranges: PackedFloat32Array = scan.ranges
	var angle_min: float = scan.angle_min
	var inc: float = scan.angle_increment
	var rmin: float = scan.range_min
	var rmax: float = min(scan.range_max, max_range)
	var count := ranges.size()

	# 先把有效点抽出来, 后面三种模式都基于它
	var points := PackedVector3Array()
	var dists := PackedFloat32Array()
	points.resize(count)
	dists.resize(count)
	var valid := 0
	for i in count:
		var r := ranges[i]
		if r <= 0.0 or r < rmin or r > rmax:
			continue
		var a := angle_min + i * inc
		# ROS 的 x 朝前、y 朝左; Godot 里 -Z 朝前、+X 朝右。
		# 这样映射之后, 车头朝屏幕上方, 左就是左。
		points[valid] = Vector3(-sin(a) * r, 0.0, -cos(a) * r)
		dists[valid] = r
		valid += 1
	points.resize(valid)
	dists.resize(valid)
	stats["raw"] = count
	stats["valid"] = valid

	_cursor = 0
	match mode:
		Mode.RAW:
			_build_raw(points, dists, rmax)
		Mode.VOXEL:
			_build_voxel(points, dists, rmax)
		Mode.WALLS:
			_build_walls(points, dists, rmax)
	stats["instances"] = _cursor
	_mm.visible_instance_count = min(_cursor, capacity)


# ---------------------------------------------------------------------------
# 三种构建方式
# ---------------------------------------------------------------------------
func _build_raw(points: PackedVector3Array, dists: PackedFloat32Array, rmax: float) -> void:
	for i in points.size():
		var p := points[i]
		_add_box(Vector3(p.x, point_size * 0.5, p.z),
			Vector3(point_size, point_size, point_size), 0.0,
			_color_for(dists[i], rmax))


func _build_voxel(points: PackedVector3Array, dists: PackedFloat32Array, rmax: float) -> void:
	# 同一格里只留离车最近的那个点, 避免多帧抖动叠在一起
	var best := {}
	for i in points.size():
		var p := points[i]
		var key := Vector2i(floori(p.x / voxel_size), floori(p.z / voxel_size))
		if not best.has(key) or dists[i] < best[key]:
			best[key] = dists[i]
	for key in best:
		var center := Vector3((key.x + 0.5) * voxel_size, voxel_size * 0.5,
			(key.y + 0.5) * voxel_size)
		_add_box(center, Vector3(voxel_size, voxel_size, voxel_size), 0.0,
			_color_for(best[key], rmax))


func _build_walls(points: PackedVector3Array, dists: PackedFloat32Array, rmax: float) -> void:
	# 1) 先按扫描顺序把点切成若干"连续段": 相邻点距离超过阈值就断
	var run: PackedVector3Array = PackedVector3Array()
	var run_dists: PackedFloat32Array = PackedFloat32Array()
	var prev := Vector3.INF
	for i in points.size():
		var p := points[i]
		var gap := p.distance_to(prev) if prev != Vector3.INF else 0.0
		if prev == Vector3.INF or gap <= wall_max_gap:
			run.append(p)
			run_dists.append(dists[i])
		else:
			_emit_run(run, run_dists, rmax)
			run = PackedVector3Array([p])
			run_dists = PackedFloat32Array([dists[i]])
		prev = p
	if run.size() > 0:
		_emit_run(run, run_dists, rmax)

	# 2) 散落的孤立点补成小方块, 否则漏掉很细的柱子
	#    (孤立点在 _emit_run 里已经处理过, 这里不重复)


## 把一段连续点做折线简化: 用尽量少的长条覆盖尽量多的点
func _emit_run(run: PackedVector3Array, run_dists: PackedFloat32Array, rmax: float) -> void:
	if run.size() == 0:
		return
	if run.size() == 1:
		_add_box(Vector3(run[0].x, point_size * 0.5, run[0].z),
			Vector3(point_size, point_size, point_size), 0.0,
			_color_for(run_dists[0], rmax))
		return

	var start := 0
	while start < run.size() - 1:
		var best := start + 1
		var probe := start + 2
		while probe < run.size():
			if run[start].distance_to(run[probe]) > wall_max_len:
				break
			if not _fits_line(run, start, probe):
				break
			best = probe
			probe += 1
		_add_segment(run[start], run[best],
			_color_for((run_dists[start] + run_dists[best]) * 0.5, rmax))
		start = best


func _fits_line(points: PackedVector3Array, a: int, b: int) -> bool:
	var pa := points[a]
	var pb := points[b]
	var dir := pb - pa
	var length := dir.length()
	if length < 0.001:
		return true
	dir /= length
	for k in range(a + 1, b):
		var v := points[k] - pa
		var perp := v - dir * v.dot(dir)
		if perp.length() > wall_tolerance:
			return false
	return true


## 在两点之间放一根长条方块
func _add_segment(a: Vector3, b: Vector3, color: Color) -> void:
	var delta := b - a
	var length := delta.length()
	if length < 0.02:
		_add_box(Vector3(b.x, point_size * 0.5, b.z),
			Vector3(point_size, point_size, point_size), 0.0, color)
		return
	var mid := (a + b) * 0.5
	# 段的朝向: 沿着 X 轴拉长, 再绕 Y 转到目标方向
	var yaw := atan2(-delta.z, delta.x)
	_add_box(Vector3(mid.x, wall_height * 0.5, mid.z),
		Vector3(length, wall_height, 0.09), yaw, color)


func _add_box(center: Vector3, size: Vector3, yaw: float, color: Color) -> void:
	if _cursor >= capacity:
		return
	# 注意顺序: 必须 R * S (先按局部轴缩放, 再旋转)。
	# Basis.scaled() 是 S * R (按行缩放), 旋转 90° 时会把长度和宽度互换,
	# 竖直方向的墙段就会塌成 0.09 宽的"点", 看起来像虚线 —— 踩过这个坑。
	var orientation := Basis(Vector3.UP, yaw) * Basis.from_scale(size)
	_mm.set_instance_transform(_cursor, Transform3D(orientation, center))
	_mm.set_instance_color(_cursor, color)
	_cursor += 1


## 距离 → 颜色: 近处橙红, 中段琥珀, 远处冷蓝
func _color_for(distance: float, rmax: float) -> Color:
	var t := clampf(distance / maxf(rmax, 0.001), 0.0, 1.0)
	t = sqrt(t)   # 近处分辨率更高一点
	if t < 0.5:
		return COLOR_NEAR.lerp(COLOR_MID, t * 2.0)
	return COLOR_MID.lerp(COLOR_FAR, (t - 0.5) * 2.0)
