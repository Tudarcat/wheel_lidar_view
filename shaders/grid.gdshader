shader_type spatial;
render_mode unshaded, cull_disabled, depth_draw_opaque;

// 地面用: 米制网格 + 2m 同心测距环 + 中轴 + 半径外淡出。
// 这样不用额外几何体就能给出距离参照。

uniform vec4 base_color : source_color = vec4(0.043, 0.055, 0.075, 1.0);
uniform vec4 line_color : source_color = vec4(0.145, 0.180, 0.235, 1.0);
uniform vec4 ring_color : source_color = vec4(0.110, 0.140, 0.185, 1.0);
uniform vec4 axis_x_color : source_color = vec4(0.28, 0.34, 0.42, 1.0);
uniform vec4 axis_z_color : source_color = vec4(0.28, 0.34, 0.42, 1.0);
uniform float cell = 1.0;
uniform float line_width = 0.016;
uniform float ring_step = 2.0;
uniform float fade_radius = 22.0;

varying vec3 world_pos;

void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec2 p = world_pos.xz;
	float r = length(p);

	// 网格线
	vec2 f = abs(fract(p / cell + 0.5) - 0.5) * cell;
	float dgrid = min(f.x, f.y);
	float grid = 1.0 - smoothstep(0.0, line_width, dgrid);

	// 同心环(每 ring_step 米)
	float dr = abs(fract(r / ring_step + 0.5) - 0.5) * ring_step;
	float ring = 1.0 - smoothstep(0.0, line_width * 1.2, dr);

	// 中轴
	float ax = 1.0 - smoothstep(0.0, line_width * 1.5, abs(p.x));
	float az = 1.0 - smoothstep(0.0, line_width * 1.5, abs(p.y));

	vec3 color = base_color.rgb;
	color = mix(color, line_color.rgb, grid);
	color = mix(color, ring_color.rgb, ring * 0.85);
	color = mix(color, axis_x_color.rgb, az * 0.55);   // 车头方向 (沿 -Z)
	color = mix(color, axis_z_color.rgb, ax * 0.45);

	// 远处淡出到背景色, 免得看到平面边缘
	float fade = 1.0 - smoothstep(fade_radius * 0.55, fade_radius, r);
	ALBEDO = mix(base_color.rgb * 0.6, color, fade);
}
