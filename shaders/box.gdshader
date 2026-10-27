shader_type spatial;
render_mode unshaded, cull_back, depth_draw_opaque;

// 平涂方块: 顶点/实例色 × tint × 一点点按法线的明暗。
// 不用光照是为了保持 Kenney 素材那种低多边形平涂质感, 也省掉灯光和阴影开销。
//
//   MultiMesh 点云:  tint = 白色, 颜色走每实例 COLOR
//   车模零件:         COLOR = 白, 颜色走 tint

uniform vec4 tint : source_color = vec4(1.0, 1.0, 1.0, 1.0);
uniform float shade_low = 0.58;
uniform float shade_high = 1.10;
uniform vec3 light_dir = vec3(0.35, 0.90, 0.25);
uniform float emission_boost = 0.0;


void fragment() {
	vec3 n = normalize(NORMAL);
	float d = dot(n, normalize(light_dir)) * 0.5 + 0.5;
	vec3 albedo = COLOR.rgb * tint.rgb * mix(shade_low, shade_high, d);
	ALBEDO = albedo;
	EMISSION = albedo * emission_boost;
}
