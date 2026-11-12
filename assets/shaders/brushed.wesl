// Brushed aluminum (js robotModels.js `brushed`): a fine grain across each face's longest way in its
// plane, in the part's own frame (so it turns with the part), lines ~3 mm apart, each its own shade
// and wandering a little, faded out where they're finer than the pixels (so it doesn't shimmer far
// off). It shades the base color a few percent and the roughness with it: the arena's reflections
// break up along the grain as on real brushed stock. grain: how much shows (anodizing mutes it).
import bevy_pbr::render::{
    forward_io::{VertexOutput, FragmentOutput},
    pbr_fragment::pbr_input_from_standard_material,
    pbr_functions::{alpha_discard, apply_pbr_lighting, main_pass_post_lighting_processing},
    mesh_functions::{get_local_from_world, get_world_from_local},
};

@group(constants::MATERIAL_BIND_GROUP) @binding(100) var<uniform> grain: f32;

fn alu_hash(n: f32) -> f32 { return fract(sin(n) * 43758.5453); }
fn alu_noise(x: f32) -> f32 {
    let i = floor(x);
    var f = fract(x);
    f = f * f * (3.0 - 2.0 * f);
    return mix(alu_hash(i), alu_hash(i + 1.0), f);
}

// The CAD's sharp edges rounded (crates/app/src/bevel.rs works out, per triangle, the distance to
// each of its edges, uv.x, uv.y, uv_b.x, and how sharply each turns, uv_b.y): near a sharp edge the
// normal leans over toward it as on a fillet, halfway to the next face's at the edge itself. The
// fillet's a few mm wide close up, a pixel and a half far off (no finer: it would shimmer), at most
// 12 mm. A few dozen operations a pixel; nothing where the mesh has no sharp edge.
fn bevel(in: VertexOutput, is_front: bool, n: vec3<f32>) -> vec3<f32> {
    var out = n;
@if(VERTEX_UVS_A && VERTEX_UVS_B) {
    let px = dpdx(in.world_position.xyz);
    let py = dpdy(in.world_position.xyz);
    let d = vec3<f32>(in.uv.x, in.uv.y, in.uv_b.x);
    let dx = dpdx(d);
    let dy = dpdy(d);
    let code = u32(round(in.uv_b.y));
    let a = dot(px, px);
    let b = dot(px, py);
    let c = dot(py, py);
    let det = a * c - b * b;
    let w = clamp(1.5 * max(length(px), length(py)), 0.004, 0.012);
    if (code != 0u && det > 1e-24) {
        // (seen from behind, a convex edge is a concave one)
        let side = select(-0.25, 0.25, is_front);
        for (var i = 0u; i < 3u; i++) {
            let s = f32(i32((code >> (4u * i)) & 15u) - 8) * side;
            if (s == 0.0) { continue; }
            // (the edge distance's gradient in the world: in the face's plane, matching its
            // change across the pixel both ways)
            let ka = (c * dx[i] - b * dy[i]) / det;
            let kb = (a * dy[i] - b * dx[i]) / det;
            let g = ka * px + kb * py;
            let gl = length(g);
            if (gl <= 0.0) { continue; }
            let t = 1.0 - d[i] / gl / w;
            if (t <= 0.0) { continue; }
            out -= g / gl * s * t;
        }
        out = normalize(out);
    }
}
    return out;
}

@fragment
fn fragment(in: VertexOutput, @builtin(front_facing) is_front: bool) -> FragmentOutput {
    var pbr_input = pbr_input_from_standard_material(in, is_front);
    pbr_input.material.base_color = alpha_discard(pbr_input.material.flags, pbr_input.material.alpha_cutoff, pbr_input.material.base_color);
    pbr_input.N = bevel(in, is_front, pbr_input.N);

    // (the part's own frame, in meters: its mesh's coordinates times its scale)
    let lfw = get_local_from_world(in.instance_index);
    let wfl = get_world_from_local(in.instance_index);
    let s = vec3<f32>(length(wfl[0].xyz), length(wfl[1].xyz), length(wfl[2].xyz));
    let p = (lfw * in.world_position).xyz * s;
    let n = abs(normalize((lfw * vec4<f32>(in.world_normal, 0.0)).xyz));
    var uv = p.xy;
    if (n.y > n.x && n.y > n.z) { uv = p.xz; } else if (n.x > n.z) { uv = p.zy; }

    let line = uv.y * 320.0 + alu_noise(uv.x * 5.0) * 4.0;
    var g = 0.65 * alu_noise(line) + 0.35 * alu_noise(line * 2.7 + 17.0);
    g = mix(g, alu_noise(uv.x * 30.0 + floor(line) * 7.13), 0.25);
    let fw = fwidth(line);
    g = mix(0.5, g, clamp(1.5 - fw, 0.0, 1.0));

    let c = pbr_input.material.base_color;
    pbr_input.material.base_color = vec4<f32>(c.rgb * (1.0 + grain * (0.14 * g - 0.07)), c.a);
    pbr_input.material.perceptual_roughness = clamp(pbr_input.material.perceptual_roughness + (g - 0.5) * 0.2 * grain, 0.05, 1.0);

    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
}
