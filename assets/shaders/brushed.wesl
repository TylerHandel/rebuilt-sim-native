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

@fragment
fn fragment(in: VertexOutput, @builtin(front_facing) is_front: bool) -> FragmentOutput {
    var pbr_input = pbr_input_from_standard_material(in, is_front);
    pbr_input.material.base_color = alpha_discard(pbr_input.material.flags, pbr_input.material.alpha_cutoff, pbr_input.material.base_color);

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
