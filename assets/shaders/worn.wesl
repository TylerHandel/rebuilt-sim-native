// Wear on painted and plastic parts (field elements, robots' plates and prints): no surface is
// perfectly even. Its color and gloss vary a few percent over a few centimeters (in the part's own
// frame, so it turns with the part), and near the carpet, where wheels, bumpers and FUEL rub, it's
// scuffed: a little darker and duller in patches. Cheap: three noise lookups a pixel.
import bevy_pbr::render::{
    forward_io::{VertexOutput, FragmentOutput},
    pbr_fragment::pbr_input_from_standard_material,
    pbr_functions::{alpha_discard, apply_pbr_lighting, main_pass_post_lighting_processing},
    mesh_functions::{get_local_from_world, get_world_from_local},
};

@group(constants::MATERIAL_BIND_GROUP) @binding(100) var<uniform> wear: f32;

fn h3(p: vec3<f32>) -> f32 {
    var q = fract(p * 0.3183099 + vec3<f32>(0.1, 0.2, 0.3));
    q *= 17.0;
    return fract(q.x * q.y * q.z * (q.x + q.y + q.z));
}
fn n3(x: vec3<f32>) -> f32 {
    let i = floor(x);
    var f = fract(x);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(h3(i), h3(i + vec3(1.0, 0.0, 0.0)), f.x), mix(h3(i + vec3(0.0, 1.0, 0.0)), h3(i + vec3(1.0, 1.0, 0.0)), f.x), f.y),
               mix(mix(h3(i + vec3(0.0, 0.0, 1.0)), h3(i + vec3(1.0, 0.0, 1.0)), f.x), mix(h3(i + vec3(0.0, 1.0, 1.0)), h3(i + vec3(1.0, 1.0, 1.0)), f.x), f.y), f.z);
}

@fragment
fn fragment(in: VertexOutput, @builtin(front_facing) is_front: bool) -> FragmentOutput {
    var pbr_input = pbr_input_from_standard_material(in, is_front);
    pbr_input.material.base_color = alpha_discard(pbr_input.material.flags, pbr_input.material.alpha_cutoff, pbr_input.material.base_color);

    let lfw = get_local_from_world(in.instance_index);
    let wfl = get_world_from_local(in.instance_index);
    let s = vec3<f32>(length(wfl[0].xyz), length(wfl[1].xyz), length(wfl[2].xyz));
    let p = (lfw * in.world_position).xyz * s;

    // (blotches a few cm across, and finer mottling, faded where finer than the pixels)
    let big = n3(p * 9.0);
    let fine = mix(0.5, n3(p * 60.0), clamp(1.5 - length(fwidth(p * 60.0)), 0.0, 1.0));
    let v = 0.6 * big + 0.4 * fine;
    // (scuffs within ~20 cm of the carpet, in patches)
    let low = clamp(1.0 - in.world_position.y / 0.22, 0.0, 1.0);
    let scuff = low * low * smoothstep(0.45, 0.75, n3(in.world_position.xyz * vec3<f32>(6.0, 14.0, 6.0)));

    let c = pbr_input.material.base_color;
    let k = (1.0 + wear * (0.07 * v - 0.035)) * (1.0 - 0.18 * wear * scuff);
    pbr_input.material.base_color = vec4<f32>(c.rgb * k, c.a);
    pbr_input.material.perceptual_roughness = clamp(pbr_input.material.perceptual_roughness + wear * ((v - 0.5) * 0.12 + 0.15 * scuff), 0.05, 1.0);

    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
}
