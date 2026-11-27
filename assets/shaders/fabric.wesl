// Bumper cloth (the cordura over the bumpers' foam and wood): never one flat color. A woven grain
// close up (faded out where it's finer than the pixels), the cloth pulled a little uneven over the
// foam, its stitched seams along the top and bottom edges (a fold and a darker line of stitching),
// and the rubs a season of matches leaves: fuzzed, lighter, duller patches, most at the corners
// and along the bottom where it meets the carpet, FUEL and other robots.
import bevy_pbr::render::{
    forward_io::{VertexOutput, FragmentOutput},
    pbr_fragment::pbr_input_from_standard_material,
    pbr_functions::{alpha_discard, apply_pbr_lighting, main_pass_post_lighting_processing},
    mesh_functions::{get_local_from_world, get_world_from_local},
};

// x: how worn (0..1); y, z: the bumpers' bottom and top (world y, standing on the carpet)
@group(constants::MATERIAL_BIND_GROUP) @binding(100) var<uniform> fab: vec4<f32>;

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
    // (in the robot's own frame, so it turns with it)
    let p = (lfw * in.world_position).xyz * s;
    let n = normalize((lfw * vec4<f32>(pbr_input.N, 0.0)).xyz);
    // the cloth's own two directions: along the bumper (round it) and up it
    let side = select(vec2<f32>(p.z, p.y), vec2<f32>(p.x, p.y), abs(n.z) > abs(n.x));
    let up = side.y;
    let along = side.x;

    // the weave: 1.4 mm threads over and under (each thread's slope bends the normal), only where
    // they're bigger than a pixel
    let k = 6.2831853 / 0.0014;
    let wv = vec2<f32>(sin(along * k) * cos(up * k * 0.5), sin(up * k) * cos(along * k * 0.5));
    let fine = clamp(1.2 - 0.6 * length(fwidth(vec2<f32>(along, up) * k)), 0.0, 1.0);
    // the cloth pulled uneven over the foam: soft ripples a few cm across
    let ripple = vec2<f32>(n3(vec3<f32>(along * 18.0, up * 30.0, 1.7)) - 0.5, n3(vec3<f32>(along * 18.0, up * 30.0, 7.3)) - 0.5);
    // (bent in the cloth's plane: along and up, in the world)
    let t_along = normalize((wfl * vec4<f32>(select(vec3<f32>(0.0, 0.0, 1.0), vec3<f32>(1.0, 0.0, 0.0), abs(n.z) > abs(n.x)), 0.0)).xyz);
    let t_up = normalize((wfl * vec4<f32>(0.0, 1.0, 0.0, 0.0)).xyz);
    let bend = wv * 0.08 * fine + ripple * 0.12;
    pbr_input.N = normalize(pbr_input.N + t_along * bend.x + t_up * bend.y);

    // the seams: a fold 8 mm in from the top and bottom edges, and the stitching beside it
    let y = in.world_position.y;
    let from_edge = min(abs(y - fab.y), abs(y - fab.z));
    let px = max(fwidth(y), 1e-4);
    let fold = 1.0 - smoothstep(0.004, 0.0085, from_edge);
    let stitch = (1.0 - smoothstep(0.0, 1.5 * px + 0.0006, abs(from_edge - 0.011))) * (0.6 + 0.4 * step(0.5, fract(along / 0.004)));

    // the rubs: patches, most near the bottom and at the corners (where the bumper turns: the
    // normal's off both axes) and on the front
    let corner = 1.0 - smoothstep(0.15, 0.6, abs(abs(n.x) - abs(n.z)));
    let low = 1.0 - smoothstep(fab.y, fab.y + 0.05, y);
    let blot = smoothstep(0.42, 0.8, n3(vec3<f32>(along * 9.0, up * 14.0, 3.1)) * 0.7 + n3(vec3<f32>(along * 41.0, up * 60.0, 5.9)) * 0.3);
    let rub = fab.x * clamp(blot * (0.35 + 0.65 * max(corner, low)), 0.0, 1.0);

    var c = pbr_input.material.base_color.rgb;
    let grain = 1.0 + 0.05 * (n3(vec3<f32>(along * 300.0, up * 300.0, 2.0)) - 0.5) * fine;
    c *= grain * (1.0 - 0.18 * fold - 0.25 * stitch);
    // (fuzzed fibers: lighter, a little washed out)
    let lum = dot(c, vec3<f32>(0.3, 0.55, 0.15));
    c = mix(c, vec3<f32>(lum) * 1.25 + c * 0.25, 0.45 * rub);
    pbr_input.material.base_color = vec4<f32>(c, pbr_input.material.base_color.a);
    pbr_input.material.perceptual_roughness = clamp(pbr_input.material.perceptual_roughness + 0.05 * rub - 0.08 * fold, 0.05, 1.0);

    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
}
