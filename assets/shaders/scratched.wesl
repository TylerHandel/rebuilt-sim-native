// Polycarbonate after a season: still clear, but its gloss isn't perfect. Fine scratches (from
// FUEL, robots and being wiped down in the pits) a few cm long, each a hair wide, in every
// direction, and soft smudges, show where the light catches them: rougher and a little more
// opaque there, so a scratch glints in a highlight and is invisible otherwise.
import bevy_pbr::render::{
    forward_io::{VertexOutput, FragmentOutput},
    pbr_fragment::pbr_input_from_standard_material,
    pbr_functions::{alpha_discard, apply_pbr_lighting, main_pass_post_lighting_processing},
    mesh_functions::{get_local_from_world, get_world_from_local},
};

// how scratched (0..1)
@group(constants::MATERIAL_BIND_GROUP) @binding(100) var<uniform> scratch: f32;

fn h2(p: vec2<f32>) -> vec2<f32> {
    let q = vec2<f32>(dot(p, vec2<f32>(127.1, 311.7)), dot(p, vec2<f32>(269.5, 183.3)));
    return fract(sin(q) * 43758.5453);
}
fn n2(x: vec2<f32>) -> f32 {
    let i = floor(x);
    var f = fract(x);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(h2(i).x, h2(i + vec2(1.0, 0.0)).x, f.x), mix(h2(i + vec2(0.0, 1.0)).x, h2(i + vec2(1.0, 1.0)).x, f.x), f.y);
}

// how much of the pixel at q (m, in the sheet's plane) a scratch covers: each 2.5 cm cell has one,
// its own length and direction, through a random point; checked in this cell and its neighbors
fn scratches(q: vec2<f32>, px: f32) -> f32 {
    let cell = 0.025;
    let c = floor(q / cell);
    var best = 0.0;
    for (var i = -1; i <= 1; i++) {
        for (var j = -1; j <= 1; j++) {
            let id = c + vec2<f32>(f32(i), f32(j));
            let r = h2(id);
            let r2 = h2(id + vec2<f32>(17.0, 3.0));
            if (r2.y > 0.55) { continue; }
            let at = (id + r) * cell;
            let a = r2.x * 6.2831853;
            let d = vec2<f32>(cos(a), sin(a));
            let len = cell * (0.4 + 1.6 * r.y);
            let v = q - at;
            let t = clamp(dot(v, d), -len, len);
            let dist = length(v - d * t);
            // (a 0.1 mm line, antialiased; thinner than a pixel it still glints, fainter, since a
            // scratch throws the light it catches about)
            let wd = max(px, 0.0001);
            let cover = clamp(1.0 - dist / wd, 0.0, 1.0) * clamp(0.0008 / wd, 0.15, 1.0);
            best = max(best, cover * (1.0 - abs(t) / len));
        }
    }
    return best;
}

@fragment
fn fragment(in: VertexOutput, @builtin(front_facing) is_front: bool) -> FragmentOutput {
    var pbr_input = pbr_input_from_standard_material(in, is_front);
    pbr_input.material.base_color = alpha_discard(pbr_input.material.flags, pbr_input.material.alpha_cutoff, pbr_input.material.base_color);

    let lfw = get_local_from_world(in.instance_index);
    let wfl = get_world_from_local(in.instance_index);
    let s = vec3<f32>(length(wfl[0].xyz), length(wfl[1].xyz), length(wfl[2].xyz));
    let p = (lfw * in.world_position).xyz * s;
    let n = abs(normalize((lfw * vec4<f32>(pbr_input.N, 0.0)).xyz));
    // (the sheet's plane: the two axes across its normal)
    var q = p.xy;
    if (n.x >= n.y && n.x >= n.z) { q = p.zy; } else if (n.y >= n.z) { q = p.xz; }
    let px = length(fwidth(q));

    let sc = scratches(q, px) * scratch;
    let smudge = smoothstep(0.55, 0.85, n2(q * 18.0)) * scratch;

    var c = pbr_input.material.base_color;
    c.a = min(1.0, c.a + 0.25 * sc + 0.03 * smudge);
    c = vec4<f32>(c.rgb * (1.0 + 0.6 * sc), c.a);
    pbr_input.material.base_color = c;
    pbr_input.material.perceptual_roughness = clamp(pbr_input.material.perceptual_roughness + 0.35 * sc + 0.12 * smudge, 0.0, 1.0);

    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
}
