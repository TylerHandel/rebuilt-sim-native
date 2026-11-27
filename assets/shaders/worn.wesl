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
// the alliance wall's cutaway (the field's parts only, crates/app/src/cutaway.rs): x how far it's
// faded (0..1), y which end (+1/-1), z where it starts (world x, times y); eye: the camera's position.
// w: how scratched a clear part is (the field's polycarbonate: as scratched.wesl draws it)
@group(constants::MATERIAL_BIND_GROUP) @binding(101) var<uniform> cut: vec4<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(102) var<uniform> eye: vec4<f32>;

// (a 4x4 ordered-dither threshold: the fade drops fragments in a fine pattern, js wallCutaway.js)
fn cut_bayer(p: vec2<f32>) -> f32 {
    let q = vec2<u32>(u32(p.x) & 3u, u32(p.y) & 3u);
    let b = array<u32, 16>(0u, 8u, 2u, 10u, 12u, 4u, 14u, 6u, 3u, 11u, 1u, 9u, 15u, 7u, 13u, 5u);
    return (f32(b[q.x + q.y * 4u]) + 0.5) / 16.0;
}

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
    if (cut.x > 0.0 && cut.y * in.world_position.x > cut.z && cut_bayer(in.position.xy) < cut.x) { discard; }
    var pbr_input = pbr_input_from_standard_material(in, is_front);
    pbr_input.material.base_color = alpha_discard(pbr_input.material.flags, pbr_input.material.alpha_cutoff, pbr_input.material.base_color);
    pbr_input.N = bevel(in, is_front, pbr_input.N);

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

    if (cut.w > 0.0) {
        let n = abs(normalize((lfw * vec4<f32>(pbr_input.N, 0.0)).xyz));
        var q = p.xy;
        if (n.x >= n.y && n.x >= n.z) { q = p.zy; } else if (n.y >= n.z) { q = p.xz; }
        let sc = scratches(q, length(fwidth(q))) * cut.w;
        let smudge = smoothstep(0.55, 0.85, n2(q * 18.0)) * cut.w;
        let b = pbr_input.material.base_color;
        pbr_input.material.base_color = vec4<f32>(b.rgb * (1.0 + 0.6 * sc), min(1.0, b.a + 0.25 * sc + 0.03 * smudge));
        pbr_input.material.perceptual_roughness = clamp(pbr_input.material.perceptual_roughness + 0.35 * sc + 0.12 * smudge, 0.0, 1.0);
    }
    let c = pbr_input.material.base_color;
    let k = (1.0 + wear * (0.07 * v - 0.035)) * (1.0 - 0.18 * wear * scuff);
    pbr_input.material.base_color = vec4<f32>(c.rgb * k, c.a);
    pbr_input.material.perceptual_roughness = clamp(pbr_input.material.perceptual_roughness + wear * ((v - 0.5) * 0.12 + 0.15 * scuff), 0.05, 1.0);

    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
}
