// Foliage in the wind (crates/app/src/nature.rs): Bevy's mesh vertex shader, with each vertex pushed
// downwind by how far up its blade or branch it is (uv_b.x, 0 at the root, 1 at the tip; squared,
// so it bends rather than leans) and how stiff it is (uv_b.y). The wind's gusts roll across the
// ground as waves (world position and time), with a quick flutter on top. Lit as standard PBR.
// It has no prepass (nature.rs: the depth prepass would have it unbent), and its shadows don't sway.
import bevy_pbr::{
    render::{
        mesh_view_bindings::globals,
        mesh_functions,
        forward_io::{Vertex, VertexOutput, decompress_vertex},
        view_transformations::position_world_to_clip,
    },
};

// how hard it blows (calm 0.4, a breeze 1, rain's squalls 1.8)
@group(constants::MATERIAL_BIND_GROUP) @binding(100) var<uniform> wind: f32;

fn wind_offset(p: vec3<f32>, bend: f32, stiff: f32) -> vec3<f32> {
    let t = globals.time;
    let dir = normalize(vec2<f32>(0.85, 0.53));
    let along = dot(p.xz, dir);
    // gusts: long waves rolling downwind, and slower swells across them
    let gust = 0.55 + 0.45 * sin(along * 0.18 - t * 1.7) * (0.6 + 0.4 * sin(p.x * 0.031 + p.z * 0.047 + t * 0.4));
    // flutter: each blade or leaf its own quick shiver
    let flutter = sin(t * 7.3 + p.x * 3.1 + p.z * 2.3) * 0.25 + sin(t * 11.0 + p.z * 4.7) * 0.12;
    let k = wind * bend * bend * stiff;
    let push = k * (0.16 * gust + 0.04 * flutter);
    // (and sinks a little as it bends, so the blade keeps its length)
    return vec3<f32>(dir.x * push, -0.5 * push * push, dir.y * push);
}

@vertex
fn vertex(vertex_in: Vertex) -> VertexOutput {
    var out: VertexOutput;
    let vertex = decompress_vertex(vertex_in, vertex_in.instance_index);
    let world_from_local = mesh_functions::get_world_from_local(vertex_in.instance_index);
@if(VERTEX_NORMALS)
    out.world_normal = mesh_functions::mesh_normal_local_to_world(vertex.normal, vertex_in.instance_index);
    var wp = mesh_functions::mesh_position_local_to_world(world_from_local, vec4<f32>(vertex.position, 1.0));
@if(VERTEX_UVS_B)
    wp = vec4<f32>(wp.xyz + wind_offset(wp.xyz, vertex.uv_b.x, vertex.uv_b.y), wp.w);
    out.world_position = wp;
    out.position = position_world_to_clip(wp.xyz);
@if(VERTEX_UVS_A)
    out.uv = vertex.uv;
@if(VERTEX_UVS_B)
    out.uv_b = vertex.uv_b;
@if(VERTEX_TANGENTS)
    out.world_tangent = mesh_functions::mesh_tangent_local_to_world(world_from_local, vertex.tangent, vertex_in.instance_index);
@if(VERTEX_COLORS)
    out.color = vertex.color;
@if(VERTEX_OUTPUT_INSTANCE_INDEX)
    out.instance_index = vertex_in.instance_index;
@if(VISIBILITY_RANGE_DITHER)
    out.visibility_range_dither = mesh_functions::get_visibility_range_dither_level(vertex_in.instance_index, world_from_local[3]);
    return out;
}
