// The prepass of the worn material (worn.wesl): Bevy's own (bevy_pbr render/pbr_prepass.wesl,
// copied with its imports pointed at bevy_pbr), and the alliance wall's cutaway in it, so a faded
// part's depth doesn't hide what's behind it (the robot) where the main pass dropped it. Only from
// the camera: a light's shadow pass sees the whole wall.
import bevy_pbr::{
    render::{
        pbr_prepass_functions,
        pbr_bindings,
        pbr_bindings::material,
        pbr_types,
        pbr_functions,
        pbr_functions::SampleBias,
        mesh_bindings::mesh,
        mesh_view_bindings::view,
    },
    prepass::io as prepass_io,
};

import bevy_render::bindless::{bindless_samplers_filtering, bindless_textures_2d};


@if(MESHLET_MESH_MATERIAL_PASS)
import bevy_pbr::meshlet::visibility_buffer_resolve::resolve_vertex_output;

@if(BINDLESS)
import bevy_pbr::render::pbr_bindings::material_indices;

@group(constants::MATERIAL_BIND_GROUP) @binding(101) var<uniform> cut: vec4<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(102) var<uniform> eye: vec4<f32>;

fn cut_bayer(p: vec2<f32>) -> f32 {
    let q = vec2<u32>(u32(p.x) & 3u, u32(p.y) & 3u);
    let b = array<u32, 16>(0u, 8u, 2u, 10u, 12u, 4u, 14u, 6u, 3u, 11u, 1u, 9u, 15u, 7u, 13u, 5u);
    return (f32(b[q.x + q.y * 4u]) + 0.5) / 16.0;
}
fn cut_away(in: prepass_io::VertexOutput) {
    if (cut.x > 0.0 && distance(view.world_position, eye.xyz) < 0.01 && cut.y * in.world_position.x > cut.z && cut_bayer(in.position.xy) < cut.x) { discard; }
}

@if(PREPASS_FRAGMENT)
@fragment
fn fragment(
@if(MESHLET_MESH_MATERIAL_PASS)
    @builtin(position) frag_coord: vec4<f32>,
@if(!MESHLET_MESH_MATERIAL_PASS)
    in: prepass_io::VertexOutput,
@if(!MESHLET_MESH_MATERIAL_PASS)
    @builtin(front_facing) is_front: bool,
) -> prepass_io::FragmentOutput {
@if(MESHLET_MESH_MATERIAL_PASS)
    let in = resolve_vertex_output(frag_coord);
@if(MESHLET_MESH_MATERIAL_PASS)
    let is_front = true;

@if(!MESHLET_MESH_MATERIAL_PASS && BINDLESS)
    let slot = mesh[in.instance_index].material_and_lightmap_bind_group_slot & 0xffffu;
@if(!MESHLET_MESH_MATERIAL_PASS && BINDLESS)
    let flags = pbr_bindings::material_array[material_indices[slot].material].flags;
@if(!MESHLET_MESH_MATERIAL_PASS && BINDLESS)
    let uv_transform = pbr_bindings::material_array[material_indices[slot].material].uv_transform;
@if(!MESHLET_MESH_MATERIAL_PASS && !BINDLESS)
    let flags = pbr_bindings::material.flags;
@if(!MESHLET_MESH_MATERIAL_PASS && !BINDLESS)
    let uv_transform = pbr_bindings::material.uv_transform;

@if(!MESHLET_MESH_MATERIAL_PASS)
    cut_away(in);

    // If we're in the crossfade section of a visibility range, conditionally
    // discard the fragment according to the visibility pattern.
@if(!MESHLET_MESH_MATERIAL_PASS && VISIBILITY_RANGE_DITHER)
    pbr_functions::visibility_range_dither(in.position, in.visibility_range_dither);

@if(!MESHLET_MESH_MATERIAL_PASS)
    pbr_prepass_functions::prepass_sample_color_and_alpha_discard(in);

    var out: prepass_io::FragmentOutput;

@if(UNCLIPPED_DEPTH_ORTHO_EMULATION)
    out.frag_depth = in.unclipped_depth;

    // NOTE: Unlit bit not set means == 0 is true, so the true case is if lit
@if(NORMAL_PREPASS)
    if (flags & pbr_types::STANDARD_MATERIAL_FLAGS_UNLIT_BIT) == 0u {
        let double_sided = (flags & pbr_types::STANDARD_MATERIAL_FLAGS_DOUBLE_SIDED_BIT) != 0u;
@if(MESHLET_MESH_MATERIAL_PASS)
        let mesh_flags = in.mesh_flags;
@else
        let mesh_flags = mesh[in.instance_index].flags;
        let is_logically_front = pbr_functions::winding_corrected_front_facing(mesh_flags, is_front);

        let world_normal = pbr_functions::prepare_world_normal(
            in.world_normal,
            double_sided,
            is_logically_front,
        );

        var normal = world_normal;

@if(VERTEX_UVS) {
@if(VERTEX_TANGENTS) {
@if(STANDARD_MATERIAL_NORMAL_MAP) {

// TODO: Transforming UVs mean we need to apply derivative chain rule for meshlet mesh material pass
@if(STANDARD_MATERIAL_NORMAL_MAP_UV_B)
        let uv = (uv_transform * vec3(in.uv_b, 1.0)).xy;
@else
        let uv = (uv_transform * vec3(in.uv, 1.0)).xy;

        // Fill in the sample bias so we can sample from textures.
        var bias: SampleBias;
@if(MESHLET_MESH_MATERIAL_PASS) {
        // Apply mip bias by scaling the gradients used by textureSampleGrad.
        let mip_scale = exp2(view.mip_bias);
        bias.ddx_uv = in.ddx_uv * mip_scale;
        bias.ddy_uv = in.ddy_uv * mip_scale;
}
@if(!MESHLET_MESH_MATERIAL_PASS)
        bias.mip_bias = view.mip_bias;

@if(BINDLESS)
        let texture = bindless_textures_2d[material_indices[slot].normal_map_texture];
@else
        let texture = pbr_bindings::normal_map_texture;
@if(BINDLESS)
        let sampler_ = bindless_samplers_filtering[material_indices[slot].normal_map_sampler];
@else
        let sampler_ = pbr_bindings::normal_map_sampler;
@if(MESHLET_MESH_MATERIAL_PASS)
        let Nt =
            textureSampleGrad(
                texture,
                sampler_,
                uv,
                bias.ddx_uv,
                bias.ddy_uv,
            ).rgb;
@if(!MESHLET_MESH_MATERIAL_PASS)
        let Nt =
            textureSampleBias(
                texture,
                sampler_,
                uv,
                bias.mip_bias,
            ).rgb;
        let TBN = pbr_functions::calculate_tbn_mikktspace(normal, in.world_tangent);

        normal = pbr_functions::apply_normal_mapping(
            flags,
            TBN,
            double_sided,
            is_logically_front,
            Nt,
        );

}
}
}

        out.normal = vec4(normal * 0.5 + vec3(0.5), 1.0);
    } else {
        out.normal = vec4(in.world_normal * 0.5 + vec3(0.5), 1.0);
    }

@if(MOTION_VECTOR_PREPASS && MESHLET_MESH_MATERIAL_PASS)
    out.motion_vector = in.motion_vector;
@if(MOTION_VECTOR_PREPASS && !MESHLET_MESH_MATERIAL_PASS)
    out.motion_vector = pbr_prepass_functions::calculate_motion_vector(in.world_position, in.previous_world_position);

    return out;
}
@else
@fragment
fn fragment(in: prepass_io::VertexOutput) {
    cut_away(in);
    pbr_prepass_functions::prepass_sample_color_and_alpha_discard(in);
}
