//! The field: its CAD model (drawn), and its colliders exactly as the sim has them (F1 shows them).

use bevy::prelude::*;
use bevy::light::CascadeShadowConfigBuilder;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::asset::RenderAssetUsages;
use rapier3d::prelude::TypedShape;

use crate::game::Game;

pub struct FieldPlugin;

impl Plugin for FieldPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(GlobalAmbientLight { brightness: 400.0, ..default() })
            .add_systems(Startup, (spawn_field, spawn_floor, spawn_colliders))
            .add_systems(Update, toggle_colliders);
    }
}

/// The field's CAD scene.
#[derive(Resource)]
pub struct FieldScene(pub Handle<WorldAsset>);

fn spawn_field(mut commands: Commands, assets: Res<AssetServer>) {
    // cad/manifest.json: the field's CAD is in meters, turned [-90, 0, 180] degrees (XYZ)
    let d = std::f32::consts::PI / 180.0;
    let scene = assets.load(GltfAssetLabel::Scene(0).from_asset("cad/field/field.glb"));
    commands.insert_resource(FieldScene(scene.clone()));
    commands
        .spawn((
            WorldAssetRoot(scene),
            Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -90.0 * d, 0.0, 180.0 * d)),
        ))
        .observe(clear_parts);
    commands.spawn((
        DirectionalLight { illuminance: 9000.0, shadow_maps_enabled: true, ..default() },
        Transform::from_xyz(4.0, 12.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
        CascadeShadowConfigBuilder { maximum_distance: 40.0, ..default() }.build(),
    ));
}

/// js cadModels.js: the field CAD's clear parts. The ALLIANCE WALL's glass (the Driver Stations'
/// panels, "Outpost Exchange Border"), which the drivers look through all match: field.js dsGlass,
/// barely tinted, 3.5% there, perfectly smooth, casting no shadow. The rest of its polycarbonate
/// (the HUB's panels and funnel, the tags' bases) tintClear: a light blue-gray tint, 7-10% there.
fn clear_parts(
    ev: On<bevy::world_serialization::WorldInstanceReady>,
    children: Query<&Children>,
    names: Query<&Name>,
    parts: Query<&MeshMaterial3d<StandardMaterial>, With<Mesh3d>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut commands: Commands,
) {
    let glass = mats.add(StandardMaterial {
        base_color: Color::srgba_u8(0xee, 0xf4, 0xfa, 9),
        alpha_mode: AlphaMode::Blend,
        perceptual_roughness: 0.0,
        reflectance: 0.2,
        cull_mode: None,
        double_sided: true,
        ..default()
    });
    // (each CAD material's tinted copy, made once)
    let mut tinted: std::collections::HashMap<_, Result<Handle<StandardMaterial>, StandardMaterial>> = std::collections::HashMap::new();
    let mut glass_parts = Vec::new();
    for e in children.iter_descendants(ev.entity) {
        if names.get(e).is_ok_and(|n| n.as_str().contains("Outpost Exchange Border")) {
            glass_parts.extend(children.iter_descendants(e).chain([e]).filter(|k| parts.contains(*k)));
        }
    }
    for e in children.iter_descendants(ev.entity) {
        let Ok(h) = parts.get(e) else { continue };
        if glass_parts.contains(&e) {
            commands.entity(e).insert((MeshMaterial3d(glass.clone()), crate::materials::Look, bevy::light::NotShadowCaster));
            continue;
        }
        let Some(m) = mats.get(&h.0) else { continue };
        if !matches!(m.alpha_mode, AlphaMode::Blend) || m.base_color.alpha() >= 0.7 { continue }
        let t = tinted.entry(h.0.id()).or_insert_with(|| Err({
            let mut m = m.clone();
            let c = m.base_color.to_srgba();
            let k = Srgba::rgb_u8(0xc4, 0xd6, 0xe8);
            let mix = |a: f32, b: f32| a + (b - a) * 0.35;
            m.base_color = Color::srgba(mix(c.red, k.red), mix(c.green, k.green), mix(c.blue, k.blue), c.alpha.clamp(0.07, 0.1));
            m.reflectance = 0.2;
            m
        }));
        let t = match t { Ok(h) => h.clone(), Err(m) => { let h = mats.add(m.clone()); *t = Ok(h.clone()); h } };
        commands.entity(e).insert((MeshMaterial3d(t), crate::materials::Look));
    }
}

#[derive(Component)]
pub struct ColliderView;

/// A triangle mesh of one collider's shape, in its own frame.
pub fn collider_mesh(shape: TypedShape) -> Option<Mesh> {
    let (v, i) = match shape {
        TypedShape::Cuboid(s) => s.to_trimesh(),
        TypedShape::ConvexPolyhedron(s) => s.to_trimesh(),
        TypedShape::Capsule(s) => s.to_trimesh(12, 6),
        TypedShape::Cylinder(s) => s.to_trimesh(16),
        TypedShape::Ball(s) => s.to_trimesh(12, 6),
        TypedShape::TriMesh(s) => (s.vertices().to_vec(), s.indices().to_vec()),
        _ => return None,
    };
    let pos: Vec<[f32; 3]> = v.iter().map(|p| [p.x, p.y, p.z]).collect();
    let mut m = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    m.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
    m.insert_indices(Indices::U32(i.into_iter().flatten().collect()));
    let m = m.with_duplicated_vertices().with_computed_flat_normals();
    Some(m)
}

fn spawn_colliders(mut commands: Commands, game: Res<Game>, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let mat = mats.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.1, 0.8, 0.45),
        depth_bias: 10.0,
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        cull_mode: None,
        ..default()
    });
    let fixed = &game.0.physics.bodies[game.0.fixed];
    for h in fixed.colliders() {
        let c = &game.0.physics.colliders[*h];
        let Some(mesh) = collider_mesh(c.shape().as_typed_shape()) else { continue };
        let (t, r) = (c.translation(), c.rotation());
        commands.spawn((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(mat.clone()),
            Transform::from_translation(t).with_rotation(r),
            Visibility::Hidden,
            ColliderView,
        ));
    }
}

fn toggle_colliders(keys: Res<ButtonInput<KeyCode>>, mut q: Query<&mut Visibility, With<ColliderView>>) {
    if keys.just_pressed(KeyCode::F1) {
        for mut v in &mut q {
            *v = if *v == Visibility::Hidden { Visibility::Visible } else { Visibility::Hidden };
        }
    }
}

/// js field.js _buildFloor: the carpet (its texture as _carpetTexture paints it) and the dark floor
/// around the field
fn spawn_floor(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>, mut images: ResMut<Assets<Image>>) {
    use sim::constants::{FIELD_L, FIELD_W};
    let (hl, hw) = ((FIELD_L / 2.0) as f32, (FIELD_W / 2.0) as f32);
    // (as three.js's PlaneGeometry turned flat: the image's top row at -z, its left column at -x)
    let mut m = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    m.insert_attribute(Mesh::ATTRIBUTE_POSITION, vec![[-hl, 0.0, -hw], [hl, 0.0, -hw], [hl, 0.0, hw], [-hl, 0.0, hw]]);
    m.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 1.0, 0.0]; 4]);
    m.insert_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    // (the fibers' normal map repeats every FIBER_TILE m: its own uvs)
    let (tu, tv) = (2.0 * hl / FIBER_TILE, 2.0 * hw / FIBER_TILE);
    m.insert_attribute(Mesh::ATTRIBUTE_UV_1, vec![[0.0, 0.0], [tu, 0.0], [tu, tv], [0.0, tv]]);
    m.insert_attribute(Mesh::ATTRIBUTE_TANGENT, vec![[1.0, 0.0, 0.0, 1.0]; 4]);
    m.insert_indices(Indices::U32(vec![0, 2, 1, 0, 3, 2]));
    let carpet = mats.add(StandardMaterial {
        base_color_texture: Some(images.add(carpet_texture())),
        normal_map_texture: Some(images.add(fiber_normals())),
        normal_map_channel: bevy::mesh::UvChannel::Uv1,
        perceptual_roughness: 0.92,
        reflectance: 0.3,
        metallic: 0.0,
        ..default()
    });
    commands.spawn((Mesh3d(meshes.add(m)), MeshMaterial3d(carpet), Transform::default()));
    // (the gym's floor round the field: varnished maple, its strips along the field)
    let (fx, fz) = (30.0f32, 20.0f32);
    let outer = mats.add(StandardMaterial {
        base_color_texture: Some(images.add(wood_texture())),
        perceptual_roughness: 0.38,
        reflectance: 0.45,
        clearcoat: 0.5,
        clearcoat_perceptual_roughness: 0.2,
        uv_transform: bevy::math::Affine2::from_scale(Vec2::new(2.0 * fx / WOOD_TILE, 2.0 * fz / WOOD_TILE)),
        ..default()
    });
    commands.spawn((Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(fx, fz)))), MeshMaterial3d(outer), Transform::from_xyz(0.0, -0.005, 0.0)));
}

/// how far the carpet's fiber texture and the wood floor's repeat (m)
const FIBER_TILE: f32 = 0.5;
const WOOD_TILE: f32 = 2.0;

/// A texture's mip chain (RGBA8, each level a 2x2 box filter of the one before) and a sampler that
/// uses it, filtered anisotropically: no shimmer far off, crisp at the low angles the drivers see
/// the carpet at
fn with_mips(w: usize, h: usize, px: Vec<u8>, format: bevy::render::render_resource::TextureFormat, repeat: bool) -> Image {
    use bevy::image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
    use bevy::render::render_resource::{Extent3d, TextureDimension};
    let mut data = px.clone();
    let (mut cw, mut ch, mut cur) = (w, h, px);
    let mut levels = 1;
    while cw > 1 || ch > 1 {
        let (nw, nh) = ((cw / 2).max(1), (ch / 2).max(1));
        let mut next = vec![0u8; nw * nh * 4];
        for y in 0..nh { for x in 0..nw { for k in 0..4 {
            let at = |xx: usize, yy: usize| cur[((yy.min(ch - 1)) * cw + xx.min(cw - 1)) * 4 + k] as u32;
            next[(y * nw + x) * 4 + k] = ((at(2 * x, 2 * y) + at(2 * x + 1, 2 * y) + at(2 * x, 2 * y + 1) + at(2 * x + 1, 2 * y + 1) + 2) / 4) as u8;
        } } }
        data.extend_from_slice(&next);
        (cw, ch, cur) = (nw, nh, next);
        levels += 1;
    }
    let mut img = Image::new(Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 }, TextureDimension::D2, data, format, RenderAssetUsages::RENDER_WORLD);
    img.texture_descriptor.mip_level_count = levels;
    let mode = if repeat { ImageAddressMode::Repeat } else { ImageAddressMode::ClampToEdge };
    img.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: mode, address_mode_v: mode,
        mag_filter: ImageFilterMode::Linear, min_filter: ImageFilterMode::Linear, mipmap_filter: ImageFilterMode::Linear,
        anisotropy_clamp: 16,
        ..default()
    });
    img
}

/// a hash of a cell (0..1)
fn hash(a: i32, b: i32, c: i32) -> f32 {
    let mut h = (a as u32).wrapping_mul(0x8da6_b343) ^ (b as u32).wrapping_mul(0xd816_3841) ^ (c as u32).wrapping_mul(0xcb1a_b31f);
    h ^= h >> 13;
    h = h.wrapping_mul(0x5bd1_e995);
    h ^= h >> 15;
    (h & 0xff_ffff) as f32 / 16_777_216.0
}

/// The carpet's fibers: a tileable bumpy normal map (a loop pile's tufts, ~2.5 mm apart)
fn fiber_normals() -> Image {
    use bevy::render::render_resource::TextureFormat;
    let n = 256usize;
    // (heights: each tuft a soft bump, jittered; wrapping round so it tiles)
    let mut ht = vec![0f32; n * n];
    for y in 0..n { for x in 0..n {
        let (cx, cy) = ((x / 2) as i32, (y / 2) as i32);
        let h = hash(cx, cy, 1) * 0.6 + hash(x as i32, y as i32, 2) * 0.4;
        let tuft = (((x % 2) as f32 - 0.5).abs() + ((y % 2) as f32 - 0.5).abs()) * 0.5;
        ht[y * n + x] = h * (1.0 - tuft);
    } }
    let at = |x: i32, y: i32| ht[((y.rem_euclid(n as i32)) as usize) * n + (x.rem_euclid(n as i32)) as usize];
    let mut px = vec![0u8; n * n * 4];
    let k = 1.2;
    for y in 0..n as i32 { for x in 0..n as i32 {
        let dx = (at(x + 1, y) - at(x - 1, y)) * k;
        let dy = (at(x, y + 1) - at(x, y - 1)) * k;
        let v = Vec3::new(-dx, -dy, 1.0).normalize();
        let i = (y as usize * n + x as usize) * 4;
        px[i] = ((v.x * 0.5 + 0.5) * 255.0) as u8;
        px[i + 1] = ((v.y * 0.5 + 0.5) * 255.0) as u8;
        px[i + 2] = ((v.z * 0.5 + 0.5) * 255.0) as u8;
        px[i + 3] = 255;
    } }
    with_mips(n, n, px, TextureFormat::Rgba8Unorm, true)
}

/// The gym floor: maple strips (2.25in wide, random lengths) under a varnish, a tile WOOD_TILE m square
fn wood_texture() -> Image {
    use bevy::render::render_resource::TextureFormat;
    let n = 1024usize;
    let ppm = n as f32 / WOOD_TILE;
    let strip = 0.057 * ppm;
    let mut px = vec![0u8; n * n * 4];
    for y in 0..n { for x in 0..n {
        let row = (y as f32 / strip) as i32;
        // (each row's joints at its own offset, every 0.6-1.6 m)
        let off = hash(row, 0, 5) * n as f32;
        let len = (0.6 + hash(row, 1, 5)) * ppm;
        let xs = (x as f32 + off) % n as f32;
        let board = (xs / len) as i32;
        let along = xs - board as f32 * len;
        let tone = hash(row, board, 9);
        let grain = ((y as f32 * 0.9 + (x as f32 * 0.02).sin() * 6.0 + tone * 40.0) * 0.35).sin() * 0.5 + 0.5;
        let fine = hash(x as i32 / 3, y as i32, 4);
        let mut c = [0x9a as f32, 0x6c as f32, 0x42 as f32];
        let k = 0.82 + 0.3 * tone + 0.08 * grain + 0.04 * fine;
        for v in &mut c { *v *= k; }
        // (the seams between strips and boards)
        let seam = (y as f32 % strip) < 1.0 || along < 1.0;
        if seam { for v in &mut c { *v *= 0.55; } }
        let i = (y * n + x) * 4;
        for j in 0..3 { px[i + j] = c[j].clamp(0.0, 255.0) as u8; }
        px[i + 3] = 255;
    } }
    with_mips(n, n, px, TextureFormat::Rgba8UnormSrgb, true)
}

/// js field.js _carpetTexture: gray carpet with a little noise, the ALLIANCE ZONES' faint tints, the
/// seam's gaffer tape, the ROBOT STARTING LINES, the CENTER LINE and the DEPOT outlines
fn carpet_texture() -> Image {
    use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
    use sim::constants::{ALLIANCE_ZONE_DEPTH, DEPOT, FIELD_L, FIELD_W, IN};
    let w = 4096usize;
    let h = (4096.0 * FIELD_W / FIELD_L).round() as usize;
    let mut px = vec![0u8; w * h * 4];
    // (the noise: Math.random there, a small LCG here)
    let mut seed = 0x2026_u32;
    for p in px.chunks_exact_mut(4) {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let n = ((seed >> 8) as f32 / 16_777_216.0 - 0.5) * 18.0;
        for (i, c) in [0x62u8, 0x66, 0x6d].iter().enumerate() { p[i] = (*c as f32 + n).round().clamp(0.0, 255.0) as u8; }
        p[3] = 255;
    }
    let (sx, sy) = (w as f64 / FIELD_L, h as f64 / FIELD_W);
    // (wear: soft darker smudges where robots run most, round the HUBs and down the middle, and a
    // few lighter scuffs, stretched along the field)
    {
        let mut r = 0x51ee_d00d_u32;
        let mut rnd = || { r = r.wrapping_mul(1664525).wrapping_add(1013904223); (r >> 8) as f64 / 16_777_216.0 };
        for i in 0..70 {
            let hub = i % 3;
            let cx = match hub { 0 => FIELD_L * (0.25 + 0.06 * (rnd() - 0.5)), 1 => FIELD_L * (0.75 + 0.06 * (rnd() - 0.5)), _ => FIELD_L * (0.2 + 0.6 * rnd()) };
            let cy = FIELD_W * (0.15 + 0.7 * rnd());
            let (rx, ry) = ((0.4 + 1.4 * rnd()) * sx, (0.25 + 0.6 * rnd()) * sy);
            let dark = rnd() < 0.8;
            let a = if dark { -5.0 } else { 3.5 } * (0.5 + rnd());
            let (x0, x1) = (((cx * sx - 2.0 * rx).max(0.0)) as usize, ((cx * sx + 2.0 * rx) as usize).min(w));
            let (y0, y1) = (((cy * sy - 2.0 * ry).max(0.0)) as usize, ((cy * sy + 2.0 * ry) as usize).min(h));
            for y in y0..y1 { for x in x0..x1 {
                let (dx, dy) = ((x as f64 - cx * sx) / rx, (y as f64 - cy * sy) / ry);
                let f = ((-(dx * dx + dy * dy)).exp() * a) as f32;
                let i = (y * w + x) * 4;
                for k in 0..3 { px[i + k] = (px[i + k] as f32 + f).round().clamp(0.0, 255.0) as u8; }
            } }
        }
    }
    // a canvas fillRect in pixels, mixed in sRGB as the canvas mixes
    let mut fill = |x0: f64, y0: f64, x1: f64, y1: f64, c: [u8; 3], a: f32| {
        let (x0, x1) = (x0.round().max(0.0) as usize, (x1.round() as usize).min(w));
        let (y0, y1) = (y0.round().max(0.0) as usize, (y1.round() as usize).min(h));
        for y in y0..y1 { for x in x0..x1 {
            let i = (y * w + x) * 4;
            for k in 0..3 { px[i + k] = (c[k] as f32 * a + px[i + k] as f32 * (1.0 - a)).round() as u8; }
        } }
    };
    // canvas x = fx, canvas y = fy mirrored
    let rect = |fill: &mut dyn FnMut(f64, f64, f64, f64, [u8; 3], f32), fx0: f64, fy0: f64, fx1: f64, fy1: f64, c: [u8; 3], a: f32| {
        fill(fx0 * sx, (FIELD_W - fy1) * sy, fx1 * sx, (FIELD_W - fy0) * sy, c, a);
    };
    let (blue, red) = ([0x2f, 0x6f, 0xf0], [0xe0, 0x34, 0x34]);
    rect(&mut fill, 0.0, 0.0, ALLIANCE_ZONE_DEPTH, FIELD_W, blue, 0.06);
    rect(&mut fill, FIELD_L - ALLIANCE_ZONE_DEPTH, 0.0, FIELD_L, FIELD_W, red, 0.04);
    let tape = 2.0 * IN;
    rect(&mut fill, 0.0, FIELD_W / 2.0 - 1.5 * IN, FIELD_L, FIELD_W / 2.0 + 1.5 * IN, [0x1e, 0x1f, 0x23], 1.0);
    rect(&mut fill, ALLIANCE_ZONE_DEPTH - tape, 0.0, ALLIANCE_ZONE_DEPTH, FIELD_W, blue, 1.0);
    rect(&mut fill, FIELD_L - ALLIANCE_ZONE_DEPTH, 0.0, FIELD_L - ALLIANCE_ZONE_DEPTH + tape, FIELD_W, red, 1.0);
    rect(&mut fill, FIELD_L / 2.0 - tape / 2.0, 0.0, FIELD_L / 2.0 + tape / 2.0, FIELD_W, [0xf2, 0xf2, 0xf2], 1.0);
    // the DEPOT outlines (a 6 px stroke, centered on the rectangle's edges)
    for (fx0, fx1, fyc, c) in [(0.0, DEPOT::depth, DEPOT::fy, blue), (FIELD_L - DEPOT::depth, FIELD_L, FIELD_W - DEPOT::fy, red)] {
        let (x0, x1) = (fx0 * sx, fx1 * sx);
        let (y0, y1) = ((FIELD_W - (fyc + DEPOT::width / 2.0)) * sy, (FIELD_W - (fyc - DEPOT::width / 2.0)) * sy);
        fill(x0 - 3.0, y0 - 3.0, x1 + 3.0, y0 + 3.0, c, 1.0);
        fill(x0 - 3.0, y1 - 3.0, x1 + 3.0, y1 + 3.0, c, 1.0);
        fill(x0 - 3.0, y0 - 3.0, x0 + 3.0, y1 + 3.0, c, 1.0);
        fill(x1 - 3.0, y0 - 3.0, x1 + 3.0, y1 + 3.0, c, 1.0);
    }
    let _ = (Extent3d::default(), TextureDimension::D2);
    with_mips(w, h, px, TextureFormat::Rgba8UnormSrgb, false)
}
