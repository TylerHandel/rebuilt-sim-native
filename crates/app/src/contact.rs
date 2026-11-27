//! Where things meet the carpet: the arena's fill light (the environment's, from every side and
//! bounced off the carpet itself, arena.rs) doesn't reach under a robot or right under a FUEL, so
//! the carpet's darker there, softly, as ambient occlusion would have it. A soft blob multiplied
//! into the floor under each robot (its footprint, bumpers and all) and under each FUEL near the
//! floor (fading as it rises), from Medium up. On Ultra, the lights' contact shadows too
//! (camera.rs): the fine shadows where parts touch that the shadow maps are too coarse for.
//! And round the field, the cable protectors on the arena floor from the scoring table out to each
//! end's Driver Stations.

use bevy::asset::RenderAssetUsages;
use bevy::light::{NotShadowCaster, NotShadowReceiver};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};

use crate::game::Game;
use crate::materials::Look;
use crate::settings::Settings;

pub struct ContactPlugin;

impl Plugin for ContactPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (spawn, cable_protectors))
            .add_systems(PostUpdate, blobs.before(TransformSystems::Propagate))
            .add_systems(Update, contact_lights);
    }
}

/// how dark the carpet is right under a robot's middle, and right under a FUEL resting on it
/// (multiplied in: 1 is untouched)
const ROBOT_AO: f32 = 0.5;
const FUEL_AO: f32 = 0.45;

/// all the blobs: one mesh, rebuilt each frame
#[derive(Component)]
struct Blobs(Handle<Mesh>);

/// a soft square (or disk): black, its alpha 1 in the middle falling to 0 at the edges (how much
/// of the darkness: AlphaMode::Multiply gives dst * (1 - alpha), the color being black)
fn blob_image(round: bool) -> Image {
    let n = 64u32;
    let mut px = Vec::with_capacity((n * n * 4) as usize);
    for y in 0..n {
        for x in 0..n {
            let (u, v) = ((x as f32 + 0.5) / n as f32 * 2.0 - 1.0, (y as f32 + 0.5) / n as f32 * 2.0 - 1.0);
            let d = if round { u.hypot(v) } else { u.abs().max(v.abs()).max(0.85 * u.hypot(v)) };
            let k = (1.0 - d).clamp(0.0, 1.0);
            let k = k * k * (3.0 - 2.0 * k);
            px.extend([0, 0, 0, (255.0 * k).round() as u8]);
        }
    }
    Image::new(Extent3d { width: n, height: n, depth_or_array_layers: 1 }, TextureDimension::D2, px, TextureFormat::Rgba8Unorm, RenderAssetUsages::default())
}

fn empty() -> Mesh {
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, Vec::<[f32; 3]>::new())
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, Vec::<[f32; 2]>::new())
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, Vec::<[f32; 4]>::new())
        .with_inserted_indices(Indices::U32(Vec::new()))
}

fn spawn(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>, mut images: ResMut<Assets<Image>>) {
    // (multiplied into what's under it: the texture's alpha the shape, the vertex color's alpha how
    // dark; a robot's square, a FUEL's round, in one atlas: left, right)
    let (sq, rd) = (blob_image(false), blob_image(true));
    let n = 64usize;
    let mut px = vec![0u8; 2 * n * n * 4];
    for y in 0..n { for x in 0..n {
        let (i, j) = ((y * n + x) * 4, (y * 2 * n + x) * 4);
        px[j..j + 4].copy_from_slice(&sq.data.as_ref().unwrap()[i..i + 4]);
        px[j + n * 4..j + n * 4 + 4].copy_from_slice(&rd.data.as_ref().unwrap()[i..i + 4]);
    } }
    let atlas = Image::new(Extent3d { width: 2 * n as u32, height: n as u32, depth_or_array_layers: 1 }, TextureDimension::D2, px, TextureFormat::Rgba8Unorm, RenderAssetUsages::default());
    let mesh = meshes.add(empty());
    let mat = mats.add(StandardMaterial {
        base_color_texture: Some(images.add(atlas)),
        unlit: true,
        alpha_mode: AlphaMode::Multiply,
        fog_enabled: false,
        depth_bias: 8.0,
        ..default()
    });
    commands.spawn((Blobs(mesh.clone()), Mesh3d(mesh), MeshMaterial3d(mat), Transform::default(), Visibility::Hidden, NotShadowCaster, NotShadowReceiver, Look,
        bevy::camera::visibility::NoFrustumCulling));
}

fn blobs(game: Res<Game>, settings: Res<Settings>, mut q: Query<(&Blobs, &mut Visibility)>, mut meshes: ResMut<Assets<Mesh>>) {
    let Ok((b, mut vis)) = q.single_mut() else { return };
    let on = settings.graphics != "low";
    let want = if on { Visibility::Visible } else { Visibility::Hidden };
    if *vis != want { *vis = want; }
    if !on { return }
    let w = &game.0;
    let (mut pos, mut uv, mut col, mut idx) = (Vec::<[f32; 3]>::new(), Vec::<[f32; 2]>::new(), Vec::<[f32; 4]>::new(), Vec::<u32>::new());
    // (a quad on the carpet: center, its two half-axes, which half of the atlas, how strong)
    let mut quad = |c: Vec3, a: Vec3, bb: Vec3, round: bool, k: f32| {
        let base = pos.len() as u32;
        let y = 0.0015;
        for (s, t) in [(-1.0f32, -1.0f32), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            let p = c + a * s + bb * t;
            pos.push([p.x, y, p.z]);
            let u0 = if round { 0.5 } else { 0.0 };
            uv.push([u0 + 0.5 * (s + 1.0) / 2.0, (t + 1.0) / 2.0]);
            col.push([1.0, 1.0, 1.0, k]);
        }
        idx.extend([base, base + 2, base + 1, base, base + 3, base + 2]);
    };
    for r in &w.robots {
        let Some(body) = w.physics.bodies.get(r.body) else { continue };
        let p = body.translation();
        if p.y > 0.3 { continue }
        let q = *body.rotation();
        // (its footprint, bumpers and all, and a little round it: the dark falls off past the edge)
        let (hl, hw) = (r.half_l as f32 + 0.085 + 0.12, r.half_w as f32 + 0.085 + 0.12);
        quad(p, q * Vec3::X * hl, q * Vec3::Z * hw, false, ROBOT_AO * (1.0 - p.y / 0.3));
    }
    let rad = sim::constants::FUEL::radius as f32;
    for f in &w.fuel.balls {
        let Some(body) = w.physics.bodies.get(f.body) else { continue };
        if !body.is_enabled() { continue }
        let p = body.translation();
        let up = p.y - rad;
        if !(-0.05..0.2).contains(&up) { continue }
        let k = FUEL_AO * (1.0 - up.max(0.0) / 0.2);
        let s = rad * (1.3 + 2.0 * up.max(0.0));
        quad(p, Vec3::X * s, Vec3::Z * s, true, k);
    }
    // (a generic robot's held FUEL are inside it: its own blob covers them)
    if let Some(mut m) = meshes.get_mut(&b.0) {
        m.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
        m.insert_attribute(Mesh::ATTRIBUTE_UV_0, uv);
        m.insert_attribute(Mesh::ATTRIBUTE_COLOR, col);
        m.insert_indices(Indices::U32(idx));
    }
}

/// On Ultra the lights cast contact shadows (the camera marches them, camera.rs): the sun and the
/// trusses' lights over the field
fn contact_lights(settings: Res<Settings>, mut suns: Query<&mut DirectionalLight>, mut spots: Query<&mut SpotLight, With<crate::arena::TrussLight>>) {
    let on = settings.graphics == "ultra";
    for mut l in &mut suns { if l.contact_shadows_enabled != on { l.contact_shadows_enabled = on; } }
    for mut l in &mut spots { if l.contact_shadows_enabled != on { l.contact_shadows_enabled = on; } }
}

/// The cable protectors on the arena floor: from the scoring table's side of the field round each
/// end to the Driver Stations, where the field's network and power run (yellow lids, black ramps)
fn cable_protectors(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    use sim::constants::{HALF_L, HALF_W};
    let (hl, hw) = (HALF_L as f32, HALF_W as f32);
    let yellow = mats.add(StandardMaterial { base_color: Color::srgb(0.85, 0.66, 0.05), perceptual_roughness: 0.7, ..default() });
    let black = mats.add(StandardMaterial { base_color: Color::srgb(0.03, 0.03, 0.03), perceptual_roughness: 0.85, ..default() });
    // (a run: from a to b on the floor; its lid 10 cm wide, 3 cm high, a ramp each side)
    let mut run = |a: Vec2, b: Vec2| {
        let d = b - a;
        let len = d.length();
        let mid = (a + b) / 2.0;
        let rot = Quat::from_rotation_y(-d.y.atan2(d.x));
        let at = |y: f32, side: f32| Transform::from_translation(Vec3::new(mid.x, y, mid.y)).with_rotation(rot) * Transform::from_xyz(0.0, 0.0, side);
        commands.spawn((Mesh3d(meshes.add(Cuboid::new(len, 0.03, 0.1))), MeshMaterial3d(yellow.clone()), at(0.015, 0.0), Look));
        for s in [-1.0f32, 1.0] {
            // (each ramp a wedge: a box turned down to the floor)
            let ramp = at(0.008, s * 0.085) * Transform::from_rotation(Quat::from_rotation_x(s * 0.32));
            commands.spawn((Mesh3d(meshes.add(Cuboid::new(len, 0.012, 0.075))), MeshMaterial3d(black.clone()), ramp, Look));
        }
    };
    // (the scoring table's side is +z, js venue.js: along it under the table, then round each end
    // behind the Driver Stations)
    let (zt, xe) = (hw + 1.6, hl + 1.9);
    run(Vec2::new(-xe, zt), Vec2::new(xe, zt));
    for s in [-1.0f32, 1.0] { run(Vec2::new(s * xe, zt), Vec2::new(s * xe, -hw + 0.6)); }
}
