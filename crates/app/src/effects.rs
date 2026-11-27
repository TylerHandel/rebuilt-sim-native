//! As the Motion effects setting asks: a fading streak behind each FUEL in flight, as wide as the
//! FUEL where it is and narrowing to nothing behind, turned to face the camera. (Nothing the real
//! game doesn't have: a HUB shows FUEL going in with its own lights, lights.rs.)
//! And always: where a robot's hit hard (its speed jerked by a wall or another robot), a puff of
//! the carpet's dust kicked up, and from a really hard one, a few sparks off the metal.

use std::collections::{HashMap, VecDeque};

use bevy::asset::RenderAssetUsages;
use bevy::light::{NotShadowCaster, NotShadowReceiver};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

use crate::camera::MainCam;
use crate::game::Game;
use crate::settings::Settings;

pub struct EffectsPlugin;

impl Plugin for EffectsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn).add_systems(PostUpdate, (trails, hits).after(bevy::transform::TransformSystems::Propagate));
    }
}

/// how long a streak trails behind its FUEL, s
const TRAIL: f32 = 0.15;

/// The streaks' one mesh (all of them, rebuilt each frame)
#[derive(Component)]
struct Streaks(Handle<Mesh>);
/// The hits' sparks and dust: one mesh, rebuilt each frame
#[derive(Component)]
struct Bits(Handle<Mesh>);

/// a speck: where, how fast, how old, how long it lasts, a spark (else dust)
struct Speck { p: Vec3, v: Vec3, age: f32, life: f32, spark: bool }
/// how hard a hit has to be to kick up dust, and to throw sparks (m/s the robot's speed jerks by in
/// a frame)
const DUST_HIT: f32 = 1.8;
const SPARK_HIT: f32 = 3.0;

fn empty() -> Mesh {
    let mut m = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    m.insert_attribute(Mesh::ATTRIBUTE_POSITION, Vec::<[f32; 3]>::new());
    m.insert_attribute(Mesh::ATTRIBUTE_COLOR, Vec::<[f32; 4]>::new());
    m.insert_indices(Indices::U32(Vec::new()));
    m
}

fn spawn(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let mesh = meshes.add(empty());
    let mat = mats.add(StandardMaterial { base_color: Color::WHITE, unlit: true, alpha_mode: AlphaMode::Blend, double_sided: true, cull_mode: None, ..default() });
    commands.spawn((Streaks(mesh.clone()), Mesh3d(mesh), MeshMaterial3d(mat), Transform::default(), Visibility::Hidden, NotShadowCaster, NotShadowReceiver,
        // (it moves with the FUEL: never culled for where it was when its bounds were last worked out)
        bevy::camera::visibility::NoFrustumCulling));
    let mesh = meshes.add(empty());
    let mat = mats.add(StandardMaterial { base_color: Color::WHITE, unlit: true, alpha_mode: AlphaMode::Blend, double_sided: true, cull_mode: None, ..default() });
    commands.spawn((Bits(mesh.clone()), Mesh3d(mesh), MeshMaterial3d(mat), Transform::default(), Visibility::Hidden, NotShadowCaster, NotShadowReceiver, bevy::camera::visibility::NoFrustumCulling));
}

/// a little hash for the specks' scatter
fn rnd(seed: &mut u32) -> f32 {
    *seed ^= *seed << 13; *seed ^= *seed >> 17; *seed ^= *seed << 5;
    (*seed & 0xffff) as f32 / 65535.0
}

#[allow(clippy::too_many_arguments)]
fn hits(
    game: Res<Game>,
    time: Res<Time>,
    mut last: Local<Vec<Option<(Vec3, Vec3)>>>,
    mut specks: Local<Vec<Speck>>,
    mut seed: Local<u32>,
    cams: Query<&GlobalTransform, With<MainCam>>,
    mut bits: Query<(&Bits, &mut Visibility)>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let Ok((bits, mut vis)) = bits.single_mut() else { return };
    let Ok(cam) = cams.single() else { return };
    if *seed == 0 { *seed = 0x9E37_79B9; }
    let dt = time.delta_secs().min(0.1);
    let w = &game.0;
    last.resize(w.robots.len(), None);
    for (i, r) in w.robots.iter().enumerate() {
        let Some(b) = w.physics.bodies.get(r.body) else { continue };
        let v = b.linvel();
        // (not where it was just put somewhere else: a restart)
        if let Some((lv, _)) = last[i].filter(|(_, lp)| lp.distance(b.translation()) < 0.5) {
            let dv = Vec3::new(v.x - lv.x, 0.0, v.z - lv.z);
            let hit = dv.length();
            if hit > DUST_HIT && dt > 0.0 {
                // (where it was hit: the side the jerk pushed it away from, at its bumpers)
                let n = -dv / hit;
                let q = *b.rotation();
                let local = q.inverse() * n;
                let reach = (r.half_l as f32 + 0.085) / local.x.abs().max(1e-3);
                let reach = reach.min((r.half_w as f32 + 0.085) / local.z.abs().max(1e-3));
                let at = b.translation() + n * reach;
                let dust = ((hit - DUST_HIT) * 6.0).clamp(4.0, 16.0) as usize;
                for _ in 0..dust {
                    let s = &mut *seed;
                    let d = Vec3::new(rnd(s) - 0.5, rnd(s) * 0.6 + 0.2, rnd(s) - 0.5) * 1.2 + n * 0.3;
                    specks.push(Speck { p: at + Vec3::new(rnd(s) - 0.5, 0.0, rnd(s) - 0.5) * 0.2 + Vec3::Y * 0.02, v: d * (0.3 + 0.2 * hit.min(5.0)), age: 0.0, life: 0.6 + 0.5 * rnd(s), spark: false });
                }
                if hit > SPARK_HIT {
                    for _ in 0..((hit - SPARK_HIT) * 8.0).clamp(4.0, 14.0) as usize {
                        let s = &mut *seed;
                        let t = Vec3::new(-n.z, 0.0, n.x) * (rnd(s) * 2.0 - 1.0);
                        let d = (n * 0.6 + t + Vec3::Y * (0.3 + 0.7 * rnd(s))).normalize();
                        specks.push(Speck { p: at + Vec3::Y * (0.1 + 0.08 * rnd(s)), v: d * (2.5 + 2.5 * rnd(s)), age: 0.0, life: 0.18 + 0.2 * rnd(s), spark: true });
                    }
                }
            }
        }
        last[i] = Some((v, b.translation()));
    }
    // (they fly: sparks fall fast and die bright; dust drifts, slows and settles, fading)
    for k in specks.iter_mut() {
        k.age += dt;
        if k.spark { k.v.y -= 9.8 * dt; } else { k.v *= (1.0 - 3.0 * dt).max(0.0); k.v.y -= 0.4 * dt; }
        k.p += k.v * dt;
        if k.p.y < 0.005 { k.p.y = 0.005; k.v.y = k.v.y.abs() * 0.3; }
    }
    specks.retain(|k| k.age < k.life);
    specks.truncate(400);
    let eye = cam.translation();
    let (mut pos, mut col, mut idx) = (Vec::<[f32; 3]>::new(), Vec::<[f32; 4]>::new(), Vec::<u32>::new());
    for k in specks.iter() {
        let f = 1.0 - k.age / k.life;
        let to_eye = (eye - k.p).normalize_or(Vec3::Y);
        let (a, b, c) = if k.spark {
            // (a streak along its flight, a few cm, turned to face the camera)
            let along = k.v * 0.012;
            let side = along.cross(to_eye).normalize_or_zero() * 0.0018;
            (k.p - along, side, [1.0, 0.65 + 0.3 * f, 0.25 * f, f.sqrt()])
        } else {
            // (a soft square of dust, a couple of cm, facing the camera)
            let size = 0.012 + 0.03 * (1.0 - f);
            let side = to_eye.cross(Vec3::Y).normalize_or(Vec3::X) * size;
            (k.p - Vec3::Y * size, side, [0.55, 0.55, 0.57, 0.35 * f])
        };
        let tip = if k.spark { k.p } else { k.p + (k.p - a) };
        let base = pos.len() as u32;
        pos.extend([(a - b).to_array(), (a + b).to_array(), (tip + b).to_array(), (tip - b).to_array()]);
        col.extend([c; 4]);
        idx.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    let want = if idx.is_empty() { Visibility::Hidden } else { Visibility::Visible };
    if *vis != want { *vis = want; }
    if want == Visibility::Hidden && pos.is_empty() { return }
    if let Some(mut m) = meshes.get_mut(&bits.0) {
        m.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
        m.insert_attribute(Mesh::ATTRIBUTE_COLOR, col);
        m.insert_indices(Indices::U32(idx));
    }
}

fn trails(
    game: Res<Game>,
    time: Res<Time>,
    settings: Res<Settings>,
    mut hist: Local<HashMap<usize, VecDeque<(Vec3, f32)>>>,
    cams: Query<&GlobalTransform, With<MainCam>>,
    mut streaks: Query<(&Streaks, &mut Visibility)>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let Ok((streaks, mut vis)) = streaks.single_mut() else { return };
    if !matches!(settings.motion.as_str(), "trails" | "both") {
        hist.clear();
        if *vis != Visibility::Hidden { *vis = Visibility::Hidden; }
        return;
    }
    let Ok(cam) = cams.single() else { return };
    let eye = cam.translation();
    let now = time.elapsed_secs();
    // (REBUILT_TRAIL_S=<s>: a longer trail, to see one where frames are slow)
    let trail: f32 = std::env::var("REBUILT_TRAIL_S").ok().and_then(|v| v.parse().ok()).unwrap_or(TRAIL);
    let w = &game.0;
    let r = sim::constants::FUEL::radius as f32;
    let (mut pos, mut col, mut idx) = (Vec::<[f32; 3]>::new(), Vec::<[f32; 4]>::new(), Vec::<u32>::new());
    for (i, b) in w.fuel.balls.iter().enumerate() {
        let Some(body) = w.physics.bodies.get(b.body) else { continue };
        let (p, v) = (body.translation(), body.linvel());
        let fast = body.is_enabled() && v.length() > 4.0 && p.y > 0.25;
        let h = hist.entry(i).or_default();
        if fast { h.push_back((p, now)); }
        while h.front().is_some_and(|(_, t)| now - t > trail) { h.pop_front(); }
        if h.len() < 2 { continue }
        // a ribbon through where it's been: at each point, across the way it went and the way the
        // camera looks (so it's seen flat on), its half-width shrinking and fading with age
        let pts: Vec<(Vec3, f32)> = h.iter().map(|(q, t)| (*q, (1.0 - (now - t) / trail).clamp(0.0, 1.0))).collect();
        let base = pos.len() as u32;
        for k in 0..pts.len() {
            let (q, a) = pts[k];
            let along = if k + 1 < pts.len() { pts[k + 1].0 - q } else { q - pts[k - 1].0 };
            let side = along.cross(eye - q).normalize_or_zero() * r * a.sqrt();
            let c = [1.0, 0.82, 0.18, 0.5 * a];
            pos.push((q - side).into()); col.push(c);
            pos.push((q + side).into()); col.push(c);
        }
        for k in 0..pts.len() as u32 - 1 {
            let (a0, a1, b0, b1) = (base + 2 * k, base + 2 * k + 1, base + 2 * k + 2, base + 2 * k + 3);
            idx.extend([a0, b0, b1, a0, b1, a1]);
        }
    }
    hist.retain(|_, h| !h.is_empty());
    let want = if idx.is_empty() { Visibility::Hidden } else { Visibility::Visible };
    if *vis != want { *vis = want; }
    if let Some(mut m) = meshes.get_mut(&streaks.0) {
        m.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
        m.insert_attribute(Mesh::ATTRIBUTE_COLOR, col);
        m.insert_indices(Indices::U32(idx));
    }
}
