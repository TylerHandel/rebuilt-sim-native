//! As the Motion effects setting asks: a fading streak behind each FUEL in flight, as wide as the
//! FUEL where it is and narrowing to nothing behind, turned to face the camera. (Nothing the real
//! game doesn't have: a HUB shows FUEL going in with its own lights, lights.rs.)

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
        app.add_systems(Startup, spawn).add_systems(PostUpdate, trails.after(bevy::transform::TransformSystems::Propagate));
    }
}

/// how long a streak trails behind its FUEL, s
const TRAIL: f32 = 0.15;

/// The streaks' one mesh (all of them, rebuilt each frame)
#[derive(Component)]
struct Streaks(Handle<Mesh>);

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
