//! Bumpers give: where a robot's hit hard (its speed jerked by a wall or another robot), its
//! bumper's foam dents in at the side it was hit on, up to a couple of cm, and springs back out over
//! a quarter of a second (drawn only). The bumper's cloth and the team numbers on it move together:
//! the dent is a shift of every point near the hit, whatever mesh it's in.

use bevy::mesh::VertexAttributeValues;
use bevy::prelude::*;

use crate::game::Game;
use crate::robot_model::RobotModel;

pub struct BumpersPlugin;

impl Plugin for BumpersPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, find).add_systems(PostUpdate, dent.before(TransformSystems::Propagate));
    }
}

/// how hard a hit has to be to dent the bumper (m/s its speed jerks by in a frame), how deep per m/s
/// past that, at most, how wide (m), and its spring back (Hz, damping)
const DENT_HIT: f32 = 1.2;
const DENT_PER: f32 = 0.006;
const DENT_MAX: f32 = 0.022;
const DENT_R: f32 = 0.2;
const DENT_HZ: f32 = 5.0;
const DENT_DAMP: f32 = 0.6;

/// a model whose bumper's been looked for
#[derive(Component)]
struct BumperFound;

/// a mesh of a robot's bumper (its cloth or its numbers), its own copy: its points at rest (in the
/// robot's frame: the meshes are in the model's frame) and its node's transform to that frame
#[derive(Component)]
struct BumperMesh { robot: usize, base: Vec<[f32; 3]>, to: bevy::math::Affine3A }

/// each robot's dent: where (the robot's frame, unit, outward), how deep, how fast that's changing;
/// its velocity last frame and where it was
#[derive(Default, Clone, Copy)]
struct Dent { n: Vec2, depth: f32, vel: f32, last: Option<(Vec3, Vec3)> }

#[allow(clippy::type_complexity)]
fn find(
    models: Query<(Entity, &RobotModel, &GlobalTransform), (Without<BumperFound>, Without<crate::showcase::Showcase>)>,
    children: Query<&Children>,
    parts: Query<(&Mesh3d, &GlobalTransform)>,
    game: Res<Game>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut wait: Local<std::collections::HashMap<Entity, u32>>,
    mut commands: Commands,
) {
    for (root, m, root_gt) in &models {
        if !m.ready { continue }
        let w = wait.entry(root).or_insert(0);
        *w += 1;
        if *w < 3 { continue }
        wait.remove(&root);
        commands.entity(root).insert(BumperFound);
        let Some(r) = game.0.robots.get(m.robot) else { continue };
        let (hl, hw) = (r.half_l as f32, r.half_w as f32);
        let to_robot = root_gt.affine().inverse();
        let mut n = 0;
        for e in children.iter_descendants(root) {
            let Ok((mh, gt)) = parts.get(e) else { continue };
            let Some(mesh) = meshes.get(&mh.0) else { continue };
            let Some(VertexAttributeValues::Float32x3(p)) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else { continue };
            let to = to_robot * gt.affine();
            // (all of it in the bumpers' band, outside the frame: the cloth, or its numbers)
            let bumper = !p.is_empty() && p.iter().all(|v| {
                let q = to.transform_point3(Vec3::from(*v));
                q.y > 0.015 && q.y < 0.19 && (q.x.abs() > hl - 0.01 || q.z.abs() > hw - 0.01)
            });
            if !bumper { continue }
            let base = p.clone();
            let mesh = mesh.clone();
            let copy = meshes.add(mesh);
            commands.entity(e).insert((Mesh3d(copy), BumperMesh { robot: m.robot, base, to }));
            n += 1;
        }
        info!("robot {}: {n} bumper meshes to dent", m.robot);
    }
}

fn dent(game: Res<Game>, time: Res<Time>, mut dents: Local<Vec<Dent>>, mut q: Query<(&BumperMesh, &Mesh3d)>, mut meshes: ResMut<Assets<Mesh>>) {
    let dt = time.delta_secs().min(0.1);
    let w = &game.0;
    dents.resize(w.robots.len(), Dent::default());
    let mut moved = vec![false; w.robots.len()];
    for (i, r) in w.robots.iter().enumerate() {
        let Some(b) = w.physics.bodies.get(r.body) else { continue };
        let (v, p) = (b.linvel(), b.translation());
        let d = &mut dents[i];
        if let Some((lv, lp)) = d.last.filter(|(_, lp)| lp.distance(p) < 0.5) {
            let _ = lp;
            let dv = Vec3::new(v.x - lv.x, 0.0, v.z - lv.z);
            let hit = dv.length();
            if hit > DENT_HIT {
                // (it's pushed away from where it's hit: that side, in its own frame)
                let local = b.rotation().inverse() * (-dv / hit);
                d.n = Vec2::new(local.x, local.z).normalize_or(Vec2::X);
                d.vel = d.vel.max(((hit - DENT_HIT) * DENT_PER).min(DENT_MAX) * std::f32::consts::TAU * DENT_HZ);
            }
        }
        d.last = Some((v, p));
        let before = d.depth;
        let (k, c) = ((std::f32::consts::TAU * DENT_HZ).powi(2), 2.0 * DENT_DAMP * std::f32::consts::TAU * DENT_HZ);
        let mut left = dt;
        while left > 0.0 {
            let h = left.min(0.003);
            d.vel += (-k * d.depth - c * d.vel) * h;
            d.depth = (d.depth + d.vel * h).clamp(0.0, DENT_MAX);
            if d.depth == 0.0 && d.vel < 0.0 { d.vel = 0.0; }
            left -= h;
        }
        if d.depth < 1e-4 && d.vel.abs() < 1e-3 { d.depth = 0.0; d.vel = 0.0; }
        moved[i] = d.depth != before;
    }
    for (bm, h) in &mut q {
        if !moved.get(bm.robot).copied().unwrap_or(false) { continue }
        let d = dents[bm.robot];
        let Some(r) = w.robots.get(bm.robot) else { continue };
        // (the hit's point on the bumper: out from the middle along n to the frame's edge, and past)
        let reach = ((r.half_l as f32 + 0.04) / d.n.x.abs().max(1e-3)).min((r.half_w as f32 + 0.04) / d.n.y.abs().max(1e-3));
        let at = d.n * reach;
        let inv = bm.to.inverse();
        let push = inv.transform_vector3(Vec3::new(-d.n.x, 0.0, -d.n.y));
        let Some(mut mesh) = meshes.get_mut(&h.0) else { continue };
        let pos: Vec<[f32; 3]> = bm.base.iter().map(|v| {
            let q = bm.to.transform_point3(Vec3::from(*v));
            let dist = Vec2::new(q.x, q.z).distance(at);
            let f = (1.0 - dist / DENT_R).clamp(0.0, 1.0);
            let f = f * f * (3.0 - 2.0 * f);
            (Vec3::from(*v) + push * d.depth * f).to_array()
        }).collect();
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, VertexAttributeValues::Float32x3(pos));
    }
}
