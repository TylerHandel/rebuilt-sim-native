//! The robots as the browser game draws them: assets/models/<key>.glb, the model it builds, and
//! <key>.rig.json, how that model's moving parts follow the robot's state (both exported by
//! tools/export-models.mjs). Each frame the parts are posed from the sim's robot as the game's
//! anim(state, dt) poses them (intake, hopper, hood; rollers spin by their rates) and the swerve
//! modules steer and roll as js/robot.js turns them. Until a model loads (or where there's none)
//! the robot is drawn as its colliders (draw.rs).

use bevy::gltf::GltfMaterialName;
use bevy::prelude::*;
use bevy::world_serialization::{WorldAssetRoot, WorldInstanceReady};
use serde::Deserialize;
use sim::constants::Alliance;
use std::collections::HashMap;
use std::sync::Arc;

use crate::game::Game;

/// js robot.js WHEEL_R: the drawn wheels' radius, which their spin is figured from
const WHEEL_R: f32 = 0.05;

#[derive(Deserialize)]
pub struct Rig {
    nodes: HashMap<String, NodeRest>,
    pose: HashMap<String, PoseInput>,
    rate: HashMap<String, HashMap<String, [f32; 3]>>,
    modules: Vec<Module>,
}

#[derive(Deserialize)]
struct NodeRest {
    /// position, Euler rotation, scale
    rest: [f32; 9],
    order: String,
}

#[derive(Deserialize)]
struct PoseInput {
    samples: Vec<f32>,
    nodes: HashMap<String, Vec<[f32; 9]>>,
}

#[derive(Deserialize)]
struct Module {
    pivot: String,
    wheel: String,
    x: f32,
    z: f32,
}

/// Where the assets are on disk (the rigs are read directly).
#[derive(Resource)]
pub struct AssetDir(pub std::path::PathBuf);

/// A robot's model: the sim's robot it draws, its rig, and its parts once the GLB is in.
#[derive(Component)]
pub struct RobotModel {
    pub robot: usize,
    rig: Arc<Rig>,
    parts: HashMap<String, Entity>,
    /// what the rates have turned each part by (Euler)
    spun: HashMap<String, [f32; 3]>,
    wheel_turn: Vec<f32>,
    steer: Vec<f32>,
    pub ready: bool,
}

/// The model of robot `i` (its key), if it has one: spawned at the robot's pose
pub fn spawn(commands: &mut Commands, assets: &AssetServer, dir: &AssetDir, i: usize, key: &str) -> Option<Entity> {
    let rig_path = dir.0.join("models").join(format!("{key}.rig.json"));
    let rig: Rig = match std::fs::read_to_string(&rig_path).map(|s| serde_json::from_str(&s)) {
        Ok(Ok(r)) => r,
        Ok(Err(e)) => { warn!("{}: {e}", rig_path.display()); return None }
        Err(_) => return None,
    };
    let n = rig.modules.len();
    let scene = assets.load(GltfAssetLabel::Scene(0).from_asset(format!("models/{key}.glb")));
    let e = commands
        .spawn((
            Transform::default(),
            Visibility::default(),
            RobotModel { robot: i, rig: Arc::new(rig), parts: HashMap::new(), spun: HashMap::new(), wheel_turn: vec![0.0; n], steer: vec![0.0; n], ready: false },
        ))
        .with_children(|p| { p.spawn((WorldAssetRoot(scene), Transform::default())).observe(on_ready); })
        .id();
    Some(e)
}

/// The GLB is in: find its moving parts by name, and put the alliance's color on the bumpers
fn on_ready(
    ev: On<WorldInstanceReady>,
    parents: Query<&ChildOf>,
    children: Query<&Children>,
    names: Query<&Name>,
    mat_names: Query<(&GltfMaterialName, &MeshMaterial3d<StandardMaterial>)>,
    mut models: Query<&mut RobotModel>,
    game: Res<Game>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut commands: Commands,
) {
    let Ok(parent) = parents.get(ev.entity) else { return };
    let Ok(mut m) = models.get_mut(parent.parent()) else { return };
    let mut parts = HashMap::new();
    let mut red = None;
    let is_red = game.0.robots.get(m.robot).is_some_and(|r| r.alliance == Alliance::Red);
    for e in children.iter_descendants(ev.entity) {
        if let Ok(n) = names.get(e) {
            if m.rig.nodes.contains_key(n.as_str()) { parts.insert(n.as_str().to_string(), e); }
        }
        if is_red {
            if let Ok((name, mat)) = mat_names.get(e) {
                if name.0 == "alliance" {
                    let h = red.get_or_insert_with(|| {
                        let mut c = mats.get(&mat.0).cloned().unwrap_or_default();
                        // js field.js ALLIANCE_COLOR_CSS.red
                        c.base_color = Color::srgb_u8(0xe0, 0x34, 0x34);
                        mats.add(c)
                    }).clone();
                    commands.entity(e).insert(MeshMaterial3d(h));
                }
            }
        }
    }
    info!("robot {} model in: {} of its {} moving parts found", m.robot, parts.len(), m.rig.nodes.len());
    m.parts = parts;
    m.ready = true;
}

fn euler(order: &str, x: f32, y: f32, z: f32) -> Quat {
    // (three.js's orders are intrinsic, as glam's plain ones are)
    let o = match order {
        "XZY" => EulerRot::XZY, "YXZ" => EulerRot::YXZ, "YZX" => EulerRot::YZX,
        "ZXY" => EulerRot::ZXY, "ZYX" => EulerRot::ZYX, _ => EulerRot::XYZ,
    };
    let (a, b, c) = match o {
        EulerRot::XZY => (x, z, y), EulerRot::YXZ => (y, x, z), EulerRot::YZX => (y, z, x),
        EulerRot::ZXY => (z, x, y), EulerRot::ZYX => (z, y, x), _ => (x, y, z),
    };
    Quat::from_euler(o, a, b, c)
}

fn lerp_row(samples: &[f32], rows: &[[f32; 9]], v: f32) -> [f32; 9] {
    let n = samples.len();
    if n == 0 { return [0.0; 9] }
    if v <= samples[0] { return rows[0] }
    if v >= samples[n - 1] { return rows[n - 1] }
    let k = samples.partition_point(|s| *s <= v).clamp(1, n - 1);
    let t = (v - samples[k - 1]) / (samples[k] - samples[k - 1]);
    let mut out = [0.0; 9];
    for j in 0..9 { out[j] = rows[k - 1][j] + (rows[k][j] - rows[k - 1][j]) * t; }
    out
}

pub struct RobotModelPlugin;

impl Plugin for RobotModelPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PostUpdate, animate.before(TransformSystems::Propagate));
    }
}

/// Each frame: the model at the robot's pose, its parts posed from its state
fn animate(game: Res<Game>, time: Res<Time>, mut models: Query<(&mut RobotModel, &mut Transform)>, mut parts: Query<&mut Transform, Without<RobotModel>>) {
    let dt = time.delta_secs();
    let w = &game.0;
    for (mut m, mut root) in &mut models {
        let Some(r) = w.robots.get(m.robot) else { continue };
        if let Some(b) = w.physics.bodies.get(r.body) {
            root.translation = b.translation();
            root.rotation = *b.rotation();
        }
        if !m.ready { continue }
        let m = &mut *m;
        // the state anim reads (js robot.js visualState().anim); what isn't simulated yet rests
        // (an arm its mechanisms turn, 8793's, drawn where it is: lifted by FUEL under it, or pushed
        // round by a TRENCH arm; its rig's arm swings linearly with intakeDeploy)
        let deploy = match &r.phys { Some(sim::robot::Phys::R8793(p)) => 1.0 - p.arm_turn() / p.arm.stow, _ => r.intake_deploy };
        // (a lid on a climber, 1678's: up as far as its climber has it, 0-1)
        let lift = match &r.phys { Some(sim::robot::Phys::R1678(p)) => p.lift_now(r.cfg["bay"]["lift"]["h"].as_f64().unwrap_or(0.0)), _ => 0.0 };
        let inputs: [(&str, f32); 5] = [("lift", lift as f32), ("intakeDeploy", deploy as f32), ("hopperDeploy", r.hopper_deploy as f32), ("hoodDeg", r.hood_deg as f32), ("turretYaw", r.turret_yaw as f32)];
        // (the hopper floor's rollers as fast as they turn, 2910's; the rest at the rates the robot's
        // drawing spins them: the intake's rollers, the flywheel, the feed when it's feeding, its
        // floor's when that's not simulated)
        let floor = match &r.phys { Some(sim::robot::Phys::R2910(p)) => p.floor_spin() as f32, _ => 0.0 };
        let feeding = if m.rig.rate.contains_key("floorSpin") { 0.0 } else { r.out.feeding as f32 };
        let rates: [(&str, f32); 4] = [("floorSpin", floor), ("feeding", feeding), ("intakeSpeed", r.out.intake_speed as f32), ("flywheel", r.flywheel as f32)];
        for (name, v) in rates {
            let Some(ch) = m.rig.rate.get(name) else { continue };
            for (node, d) in ch {
                let s = m.spun.entry(node.clone()).or_insert([0.0; 3]);
                for j in 0..3 { s[j] = (s[j] + d[j] * v * dt) % std::f32::consts::TAU; }
            }
        }
        for (node, rest) in &m.rig.nodes {
            let Some(&e) = m.parts.get(node) else { continue };
            let mut p = rest.rest;
            for (name, v) in inputs {
                let Some(ch) = m.rig.pose.get(name) else { continue };
                let Some(rows) = ch.nodes.get(node) else { continue };
                let row = lerp_row(&ch.samples, rows, v);
                for j in 0..9 { p[j] += row[j] - rest.rest[j]; }
            }
            if let Some(s) = m.spun.get(node) { for j in 0..3 { p[3 + j] += s[j]; } }
            if let Ok(mut t) = parts.get_mut(e) {
                t.translation = Vec3::new(p[0], p[1], p[2]);
                t.rotation = euler(&rest.order, p[3], p[4], p[5]);
                t.scale = Vec3::new(p[6], p[7], p[8]);
            }
        }
        // the swerve modules steer to their motion and roll (js robot.js)
        let (c, sn) = (r.yaw.cos() as f32, r.yaw.sin() as f32);
        let (vx0, vz0) = (r.vel[0] as f32, r.vel[2] as f32);
        let (lx, lz) = (vx0 * c - vz0 * sn, vx0 * sn + vz0 * c);
        let om = r.omega as f32;
        for (i, md) in m.rig.modules.iter().enumerate() {
            let (vx, vz) = (lx + om * md.z, lz - om * md.x);
            let sp = vx.hypot(vz);
            if sp > 0.05 { m.steer[i] = -vz.atan2(vx); }
            m.wheel_turn[i] = (m.wheel_turn[i] - sp * dt / WHEEL_R) % std::f32::consts::TAU;
            for (node, y) in [(&md.pivot, m.steer[i]), (&md.wheel, m.wheel_turn[i])] {
                let (Some(&e), Some(rest)) = (m.parts.get(node), m.rig.nodes.get(node)) else { continue };
                let p = rest.rest;
                if let Ok(mut t) = parts.get_mut(e) { t.rotation = euler(&rest.order, p[3], y, p[5]); }
            }
        }
    }
}
