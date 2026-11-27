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
/// the chassis's rock on its tires (drawn only): rad per m/s^2 of acceleration, at most, how fast
/// it rocks (Hz) and how soon that dies down
const TILT_PER_ACCEL: f32 = 0.0017;
const TILT_MAX: f32 = 0.021;
const TILT_HZ: f32 = 3.2;
const TILT_DAMP: f32 = 0.45;

#[derive(Deserialize)]
pub struct Rig {
    /// its robots.json key
    #[serde(default)]
    key: String,
    nodes: HashMap<String, NodeRest>,
    pose: HashMap<String, PoseInput>,
    rate: HashMap<String, HashMap<String, [f32; 3]>>,
    modules: Vec<Module>,
    /// the nets over its load (net.rs)
    #[serde(default)]
    nets: Vec<crate::net::RigNet>,
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
    /// the inputs its parts were posed from this frame (for its nets)
    inputs: Vec<(String, f32)>,
    nets_made: bool,
    /// drawn only: how far the chassis is pitched (nose up +) and rolled (right side down +) by
    /// its acceleration, rad; how fast those are changing; its velocity last frame
    tilt: Vec2,
    tilt_v: Vec2,
    last_v: Option<Vec3>,
}

impl RobotModel {
    /// its robots.json key
    pub fn key(&self) -> &str { &self.rig.key }
    /// its swerve modules' centers (x forward, z right, m)
    pub fn module_points(&self) -> Vec<Vec2> { self.rig.modules.iter().map(|m| Vec2::new(m.x, m.z)).collect() }
    /// the parts that move (each with what's under it)
    pub fn moving_parts(&self) -> impl Iterator<Item = Entity> + '_ { self.parts.values().copied() }
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
            RobotModel { robot: i, rig: Arc::new(rig), parts: HashMap::new(), spun: HashMap::new(), wheel_turn: vec![0.0; n], steer: vec![0.0; n], ready: false, inputs: Vec::new(), nets_made: false, tilt: Vec2::ZERO, tilt_v: Vec2::ZERO, last_v: None },
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
        app.add_systems(PostUpdate, (animate, drape_nets, find_plates, bow_plates).chain().before(TransformSystems::Propagate));
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
            // the chassis rocks on its tires as it speeds up, brakes, turns and is hit (drawn only:
            // the sim's body stays as it is): the weight goes to the side it's thrown toward, which
            // squats as the other lifts, about that side's wheels
            let v = b.linvel();
            let q = *b.rotation();
            let (fwd, right) = (q * Vec3::X, q * Vec3::Z);
            if let (Some(lv), true) = (m.last_v, dt > 1e-4) {
                let a = (v - lv) / dt.max(1.0 / 120.0);
                // (rad per m/s^2: 12 m/s^2, the tires' grip, rocks it 1.2 degrees)
                let want = Vec2::new(a.dot(fwd), -a.dot(right)) * TILT_PER_ACCEL;
                let want = want.clamp(Vec2::splat(-TILT_MAX), Vec2::splat(TILT_MAX));
                let (k, c) = ((std::f32::consts::TAU * TILT_HZ).powi(2), 2.0 * TILT_DAMP * std::f32::consts::TAU * TILT_HZ);
                let (mut t, mut tv) = (m.tilt, m.tilt_v);
                let mut left = dt.min(0.1);
                while left > 0.0 {
                    let h = left.min(0.004);
                    tv += ((want - t) * k - tv * c) * h;
                    t += tv * h;
                    left -= h;
                }
                m.tilt = t.clamp(Vec2::splat(-1.5 * TILT_MAX), Vec2::splat(1.5 * TILT_MAX));
                m.tilt_v = tv;
            }
            m.last_v = Some(v);
            // (about the edge it tips toward: the wheels on that side stay on the carpet)
            let reach = m.rig.modules.iter().fold(Vec2::splat(0.2), |a, md| a.max(Vec2::new(md.x.abs(), md.z.abs())));
            let tilt = m.tilt;
            let pivot = Vec3::new(-tilt.x.signum() * reach.x, 0.0, tilt.y.signum() * reach.y);
            let tq = Quat::from_rotation_z(tilt.x) * Quat::from_rotation_x(tilt.y);
            root.translation += q * (pivot - tq * pivot);
            root.rotation = q * tq;
        }
        if !m.ready { continue }
        let m = &mut *m;
        // the state anim reads (js robot.js visualState().anim); what isn't simulated yet rests
        use sim::robot::Phys;
        // (a lid on a climber, 1678's: up as far as its climber has it, 0-1)
        let lift = match &r.phys { Some(Phys::R1678(p)) => p.lift_now(r.cfg["bay"]["lift"]["h"].as_f64().unwrap_or(0.0)), _ => 0.0 };
        let mut inputs: Vec<(&str, f32)> = vec![("lift", lift as f32), ("intakeDeploy", r.intake_deploy as f32), ("hopperDeploy", r.hopper_deploy as f32), ("hoodDeg", r.hood_deg as f32), ("turretYaw", r.turret_yaw as f32)];
        // (twin turrets: each its own yaw)
        if r.turret_yaws.len() > 1 { inputs.push(("turretYaws.0", r.turret_yaws[0] as f32)); inputs.push(("turretYaws.1", r.turret_yaws[1] as f32)); }
        // (the hopper floor's rollers as fast as they turn, 2910's; the rest at the rates the robot's
        // drawing spins them: the intake's rollers, the flywheel, the feed when it's feeding, its
        // floor's when that's not simulated)
        let floor = match &r.phys { Some(Phys::R2910(p)) => p.floor_spin() as f32, Some(Phys::R1114(p)) => p.floor_spin() as f32, _ => 0.0 };
        let feeding = if m.rig.rate.contains_key("floorSpin") { 0.0 } else { r.out.feeding as f32 };
        let mut rates: Vec<(&str, f32)> = vec![("floorSpin", floor), ("feeding", feeding), ("intakeSpeed", r.out.intake_speed as f32), ("flywheel", r.flywheel as f32)];
        // (a generic robot: its rotor's turn, its intake's rollers as fast as they turn, slowed while
        // FUEL waits in them, and its feed while it's feeding)
        if let Some(g) = &r.generic {
            inputs.push(("rotorAngle", g.hopper.fin_angle.rem_euclid(std::f64::consts::TAU) as f32));
            let s = g.intake_speed * if g.intake_speed > 0.0 { g.roller_speed } else { 1.0 };
            for (k, v) in rates.iter_mut() {
                if *k == "intakeSpeed" { *v = s as f32; }
                if *k == "feeding" && !m.rig.rate.contains_key("floorSpin") { *v = if g.feeding { 1.0 } else { 0.0 }; }
            }
        }
        // (what a physical robot says of its parts itself: an arm's turn, where its mechanisms have it
        // (lifted by FUEL under it, pushed by a TRENCH arm); 4414's hood and Dye Rotor; 1690's
        // mechanisms, mechPose)
        match &r.phys {
            Some(Phys::R8793(p)) => inputs.push(("armTurn", p.arm_turn() as f32)),
            Some(Phys::R1678(p)) => inputs.push(("armTurn", p.arm_turn() as f32)),
            Some(Phys::R341(p)) => { inputs.push(("armTurn", p.arm_turn() as f32)); inputs.push(("hoodTurn", p.hood_turn() as f32)); }
            Some(Phys::R6328(p)) => { inputs.push(("armTurn", p.arm_turn() as f32)); inputs.push(("hoodTurn", p.hood_turn() as f32)); }
            Some(Phys::R581(p)) => { inputs.push(("floorTurn", p.floor_turn() as f32)); inputs.push(("hoodTurn", p.hood_turn() as f32)); }
            Some(Phys::R604(p)) => { inputs.push(("armTurn", p.arm_turn() as f32)); inputs.push(("hoodTurn", p.hood_turn() as f32)); inputs.push(("rotorTurn", p.rotor_turn().rem_euclid(std::f64::consts::TAU) as f32)); }
            Some(Phys::R4414(p)) => { inputs.push(("hoodTurn", p.hood_turn() as f32)); inputs.push(("rotorTurn", p.rotor_turn().rem_euclid(std::f64::consts::TAU) as f32)); }
            Some(Phys::R1690(p)) => for (k, v) in p.mech_pose() { if m.rig.rate.contains_key(k) { rates.push((k, v as f32)); } else { inputs.push((k, v as f32)); } },
            Some(Phys::R7769(p)) => for (k, v) in p.mech_pose() { if m.rig.rate.contains_key(k) { rates.push((k, v as f32)); } else { inputs.push((k, v as f32)); } },
            Some(Phys::R1706(p)) => {
                inputs.push(("armTurn", p.arm_turn() as f32));
                // (the spindexers' turns, 0-2pi as the rig samples them)
                for (k, v) in p.mech_pose() { if k.starts_with("mech.spin") { inputs.push((k, v.rem_euclid(std::f64::consts::TAU) as f32)); } }
            }
            Some(Phys::R2056(p)) => for (k, v) in p.mech_pose() { if m.rig.rate.contains_key(k) { rates.push((k, v as f32)); } else { inputs.push((k, v as f32)); } },
            _ => {}
        }
        pose_parts(m, &inputs, &rates, dt, &mut parts, &time);
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

/// The parts posed from the inputs anim reads (and spun by the rates): js anim(state, dt)
fn pose_parts(m: &mut RobotModel, inputs: &[(&str, f32)], rates: &[(&str, f32)], dt: f32, parts: &mut Query<&mut Transform, Without<RobotModel>>, time: &Time) {
    // (and those take over the parts they move from the inputs anim reads otherwise)
    let pose_taken = |node: &str, b: &str| inputs.iter().any(|(a, _)| overrides(a, b) && m.rig.pose.get(*a).is_some_and(|ch| ch.nodes.contains_key(node)));
    let rate_taken = |node: &str, b: &str| rates.iter().any(|(a, _)| overrides(a, b) && m.rig.rate.get(*a).is_some_and(|ch| ch.contains_key(node)));
    let mut spin: Vec<(String, [f32; 3])> = Vec::new();
    for (name, v) in rates {
        let Some(ch) = m.rig.rate.get(*name) else { continue };
        for (node, d) in ch {
            if rate_taken(node, name) { continue; }
            spin.push((node.clone(), [d[0] * v * dt, d[1] * v * dt, d[2] * v * dt]));
        }
    }
    for (node, d) in spin {
        let s = m.spun.entry(node).or_insert([0.0; 3]);
        for j in 0..3 { s[j] = (s[j] + d[j]) % std::f32::consts::TAU; }
    }
    m.inputs = inputs.iter().map(|(k, v)| (k.to_string(), *v)).collect();
    let skip: Vec<(String, String)> = m.rig.nodes.keys().flat_map(|node| inputs.iter().filter(|(b, _)| pose_taken(node, b)).map(|(b, _)| (node.clone(), b.to_string())).collect::<Vec<_>>()).collect();
    for (node, rest) in &m.rig.nodes {
        let Some(&e) = m.parts.get(node) else { continue };
        let mut p = rest.rest;
        for &(name, v) in inputs {
            if skip.iter().any(|(n, b)| n == node && b == name) { continue; }
            let Some(ch) = m.rig.pose.get(name) else { continue };
            let Some(rows) = ch.nodes.get(node) else { continue };
            let row = lerp_row(&ch.samples, rows, v);
            for j in 0..9 { p[j] += row[j] - rest.rest[j]; }
        }
        if let Some(s) = m.spun.get(node) { for j in 0..3 { p[3 + j] += s[j]; } }
        if let Ok(mut t) = parts.get_mut(e) {
            // (REBUILT_RIG_LOG=1: each posed part where it's put, once a second)
            if std::env::var_os("REBUILT_RIG_LOG").is_some() && (time.elapsed_secs() % 1.0) < dt && m.rig.pose.values().any(|ch| ch.nodes.contains_key(node)) {
                info!("rig {node}: {:?} (inputs {:?})", &p[..6], inputs);
            }
            t.translation = Vec3::new(p[0], p[1], p[2]);
            t.rotation = euler(&rest.order, p[3], p[4], p[5]);
            t.scale = Vec3::new(p[6], p[7], p[8]);
        }
    }
}

/// Which of a robot's nets this is
#[derive(Component)]
struct NetOf { model: Entity, i: usize }

/// js `net.drape(st.load, ex, at)` each frame: its nets made once its model's in (in the robot's
/// frame, on the model), then draped over the FUEL it holds as its state has them
fn drape_nets(
    game: Res<Game>,
    mut models: Query<(Entity, &mut RobotModel)>,
    mut nets: Query<(&mut crate::net::Net, &NetOf, &mut Visibility)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut tex: Local<Option<Handle<Image>>>,
    mut commands: Commands,
) {
    let w = &game.0;
    for (e, mut m) in &mut models {
        if !m.ready || m.nets_made || m.rig.nets.is_empty() { continue; }
        m.nets_made = true;
        let t = tex.get_or_insert_with(|| crate::net::net_texture(&mut images)).clone();
        for (i, rn) in m.rig.nets.iter().enumerate() {
            // (nets hang on the model's root: others, on a part that moves, aren't drawn yet)
            if rn.parent != "rig0" { continue; }
            let (net, mesh) = crate::net::Net::new(rn.spec.clone(), &mut meshes);
            let c = rn.spec.color.unwrap_or(0x15171a);
            let mat = mats.add(StandardMaterial {
                base_color: Color::srgb_u8((c >> 16) as u8, (c >> 8) as u8, c as u8),
                base_color_texture: Some(t.clone()),
                alpha_mode: AlphaMode::Blend,
                double_sided: true,
                cull_mode: None,
                perceptual_roughness: 0.9,
                ..default()
            });
            let child = commands.spawn((Mesh3d(mesh), MeshMaterial3d(mat), Transform::default(), Visibility::Hidden, bevy::light::NotShadowCaster, net, NetOf { model: e, i })).id();
            commands.entity(e).add_child(child);
            info!("robot {} net {i}: {}x{} over x {:.2}..{:.2}", m.robot, rn.spec.nx, rn.spec.nz, rn.spec.x0, rn.spec.x1_out);
        }
    }
    for (mut net, of, mut vis) in &mut nets {
        let Ok((_, m)) = models.get(of.model) else { continue };
        let Some(r) = w.robots.get(m.robot) else { continue };
        let rn = &m.rig.nets[of.i];
        // (an input another one takes over from: as the parts' poses, robot_model's overrides)
        let taken = |b: &str| m.inputs.iter().any(|(a, _)| overrides(a, b) && rn.by.contains_key(a.as_str()));
        let a = rn.args(&m.inputs, &taken);
        let shown = a[4].unwrap_or(1.0) > 0.5;
        let v = if shown { Visibility::Inherited } else { Visibility::Hidden };
        if *vis != v { *vis = v; }
        if !shown { continue; }
        // (the FUEL it holds, in its frame: js netLoad)
        let Some(b) = w.physics.bodies.get(r.body) else { continue };
        let (bt, bq) = (b.translation(), b.rotation().inverse());
        let load: Vec<[f64; 3]> = if let Some(g) = &r.generic { g.hopper.list.iter().map(|e| e.p).collect() } else { r.stored.iter().map(|&k| {
            let p = w.fuel.balls[k].pos;
            let l = bq * (Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32) - bt);
            [l.x as f64, l.y as f64, l.z as f64]
        }).collect() };
        net.drape(&load, a[0].unwrap_or(0.0), a[1], a[2], a[3], &mut meshes);
    }
}

/// Whether input a, when a robot gives it, takes over the parts input b would move
fn overrides(a: &str, b: &str) -> bool {
    match a {
        "armTurn" => b == "intakeDeploy",
        "hoodTurn" => b == "hoodDeg",
        "rotorTurn" => b == "rotorAngle",
        "turretYaws.0" | "turretYaws.1" => b == "turretYaw",
        _ if a.starts_with("mech.") => matches!(b, "intakeDeploy" | "hopperDeploy" | "hoodDeg" | "intakeSpeed" | "flywheel" | "feeding" | "floorSpin"),
        _ => false,
    }
}

/// A model with no robot under it (the robot select screen's, showcase.rs): every posed part
/// swung through its range in turn, every spinning one turning, its modules steering and rolling
pub fn pose_demo(m: &mut RobotModel, t: f32, dt: f32, parts: &mut Query<&mut Transform, Without<RobotModel>>, time: &Time) {
    if !m.ready { return }
    let mut keys: Vec<&String> = m.rig.pose.keys().collect();
    keys.sort();
    // (each input out and back over 5 s, one after another a second and a half apart; the
    // nets' and lid's inputs too: they rest at their first sample)
    let vals: Vec<(String, f32)> = keys.iter().enumerate().map(|(i, k)| {
        let s = &m.rig.pose[*k].samples;
        let (a, b) = (s.first().copied().unwrap_or(0.0), s.last().copied().unwrap_or(0.0));
        let ph = ((t - i as f32 * 1.5) / 5.0).rem_euclid(1.0 + keys.len() as f32 * 0.3);
        let u = if ph < 1.0 { 0.5 - 0.5 * (ph * std::f32::consts::TAU).cos() } else { 0.0 };
        (k.to_string(), a + (b - a) * u)
    }).collect();
    let inputs: Vec<(&str, f32)> = vals.iter().map(|(k, v)| (k.as_str(), *v)).collect();
    let rk: Vec<String> = m.rig.rate.keys().cloned().collect();
    let rates: Vec<(&str, f32)> = rk.iter().map(|k| (k.as_str(), 1.0)).collect();
    pose_parts(m, &inputs, &rates, dt, parts, time);
    let steer = 0.5 * (t * 0.7).sin();
    for (i, md) in m.rig.modules.iter().enumerate() {
        m.steer[i] = steer;
        m.wheel_turn[i] = (m.wheel_turn[i] - 1.2 * dt / WHEEL_R) % std::f32::consts::TAU;
        for (node, y) in [(&md.pivot, m.steer[i]), (&md.wheel, m.wheel_turn[i])] {
            let (Some(&e), Some(rest)) = (m.parts.get(node), m.rig.nodes.get(node)) else { continue };
            let p = rest.rest;
            if let Ok(mut tr) = parts.get_mut(e) { tr.rotation = euler(&rest.order, p[3], y, p[5]); }
        }
    }
}

/// A lattice top plate the load bows up (1690's: js robotModels.js bowPlate): its mesh (this
/// robot's own copy), its vertices as the CAD has them (robot frame: the plate hangs on the model's
/// root unmoved), and the bow it was last drawn at
#[derive(Component)]
struct BowPlate { robot: usize, base: Vec<[f32; 3]>, last: f32 }
/// a model whose plates have been looked for
#[derive(Component)]
struct PlatesFound;

/// Once a 1690's model is in: its top plate's meshes (thin, flat, ~0.52-0.55 m up, spanning the
/// hopper), each given its own copy of its mesh to bend
fn find_plates(
    models: Query<(Entity, &RobotModel), Without<PlatesFound>>,
    children: Query<&Children>,
    meshes_q: Query<&Mesh3d>,
    game: Res<Game>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut commands: Commands,
) {
    use bevy::mesh::VertexAttributeValues;
    for (root, m) in &models {
        if !m.ready { continue }
        commands.entity(root).insert(PlatesFound);
        let Some(r) = game.0.robots.get(m.robot) else { continue };
        if !matches!(r.phys, Some(sim::robot::Phys::R1690(_))) { continue }
        for e in children.iter_descendants(root) {
            let Ok(h) = meshes_q.get(e) else { continue };
            let Some(mesh) = meshes.get(&h.0) else { continue };
            let Some(VertexAttributeValues::Float32x3(p)) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else { continue };
            let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
            for v in p { for k in 0..3 { lo[k] = lo[k].min(v[k]); hi[k] = hi[k].max(v[k]); } }
            if !(hi[1] - lo[1] < 0.045 && lo[1] > 0.48 && hi[0] - lo[0] > 0.4 && hi[2] - lo[2] > 0.4) { continue }
            let base = p.clone();
            let mesh = mesh.clone();
            let copy = meshes.add(mesh);
            info!("robot {} top plate: {} vertices to bow", m.robot, base.len());
            commands.entity(e).insert((Mesh3d(copy), BowPlate { robot: m.robot, base, last: 0.0 }));
        }
    }
}

/// Each frame: the plate bent as far as the load bows it (js r1690.js bowShape: most at its free
/// front edge's middle, nothing at its supports)
fn bow_plates(mut plates: Query<(&mut BowPlate, &Mesh3d)>, game: Res<Game>, mut meshes: ResMut<Assets<Mesh>>) {
    use bevy::mesh::VertexAttributeValues;
    for (mut bp, h) in &mut plates {
        let Some(r) = game.0.robots.get(bp.robot) else { continue };
        let Some(sim::robot::Phys::R1690(p)) = &r.phys else { continue };
        // (REBUILT_SHOT_BOW=m: a bow to see it by, for the screenshot harness: shot.rs)
        let bow = std::env::var("REBUILT_SHOT_BOW").ok().and_then(|v| v.parse().ok()).unwrap_or(p.bow() as f32);
        if (bow - bp.last).abs() < 0.0005 { continue }
        bp.last = bow;
        let Some(mut mesh) = meshes.get_mut(&h.0) else { continue };
        let pos: Vec<[f32; 3]> = bp.base.iter().map(|v| [v[0], v[1] + bow * p.bow_shape(v[0] as f64, v[2] as f64) as f32, v[2]]).collect();
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, VertexAttributeValues::Float32x3(pos));
        if mesh.indices().is_some() { mesh.compute_smooth_normals(); } else { mesh.compute_flat_normals(); }
    }
}
