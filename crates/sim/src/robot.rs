//! js/robot.js, so far for a physical robot driving with nothing in it: its rigid body as built
//! (_createBody), spawn, the drive (_preStepDrive, _alongWalls, _wheelsDown, tip damping), the
//! hopper sliding out (_hopper) and the intake's collider (_stepTopColliders). Shooting, the
//! intake's FUEL and the pose estimate arrive with the mechanisms (M3).
use crate::constants::*;
use crate::drivetrain::{DriveCfg, Drivetrain, SwerveDrive};
use crate::mech::phys2910::{Phys2910, PostIn, PostOut};
use crate::mech::phys4414::Phys4414;
use crate::mech::phys8793::Phys8793;
use crate::mech::phys1678::Phys1678;
use crate::mech::phys1690::Phys1690;
use crate::mech::phys7769::Phys7769;
use crate::mech::phys341::Phys341;
use crate::mech::phys1114::Phys1114;
use crate::mech::physical::{Ctx, Stats};
use crate::fuel::Fuel;
use crate::parity::Cmd;
use jsmath as m;
use rapier3d::prelude::*;
use serde_json::Value;

const WHEEL_R: f64 = 0.05;
const TIP_RATE: f64 = 2.5;
const TIP_DAMP: f64 = 4.0;
const DEG: f64 = m::PI / 180.0;
/// js AIM_VEL_TAU, AIM_GAIN
const AIM_VEL_TAU: f64 = 0.05;
const AIM_GAIN: f64 = 14.0;
/// js CLIMB_LIFT: the body's lift that makes LEVEL 1, 2, 3
pub const CLIMB_LIFT: [f64; 4] = [0.0, 0.16, 0.74, 1.2];

/// js robot.climbState (no automatic lining up here: a climber is worked by hand)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClimbState { None, Climbing, Hanging, Descending }
/// js cfg.climber: { maxLevel, times }
#[derive(Clone, Debug)]
pub struct ClimberCfg { pub max_level: usize, pub times: Vec<f64> }

fn num(v: &Value, k: &str) -> f64 { v[k].as_f64().unwrap_or_else(|| panic!("config: no number {k}")) }
fn opt(v: &Value, k: &str) -> Option<f64> { v.get(k).and_then(|x| x.as_f64()) }
fn vf(x: f64, y: f64, z: f64) -> Vector { Vector::new(x as f32, y as f32, z as f32) }
fn grp(g: u32) -> InteractionGroups { crate::groups(g) }

/// three.js Vector3.applyQuaternion (r169), on f64s
pub fn apply_quat(v: [f64; 3], q: [f64; 4]) -> [f64; 3] {
    let [vx, vy, vz] = v;
    let [qx, qy, qz, qw] = q;
    let tx = 2.0 * (qy * vz - qz * vy);
    let ty = 2.0 * (qz * vx - qx * vz);
    let tz = 2.0 * (qx * vy - qy * vx);
    [vx + qw * tx + qy * tz - qz * ty, vy + qw * ty + qz * tx - qx * tz, vz + qw * tz + qx * ty - qy * tx]
}

/// The robots whose mechanisms are ported (a robot.json config's "physical"): the others' Robot::new
/// panics until theirs are
pub const PORTED: &[&str] = &["2910", "4414", "8793", "1678", "1690", "7769", "341", "1114"];
/// The ported robots whose intake folds (on a pivot their mechanisms turn)
pub const FOLDS: &[&str] = &["8793"];

/// Whether this sim can build robot `cfg` yet (its mechanisms and intake are ported)
pub fn ported(cfg: &Value) -> bool {
    let ic = &cfg["intake"];
    let folds = ic.get("fold").is_some_and(|x| !x.is_null()) || ic.get("slide").is_some_and(|x| !x.is_null());
    let p = cfg.get("physical").and_then(|x| x.as_str());
    p.is_some_and(|p| PORTED.contains(&p)) && (!folds || p.is_some_and(|p| FOLDS.contains(&p)))
}

/// js robotConfigs `frameShape(cfg)`
pub fn frame_shape(cfg: &Value) -> Vec<[f64; 2]> {
    let f = &cfg["frame"];
    let (l, w) = (num(f, "length"), num(f, "width"));
    if let Some(round) = f.get("round").filter(|x| !x.is_null()) {
        let (r, cx, front) = (num(round, "r"), num(round, "cx"), num(round, "front"));
        let mut pts = vec![[l / 2.0, front / 2.0]];
        for i in 0..=24 { let a = m::PI / 2.0 + (i as f64 / 24.0) * m::PI; pts.push([cx + r * m::cos(a), r * m::sin(a)]); }
        pts.push([l / 2.0, -front / 2.0]);
        return pts;
    }
    let c = opt(f, "chamfer").unwrap_or(0.0);
    let mut pts = vec![[l / 2.0, w / 2.0], [l / 2.0, -w / 2.0]];
    if c > 0.0 { pts.extend([[-l / 2.0 + c, -w / 2.0], [-l / 2.0, -w / 2.0 + c], [-l / 2.0, w / 2.0 - c], [-l / 2.0 + c, w / 2.0]]); }
    else { pts.extend([[-l / 2.0, -w / 2.0], [-l / 2.0, w / 2.0]]); }
    pts
}

/// js robotConfigs `offsetShape(pts, d)`
pub fn offset_shape(pts: &[[f64; 2]], d: f64) -> Vec<[f64; 2]> {
    let n = pts.len();
    let mut area = 0.0;
    for i in 0..n { let (a, b) = (pts[i], pts[(i + 1) % n]); area += a[0] * b[1] - b[0] * a[1]; }
    let s = if area > 0.0 { 1.0 } else { -1.0 };
    let nrm = |a: [f64; 2], b: [f64; 2]| { let (dx, dz) = (b[0] - a[0], b[1] - a[1]); let l0 = m::hypot2(dx, dz); let l = if l0 == 0.0 { 1.0 } else { l0 }; [(s * dz) / l, (-s * dx) / l] };
    (0..n).map(|i| {
        let p = pts[i];
        let n0 = nrm(pts[(i + n - 1) % n], p);
        let n1 = nrm(p, pts[(i + 1) % n]);
        let (mx, mz) = (n0[0] + n1[0], n0[1] + n1[1]);
        let k = d / m::max(0.2, mx * n0[0] + mz * n0[1]);
        [p[0] + mx * k, p[1] + mz * k]
    }).collect()
}

/// js robotConfigs `modulePoints(cfg)`
pub fn module_points(cfg: &Value) -> Vec<[f64; 2]> {
    let f = &cfg["frame"];
    let inset = 0.075;
    if let Some(at) = cfg["drive"].get("moduleAt").and_then(|x| x.as_array()) {
        let (x, z) = (at[0].as_f64().unwrap(), at[1].as_f64().unwrap());
        return vec![[x, z], [x, -z], [-x, z], [-x, -z]];
    }
    let rw = opt(&cfg["drive"], "wheel").unwrap_or(4.0 * IN) / 2.0;
    let chamfer = opt(f, "chamfer").unwrap_or(0.0);
    let cut = if chamfer != 0.0 { m::max(inset, chamfer / 2.0 + (rw + 0.006) * m::SQRT1_2) } else { inset };
    if let Some(round) = f.get("round").filter(|x| !x.is_null()) {
        let (r, cx) = (num(round, "r"), num(round, "cx"));
        let d = (r - 0.1) * m::SQRT1_2;
        return vec![[cx + d, d], [cx + d, -d], [cx - d, d], [cx - d, -d]];
    }
    let (l, w) = (num(f, "length"), num(f, "width"));
    [[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]].iter().map(|&[sx, sz]| { let d = if sx < 0.0 { cut } else { inset }; [sx * (l / 2.0 - d), sz * (w / 2.0 - d)] }).collect()
}

/// What decides whether a collider reaches a TRENCH arm by the end of the step (physics.js
/// passesUnder's top())
#[derive(Debug, Clone)]
pub enum Top { Frame { body: RigidBodyHandle, pts: Vec<f64> }, Lid { body: RigidBodyHandle, lid: ColliderHandle, hl: f64, hw: f64 } }

/// js physics.underArm entry
#[derive(Debug, Clone)]
pub struct UnderArm { pub collider: ColliderHandle, pub top: Top, pub under: bool }

pub enum Phys { R2910(Box<Phys2910>), R4414(Box<Phys4414>), R8793(Box<Phys8793>), R1678(Box<Phys1678>), R1690(Box<Phys1690>), R7769(Box<Phys7769>), R341(Box<Phys341>), R1114(Box<Phys1114>) }

impl Phys {
    /// the shared part (js PhysicalRobot)
    pub fn physical(&self) -> &crate::mech::physical::Physical { match self { Phys::R2910(p) => &p.p, Phys::R4414(p) => &p.p, Phys::R8793(p) => &p.p, Phys::R1678(p) => &p.p, Phys::R1690(p) => &p.p, Phys::R7769(p) => &p.p, Phys::R341(p) => &p.p, Phys::R1114(p) => &p.p } }
    pub fn physical_mut(&mut self) -> &mut crate::mech::physical::Physical { match self { Phys::R2910(p) => &mut p.p, Phys::R4414(p) => &mut p.p, Phys::R8793(p) => &mut p.p, Phys::R1678(p) => &mut p.p, Phys::R1690(p) => &mut p.p, Phys::R7769(p) => &mut p.p, Phys::R341(p) => &mut p.p, Phys::R1114(p) => &mut p.p } }
}

pub struct Robot {
    pub key: String,
    pub cfg: Value,
    pub alliance: Alliance,
    pub body: RigidBodyHandle,
    pub bumper: ColliderHandle,
    pub frame: ColliderHandle,
    pub hopper: Option<(ColliderHandle, ColliderHandle)>,
    pub hopper_y: f64,
    pub arm_collider: ColliderHandle,
    pub arm_wall: Option<ColliderHandle>,
    pub intake_block: Option<(f64, f64, f64)>,
    pub lid: Option<(ColliderHandle, f64)>,
    pub wheel_pts: Vec<[f64; 2]>,
    pub half_l: f64,
    pub half_w: f64,
    pub height: f64,
    pub swerve: SwerveDrive,
    pub max_speed: f64,
    pub max_omega: f64,
    pub phys: Option<Phys>,
    pub cmd: Cmd,
    pub enabled: bool,
    pub intake_deploy: f64,
    /// js intakeWant: whether the game wants a physical robot's intake down (its arm, 8793's)
    pub intake_want: bool,
    /// js humanDriven: a person drives it (its climber is theirs to work, in TELEOP)
    pub human_driven: bool,
    /// js climberCfg, and the climb (climbState, climbLift, climbLevel, climbTarget; a climber worked
    /// by hand: climberUp, climberExt, hooked, _manual; climbBase)
    pub climber: Option<ClimberCfg>,
    pub climb_state: ClimbState,
    pub climb_lift: f64,
    pub climb_level: u8,
    pub climb_target: usize,
    pub climber_up: bool,
    pub climber_ext: f64,
    pub hooked: bool,
    manual: bool,
    climb_base: [f64; 3],
    /// whether it's TELEOP (as its last step was told)
    pub teleop: bool,
    /// js armGroups, armGroupsRigid: a folding arm's collision groups (yielding to a TRENCH arm, or not)
    pub arm_groups: u32,
    pub arm_groups_rigid: u32,
    pub hopper_deploy: f64,
    pub last_t: f64,
    pub pos: [f64; 3],
    pub quat: [f64; 4],
    pub yaw: f64,
    pub vel: [f64; 3],
    pub omega: f64,
    pub traction: f64,
    /// js aimVel: the chassis' velocity as a moving shot is aimed with (smoothed: AIM_VEL_TAU)
    pub aim_vel: [f64; 2],
    /// the FUEL it holds (fuel.balls indices), its counts, full
    pub stored: Vec<usize>,
    pub stats: Stats,
    pub full: bool,
    /// js lastInZone, wantShoot, shotSetpoint, aimOverride, aimErr, passClear
    pub last_in_zone: bool,
    pub want_shoot: bool,
    pub shot_setpoint: f64,
    pub shot: Option<Shot>,
    pub last_pass: bool,
    pub aim_override: Option<f64>,
    pub aim_err: Option<f64>,
    pub pass_clear: Option<bool>,
    /// what its mechanisms say (js hoodDeg, flywheel, ready, feeding, intakeSpeed)
    pub out: PostOut,
    /// js hoodDeg (the shot's elevation, deg), flywheel (exit speed, m/s), ready, status
    pub hood_deg: f64,
    pub flywheel: f64,
    pub ready: bool,
    pub status: String,
    /// js aimOffset: a fixed shooter facing back shoots along the heading turned around
    pub aim_offset: f64,
    /// js hubTable, passTable
    pub tables: std::sync::Arc<Tables>,
    /// js loc: what its code knows of its pose, which its aims work from (localize.rs)
    pub loc: crate::localize::Localizer,
    /// js turrets (x, z: robot frame) and their yaws (rad, on the robot), turretYaw (the first's)
    pub turrets: Vec<[f64; 2]>,
    pub turret_yaws: Vec<f64>,
    pub turret_yaw: f64,
    /// this step's dt (js _shooter's), for the turret
    step_dt: f64,
}

/// js robot.hubTable and passTable: its shot tables (built once per robot)
pub struct Tables { pub hub: crate::ballistics::ShotTable, pub pass: crate::ballistics::ShotTable }

/// What the localizer reads of a robot (js its this.r)
macro_rules! loc_view {
    ($r:expr, $others:expr) => {
        crate::localize::RobotView { pos: $r.pos, quat: $r.quat, yaw: $r.yaw, vel: $r.vel, omega: $r.omega, swerve: &$r.swerve, ready_shoot: $r.ready && $r.want_shoot, others: $others, turrets: &$r.turrets, turret_yaws: &$r.turret_yaws }
    };
}

/// js `turretGoal(rel, cur, lim, shooting)`: where the turret goes for a target rel (rad from the
/// middle of its travel), from cur, within +-lim: (goal, over: how far past a stop it's held)
pub fn turret_goal(rel: f64, cur: f64, lim: f64, shooting: bool) -> (f64, f64) {
    let (unwind, hold) = (-1.0 * DEG, 0.0 * DEG);
    let b = cur + wrap_angle(rel - cur);
    let cands: Vec<f64> = [rel, rel + 2.0 * m::PI, rel - 2.0 * m::PI].into_iter().filter(|a| m::abs(*a) <= lim).collect();
    if shooting {
        if m::abs(b) <= lim { return (b, 0.0); }
        let g = crate::fuel::clamp(b, -lim, lim);
        if m::abs(b - g) < hold || cands.is_empty() { return (g, b - g); }
        return (cands.iter().copied().reduce(|a, c| if m::abs(c - cur) < m::abs(a - cur) { c } else { a }).unwrap(), 0.0);
    }
    if cands.is_empty() { return (crate::fuel::clamp(rel, -lim, lim), 0.0); }
    let mid = cands.iter().copied().reduce(|a, c| if m::abs(c) < m::abs(a) { c } else { a }).unwrap();
    (if m::abs(b) <= lim && m::abs(b) < lim - unwind { b } else { mid }, 0.0)
}

/// js util `wrapAngle(a)`
pub fn wrap_angle(mut a: f64) -> f64 {
    while a > m::PI { a -= 2.0 * m::PI; }
    while a < -m::PI { a += 2.0 * m::PI; }
    a
}

/// The robot's shot tables (js Robot's constructor): built from the drag model, once per robot
fn tables_for(key: &str, cfg: &Value) -> std::sync::Arc<Tables> {
    use crate::ballistics::{ShotTable, TableOpts, H0};
    static CACHE: std::sync::OnceLock<std::sync::Mutex<std::collections::HashMap<String, std::sync::Arc<Tables>>>> = std::sync::OnceLock::new();
    let cache = CACHE.get_or_init(Default::default);
    if let Some(t) = cache.lock().unwrap().get(key) { return t.clone(); }
    let sh = &cfg["shooter"];
    let fixed = sh["type"].as_str() == Some("fixed");
    let hood = &sh["hood"];
    let exit_y = |deg: f64| {
        let th = (deg * m::PI) / 180.0;
        num(hood, "y") + num(hood, "r") * m::cos(th)
    };
    fn h0<'a>(fixed: bool, sh: &Value, o: &'a dyn Fn(f64) -> f64) -> H0<'a> {
        if fixed && !sh["hood"].is_null() { H0::Of(o) } else if fixed { H0::At(num(&sh["exit"], "y")) } else { H0::At(num(sh, "exitY")) }
    }
    let spin = opt(sh, "spin").unwrap_or(0.0);
    let (hmin, hmax, smax) = (num(sh, "hoodMin"), num(sh, "hoodMax"), num(sh, "speedMax"));
    let mut ho = TableOpts::new(h0(fixed, sh, &exit_y), HUB::targetHeight, hmin, hmax, smax);
    ho.clear_dist = HUB::size / 2.0 + FUEL::radius + 0.02;
    ho.clear_height = HUB::rimFront + FUEL::radius + 0.04;
    ho.spin = spin;
    let mut po = TableOpts::new(h0(fixed, sh, &exit_y), FUEL::radius + 0.02, hmin, hmax, smax);
    po.pass = true; po.pass_theta = 55.0; po.d_max = 22.0; po.spin = spin;
    let t = std::sync::Arc::new(Tables { hub: ShotTable::new(&ho), pass: ShotTable::new(&po) });
    cache.lock().unwrap().insert(key.to_string(), t.clone());
    t
}

/// js robot.shot: the shot it's lined up (what its physical robot reads of it)
#[derive(Debug, Clone, Copy)]
pub struct Shot { pub pass: bool, pub target: [f64; 2], pub reach: bool }

impl Robot {
    /// js `new Robot(...)`: its body made in the world now (after the FUEL and any robot before it);
    /// under: the world's physics.underArm, which its frame (and any lid) join
    pub fn new(key: &str, cfg: Value, alliance: Alliance, phys: &mut PhysicsWorld, under: &mut Vec<UnderArm>) -> Self {
        let t = BUMPER_T;
        let half_l = num(&cfg["frame"], "length") / 2.0 + t;
        let half_w = num(&cfg["frame"], "width") / 2.0 + t;
        let height = num(&cfg, "height");
        let mass = num(&cfg, "mass");
        let pts = module_points(&cfg);
        let dcfg: DriveCfg = serde_json::from_value(cfg["drive"].clone()).unwrap();
        let drivetrain = Drivetrain::new(&dcfg, mass, m::hypot2(pts[0][0], pts[0][1]));
        let swerve = SwerveDrive::new(drivetrain, pts.clone());
        let tables = tables_for(key, &cfg);
        let cfg_loc = cfg.clone();
        let mut r = Robot {
            key: key.into(), cfg, alliance, body: RigidBodyHandle::invalid(), bumper: ColliderHandle::invalid(), frame: ColliderHandle::invalid(),
            hopper: None, hopper_y: 0.0, arm_collider: ColliderHandle::invalid(), arm_wall: None, intake_block: None, lid: None,
            wheel_pts: Vec::new(), half_l, half_w, height, swerve,
            max_speed: 0.0, max_omega: 0.0, phys: None, cmd: Cmd::default(), enabled: false, intake_deploy: 0.0, intake_want: false, human_driven: false, climber: None, climb_state: ClimbState::None, climb_lift: 0.0, climb_level: 0, climb_target: 0,
            climber_up: false, climber_ext: 0.0, hooked: false, manual: false, climb_base: [0.0; 3], teleop: false, arm_groups: 0, arm_groups_rigid: 0, hopper_deploy: 0.0,
            last_t: 0.0, pos: [0.0; 3], quat: [0.0, 0.0, 0.0, 1.0], yaw: 0.0, vel: [0.0; 3], omega: 0.0, traction: 0.0,
            aim_vel: [0.0; 2], stored: Vec::new(), stats: Stats::default(), full: false, last_in_zone: false, want_shoot: false, shot_setpoint: 0.0,
            shot: None, last_pass: false, aim_override: None, aim_err: None, pass_clear: None, out: PostOut::default(),
            hood_deg: 0.0, flywheel: 0.0, ready: false, status: String::new(), aim_offset: 0.0, tables, loc: crate::localize::Localizer::new(&cfg_loc, false),
            turrets: Vec::new(), turret_yaws: Vec::new(), turret_yaw: 0.0, step_dt: 0.0,
        };
        let sh = &r.cfg["shooter"];
        r.aim_offset = if sh["type"].as_str() == Some("fixed") && sh["facing"].as_str() == Some("back") { m::PI } else { 0.0 };
        // (turret shooters: one or more turrets, each aiming itself)
        if sh["type"].as_str() == Some("turret") {
            let list = if sh["turrets"].is_array() { sh["turrets"].as_array().unwrap().clone() } else { vec![sh["turretPos"].clone()] };
            assert!(list.len() == 1, "twin turrets arrive with 1706 (M3c)");
            r.turrets = list.iter().map(|t| [num(t, "x"), num(t, "z")]).collect();
        }
        r.climber = r.cfg.get("climber").filter(|c| c.is_object()).map(|c| ClimberCfg {
            max_level: c["maxLevel"].as_u64().unwrap() as usize,
            times: c["times"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect(),
        });
        r.max_speed = num(&r.cfg["drive"], "maxSpeed");
        r.max_omega = num(&r.cfg["drive"], "maxOmega");
        r.create_body(phys, under);
        r.phys = match r.cfg.get("physical").and_then(|x| x.as_str()) {
            Some("2910") => Some(Phys::R2910(Box::new(Phys2910::new(r.body)))), // (and in PORTED)
            Some("4414") => Some(Phys::R4414(Box::new(Phys4414::new(r.body)))),
            Some("1678") => Some(Phys::R1678(Box::new(Phys1678::new(r.body)))),
            Some("1690") => Some(Phys::R1690(Box::new(Phys1690::new(r.body)))),
            Some("7769") => Some(Phys::R7769(Box::new(Phys7769::new(r.body)))),
            Some("341") => Some(Phys::R341(Box::new(Phys341::new(r.body)))),
            Some("1114") => Some(Phys::R1114(Box::new(Phys1114::new(r.body)))),
            Some("8793") => {
                let f = &r.cfg["intake"]["fold"];
                let hull: Vec<[f64; 2]> = f["hull"].as_array().unwrap().iter().map(|p| [p[0].as_f64().unwrap(), p[1].as_f64().unwrap()]).collect();
                let pv = [f["pivot"][0].as_f64().unwrap(), f["pivot"][1].as_f64().unwrap()];
                let mut p = Phys8793::new(r.body, hull, pv, num(&r.cfg["intake"], "width"), num(f, "stowDeg"));
                (p.arm_collider, p.arm_groups, p.arm_groups_rigid) = (r.arm_collider, r.arm_groups, r.arm_groups_rigid);
                Some(Phys::R8793(Box::new(p)))
            }
            Some(p) => panic!("robot {p}'s mechanisms aren't ported yet"),
            None => panic!("robot {key} isn't physical: not in the native game (PLAN.md D11)"),
        };
        r.reset();
        r
    }

    fn create_body(&mut self, phys: &mut PhysicsWorld, under: &mut Vec<UnderArm>) {
        let cfg = self.cfg.clone();
        let mass = num(&cfg, "mass");
        let dt = phys.integration_parameters.dt as f64;
        self.body = phys.insert_body(RigidBodyBuilder::dynamic().ccd_enabled(true).can_sleep(false).linear_damping(0.0).angular_damping(0.0));
        let body = self.body;
        let robot_groups = groups(GROUP::ROBOT, GROUP::STATIC | GROUP::TERRAIN | GROUP::BALL | GROUP::ROBOT_BARRIER | GROUP::ROBOT | GROUP::INTAKE);
        let (y0, y1) = (BUMPER_Y0, BUMPER_Y1);
        let (bc, cr) = (0.03, 0.04);
        let shape = frame_shape(&cfg);
        let mut pts: Vec<Vector> = Vec::new();
        for (y, inset) in [(y0, bc), (y0 + bc, 0.0), (y1 - 0.025, 0.0), (y1, 0.025)] {
            for [x, z] in offset_shape(&shape, BUMPER_T - inset - cr) {
                for k in 0..8 { let a = (k as f64 / 8.0) * m::PI * 2.0; pts.push(vf(x + cr * m::cos(a), y, z + cr * m::sin(a))); }
            }
        }
        self.bumper = phys.insert_collider(ColliderBuilder::convex_hull(&pts).unwrap().mass((mass * 0.15) as f32)
            .friction(0.05).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.1).collision_groups(grp(robot_groups)), Some(body));
        let (sl, sw) = (num(&cfg["frame"], "length") / 2.0, num(&cfg["frame"], "width") / 2.0);
        let mut up: Vec<f64> = Vec::new();
        for [x, z] in offset_shape(&shape, -0.05) { up.extend([x, y1, z]); }
        for [x, z] in &shape { up.extend([*x, y1 + 0.05, *z, *x, self.height, *z]); }
        let upv: Vec<Vector> = up.chunks_exact(3).map(|p| vf(p[0], p[1], p[2])).collect();
        self.frame = phys.insert_collider(ColliderBuilder::convex_hull(&upv).unwrap().mass((mass * 0.3) as f32)
            .friction(0.05).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.1)
            .collision_groups(grp(groups(GROUP::ROBOT, GROUP::STATIC | GROUP::TERRAIN | GROUP::BALL | GROUP::ROBOT_BARRIER | GROUP::ROBOT | GROUP::INTAKE))), Some(body));
        passes_under(phys, under, self.frame, Top::Frame { body, pts: up });
        let _ = dt;
        // most of a robot's weight is low: mass only, it touches nothing
        phys.insert_collider(ColliderBuilder::cuboid(sl as f32, 0.03, sw as f32).translation(vf(0.0, 0.05, 0.0)).mass((mass * 0.5) as f32).collision_groups(grp(0)), Some(body));
        for [mx, mz] in module_points(&cfg) {
            self.wheel_pts.push([mx, mz]);
            phys.insert_collider(ColliderBuilder::ball(WHEEL_R as f32).translation(vf(mx, WHEEL_R, mz)).mass((mass * 0.0125) as f32)
                .friction(0.0).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.0)
                .collision_groups(grp(groups(GROUP::WHEEL, GROUP::STATIC | GROUP::TERRAIN | GROUP::ROBOT_BARRIER | GROUP::BALL))), Some(body));
        }
        // the extending hopper section
        let st = &cfg["storage"];
        let ext = opt(st, "extLen").unwrap_or(0.0);
        let bx = st.get("box");
        let box_false = matches!(bx, Some(Value::Bool(false)));
        let box_obj = bx.filter(|b| b.is_object());
        let ic = &cfg["intake"];
        let fold = ic.get("fold").filter(|x| !x.is_null());
        if ext != 0.0 && !box_false && (fold.is_none() || box_obj.is_some()) {
            let (by0, by1) = match box_obj { Some(b) => (num(b, "y0"), num(b, "y1")), None => (0.17, self.height) };
            let half_h = (by1 - by0) / 2.0;
            self.hopper_y = by0 + half_h;
            let (hx, hy, hz) = (ext / 2.0, half_h, match box_obj { Some(b) => num(b, "hw"), None => num(&cfg["frame"], "width") / 2.0 - 0.02 });
            let shp = || -> ColliderBuilder {
                if let Some(lip) = st.get("lip").and_then(|x| x.as_array()) {
                    let (lx, ly) = (lip[0].as_f64().unwrap(), lip[1].as_f64().unwrap());
                    let cx = num(&cfg["frame"], "length") / 2.0 + hx;
                    let mut p = Vec::new();
                    for z in [-hz, hz] {
                        for [x, y] in [[-hx, -hy], [-hx, hy], [hx, hy], [hx, ly - self.hopper_y], [lx - cx, -hy]] { p.push(vf(x, y, z)); }
                    }
                    ColliderBuilder::convex_hull(&p).unwrap()
                } else { ColliderBuilder::cuboid(hx as f32, hy as f32, hz as f32) }
            };
            let tr = vf(num(&cfg["frame"], "length") / 2.0 - ext / 2.0, self.hopper_y, 0.0);
            let col = phys.insert_collider(shp().translation(tr).mass(if box_obj.is_some() { 0.0 } else { 0.5 }).friction(0.05).restitution(0.1).collision_groups(grp(0)), Some(body));
            let wall = phys.insert_collider(shp().translation(tr).density(0.0).friction(0.1).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.1).collision_groups(grp(0)), Some(body));
            self.hopper = Some((col, wall));
        }
        let bay = &cfg["bay"];
        let solid = groups(GROUP::ROBOT, GROUP::STATIC | GROUP::ROBOT | GROUP::INTAKE);
        if let Some(lift) = bay.get("lift").filter(|x| !x.is_null()) {
            let _ = lift;
            let x1 = bay.get("slope").filter(|x| !x.is_null()).map_or(num(bay, "x1"), |s| num(s, "x"));
            let x0 = num(bay, "x0");
            let lid_x = (x0 + x1) / 2.0;
            let (hl, hw) = ((x1 - x0) / 2.0, num(bay, "hw"));
            let lid = phys.insert_collider(ColliderBuilder::cuboid(hl as f32, 0.006, hw as f32).translation(vf(lid_x, num(bay, "top"), 0.0))
                .density(0.0).friction(0.05).restitution(0.1).collision_groups(grp(solid)), Some(body));
            passes_under(phys, under, lid, Top::Lid { body, lid, hl, hw });
            self.lid = Some((lid, lid_x));
        }
        let intake_groups = groups(GROUP::INTAKE, GROUP::STATIC | GROUP::ROBOT | GROUP::INTAKE | GROUP::ROBOT_BARRIER);
        if let Some(fold) = fold {
            // (its side outline across its width, turning about its pivot; one that yields to a
            // TRENCH arm passes through it here, and its mechanisms work out what the arm does to it)
            let hw = opt(fold, "hw").unwrap_or(num(ic, "width") / 2.0 + 0.01);
            let mut pts: Vec<Vector> = Vec::new();
            for p in fold["hull"].as_array().unwrap() { for z in [-hw, hw] { pts.push(vf(p[0].as_f64().unwrap(), p[1].as_f64().unwrap(), z)); } }
            self.arm_groups_rigid = intake_groups;
            self.arm_groups = if fold["yields"].as_bool() == Some(true) { intake_groups & !GROUP::TRENCH_ARM } else { intake_groups };
            let pv = (fold["pivot"][0].as_f64().unwrap(), fold["pivot"][1].as_f64().unwrap());
            self.arm_collider = phys.insert_collider(ColliderBuilder::convex_hull(&pts).unwrap().translation(vf(pv.0, pv.1, 0.0))
                .density(0.0).friction(0.1).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.05).collision_groups(grp(self.arm_groups)), Some(body));
        } else if ic.get("slide").filter(|x| !x.is_null()).is_some() {
            panic!("sliding intakes arrive with their robots (M3c)");
        } else {
            let (bhx, by0, by1) = (num(ic, "reach") / 2.0 + 0.02, 0.03, 0.19);
            self.intake_block = Some((bhx, by0, by1));
            self.arm_collider = phys.insert_collider(ColliderBuilder::cuboid(bhx as f32, ((by1 - by0) / 2.0) as f32, (num(ic, "width") / 2.0) as f32)
                .translation(vf(self.half_l - bhx, (by0 + by1) / 2.0, 0.0))
                .density(0.0).friction(0.05).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.05).collision_groups(grp(0)), Some(body));
        }
        // deployed intake roller, and its side plates
        let (reach, iw) = (num(ic, "reach"), num(ic, "width"));
        phys.insert_collider(ColliderBuilder::cuboid(0.03, 0.05, (iw / 2.0) as f32).translation(vf(self.half_l + reach - 0.05, 0.1, 0.0))
            .mass(0.1).friction(0.1).restitution(0.05).collision_groups(grp(0)), Some(body));
        for sz in [-1.0, 1.0] {
            phys.insert_collider(ColliderBuilder::cuboid((reach / 2.0) as f32, 0.08, 0.006).translation(vf(self.half_l + reach / 2.0, 0.11, sz * (iw / 2.0 + 0.006)))
                .density(0.0).friction(0.1).restitution(0.05).collision_groups(grp(0)), Some(body));
        }
    }

    /// js `reset()` (what play reads so far)
    pub fn reset(&mut self) {
        match &mut self.phys { Some(Phys::R2910(p)) => p.reset(), Some(Phys::R4414(p)) => p.reset(), Some(Phys::R8793(p)) => p.reset(), Some(Phys::R1678(p)) => p.reset(), Some(Phys::R1690(p)) => p.reset(), Some(Phys::R7769(p)) => p.reset(), Some(Phys::R341(p)) => p.reset(), Some(Phys::R1114(p)) => p.reset(), None => {} }
        // (each turret starts at the middle of its travel)
        let sh = &self.cfg["shooter"];
        let list = if sh["turrets"].is_array() { sh["turrets"].as_array().unwrap().clone() } else if sh["turretPos"].is_object() { vec![sh["turretPos"].clone()] } else { vec![] };
        self.turret_yaws = list.iter().map(|t| opt(t, "start").or(opt(t, "center")).unwrap_or(0.0) * DEG).collect();
        self.turret_yaw = self.turret_yaws.first().copied().unwrap_or(0.0);
        self.last_t = 0.0;
        self.intake_deploy = 0.0;
        self.hopper_deploy = 0.0;
        self.climb_state = ClimbState::None;
        self.climb_lift = 0.0;
        self.climb_target = self.climber.as_ref().map_or(0, |c| c.max_level);
        self.climb_level = 0;
        self.climber_up = false; self.climber_ext = 0.0; self.hooked = false; self.manual = false;
        self.enabled = false;
        self.cmd = Cmd::default();
        self.stored.clear();
        self.full = false;
        self.aim_override = None;
        self.aim_vel = [0.0; 2];
        self.shot = None;
        self.stats = Stats::default();
        self.last_in_zone = false;
        self.out = PostOut::default();
        let sh = &self.cfg["shooter"];
        self.hood_deg = if sh["hood"].is_null() { num(sh, "hoodMin") } else { num(&sh["hood"], "stowDeg") };
        self.flywheel = 0.0;
        self.ready = false;
        self.status = "Idle".into();
    }

    /// js `spawn(x, z, yaw)`
    pub fn spawn(&mut self, phys: &mut PhysicsWorld, x: f64, z: f64, yaw: f64) {
        let b = &mut phys.bodies[self.body];
        b.set_body_type(RigidBodyType::Dynamic, true);
        b.set_translation(vf(x, 0.003, z), true);
        let q = [0.0, m::sin(yaw / 2.0), 0.0, m::cos(yaw / 2.0)];
        b.set_rotation(Rotation::from_xyzw(q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32), true);
        b.set_linvel(vf(0.0, 0.0, 0.0), true);
        b.set_angvel(vf(0.0, 0.0, 0.0), true);
        self.yaw = yaw;
        let h = yaw / 2.0;
        self.quat = [0.0, m::sin(h), 0.0, m::cos(h)];
        self.pos = [x, 0.0, z];
        self.vel = [0.0; 3];
        self.omega = 0.0;
        self.aim_vel = [0.0; 2];
        self.swerve.reset();
        let view = loc_view!(self, &[]);
        self.loc.reset(&view);
    }

    /// js `loadFuel(balls)`: its preload
    pub fn load_fuel(&mut self, phys: &mut PhysicsWorld, fuel: &mut Fuel, balls: &[usize]) {
        let mut c = Ctx { body: self.body, alliance: self.alliance, frame: self.frame_lw(), fuel, phys };
        if let Some(p) = &mut self.phys { self.stored = p.physical_mut().load(&mut c, balls); }
    }

    /// js `preStep(dt)` for a robot alone (its mechanisms between its two halves)
    pub fn pre_step(&mut self, phys: &mut PhysicsWorld, fuel: &mut Fuel, dt: f64, teleop: bool) {
        self.teleop = teleop;
        self.pre_step_drive(phys, dt, teleop);
        let (on, t) = (self.enabled, self.last_t);
        let (bay_top, lift_h) = (opt(&self.cfg["bay"], "top").unwrap_or(0.0), self.lift_h());
        let driver_climb = self.driver_climb();
        let mut c = Ctx { body: self.body, alliance: self.alliance, frame: self.frame_lw(), fuel, phys };
        match &mut self.phys {
            Some(Phys::R2910(p)) => { p.pre_mech(dt, on, teleop); p.mechanisms(&mut c, dt, t); self.intake_deploy = p.post_mech(dt); }
            Some(Phys::R4414(p)) => {
                p.pre_mech(dt, &crate::mech::phys4414::PreIn { enabled: on, intake_deploy: self.intake_deploy, turret_yaw: self.turret_yaw });
                p.mechanisms(&mut c, dt, t);
                self.intake_deploy = p.post_mech(dt);
            }
            Some(Phys::R1678(p)) => {
                p.pre_mech(c.phys, c.fuel, self.body, dt, &crate::mech::phys1678::PreIn { enabled: on, bay_top, lift_h, driver_climb, climber_ext: self.climber_ext });
                p.mechanisms(&mut c, dt, t);
                self.intake_deploy = p.post_mech(dt);
            }
            Some(Phys::R1690(p)) => {
                p.pre_mech(dt, &crate::mech::phys1690::PreIn { enabled: on, intake_deploy: self.intake_deploy, turret_yaw: self.turret_yaw });
                p.mechanisms(&mut c, dt, t);
                self.intake_deploy = p.post_mech(&mut c.phys.bodies, dt);
            }
            Some(Phys::R1114(p)) => {
                p.pre_mech(dt, &crate::mech::phys1114::PreIn { enabled: on });
                p.mechanisms(&mut c, dt, t);
                self.intake_deploy = p.post_mech(dt);
            }
            Some(Phys::R341(p)) => {
                p.pre_mech(dt, &crate::mech::phys341::PreIn { enabled: on, turret_yaw: self.turret_yaw });
                p.mechanisms(&mut c, dt, t);
                self.intake_deploy = p.post_mech(dt);
            }
            Some(Phys::R7769(p)) => {
                p.pre_mech(dt, &crate::mech::phys7769::PreIn { enabled: on });
                p.mechanisms(&mut c, dt, t);
                self.intake_deploy = p.post_mech(dt);
            }
            Some(Phys::R8793(p)) => {
                p.pre_mech(c.phys, dt, &crate::mech::phys8793::PreIn {
                    enabled: on, intake_deploy: self.intake_deploy, intake_want: self.intake_want, turret_yaw: self.turret_yaw, hood_deg: self.hood_deg, pos: self.pos, quat: self.quat,
                });
                p.mechanisms(&mut c, dt, t, self.intake_deploy);
                self.intake_deploy = p.post_mech(c.phys, self.body, dt);
            }
            None => {}
        }
    }
    /// cfg.bay.lift.h (0: no lid that lifts)
    fn lift_h(&self) -> f64 { self.cfg["bay"].get("lift").filter(|x| !x.is_null()).and_then(|l| opt(l, "h")).unwrap_or(0.0) }
    /// cfg.frame's length and width
    fn frame_lw(&self) -> (f64, f64) { (num(&self.cfg["frame"], "length"), num(&self.cfg["frame"], "width")) }

    /// js `localToWorld(lx, ly, lz)`
    pub fn local_to_world(&self, lx: f64, ly: f64, lz: f64) -> [f64; 3] {
        let (c, s) = (m::cos(self.yaw), m::sin(self.yaw));
        [self.pos[0] + lx * c + lz * s, self.pos[1] + ly, self.pos[2] - lx * s + lz * c]
    }
    /// js `inAllianceZone(alliance)`: its BUMPERS partly or fully within the ALLIANCE ZONE (G407)
    pub fn in_alliance_zone(&self, a: Alliance) -> bool {
        let lx = crate::field::alliance_line_x(a);
        [[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]].iter().any(|[sa, sb]| {
            let x = self.local_to_world(sa * self.half_l, 0.0, sb * self.half_w)[0];
            if a == Alliance::Blue { x <= lx } else { x >= lx }
        })
    }

    /// js `_target()`: the HUB when legal (in its own ALLIANCE ZONE), otherwise a pass: (pass, x, z, inZone)
    fn target(&self) -> (bool, [f64; 2], bool) {
        let in_zone = self.in_alliance_zone(self.alliance);
        let pass = self.cmd.pass || !in_zone;
        if !pass { return (false, self.aim_hub(), in_zone); }
        // (a physical robot passes where its code does: 2910's FieldConstants' points)
        match &self.phys {
            Some(Phys::R2910(_)) => (true, self.aim_pt(Phys2910::pass_target(self.alliance, [self.pos[0], self.pos[2]])), in_zone),
            Some(Phys::R1678(_)) => (true, self.aim_pt(crate::mech::phys1678::ferry(self.alliance, self.pos).1), in_zone),
            Some(Phys::R7769(_)) => (true, self.aim_pt(Phys7769::pass_target(self.alliance, self.pos)), in_zone),
            Some(Phys::R1114(_)) => (true, self.aim_pt(Phys1114::pass_target(self.alliance, self.pos)), in_zone),
            _ => {
                let s = if self.alliance == Alliance::Blue { 1.0 } else { -1.0 };
                let x = s * (-HALF_L + 1.7);
                let zc = [HALF_W - 1.8, -HALF_W + 1.8];
                let z = if m::abs(self.pos[2] - zc[0]) < m::abs(self.pos[2] - zc[1]) { zc[0] } else { zc[1] };
                (true, self.aim_pt([x, z]), in_zone)
            }
        }
    }

    /// js `capacity()`: how many it holds, about (a physical robot's own rating)
    pub fn capacity(&self) -> usize {
        match &self.phys { Some(Phys::R2910(p)) => p.capacity(), Some(Phys::R4414(p)) => p.capacity(), Some(Phys::R8793(p)) => p.capacity(), Some(Phys::R1678(p)) => p.capacity(), Some(Phys::R1690(p)) => p.capacity(), Some(Phys::R7769(p)) => p.capacity(), Some(Phys::R341(p)) => p.capacity(), Some(Phys::R1114(p)) => p.capacity(), None => 0 }
    }

    /// js `aimPt(p)`: where its pose estimate puts world point p, seen from its true pose
    pub fn aim_pt(&self, p: [f64; 2]) -> [f64; 2] { self.loc.aim_pt(p, self.pos, self.yaw) }
    /// js `aimHub()`
    pub fn aim_hub(&self) -> [f64; 2] { let c = crate::field::hub_center(self.alliance); self.aim_pt([c.0, c.1]) }

    /// js `velocityAt(p)`
    pub fn velocity_at(&self, p: [f64; 3]) -> [f64; 2] {
        [self.vel[0] + self.omega * (p[2] - self.pos[2]), self.vel[2] - self.omega * (p[0] - self.pos[0])]
    }
    /// js `fixedExit(sh, deg)` (robot frame: x, y)
    fn fixed_exit(&self, deg: f64) -> [f64; 2] {
        let sh = &self.cfg["shooter"];
        let h = &sh["hood"];
        if h.is_null() { return [num(&sh["exit"], "x"), num(&sh["exit"], "y")]; }
        let th = (deg * m::PI) / 180.0;
        let s = if sh["facing"].as_str() == Some("back") { 1.0 } else { -1.0 };
        [num(h, "x") + s * num(h, "r") * m::sin(th), num(h, "y") + num(h, "r") * m::cos(th)]
    }

    /// js `_turretExit(ti)`: where FUEL leaves turret ti's hood now (robot frame)
    fn turret_exit(&self, ti: usize) -> [f64; 3] {
        let sh = &self.cfg["shooter"];
        let (t, a, r) = (self.turrets[ti], self.turret_yaws[ti], num(sh, "exitRadius"));
        [t[0] + m::cos(a) * r, num(sh, "exitY"), t[1] - m::sin(a) * r]
    }
    /// js `estVelAt(p)`: the chassis velocity at p as its wheels and gyro measure it
    pub fn est_vel_at(&self, p: [f64; 3]) -> [f64; 2] {
        let Some(v) = self.loc.vel(self.yaw) else { return self.velocity_at(p) };
        let w = self.loc.omega().unwrap();
        [v[0] + w * (p[2] - self.pos[2]), v[1] - w * (p[0] - self.pos[0])]
    }

    /// js `_passLands(0, anywhere)`: where a FUEL launched now comes down is on
    /// its own half, clear of the walls (anywhere: only that it misses both HUBS)
    fn pass_lands(&self, short: bool, anywhere: bool) -> bool {
        let sh = &self.cfg["shooter"];
        let (psi, exit) = if sh["type"].as_str() == Some("fixed") {
            let lanes = sh["lanes"].as_array().expect("lanes");
            let lane_z = lanes[0].as_f64().unwrap();
            let e = self.fixed_exit(self.hood_deg);
            (self.yaw + self.aim_offset, self.local_to_world(e[0], e[1], lane_z))
        } else {
            let e = self.turret_exit(0);
            (self.yaw + self.turret_yaws[0], self.local_to_world(e[0], e[1], e[2]))
        };
        let lv = self.velocity_at(exit);
        let (th, v) = (self.hood_deg * DEG, self.flywheel);
        let sv = [m::cos(psi) * m::cos(th) * v, -m::sin(psi) * m::cos(th) * v];
        let spin = opt(sh, "spin").unwrap_or(0.0);
        let w = crate::ballistics::spin_for(sv, (spin * v) / FUEL::radius);
        let pts = crate::ballistics::trajectory_points(exit, [sv[0] + lv[0], m::sin(th) * v, sv[1] + lv[1]], 3.0, 0.05, Some(w));
        let (x, z) = (pts[pts.len() - 3], pts[pts.len() - 1]);
        let s = if self.alliance == Alliance::Blue { 1.0 } else { -1.0 };
        if !anywhere && !(m::abs(x) < HALF_L - 1.2 && m::abs(z) < HALF_W - 1.2 && (short || s * x < -0.5)) { return false; }
        for a in [Alliance::Blue, Alliance::Red] {
            let c = crate::field::hub_center(a);
            for i in (0..pts.len()).step_by(3) {
                if pts[i + 1] < HUB::rimFront + 0.6 && m::hypot2(pts[i] - c.0, pts[i + 2] - c.1) < HUB::size / 2.0 + 0.3 { return false; }
            }
        }
        true
    }

    /// js `_shooter(dt, t, on)` for a fixed shooter whose physical robot feeds itself (2910's): the
    /// target, the moving shot's solution, the chassis' heading loop, ready or not
    fn shooter_step(&mut self, on: bool) {
        // (8793, as its code's ShooterMode: out of its zone it shuttles whatever it picks up, all the
        // time, except under a TRENCH)
        let auto_shuttle = matches!(self.phys, Some(Phys::R8793(_))) && !self.in_alliance_zone(self.alliance) && !crate::mech::phys8793::under_trench(self.pos[0], self.pos[2], self.half_l);
        let want_shoot = on && (self.cmd.shoot || self.cmd.pass || auto_shuttle);
        self.want_shoot = want_shoot;
        let has = !self.stored.is_empty();
        self.aim_override = None;
        self.shot = None;
        let mut setpoint = 0.0;
        let mut ready = false;
        let mut status: &str = if has { "Holding FUEL" } else { "Empty" };
        let (pass, tgt, in_zone) = self.target();
        self.last_in_zone = in_zone;
        let prespin = on && has && in_zone;
        let mut out_of_range = false;
        let fixed = self.cfg["shooter"]["type"].as_str() == Some("fixed");
        if (want_shoot || prespin) && has {
            let (exit, lv);
            let mut sol;
            let table = if pass { &self.tables.pass } else { &self.tables.hub };
            if fixed {
                let (dx, dz) = (tgt[0] - self.pos[0], tgt[1] - self.pos[2]);
                let dd = { let h = m::hypot2(dx, dz); if h == 0.0 { 1.0 } else { h } };
                let exit_at = |deg: f64| {
                    let e = self.fixed_exit(deg);
                    let along = if self.aim_offset != 0.0 { -e[0] } else { e[0] };
                    [self.pos[0] + (dx / dd) * along, self.pos[1] + e[1], self.pos[2] + (dz / dd) * along]
                };
                exit = exit_at(self.hood_deg);
                lv = self.aim_vel;
                sol = crate::ballistics::solve_moving_shot(table, exit, lv, tgt);
                if let Some(s1) = sol {
                    if !self.cfg["shooter"]["hood"].is_null() {
                        if let Some(s2) = crate::ballistics::solve_moving_shot(table, exit_at(s1.theta / DEG), lv, tgt) { sol = Some(s2); }
                    }
                }
            } else {
                // (FUEL leaves the hood exitRadius in front of the turret's axis, along the shot line)
                let sh = &self.cfg["shooter"];
                let c = self.local_to_world(self.turrets[0][0], num(sh, "exitY"), self.turrets[0][1]);
                let (dx, dz) = (tgt[0] - c[0], tgt[1] - c[2]);
                let dd = { let h = m::hypot2(dx, dz); if h == 0.0 { 1.0 } else { h } };
                let er = num(sh, "exitRadius");
                exit = [c[0] + (dx / dd) * er, c[1], c[2] + (dz / dd) * er];
                lv = self.est_vel_at(exit);
                sol = crate::ballistics::solve_moving_shot(table, exit, lv, tgt);
                if let Some(s1) = sol {
                    if !self.cfg["shooter"]["hood"].is_null() { let _ = s1; panic!("a turret with a swinging hood's exit: not yet"); }
                }
            }
            let _ = exit;
            if let Some(sol) = sol {
                self.shot = Some(Shot { pass, target: tgt, reach: sol.reach });
                setpoint = sol.v;
                // (a robot whose mechanisms don't set the hood themselves: 8793's)
                if matches!(self.phys, Some(Phys::R8793(_))) { self.hood_deg = approach(self.hood_deg, sol.theta / DEG, 260.0 * self.step_dt); }
                let aim_err;
                if fixed {
                aim_err = wrap_angle(sol.psi - self.yaw - self.aim_offset);
                if want_shoot {
                    let (max_omega, max_alpha) = (num(&self.cfg["drive"], "maxOmega"), num(&self.cfg["drive"], "maxAlpha"));
                    let (rx, rz) = (tgt[0] - self.pos[0], tgt[1] - self.pos[2]);
                    let r2 = m::max(0.25, rx * rx + rz * rz);
                    let ff = crate::fuel::clamp((rx * lv[1] - rz * lv[0]) / r2, -3.0, 3.0);
                    let a = m::abs(aim_err);
                    let w = m::min(m::min(max_omega, m::sqrt(2.0 * 0.7 * max_alpha * a)), AIM_GAIN * a);
                    self.aim_override = Some(crate::fuel::clamp(m::sign(aim_err) * w + ff, -max_omega, max_omega));
                }
                } else {
                    // (measured from the middle of the turret's travel: its hard stops are +-lim from there)
                    let sh = &self.cfg["shooter"];
                    let lim = num(sh, "turretRange") * DEG;
                    let mid = opt(&sh["turretPos"], "center").unwrap_or(0.0) * DEG;
                    // (a physical robot that aims its own way says where: phys.aimPsi)
                    let psi = match &self.phys { Some(Phys::R4414(p)) => p.aim_psi.unwrap_or(sol.psi), Some(Phys::R1690(p)) => p.aim_psi.unwrap_or(sol.psi), Some(Phys::R341(p)) => p.aim_psi.unwrap_or(sol.psi), _ => sol.psi };
                    let rel = wrap_angle(psi - self.yaw - mid);
                    let cur = self.turret_yaw - mid;
                    let (goal, over) = turret_goal(rel, cur, lim, want_shoot);
                    if over != 0.0 && want_shoot {
                        let (max_omega, max_alpha) = (num(&self.cfg["drive"], "maxOmega"), num(&self.cfg["drive"], "maxAlpha"));
                        let a = m::abs(over);
                        self.aim_override = Some(m::sign(over) * m::min(m::min(max_omega, m::sqrt(2.0 * 0.7 * max_alpha * a)), AIM_GAIN * a));
                    }
                    let ty = match &mut self.phys {
                        Some(Phys::R4414(p)) => p.turret_step(mid + cur, mid + goal, self.step_dt, self.last_t),
                        Some(Phys::R8793(p)) => p.turret_step(mid + cur, mid + goal, self.step_dt, self.last_t),
                        Some(Phys::R1690(p)) => p.turret_step(mid + cur, mid + goal, self.step_dt, self.last_t),
                        _ => mid + approach(cur, goal, num(sh, "turretRate") * DEG * self.step_dt),
                    };
                    self.turret_yaw = ty;
                    self.turret_yaws[0] = ty;
                    aim_err = wrap_angle(psi - (self.yaw + self.turret_yaw));
                }
                self.aim_err = Some(aim_err);
                let tol = if pass { 4.0 * DEG } else { m::max(0.6 * DEG, m::atan2(0.14, sol.dist)) };
                let spin_ok = if pass { self.flywheel > 0.75 * sol.v } else { m::abs(self.flywheel - sol.v) / sol.v < 0.025 };
                let hood_ok = m::abs(self.hood_deg - sol.theta / DEG) < 1.5;
                let aim_ok = m::abs(aim_err) < tol;
                ready = if pass { spin_ok && self.pass_lands(!sol.reach, false) } else { spin_ok && hood_ok && aim_ok };
                self.pass_clear = Some(!pass || self.pass_lands(!sol.reach, true));
                status = if !spin_ok { "Spinning up" } else if !aim_ok { "Aiming" } else if !hood_ok { "Hood" } else { "READY" };
            } else { out_of_range = true; }
        } else if !in_zone && has { status = "Outside ALLIANCE ZONE"; }
        let mut st = if out_of_range { if pass { "Pass out of range".to_string() } else { "Out of range".to_string() } }
            else if self.shot.is_some_and(|s| s.pass) { if ready { "PASS READY".to_string() } else { format!("Pass: {}", status.to_lowercase()) } }
            else { status.to_string() };
        if !on && !self.enabled { st = "Disabled".into(); }
        self.status = st;
        self.shot_setpoint = setpoint;
        if let Some(s) = self.shot { self.last_pass = s.pass; }
        self.ready = ready;
    }

    fn pre_step_drive(&mut self, phys: &mut PhysicsWorld, dt: f64, teleop: bool) {
        {
            let b = &mut phys.bodies[self.body];
            b.reset_forces(true);
            b.reset_torques(true);
        }
        // (a climber a person works: their button raises it and lowers it, and its hooks do the climb)
        if self.driver_climb() { self.manual_climb(phys, dt); } else { self.manual = false; }
        if self.climb_state != ClimbState::None { return; }
        let (mut tvx, mut tvz, mut tw) = (0.0, 0.0, 0.0);
        if self.enabled {
            tvx = self.cmd.vx; tvz = self.cmd.vz; tw = self.cmd.omega;
            let w = self.along_walls(phys, tvx, tvz);
            tvx = w[0]; tvz = w[1];
            if let Some(a) = self.aim_override { tw = a; }
        }
        let sp = m::hypot2(tvx, tvz);
        if sp > self.max_speed { tvx *= self.max_speed / sp; tvz *= self.max_speed / sp; }
        tw = crate::fuel::clamp(tw, -self.max_omega, self.max_omega);
        let wheel = m::hypot2(tvx, tvz) + m::abs(tw) * self.swerve.d.r;
        if wheel > self.max_speed { let k = self.max_speed / wheel; tvx *= k; tvz *= k; tw *= k; }
        let down = self.wheels_down(phys);
        self.traction = down.iter().filter(|x| **x).count() as f64 / self.wheel_pts.len() as f64;
        {
            let b = &mut phys.bodies[self.body];
            let av = b.angvel();
            let k = 1.0 / (1.0 + TIP_DAMP * dt);
            let (mut ax, mut az) = (av.x as f64 * k, av.z as f64 * k);
            let h = m::hypot2(ax, az);
            if h > TIP_RATE { ax *= TIP_RATE / h; az *= TIP_RATE / h; }
            b.set_angvel(vf(ax, av.y as f64, az), true);
        }
        let b = &mut phys.bodies[self.body];
        let q = b.rotation();
        let (qx, qy, qz, qw) = (q.x as f64, q.y as f64, q.z as f64, q.w as f64);
        let yaw = m::atan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz));
        self.swerve.d.limits_for(teleop);
        self.swerve.step(b, yaw, tvx, tvz, tw, down, dt);
    }

    /// js `climbPose()`: where it backs up to its TOWER's RUNGS (its climber's on its back)
    pub fn climb_pose(&self) -> [f64; 3] {
        let s = if self.alliance == Alliance::Blue { 1.0 } else { -1.0 };
        let ladder_x = -HALF_L + TOWER::uprightFx + TOWER::uprightDeep / 2.0;
        let x = ladder_x + self.half_l + 0.03;
        let z = HALF_W - TOWER::fy;
        [s * x, s * z, if self.alliance == Alliance::Blue { 0.0 } else { m::PI }]
    }
    /// js `driverClimb`: a person drives it and it has a climber, in TELEOP: the climber is theirs
    pub fn driver_climb(&self) -> bool { self.human_driven && self.climber.is_some() && self.teleop }
    /// js `toggleClimber()` (the driver's button): what it says
    pub fn toggle_climber(&mut self) -> &'static str {
        if self.climber.is_none() { return "No climber installed"; }
        self.climber_up = !self.climber_up;
        if self.climber_up { if self.climb_state != ClimbState::None { "Climber up: letting the robot down" } else { "Climber up" } }
        else if self.hooked { "Climber down: climbing" } else { "Climber down" }
    }
    /// js `_manualClimb(dt)`: the climber as the driver works it. Raised with the robot's back at its
    /// TOWER, its hooks go over the RUNG; lowered then, they pull the robot up to climbTarget's LEVEL
    /// over that LEVEL's climb time, and raised again they let it back down
    fn manual_climb(&mut self, phys: &mut PhysicsWorld, dt: f64) {
        let c = self.climber.clone().unwrap();
        if !self.manual {
            self.manual = true;
            let lh = self.cfg["bay"].get("lift").filter(|x| !x.is_null()).and_then(|l| opt(l, "h")).unwrap_or(0.0);
            self.climber_ext = match &self.phys { Some(Phys::R1678(p)) => p.lift_now(lh), _ => 0.0 };
            self.climber_up |= self.climber_ext > 0.5;
        }
        let lvl = self.climb_target.max(1);
        let lift = CLIMB_LIFT[lvl];
        let t_up = m::max(0.3, c.times.get(1).copied().unwrap_or(1.6));
        let t_climb = m::max(0.3, c.times.get(lvl).copied().unwrap_or(2.0));
        if self.climb_state == ClimbState::None {
            let p = self.climb_pose();
            let at_tower = m::hypot2(self.pos[0] - p[0], self.pos[2] - p[1]) < 0.3 && m::abs(wrap_angle(p[2] - self.yaw)) < 20.0 * DEG;
            if self.hooked && !at_tower { self.hooked = false; }
            if !self.hooked && at_tower && self.climber_ext >= 0.98 { self.hooked = true; }
        }
        let rate = if self.hooked { 1.0 / t_climb } else { 1.0 / t_up };
        self.climber_ext = crate::fuel::clamp(self.climber_ext + (if self.climber_up { rate } else { -rate }) * dt, 0.0, 1.0);
        if !self.hooked { return; }
        let want = (1.0 - self.climber_ext) * lift;
        let b = &mut phys.bodies[self.body];
        if self.climb_state == ClimbState::None {
            if want <= 0.0 { return; }
            // (the hooks bite: from here the climber carries the robot)
            let h = self.yaw / 2.0;
            b.set_rotation(Rotation::from_xyzw(0.0, m::sin(h) as f32, 0.0, m::cos(h) as f32), true);
            b.set_body_type(RigidBodyType::KinematicPositionBased, true);
            self.climb_base = self.pos;
        }
        self.climb_lift = want;
        self.climb_state = if want <= 0.0 { ClimbState::None } else if self.climber_ext <= 0.0 { ClimbState::Hanging } else if self.climber_up { ClimbState::Descending } else { ClimbState::Climbing };
        self.climb_level = if self.climb_state == ClimbState::None { 0 } else { climb_level_from_lift(want) };
        if self.climb_state == ClimbState::None {
            b.set_body_type(RigidBodyType::Dynamic, true);
            b.set_linvel(vf(0.0, 0.0, 0.0), true);
        } else {
            let cb = self.climb_base;
            b.set_next_kinematic_translation(vf(cb[0], cb[1] + want, cb[2]));
        }
    }

    /// js `_alongWalls(vx, vz)`
    fn along_walls(&self, phys: &PhysicsWorld, mut vx: f64, mut vz: f64) -> [f64; 2] {
        let sp = m::hypot2(vx, vz);
        if sp < 0.05 { return [vx, vz]; }
        let wall = GROUP::STATIC | GROUP::ROBOT_BARRIER;
        let mut walls: Vec<[f64; 2]> = Vec::new();
        let mut cs = vec![self.bumper, self.frame, self.arm_wall.unwrap_or(self.arm_collider)];
        if let Some((_, w)) = self.hopper { cs.push(w); }
        let fixed_parent = |h: ColliderHandle| phys.colliders[h].parent().map(|p| phys.bodies[p].is_fixed()).unwrap_or(false);
        for c in cs {
            let cg = phys.colliders[c].collision_groups();
            if cg.memberships.bits() == 0 && cg.filter.bits() == 0 { continue; }
            for pair in phys.narrow_phase.contact_pairs_with(c) {
                let o = if pair.collider1 == c { pair.collider2 } else { pair.collider1 };
                let og = phys.colliders[o].collision_groups().memberships.bits();
                if !fixed_parent(o) || (og & wall) == 0 { continue; }
                let flipped = pair.collider1 != c;
                for mf in pair.manifolds() {
                    if mf.points.is_empty() || mf.points[0].dist > 0.02 { continue; }
                    let n = mf.data.normal;
                    let k = if flipped { -1.0 } else { 1.0 };
                    let (nx, nz) = (n.x as f64, n.z as f64);
                    let h = m::hypot2(nx, nz);
                    if h < 0.7 { continue; }
                    walls.push([(k * nx) / h, (k * nz) / h]);
                }
            }
        }
        for [nx, nz] in walls {
            let into = vx * nx + vz * nz;
            if into <= 0.05 || into > 0.966 * m::hypot2(vx, vz) { continue; }
            vx -= (into - 0.05) * nx; vz -= (into - 0.05) * nz;
        }
        [vx, vz]
    }

    /// js `_wheelsDown()`: a short ray down from each wheel's center
    fn wheels_down(&self, phys: &PhysicsWorld) -> Vec<bool> {
        let b = &phys.bodies[self.body];
        let q = b.rotation();
        let qv = [q.x as f64, q.y as f64, q.z as f64, q.w as f64];
        let p = b.translation();
        let floor = groups(GROUP::WHEEL, GROUP::STATIC | GROUP::TERRAIN | GROUP::BALL);
        self.wheel_pts.iter().map(|w| {
            let v = apply_quat([w[0], WHEEL_R, w[1]], qv);
            let ray = Ray::new(vf(p.x as f64 + v[0], p.y as f64 + v[1], p.z as f64 + v[2]), Vector::new(0.0, -1.0, 0.0));
            phys.cast_ray(&ray, (WHEEL_R + 0.02) as f32, true, QueryFilter::default().groups(grp(floor)).exclude_rigid_body(self.body)).is_some()
        }).collect()
    }

    /// js `postStep(dt, t)` (what moves the robot: its pose read back, the hopper, the intake's collider)
    /// others: the other robots as they are now (x, z, halfL, halfW), for the cameras' view
    pub fn post_step(&mut self, phys: &mut PhysicsWorld, fuel: &mut Fuel, dt: f64, t: f64, others: &[[f64; 4]]) {
        let b = &phys.bodies[self.body];
        let (tr, q) = (b.translation(), *b.rotation());
        self.pos = [tr.x as f64, tr.y as f64, tr.z as f64];
        self.quat = [q.x as f64, q.y as f64, q.z as f64, q.w as f64];
        let [qx, qy, qz, qw] = self.quat;
        let fx = 1.0 - 2.0 * (qy * qy + qz * qz);
        let fz = 2.0 * (qx * qz - qw * qy);
        self.yaw = m::atan2(-fz, fx);
        let lv = b.linvel();
        self.vel = [lv.x as f64, 0.0, lv.z as f64];
        self.omega = b.angvel().y as f64;
        // (its pose estimate: odometry, gyro, vision)
        let view = loc_view!(self, others);
        self.loc.step(dt, t, &view);
        // (what a moving shot is aimed with: AIM_VEL_TAU; the velocity its wheels measure)
        let ka = 1.0 - m::exp(-dt / AIM_VEL_TAU);
        let mv = self.loc.vel(self.yaw).unwrap_or([lv.x as f64, lv.z as f64]);
        self.aim_vel[0] += ka * (mv[0] - self.aim_vel[0]);
        self.aim_vel[1] += ka * (mv[1] - self.aim_vel[1]);
        let on = self.enabled && self.climb_state == ClimbState::None;
        self.last_t = t;
        self.step_dt = dt;
        // (2910's intake pivot is its mechanisms': its hopper still slides out with the first deploy)
        if matches!(self.phys, Some(Phys::R8793(_))) { self.intake_phys(phys, fuel, on); } else { self.hopper_step(phys, dt, on); }
        self.shooter_step(on);
        self.step_top_colliders(phys);
        let lv = phys.bodies[self.body].linvel();
        let est_vel = self.loc.vel(self.yaw).unwrap_or([lv.x as f64, lv.z as f64]);
        let aim_hub = self.aim_hub();
        let pin = PostIn {
            on, feeding: self.want_shoot, outtake: on && self.cmd.outtake, setpoint: self.shot_setpoint,
            pass: match self.shot { Some(s) => s.pass, None => self.last_pass }, intake: self.cmd.intake,
            target: self.shot.map(|s| s.target), pos: [self.pos[0], self.pos[2]], est_vel,
            aim_hub, aim_err: self.aim_err, pass_clear: self.pass_clear, legal: self.last_in_zone,
        };
        let dc = self.driver_climb();
        let in_zone = self.in_alliance_zone(self.alliance);
        let mut c = Ctx { body: self.body, alliance: self.alliance, frame: self.frame_lw(), fuel, phys };
        match &mut self.phys {
            Some(Phys::R2910(p)) => {
                self.stored = p.post_step(&mut c, dt, t, &pin, &mut self.stats, &mut self.out);
                self.hood_deg = self.out.hood_deg;
                self.flywheel = self.out.flywheel;
                self.ready = self.out.ready;
                self.full = p.p.watch_full(c.phys, c.fuel, self.body, dt, on && self.cmd.intake);
            }
            Some(Phys::R4414(p)) => {
                let est_omega = self.loc.omega().unwrap_or(c.phys.bodies[self.body].angvel().y as f64);
                let pin4 = crate::mech::phys4414::PostIn {
                    on, feeding: self.want_shoot, outtake: on && self.cmd.outtake, setpoint: self.shot_setpoint, pass: pin.pass, intake: self.cmd.intake,
                    has_shot: self.shot.is_some(), target: pin.target, aim_hub: pin.aim_hub, aim_err: self.aim_err,
                    est_vel: pin.est_vel, est_omega, yaw: self.yaw, spin: opt(&self.cfg["shooter"], "spin").unwrap_or(0.0), wheel_pts: &self.wheel_pts, legal: self.last_in_zone,
                };
                self.stored = p.post_step(&mut c, dt, t, &pin4, &mut self.stats, &mut self.out);
                self.hood_deg = self.out.hood_deg;
                self.flywheel = self.out.flywheel;
                self.ready = self.out.ready;
                // (full at its rated count: it isn't one whose intake says so)
                self.full = self.stored.len() >= p.capacity();
            }
            Some(Phys::R1690(p)) => {
                let est_omega = self.loc.omega().unwrap_or(c.phys.bodies[self.body].angvel().y as f64);
                let pin0 = crate::mech::phys1690::PostIn {
                    on, feeding: self.want_shoot, outtake: on && self.cmd.outtake, pass: pin.pass, legal: self.last_in_zone, intake: self.cmd.intake,
                    target: pin.target, aim_hub: pin.aim_hub, table: if self.shot.is_some_and(|s| s.pass) { &self.tables.pass } else { &self.tables.hub },
                    est_vel: pin.est_vel, est_omega, yaw: self.yaw, pos: self.pos, aim_err: self.aim_err,
                };
                self.stored = p.post_step(&mut c, dt, t, &pin0, &mut self.stats, &mut self.out);
                self.hood_deg = self.out.hood_deg;
                self.flywheel = self.out.flywheel;
                self.ready = self.out.ready;
                self.full = self.stored.len() >= p.capacity();
            }
            Some(Phys::R1114(p)) => {
                let pin4 = crate::mech::phys1114::PostIn {
                    on, feeding: self.want_shoot, outtake: on && self.cmd.outtake, setpoint: self.shot_setpoint, pass: pin.pass, legal: self.last_in_zone,
                    intake: self.cmd.intake, auto: !self.teleop, spin: None, lineup: false, has_shot: self.shot.is_some(),
                    aim_pass: self.loc.aim_pt(Phys1114::pass_target(self.alliance, self.pos), self.pos, self.yaw), aim_hub: pin.aim_hub, pos: self.pos, aim_err: self.aim_err,
                };
                self.stored = p.post_step(&mut c, dt, t, &pin4, &mut self.stats, &mut self.out);
                self.hood_deg = self.out.hood_deg;
                self.flywheel = self.out.flywheel;
                self.ready = self.out.ready;
                self.full = p.p.watch_full(c.phys, c.fuel, self.body, dt, on && self.cmd.intake);
            }
            Some(Phys::R341(p)) => {
                let est_omega = self.loc.omega().unwrap_or(c.phys.bodies[self.body].angvel().y as f64);
                let pin3 = crate::mech::phys341::PostIn {
                    on, feeding: self.want_shoot, outtake: on && self.cmd.outtake, setpoint: self.shot_setpoint, pass: pin.pass, legal: self.last_in_zone,
                    intake: self.cmd.intake, shot: pin.target, aim_hub: pin.aim_hub, est_vel: pin.est_vel, est_omega, pos: self.pos, alliance: self.alliance, aim_err: self.aim_err,
                };
                self.stored = p.post_step(&mut c, dt, t, &pin3, &mut self.stats, &mut self.out);
                self.hood_deg = self.out.hood_deg;
                self.flywheel = self.out.flywheel;
                self.ready = self.out.ready;
                self.full = self.stored.len() >= p.capacity();
            }
            Some(Phys::R7769(p)) => {
                let pin7 = crate::mech::phys7769::PostIn {
                    on, feeding: self.want_shoot, outtake: on && self.cmd.outtake, setpoint: self.shot_setpoint, pass: pin.pass, legal: self.last_in_zone,
                    intake: self.cmd.intake, has_shot: self.shot.is_some(),
                    aim_pass: self.loc.aim_pt(Phys7769::pass_target(self.alliance, self.pos), self.pos, self.yaw), aim_hub: pin.aim_hub,
                    pos: self.pos, yaw: self.yaw, est_vel: pin.est_vel, aim_err: self.aim_err, pass_clear: self.pass_clear, in_zone,
                };
                self.stored = p.post_step(&mut c, dt, t, &pin7, &mut self.stats, &mut self.out);
                self.hood_deg = self.out.hood_deg;
                self.flywheel = self.out.flywheel;
                self.ready = self.out.ready;
                self.full = p.p.watch_full(c.phys, c.fuel, self.body, dt, on && self.cmd.intake);
            }
            Some(Phys::R1678(p)) => {
                let est_vel2 = self.loc.vel(self.yaw).unwrap_or({ let v = c.phys.bodies[self.body].linvel(); [v.x as f64, v.z as f64] });
                let lv = c.phys.bodies[self.body].linvel();
                let pin6 = crate::mech::phys1678::PostIn {
                    on, feeding: self.want_shoot, outtake: on && self.cmd.outtake, setpoint: self.shot_setpoint, pass: pin.pass, legal: self.last_in_zone,
                    intake: self.cmd.intake, has_shot: self.shot.is_some(), aim_hub: pin.aim_hub,
                    aim_ferry: self.loc.aim_pt(crate::mech::phys1678::ferry(self.alliance, self.pos).1, self.pos, self.yaw),
                    pos: self.pos, alliance: self.alliance, est_vel: est_vel2, aim_err: self.aim_err, pass_clear: self.pass_clear,
                    half_l: self.half_l, linvel: [lv.x as f64, lv.y as f64, lv.z as f64], bay_top: opt(&self.cfg["bay"], "top").unwrap_or(0.0), lift_h: 0.0,
                };
                self.stored = p.post_step(&mut c, dt, t, &pin6, &mut self.stats, &mut self.out);
                self.hood_deg = self.out.hood_deg;
                self.flywheel = self.out.flywheel;
                self.ready = self.out.ready;
                let lift_h = self.cfg["bay"].get("lift").filter(|x| !x.is_null()).and_then(|l| opt(l, "h")).unwrap_or(0.0);
                self.full = p.watch_full(c.phys, c.fuel, self.body, dt, on && self.cmd.intake, lift_h, dc);
            }
            Some(Phys::R8793(p)) => {
                let pin8 = crate::mech::phys8793::PostIn {
                    on, feeding: self.want_shoot, outtake: on && self.cmd.outtake, setpoint: self.shot_setpoint, pass: pin.pass, legal: self.last_in_zone,
                    aim_err: self.aim_err, pass_clear: self.pass_clear,
                };
                let mut o = crate::mech::phys8793::PostOut8793::default();
                self.stored = p.post_step(&mut c, dt, t, &pin8, &mut self.stats, &mut o);
                self.flywheel = o.flywheel;
                self.out.flywheel = o.flywheel;
                self.out.feeding = o.feeding;
                self.out.intake_speed = o.intake_speed;
                self.out.hood_deg = self.hood_deg;
                self.full = self.stored.len() >= p.capacity();
            }
            None => {}
        }
    }

    /// js `_intakePhys(dt, on)` for a robot whose mechanisms turn its arm (8793's): only whether
    /// the game wants it down. It stays down while FUEL is up on it.
    fn intake_phys(&mut self, _phys: &PhysicsWorld, fuel: &Fuel, on: bool) {
        let ic = &self.cfg["intake"];
        let mut want = on && (self.cmd.intake || self.cmd.outtake);
        if ic["latched"].as_bool() == Some(true) && self.intake_deploy >= 1.0 { want = true; }
        let fl = num(&self.cfg["frame"], "length");
        let (c, s) = (m::cos(self.yaw), m::sin(self.yaw));
        if self.stored.iter().any(|&b| { let p = fuel.balls[b].pos; let (dx, dz) = (p[0] - self.pos[0], p[2] - self.pos[2]); dx * c - dz * s > fl / 2.0 }) { want = true; }
        self.intake_want = want;
    }

    /// js `_hopper(dt, on)` (nothing held: what the load needs of the extension is nothing)
    fn hopper_step(&mut self, phys: &mut PhysicsWorld, dt: f64, on: bool) {
        let st = &self.cfg["storage"];
        let ext = opt(st, "extLen").unwrap_or(0.0);
        if ext == 0.0 { return; }
        // (a physical robot's extension goes where its mechanism puts it: 4414's box on its racks)
        let ext_now = match &self.phys { Some(Phys::R4414(p)) => Some(p.ext_now()), Some(Phys::R1678(p)) => Some(p.ext_now()), Some(Phys::R1690(p)) => Some(p.ext_now()), Some(Phys::R7769(p)) => Some(p.ext_now()), Some(Phys::R1114(p)) => Some(p.ext_now()), _ => None };
        if let Some(e) = ext_now { self.hopper_deploy = crate::fuel::clamp(e / ext, 0.0, 1.0); }
        else {
            let latched = st["extend"].as_str() == Some("latched");
            let mut want = if latched { if self.hopper_deploy > 0.02 || self.intake_deploy > 0.3 { 1.0 } else { 0.0 } } else { self.intake_deploy };
            want = m::max(want, 0.0);
            if on || want < self.hopper_deploy { self.hopper_deploy = approach(self.hopper_deploy, want, dt / 0.45); }
        }
        let Some((col, wall)) = self.hopper else { return };
        let out = self.hopper_deploy * ext;
        let fx = num(&self.cfg["frame"], "length") / 2.0 - ext / 2.0 + out;
        phys.colliders[col].set_position_wrt_parent(Pose::from_translation(vf(fx, self.hopper_y, 0.0)));
        phys.colliders[wall].set_position_wrt_parent(Pose::from_translation(vf(fx, self.hopper_y, 0.0)));
        let ball = if st.get("box").filter(|b| b.is_object()).is_some() { 0 } else { GROUP::BALL };
        let is_out = self.hopper_deploy > 0.05;
        phys.colliders[col].set_collision_groups(grp(if is_out { groups(GROUP::ROBOT, ball | GROUP::ROBOT | GROUP::INTAKE) } else { 0 }));
        phys.colliders[wall].set_collision_groups(grp(if is_out { groups(GROUP::ROBOT, GROUP::ROBOT_BARRIER | GROUP::STATIC) } else { 0 }));
    }

    /// js `_stepTopColliders()` (an intake block; lids and folding or sliding intakes later)
    fn step_top_colliders(&mut self, phys: &mut PhysicsWorld) {
        let ic = &self.cfg["intake"];
        // (the lid where the climber has it: js hopper.liftScale, a physical robot's liftNow)
        if let Some((lid, lid_x)) = self.lid {
            let h = self.lift_h();
            let scale = match &self.phys { Some(Phys::R1678(p)) => p.lift_now(h), _ => 0.0 };
            phys.colliders[lid].set_translation_wrt_parent(vf(lid_x, num(&self.cfg["bay"], "top") + h * scale + 0.006, 0.0));
        }
        if let Some(fold) = ic.get("fold").filter(|x| !x.is_null()) {
            let a = (1.0 - self.intake_deploy) * num(fold, "stowDeg") * DEG;
            // (as rapier.js sets it: normalized, through its scaled axis)
            let q = Rotation::from_xyzw(0.0, 0.0, m::sin(a / 2.0) as f32, m::cos(a / 2.0) as f32);
            phys.colliders[self.arm_collider].set_rotation_wrt_parent(q.normalize().to_scaled_axis());
        }
        if let Some((bhx, by0, by1)) = self.intake_block {
            let out = self.intake_deploy * (num(ic, "reach") + 0.04);
            let x = match opt(ic, "front") { Some(front) => self.half_l + self.intake_deploy * (front - self.half_l) - bhx, None => self.half_l - bhx + out - 0.02 };
            phys.colliders[self.arm_collider].set_position_wrt_parent(Pose::from_translation(vf(x, (by0 + by1) / 2.0, 0.0)));
            phys.colliders[self.arm_collider].set_collision_groups(grp(if self.intake_deploy > 0.05 { groups(GROUP::INTAKE, GROUP::STATIC | GROUP::ROBOT | GROUP::INTAKE | GROUP::ROBOT_BARRIER) } else { 0 }));
        }
    }
}

/// js `climbLevelFromLift(lift)`
pub fn climb_level_from_lift(lift: f64) -> u8 {
    let mut l = 0;
    for i in 1..=3 { if lift >= CLIMB_LIFT[i] - 1e-3 { l = i as u8; } }
    l
}

/// js util `approach(cur, target, maxStep)`
pub fn approach(cur: f64, target: f64, max_step: f64) -> f64 {
    let d = target - cur;
    if m::abs(d) <= max_step { return target; }
    cur + m::sign(d) * max_step
}

/// js physics `passesUnder(collider, top)`
fn passes_under(phys: &mut PhysicsWorld, under: &mut Vec<UnderArm>, c: ColliderHandle, top: Top) {
    let ug = (0xffff0000u32 | (0xffff & !GROUP::TRENCH_ARM)) as u32;
    phys.colliders[c].set_solver_groups(grp(ug));
    under.push(UnderArm { collider: c, top, under: true });
}

/// js physics.step()'s TRENCH arm rule, before the world's step
pub fn under_arms(phys: &mut PhysicsWorld, under: &mut [UnderArm], arms: &crate::world::TrenchArms) {
    let dt = phys.integration_parameters.dt as f64;
    for u in under.iter_mut() {
        let p = phys.colliders[u.collider].translation();
        let (px, pz) = (p.x as f64, p.z as f64);
        if !arms.arms.iter().any(|a| m::abs(px - a.x) < a.hx + 1.0 && m::abs(pz - a.z) < a.hz + 1.0) { continue; }
        let is_under = top(phys, &u.top, dt) < arms.arms_bottom;
        if is_under != u.under {
            u.under = is_under;
            phys.colliders[u.collider].set_solver_groups(grp(if is_under { arms.under_solver_groups } else { 0xffffffff }));
        }
    }
}

fn top(phys: &PhysicsWorld, t: &Top, dt: f64) -> f64 {
    match t {
        Top::Frame { body, pts } => {
            let b = &phys.bodies[*body];
            let (p, q, v, w, c) = (b.translation(), *b.rotation(), b.linvel(), b.angvel(), b.center_of_mass());
            let (qx, qy, qz, qw) = (q.x as f64, q.y as f64, q.z as f64, q.w as f64);
            let rx = [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)];
            let ry = [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx)];
            let rz = [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy)];
            let mut tp = f64::NEG_INFINITY;
            for k in 0..pts.len() / 3 {
                let (x, y, z) = (pts[3 * k], pts[3 * k + 1], pts[3 * k + 2]);
                let wx = p.x as f64 + rx[0] * x + rx[1] * y + rx[2] * z;
                let wy = p.y as f64 + ry[0] * x + ry[1] * y + ry[2] * z;
                let wz = p.z as f64 + rz[0] * x + rz[1] * y + rz[2] * z;
                let vy = v.y as f64 + w.z as f64 * (wx - c.x as f64) - w.x as f64 * (wz - c.z as f64);
                tp = m::max(tp, wy + m::max(0.0, vy) * dt);
            }
            tp
        }
        Top::Lid { body, lid, hl, hw } => {
            let b = &phys.bodies[*body];
            let t = phys.colliders[*lid].translation();
            let (q, v, w) = (*b.rotation(), b.linvel(), b.angvel());
            let (qx, qy, qz, qw) = (q.x as f64, q.y as f64, q.z as f64, q.w as f64);
            let tip = m::abs(2.0 * (qw * qz + qx * qy)) * hl + m::abs(2.0 * (qw * qx - qy * qz)) * hw;
            t.y as f64 + 0.006 + tip + m::max(0.0, v.y as f64 + m::abs(w.z as f64) * hl + m::abs(w.x as f64) * hw) * dt
        }
    }
}
