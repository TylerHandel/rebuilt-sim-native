//! js/mech/r341.js: 341's Miss Daisy XXIV as physical mechanisms, from their CAD, their design
//! binder (the gearings) and their code (Team341/FRC2026-Public). Robot frame: x forward (the
//! intake), y up, z to its right.
//!
//! The ball path, 4 wide to 1 ("the zipper"): the slapdown intake's rollers bring FUEL up over the
//! bumper onto the serializer's floor, where the table's top rollers and three 6in omnis lying flat
//! steer them into the uptake behind the turret; its flex wheels push them up into the turret,
//! where a 1in wheel hands them to the 4in flywheel, which wraps them round under the hood's Stealth
//! wheels and flings them out forward.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{Surface, Wheel, BUMPER_MU};
use jsmath as m;

/// js MU (estimates)
pub mod mu {
    pub const COMPLIANT: f64 = 1.6;
    pub const URETHANE: f64 = 1.3;
    pub const STEALTH: f64 = 1.4;
    pub const SILICONE: f64 = 1.0;
    pub const MECANUM: f64 = 0.8;
    pub const OMNI: f64 = 0.6;
    pub const POLYCARB: f64 = 0.5;
}

/// js HALF: the frame's half length and width (27in)
pub fn half() -> f64 { 13.5 * IN }
/// js PIVOT: the intake's pivot (the top roller's shaft)
pub const PIVOT: [f64; 2] = [0.255, 0.337];
/// js INTAKE
pub mod intake {
    pub const BOTTOM: ([f64; 2], f64, f64) = ([0.291, 0.680], 0.034, 0.583);
    pub const TOP: ([f64; 2], f64, f64) = ([0.119, 0.680], 0.034, 0.565);
    pub const DEAD: [[f64; 2]; 3] = [[0.192, 0.397], [0.121, 0.468], [0.083, 0.574]];
    pub fn dead_d() -> f64 { 2.0 * crate::constants::IN }
    pub const DEAD_W: f64 = 0.562;
    pub const COMPLIANT: [f64; 3] = [-0.056, -0.018, 0.020];
    pub const LEFT: [f64; 7] = [-0.298, -0.263, -0.228, -0.193, -0.158, -0.123, -0.088];
    pub const RIGHT: [f64; 7] = [0.052, 0.087, 0.122, 0.157, 0.192, 0.227, 0.262];
    pub const DOWN: f64 = 149.0;
    pub const UP: f64 = 44.0;
    pub const STOWED: f64 = 0.0;
    pub const CAD: f64 = 9.0;
    pub const MASS: f64 = 3.0;
    pub const COM: f64 = 0.25;
    pub const J: f64 = 0.2;
}
/// js OMNI: the serializer's three 6in omnis lying flat
pub mod omni {
    pub fn d() -> f64 { 6.0 * crate::constants::IN }
    pub const Y: f64 = 0.248;
    pub const W: f64 = 0.037;
    pub const FEEDER: [f64; 2] = [-0.044, 0.075];
    pub const FREE: [[f64; 2]; 2] = [[0.108, 0.219], [0.121, -0.229]];
}
/// js TABLE: the table's top rollers over the floor
pub mod table {
    pub const Y: f64 = 0.337;
    pub const A_X: f64 = 0.093;
    pub const A_COMPLIANT: [f64; 3] = [-0.098, -0.063, -0.029];
    pub const A_RIGHT: [f64; 6] = [0.002, 0.031, 0.059, 0.088, 0.116, 0.145];
    pub const B_X: f64 = 0.168;
    pub const B_COMPLIANT: [f64; 5] = [-0.087, -0.043, 0.0, 0.043, 0.087];
    pub const B_LEFT: [f64; 2] = [-0.158, -0.129];
    pub const B_RIGHT: [f64; 2] = [0.129, 0.158];
}
/// js UPTAKE
pub mod uptake {
    pub const BACK: [f64; 2] = [-0.187, 0.328];
    pub const FRONT: [f64; 2] = [-0.016, 0.328];
    pub const Z: [f64; 2] = [-0.064, -0.089];
    pub fn d() -> f64 { 1.625 * crate::constants::IN }
}
/// js TURRET: the turret's axis
pub const TURRET: [f64; 2] = [-0.102, -0.076];
/// js SHOOTER
pub mod shooter {
    use crate::constants::IN;
    pub const FLY_AT: [f64; 2] = [0.112, 0.466];
    pub fn fly_d() -> f64 { 4.0 * IN }
    pub const FLY_W: f64 = 0.1;
    pub const FEED_AT: [f64; 2] = [0.074, 0.410];
    pub fn feed_d() -> f64 { 1.0 * IN }
    pub const FEED_W: f64 = 0.1;
    pub const HOOD: [[f64; 2]; 2] = [[-0.202, -0.002], [-0.196, 0.052]];
    pub fn hood_d() -> f64 { 2.0 * IN }
    pub const HOOD_W: f64 = 0.05;
    pub const J: f64 = 1.228e-3;
    pub const J_STEALTH: f64 = 8.66e-5;
    pub const J_FEED: f64 = 8.3e-6;
    pub const KV: f64 = 0.135;
    pub const KP: f64 = 0.125;
    pub const TOL: f64 = 130.0;
    pub const HOOD_MAX: f64 = 31.3;
    pub const HOOD_TURN: [[f64; 2]; 2] = [[0.0, 0.0], [31.3, 31.3]];
}
/// js REGRESSIONS.hood and feedHood; SIM_SHOTS (SUB341 '1', the default)
const HOOD_TABLE: [[f64; 2]; 7] = [[1.51, 0.0], [2.38, 6.0], [3.03, 11.0], [3.5, 13.5], [4.24, 20.0], [5.5, 29.5], [14.0, 31.0]];
const FEED_HOOD: f64 = 28.0;
const SIM_RPM: [[f64; 2]; 7] = [[1.51, 2560.0], [2.38, 2290.0], [3.07, 2210.0], [3.5, 2265.0], [4.24, 2285.0], [5.5, 2515.0], [14.0, 3365.0]];
const SIM_FEED_RPM: [[f64; 2]; 7] = [[4.0, 1710.0], [5.5, 2115.0], [7.5, 2595.0], [9.0, 2925.0], [10.5, 3250.0], [13.94, 4520.0], [16.0, 5500.0]];
fn interp(t: &[[f64; 2]], x: f64) -> f64 {
    if x <= t[0][0] { return t[0][1]; }
    for k in 1..t.len() {
        if x <= t[k][0] { return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); }
    }
    t[t.len() - 1][1]
}
/// js `shotFor(kind, d)`: (rpm, hood)
pub fn shot_for(ferry: bool, d: f64) -> (f64, f64) {
    let rpm = interp(if ferry { &SIM_FEED_RPM } else { &SIM_RPM }, d);
    (rpm, if ferry { FEED_HOOD } else { interp(&HOOD_TABLE, d) })
}
/// js PRELOAD
pub const PRELOAD: [[f64; 3]; 8] = [
    [-0.095, 0.285, -0.077], [0.044, 0.248, -0.072],
    [0.241, 0.242, -0.138], [0.21, 0.246, 0.008], [0.243, 0.242, 0.153],
    [0.382, 0.241, -0.225], [0.382, 0.241, -0.075], [0.382, 0.241, 0.075],
];

/// js build341()'s answer (indices into Drives)
pub struct Trains {
    pub intake: usize, pub bottom: usize, pub top: usize, pub pivot_roller: usize, pub dead: [usize; 3],
    pub serializer: usize, pub table_a: usize, pub table_b: usize, pub up_back: usize, pub up_front: usize,
    pub feeder_t: usize, pub feeder: usize, pub omni_free: [usize; 2],
    pub shooter: usize, pub fly: usize, pub stealth: usize, pub feed: usize, pub pivot: usize,
}
/// js `build341()`
pub fn build341(d: &mut Drives) -> Trains {
    let sh = |name: &str, j: f64| Shaft::new(name, None, j, 2e-4, 0.0);
    let intake = d.train("intake rollers", Motor::new("krakenX60", 2.0, 1.0, 0.9, 90.0, 30.0));
    let k = (12.0 / 20.0) * (18.0 / 22.0);
    let bottom = d.add(intake, sh("intake bottom roller", 3e-5), k);
    let top = d.add(intake, sh("intake top roller", 3e-5), -k);
    let pivot_roller = d.add(intake, sh("intake top shaft (on the pivot)", 1.5e-4), -k);
    let dead = [0, 1, 2].map(|i| d.add(intake, sh(&format!("intake 2in roller {}", i + 1), 6e-5), -k));
    let serializer = d.train("top rollers and uptake", Motor::new("krakenX60", 2.0, 1.0, 0.9, 90.0, 30.0));
    let table_a = d.add(serializer, sh("table top roller (front)", 1e-4), -1.0 / 2.0);
    let table_b = d.add(serializer, sh("table top roller (back)", 1e-4), -1.0 / 2.0);
    let up_back = d.add(serializer, sh("uptake (back)", 3e-5), (1.0 / 2.0) * (7.0 / 5.0));
    let up_front = d.add(serializer, sh("uptake (front)", 3e-5), -(1.0 / 2.0) * (7.0 / 5.0));
    let feeder_t = d.train("feeder", Motor::new("falcon500", 1.0, 1.0, 0.9, 90.0, 30.0));
    let feeder = d.add(feeder_t, sh("feeder 6in omni", 6e-4), 1.0 / 6.0);
    let omni_free = [0, 1].map(|i| d.shaft(Shaft::new(&format!("free 6in omni {}", i + 1), None, 6e-4, 1e-4, 0.0)));
    let shooter = d.train("shooter", Motor::new("krakenX60", 2.0, 1.0, 0.9, 120.0, 200.0));
    let fly = d.add(shooter, sh("flywheel (4in urethane)", shooter::J), -1.0);
    let stealth = d.add(shooter, sh("hood Stealth wheels", shooter::J_STEALTH), 1.0);
    let feed = d.add(shooter, sh("1in urethane wheel", shooter::J_FEED), -1.0);
    let pivot = d.shaft(Shaft::new("intake pivot", Some(Motor::new("krakenX44", 1.0, 3.4 * (48.0 / 14.0) * (48.0 / 16.0), 0.9, 40.0, 30.0)), intake::J, 0.3, 0.0));
    Trains { intake, bottom, top, pivot_roller, dead, serializer, table_a, table_b, up_back, up_front, feeder_t, feeder, omni_free, shooter, fly, stealth, feed, pivot }
}

/// js `hand(dz)`: a mecanum's hand, pushing a FUEL under it toward dz
fn hand(dz: f64) -> f64 { 45.0 * m::sign(-dz) }

/// js wheels341()'s answer
pub struct Wheels { pub pivot_row: Vec<Wheel>, pub arm: Vec<Wheel>, pub arm_home: Vec<[f64; 3]>, pub table: Vec<Wheel>, pub uptake: Vec<Wheel>, pub omnis: Vec<Wheel> }
/// js `wheels341(t)`
pub fn wheels341(t: &Trains, d: &Drives) -> Wheels {
    let (zx, yx) = ([0.0, 0.0, 1.0], [0.0, 1.0, 0.0]);
    let comp = |shaft: usize, x: f64, y: f64, z: f64, name: &str| Wheel::new(name, shaft, [x, y, z], zx, (2.25 * IN) / 2.0, 1.0 * IN).mu(mu::COMPLIANT).tread(5000.0);
    let mec = |shaft: usize, x: f64, y: f64, z: f64, dz: f64, name: &str| Wheel::new(name, shaft, [x, y, z], zx, ((2.0 * IN) / 2.0) * 1.05, 0.9 * IN).mecanum(hand(dz)).mu(mu::MECANUM).tread(20000.0);
    let [px, py] = PIVOT;
    let mut pivot_row: Vec<Wheel> = intake::COMPLIANT.iter().map(|&z| comp(t.pivot_roller, px, py, z, "intake top shaft compliant")).collect();
    pivot_row.extend(intake::LEFT.iter().map(|&z| mec(t.pivot_roller, px, py, z, 1.0, "intake top shaft mecanum")));
    pivot_row.extend(intake::RIGHT.iter().map(|&z| mec(t.pivot_roller, px, py, z, -1.0, "intake top shaft mecanum")));
    let (b, tp) = (intake::BOTTOM, intake::TOP);
    let mut arm = vec![
        Wheel::new("intake bottom roller (silicone)", t.bottom, [b.0[0], b.0[1], 0.0], zx, b.1 / 2.0, b.2).mu(mu::SILICONE).tread(20000.0),
        Wheel::new("intake top roller (silicone)", t.top, [tp.0[0], tp.0[1], 0.0], zx, tp.1 / 2.0, tp.2).mu(mu::SILICONE).tread(20000.0),
    ];
    arm.extend(intake::DEAD.iter().enumerate().map(|(i, at)| Wheel::new(&d.shafts[t.dead[i]].name, t.dead[i], [at[0], at[1], 0.0], zx, intake::dead_d() / 2.0, intake::DEAD_W).mu(mu::POLYCARB)));
    let arm_home = arm.iter().map(|w| w.center).collect();
    let mut tb: Vec<Wheel> = table::A_COMPLIANT.iter().map(|&z| comp(t.table_a, table::A_X, table::Y, z, "table front roller compliant")).collect();
    tb.extend(table::A_RIGHT.iter().map(|&z| mec(t.table_a, table::A_X, table::Y, z, -1.0, "table front roller mecanum")));
    tb.extend(table::B_COMPLIANT.iter().map(|&z| comp(t.table_b, table::B_X, table::Y, z, "table back roller compliant")));
    tb.extend(table::B_LEFT.iter().map(|&z| mec(t.table_b, table::B_X, table::Y, z, 1.0, "table back roller mecanum")));
    tb.extend(table::B_RIGHT.iter().map(|&z| mec(t.table_b, table::B_X, table::Y, z, -1.0, "table back roller mecanum")));
    let ud = uptake::d();
    let mut up: Vec<Wheel> = uptake::Z.iter().map(|&z| Wheel::new("uptake back flex wheel", t.up_back, [uptake::BACK[0], uptake::BACK[1], z], zx, ud / 2.0, 1.0 * IN).mu(mu::COMPLIANT).tread(4000.0)).collect();
    up.extend(uptake::Z.iter().map(|&z| Wheel::new("uptake front flex wheel", t.up_front, [uptake::FRONT[0], uptake::FRONT[1], z], zx, ud / 2.0, 1.0 * IN).mu(mu::COMPLIANT).tread(4000.0)));
    let od = omni::d();
    let mut omnis = vec![Wheel::new("feeder 6in omni", t.feeder, [omni::FEEDER[0], omni::Y, omni::FEEDER[1]], yx, od / 2.0, omni::W).kind(crate::mech::mechanism::Kind::Omni).mu(mu::OMNI)];
    omnis.extend(omni::FREE.iter().enumerate().map(|(i, [x, z])| Wheel::new(&d.shafts[t.omni_free[i]].name, t.omni_free[i], [*x, omni::Y, *z], yx, od / 2.0, omni::W).kind(crate::mech::mechanism::Kind::Omni).mu(mu::OMNI)));
    Wheels { pivot_row, arm, arm_home, table: tb, uptake: up, omnis }
}
/// js `shooterWheels(t)`
pub fn shooter_wheels(t: &Trains) -> Vec<Wheel> {
    let z = [0.0, 0.0, 1.0];
    let mut w = vec![
        Wheel::new("flywheel (4in urethane)", t.fly, [0.0; 3], z, shooter::fly_d() / 2.0, shooter::FLY_W).mu(mu::URETHANE).tread(40000.0),
        Wheel::new("1in urethane wheel", t.feed, [0.0; 3], z, shooter::feed_d() / 2.0, shooter::FEED_W).mu(mu::URETHANE).tread(20000.0),
    ];
    for i in 0..shooter::HOOD.len() { w.push(Wheel::new(&format!("hood Stealth row {}", i + 1), t.stealth, [0.0; 3], z, shooter::hood_d() / 2.0, shooter::HOOD_W).mu(mu::STEALTH).tread(30000.0)); }
    place_shooter(&mut w, 0.0, 0.0);
    w
}

/// js `intakeRot(deg)`: the arm's turn from the CAD's pose (rad) at their angle
pub fn intake_rot(deg: f64) -> f64 { (-(deg - intake::CAD) * m::PI) / 180.0 }
/// js `rot(p, c, a)`
fn rot([x, y]: [f64; 2], [cx, cy]: [f64; 2], a: f64) -> [f64; 2] {
    let (c, s, dx, dy) = (m::cos(a), m::sin(a), x - cx, y - cy);
    [cx + dx * c - dy * s, cy + dx * s + dy * c]
}
/// js `placeIntake(wheels, deg)`
pub fn place_intake(wheels: &mut [Wheel], home: &[[f64; 3]], deg: f64) {
    let a = intake_rot(deg);
    for (w, h) in wheels.iter_mut().zip(home) { let [x, y] = rot([h[0], h[1]], PIVOT, a); w.center = [x, y, 0.0]; }
}
/// js `lerpT(t, x)`
fn lerp_t(t: &[[f64; 2]], x: f64) -> f64 {
    for k in 1..t.len() { if x <= t[k][0] { return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); } }
    t[t.len() - 1][1]
}
/// js `hoodRot(deg)`: the hood's turn about the flywheel's axle (rad) at their angle
pub fn hood_rot(deg: f64) -> f64 { (-lerp_t(&shooter::HOOD_TURN, m::max(0.0, m::min(shooter::HOOD_MAX, deg))) * m::PI) / 180.0 }
/// js `placeShooter([fly, feed, ...hood], yaw, hoodDeg)`
pub fn place_shooter(w: &mut [Wheel], yaw: f64, hood_deg: f64) {
    let (c, s) = (m::cos(yaw), m::sin(yaw));
    let at = |[x, y]: [f64; 2]| [TURRET[0] + x * c, y, TURRET[1] - x * s];
    w[0].center = at(shooter::FLY_AT);
    w[1].center = at(shooter::FEED_AT);
    let h = hood_rot(hood_deg);
    for (i, p) in shooter::HOOD.iter().enumerate() { let [hx, hy] = rot(*p, [0.0, 0.0], h); w[2 + i].center = at([shooter::FLY_AT[0] + hx, shooter::FLY_AT[1] + hy]); }
    for x in w.iter_mut() { x.axis = [s, 0.0, c]; }
}
/// js `hoodElevation(deg)`
pub fn hood_elevation(deg: f64) -> f64 {
    let [hx, hy] = shooter::HOOD[shooter::HOOD.len() - 1];
    let th = m::atan2(hy, hx) + hood_rot(deg);
    (m::atan2(-m::cos(th), m::sin(th)) * 180.0) / m::PI
}

#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
struct Path { body: PathMesh, intake: PathMesh, turret: PathMesh, hood: PathMesh }

/// pathSurfaces341's surfaces, in its order (Object.values: the CAD's and the guards)
pub const S_BODY: usize = 0;
pub const S_BUMPER: usize = 1;
pub const S_INTAKE: usize = 2;
pub const S_TURRET: usize = 3;
pub const S_HOOD: usize = 4;
pub const S_LID: usize = 5;
pub const S_WALL: usize = 6;
pub const S_OMNI: usize = 7;

/// js `pathSurfaces341(mesh, t)` (with guards341's lid, wall and omni cover)
pub fn path_surfaces341(t: &Trains) -> Vec<Surface> {
    static P: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    let p = P.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path341.json")).expect("path341.json"));
    let s = |k: &str, mesh: &PathMesh, pos: [f64; 3]| {
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{k} (CAD)"), &v, &mesh.i, 0.35, pos, 0.0, 0.0, 0.08, f64::INFINITY)
    };
    let piv = [PIVOT[0], PIVOT[1], 0.0];
    let x1 = 0.364 + 2.5 * IN;
    let guard = |name: &str, x0: f64, x1: f64, y0: f64, y1: f64, hw: f64, z: f64| {
        let mut b = Surface::block(name, x0, x1, y0, y1, hw, 0.005, 0.35, f64::INFINITY);
        b.set_pose([0.0, 0.0, z], 0.0, 0.0);
        b
    };
    vec![
        s("body", &p.body, [0.0; 3]),
        Surface::block("bumper", half(), x1, 0.041, 0.168, x1, 0.01, BUMPER_MU, 20000.0),
        s("intake", &p.intake, piv).on_shaft(t.pivot, piv, [0.0, 0.0, -1.0]),
        s("turret", &p.turret, [TURRET[0], 0.0, TURRET[1]]),
        s("hood", &p.hood, [TURRET[0] + shooter::FLY_AT[0], shooter::FLY_AT[1], TURRET[1]]),
        guard("lane lid (not in the CAD)", 0.008, 0.062, 0.326, 0.345, 0.07, -0.075),
        guard("lane wall to the table (not in the CAD)", -0.152, 0.095, 0.175, 0.372, 0.0475, -0.1965),
        guard("feeder omni cover (not in the CAD)", -0.094, 0.006, 0.272, 0.37, 0.05, 0.075),
    ]
}
/// js `placeTurretSurfaces({ turret, hood }, yaw, hoodDeg)`
pub fn place_turret_surfaces(sf: &mut [Surface], yaw: f64, hood_deg: f64) {
    sf[S_TURRET].set_pose([TURRET[0], 0.0, TURRET[1]], yaw, 0.0);
    let (c, s, fx) = (m::cos(yaw), m::sin(yaw), shooter::FLY_AT[0]);
    sf[S_HOOD].set_pose([TURRET[0] + fx * c, shooter::FLY_AT[1], TURRET[1] - fx * s], yaw, hood_rot(hood_deg));
}
