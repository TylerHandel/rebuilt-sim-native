//! js/mech/r8793.js: 8793 (Pumpkin Bots) built as a physical mechanism: its drive trains as the
//! team describes them, the rollers, wheels and their places from their CAD, in the robot's frame
//! (x forward, y up, z to its right, from the frame's center on the floor). The browser game's file
//! has the why of every number; here are the numbers.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{Kind, Surface, Wheel, BUMPER_MU};
use jsmath as m;

const DEG: f64 = m::PI / 180.0;
const FUEL_R: f64 = FUEL::radius;

/// js MU: what each grips FUEL with
pub mod mu {
    pub const COMPLIANT: f64 = 1.6;
    pub const GRIP_TAPE: f64 = 1.3;
    pub const OMNI: f64 = 0.6;
    pub const MECANUM: f64 = 0.8;
    pub const ALUMINUM: f64 = 0.45;
}
/// js COMPLIANT_TREAD
pub const COMPLIANT_TREAD: f64 = 12000.0;
/// js HORIZONTAL: [name, the stage from the shaft before (driver:driven), diameter, mu] (the tip on 18:32)
pub fn horizontal() -> [(&'static str, [f64; 2], f64, f64); 7] {
    [
        ("conveyor 1", [18.0, 33.0], 2.0 * IN, mu::MECANUM),
        ("conveyor 2", [25.0, 24.0], 2.0 * IN, mu::MECANUM),
        ("conveyor 3 (pivot)", [24.0, 43.0], 2.0 * IN, mu::MECANUM),
        ("intake 1", [23.0, 20.0], 2.0 * IN, mu::ALUMINUM),
        ("intake 2", [20.0, 20.0], 2.0 * IN, mu::ALUMINUM),
        ("intake 3", [20.0, 20.0], 2.0 * IN, mu::ALUMINUM),
        ("intake 4 (tip)", [18.0, 32.0], 2.0 * IN, mu::GRIP_TAPE),
    ]
}
/// js VERTICAL
pub mod vertical {
    pub const RATIO: [[f64; 2]; 3] = [[1.0, 5.0], [28.0, 18.0], [1.0, 1.0]];
    pub const DIAMETER: f64 = 4.0 * crate::constants::IN;
    pub const PER_X60: f64 = 1.2;
}
/// js NECK: x, y; each side's z, diameter, belt, omni or not
pub const NECK_X: f64 = 0.065;
pub const NECK_Y: f64 = 0.237;
pub struct NeckSide { pub z: f64, pub d: f64, pub belt: [f64; 2], pub omni: bool }
pub const NECK: [NeckSide; 2] = [
    NeckSide { z: -0.122, d: 4.0 * IN, belt: [18.0, 18.0], omni: true },
    NeckSide { z: 0.119, d: 4.0 * IN, belt: [18.0, 28.0], omni: false },
];
/// js DUTY
pub const DUTY_ROLLERS: f64 = 0.95;
pub const DUTY_VERTICAL: f64 = 0.95 * 0.9244;

/// js build8793()'s answer: the trains and shafts (drives indices), and each roller's diameter and grip
pub struct Trains {
    pub horizontal: usize,
    pub rollers: Vec<usize>,
    /// (the X44s coupled to the horizontal train: its coupled indices)
    pub x44: [usize; 2],
    pub vshafts: [usize; 2],
    pub necks: [usize; 2],
    pub indexer: usize,
    pub ishaft: usize,
    pub shooter: usize,
    pub large: usize,
    pub small: usize,
    pub pivot: usize,
}

/// js `build8793()`
pub fn build8793(d: &mut Drives) -> Trains {
    let horizontal = d.train("horizontal rollers", Motor::new("krakenX60", 1.0, 1.0, 0.9, 70.0, 40.0));
    let mut k = 1.0;
    let stages = self::horizontal();
    let rollers = stages.iter().map(|&(name, [a, b], _, _)| {
        k *= a / b;
        d.add(horizontal, Shaft::new(name, None, 6e-5, 2e-4, 0.0), k)
    }).collect();
    let vk = vertical::RATIO.iter().fold(1.0, |p, [a, b]| (p * a) / b);
    let mut x44 = [0; 2];
    for x in &mut x44 {
        d.trains[horizontal].coupled.push(crate::mech::drives::Coupled { motor: Motor::new("krakenX44", 1.0, 1.0, 0.9, 50.0, 40.0), k: vertical::PER_X60, cmd: Some(0.0) });
        *x = d.trains[horizontal].coupled.len() - 1;
    }
    let (mut vshafts, mut necks) = ([0; 2], [0; 2]);
    for (i, side) in ["left", "right"].iter().enumerate() {
        vshafts[i] = d.add(horizontal, Shaft::new(&format!("vertical roller, {side}"), None, 1e-4, 2e-4, 0.0), -vertical::PER_X60 * vk);
        let nk = &NECK[i];
        let name = format!("neck {}in {}, {side}", nk.d / IN, if nk.omni { "omni" } else { "compliant 45A" });
        necks[i] = d.add(horizontal, Shaft::new(&name, None, 6e-5, 2e-4, 0.0), (-vertical::PER_X60 * vk * nk.belt[0]) / nk.belt[1]);
    }
    let indexer = d.train("turret indexer", Motor::new("krakenX44", 1.0, 1.0, 0.9, 60.0, 40.0));
    let ishaft = d.add(indexer, Shaft::new("turret indexer", None, 4e-5, 2e-4, 0.0), -1.0);
    let shooter = d.train("shooter", Motor::new("krakenX60", 1.0, 1.0, 0.9, 160.0, 60.0));
    let large = d.add(shooter, Shaft::new("shooter 4in wheels", None, shooter::MOI, 2e-4, 0.0), -1.0 / shooter::RATIO);
    let small = d.add(shooter, Shaft::new("shooter 2in wheels", None, shooter::SMALL_MOI, 2e-4, 0.0), shooter::SMALL_PER_LARGE / shooter::RATIO);
    let pivot = d.shaft(Shaft::new("intake pivot", Some(Motor::new("krakenX60", 2.0, pivot::RATIO, 0.9, pivot::STATOR, pivot::SUPPLY)), pivot::J, pivot::DRAG, 0.0));
    Trains { horizontal, rollers, x44, vshafts, necks, indexer, ishaft, shooter, large, small, pivot }
}

/// js `intake8793(t, on, { dir })`: run the intake as their code does (dir -1: reversed, at full)
pub fn intake8793(d: &mut Drives, t: &Trains, on: bool, dir: f64) {
    let h = &mut d.trains[t.horizontal];
    h.cmd = Some(if !on { 0.0 } else if dir > 0.0 { -DUTY_ROLLERS } else { 1.0 });
    for &i in &t.x44 { h.coupled[i].cmd = Some(if !on { 0.0 } else if dir > 0.0 { -DUTY_VERTICAL } else { 1.0 }); }
}

/// js CONVEYOR: [roller index, x, y, the wheels' z (or the half width they spread over)]
enum Spread { At(&'static [f64]), Half(f64) }
const CONVEYOR: [(usize, f64, f64, Spread); 3] = [
    (0, 0.122, 0.315, Spread::At(&[-0.077, -0.039, -0.001, 0.037, 0.075])),
    (1, 0.217, 0.34, Spread::Half(0.24 - 2.0 * IN)),
    (2, 0.292, 0.336, Spread::Half(0.31)),
];
const VERT: [f64; 3] = [0.217, 0.241, 0.24];

/// js `conveyorWheels(t)`
pub fn conveyor_wheels(t: &Trains) -> Vec<Wheel> {
    let mut wheels = Vec::new();
    let w = 1.0 * IN;
    let hand = |dz: f64| 45.0 * m::sign(dz * -1.0);
    let spread = |n: usize, hw: &Spread| -> Vec<f64> {
        match hw {
            Spread::At(a) => a.to_vec(),
            Spread::Half(hw) => (0..n).map(|i| -hw + w / 2.0 + ((2.0 * hw - w) * i as f64) / (n - 1) as f64).collect(),
        }
    };
    let hz = horizontal();
    for (ri, x, y, hw) in &CONVEYOR {
        let (s, (name, _, dia, smu)) = (t.rollers[*ri], hz[*ri]);
        let base = |n: &str, z: f64| Wheel::new(n, s, [*x, *y, z], [0.0, 0.0, 1.0], dia / 2.0, w).mu(smu);
        wheels.push(Wheel::new(&format!("{name} shaft"), s, [*x, *y, 0.0], [0.0, 0.0, 1.0], 0.29 * IN, 0.62).mu(0.3));
        match ri {
            0 => for (i, z) in spread(5, hw).into_iter().enumerate() {
                wheels.push(if i == 2 { base("conveyor 1 compliant", z).tread(5000.0).mu(mu::COMPLIANT) } else { base("conveyor 1 mecanum", z).tread(20000.0).mecanum(hand(-m::sign(z))) });
            },
            1 => for z in spread(8, hw) { wheels.push(base("conveyor 2 mecanum", z).tread(20000.0).mecanum(hand(-m::sign(z)))); },
            _ => for (i, z) in spread(12, hw).into_iter().enumerate() {
                wheels.push(base("pivot mecanum", z).tread(20000.0).mecanum(hand(if i >= 8 { -1.0 } else { 1.0 })));
            },
        }
    }
    let r = vertical::DIAMETER / 2.0;
    for dy in [-0.6 * IN, 0.6 * IN] {
        wheels.push(Wheel::new("vertical omni", t.vshafts[0], [VERT[0], VERT[1] + dy, -VERT[2]], [0.0, -1.0, 0.0], r, w).kind(Kind::Omni).mu(mu::OMNI));
        wheels.push(Wheel::new("vertical compliant 45A", t.vshafts[1], [VERT[0], VERT[1] + dy, VERT[2]], [0.0, 1.0, 0.0], r, w).mu(mu::COMPLIANT).tread(COMPLIANT_TREAD));
    }
    for i in 0..2 {
        let nk = &NECK[i];
        let name = format!("neck {}in {}, {}", nk.d / IN, if nk.omni { "omni" } else { "compliant 45A" }, if i == 0 { "left" } else { "right" });
        let wh = Wheel::new(&name, t.necks[i], [NECK_X, NECK_Y, nk.z], [0.0, if i == 0 { -1.0 } else { 1.0 }, 0.0], nk.d / 2.0, w);
        wheels.push(if nk.omni { wh.kind(Kind::Omni).mu(mu::OMNI) } else { wh.mu(mu::COMPLIANT).tread(COMPLIANT_TREAD) });
    }
    wheels
}

/// js ARM: the pivot, the four rollers (from it), the half width
pub const ARM_PIVOT: [f64; 2] = [0.292, 0.336];
pub const ARM_ROLLERS: [[f64; 2]; 4] = [[0.0835, -0.0059], [0.1834, -0.0152], [0.2593, -0.0803], [0.2764, -0.1823]];
pub const ARM_HALF_WIDTH: f64 = 0.31;
/// js `armWheels(t)`
pub fn arm_wheels(t: &Trains) -> Vec<Wheel> {
    let [px, py] = ARM_PIVOT;
    let hz = horizontal();
    ARM_ROLLERS.iter().enumerate().map(|(i, [dx, dy])| {
        let (name, _, dia, smu) = hz[3 + i];
        Wheel::new(name, t.rollers[3 + i], [px + dx, py + dy, 0.0], [0.0, 0.0, 1.0], dia / 2.0, 2.0 * ARM_HALF_WIDTH).mu(smu).on_pivot(t.pivot, [px, py, 0.0], [0.0, 0.0, 1.0])
    }).collect()
}
/// js `placeArm(wheels, roll)`
pub fn place_arm(wheels: &mut [Wheel], roll: f64) {
    let [px, py] = ARM_PIVOT;
    let (c, s) = (m::cos(roll), m::sin(roll));
    for (i, w) in wheels.iter_mut().enumerate() {
        let [dx, dy] = ARM_ROLLERS[i];
        w.center = [px + dx * c - dy * s, py + dx * s + dy * c, 0.0];
    }
}

/// js PIVOT
pub mod pivot {
    pub const RATIO: f64 = 18.0;
    pub const STATOR: f64 = 80.0;
    pub const SUPPLY: f64 = 40.0;
    pub const DRAG: f64 = 0.5;
    pub const MASS: f64 = 3.8;
    pub const COM: f64 = 0.207;
    pub const COM_DEG: f64 = -28.0;
    pub const J: f64 = 0.215;
    pub const DOWN: f64 = 0.355;
    pub const PAST: f64 = 0.02;
    pub const MM_V: f64 = 1.0;
    pub const MM_A: f64 = 2.0;
    pub const KP: f64 = 60.0;
    pub const KD: f64 = 2.0;
    pub const KV: f64 = 0.96;
    pub const KG: f64 = 1.3;
    pub const G_OFFSET: f64 = 0.2;
    pub const TOL: f64 = 0.02;
}
/// js `profile(p, v, goal, vmax, amax, dt)` (Motion Magic)
fn profile(p: f64, v: f64, goal: f64, vmax: f64, amax: f64, dt: f64) -> [f64; 2] {
    let d = goal - p;
    let dir = m::sign(d);
    if m::abs(d) < 1e-9 && m::abs(v) < 1e-9 { return [goal, 0.0]; }
    let want = dir * m::min(vmax, m::sqrt(2.0 * amax * m::abs(d)));
    let nv = v + m::max(-amax * dt, m::min(amax * dt, want - v));
    let np = p + nv * dt;
    if (goal - np) * dir < 0.0 { return [goal, 0.0]; }
    [np, nv]
}
/// js `armState(stow)`: roll, how far it's swung up from down on its stop (rad); stow, the upper
/// stop; set, their Motion Magic's setpoint
#[derive(Clone, Copy, Debug)]
pub struct ArmState { pub roll: f64, pub stow: f64, pub set: Option<[f64; 2]> }
pub fn arm_state(stow: f64) -> ArmState { ArmState { roll: stow, stow, set: None } }
/// js `armPos(st)`: their units, arm rotations from their zero, positive down
pub fn arm_pos(st: &ArmState) -> f64 { pivot::DOWN - pivot::PAST - st.roll / (2.0 * m::PI) }
/// js `servoArm(t, st, down, dt, on)`
pub fn servo_arm(d: &mut Drives, piv: usize, st: &mut ArmState, down: bool, dt: f64, on: bool) {
    let v_ = 12.0;
    let p = arm_pos(st);
    let v = -d.omega(piv) / (2.0 * m::PI);
    if !on || st.set.is_none() { st.set = Some([p, if on { v } else { 0.0 }]); }
    if on {
        let [s0, s1] = st.set.unwrap();
        let set = profile(s0, s1, if down { pivot::DOWN } else { pivot::DOWN - pivot::PAST - st.stow / (2.0 * m::PI) }, pivot::MM_V, pivot::MM_A, dt);
        st.set = Some(set);
        let volts = pivot::KV * set[1] + pivot::KP * (set[0] - p) + pivot::KD * (set[1] - v) + pivot::KG * m::cos(2.0 * m::PI * (p + pivot::G_OFFSET));
        d.shafts[piv].cmd = Some(-m::max(-v_, m::min(v_, volts)) / v_);
    } else { d.shafts[piv].cmd = Some(0.0); }
    d.shafts[piv].load = -pivot::MASS * 9.81 * pivot::COM * m::cos(pivot::COM_DEG * DEG + st.roll);
    d.stop(piv, if st.roll <= 0.0 { 0.0 } else { f64::NEG_INFINITY }, if st.roll >= st.stow { 0.0 } else { f64::INFINITY });
}
/// js `turnArm(t, st, dt)`
pub fn turn_arm(d: &mut Drives, piv: usize, st: &mut ArmState, dt: f64) -> f64 {
    st.roll += d.omega(piv) * dt;
    if st.roll <= 0.0 { st.roll = 0.0; if d.shafts[piv].omega_ < 0.0 { d.shafts[piv].omega_ = 0.0; } }
    if st.roll >= st.stow { st.roll = st.stow; if d.shafts[piv].omega_ > 0.0 { d.shafts[piv].omega_ = 0.0; } }
    st.roll
}
/// js PRELOAD
pub const PRELOAD: [[f64; 3]; 6] = [[-0.052, 0.131, 0.0], [-0.137, 0.248, 0.006], [-0.122, 0.392, 0.0], [0.073, 0.213, 0.0], [0.19, 0.25, -0.116], [0.193, 0.249, 0.082]];

/// js INDEXER, DUTY_INDEXER
const INDEXER_SHAFT: [f64; 2] = [-0.039, 0.220];
const INDEXER_WHEEL_Z: [f64; 3] = [-0.025, 0.001, 0.026];
pub const DUTY_INDEXER: f64 = 0.65;
/// js `indexerWheels(t)`
pub fn indexer_wheels(t: &Trains) -> Vec<Wheel> {
    let [x, y] = INDEXER_SHAFT;
    let mut wheels: Vec<Wheel> = INDEXER_WHEEL_Z.iter().map(|&z| Wheel::new("turret indexer", t.ishaft, [x, y, z], [0.0, 0.0, 1.0], 1.0 * IN, 1.0 * IN).mu(mu::COMPLIANT).tread(5000.0)).collect();
    wheels.push(Wheel::new("turret indexer shaft", t.ishaft, [x, y, -0.05], [0.0, 0.0, 1.0], 0.29 * IN, 0.28).mu(0.3));
    wheels
}
/// js PATH_FAST: (speed, sub)
pub const PATH_FAST: (f64, usize) = (4.0, 4);

/// js SHOOTER
pub mod shooter {
    use crate::constants::IN;
    pub const RATIO: f64 = 28.0 / 18.0;
    pub const SMALL_PER_LARGE: f64 = 28.0 / 18.0;
    pub const MOI: f64 = 1.046e-3;
    pub const SMALL_MOI: f64 = 6.6e-5;
    pub const LARGE_X: f64 = 0.127;
    pub const LARGE_Y: f64 = 0.411;
    pub const LARGE_R: f64 = 2.0 * IN;
    pub const SMALL_R: f64 = 1.0 * IN;
    pub const SMALL_D: f64 = 0.203;
    pub const ELEVATION: [f64; 2] = [52.0, 72.0];
    pub const KV: f64 = 0.125;
    pub const KP: f64 = 0.4;
    pub const MAX_RPS: f64 = 90.0;
    pub const MU: f64 = 1.7;
}
/// js `shooterSmallAt(e)`
pub fn shooter_small_at(e: f64) -> [f64; 2] {
    let a = (e * m::PI) / 180.0;
    [shooter::LARGE_X - shooter::SMALL_D * m::sin(a), shooter::LARGE_Y + shooter::SMALL_D * m::cos(a)]
}
/// js `shooterWheels(t, e)` (the turret at its yaw 0, turretX -0.127)
pub fn shooter_wheels(t: &Trains, e: f64) -> Vec<Wheel> {
    let mut w = vec![
        Wheel::new("shooter 4in 45A roller", t.large, [0.0; 3], [0.0, 0.0, 1.0], shooter::LARGE_R, 1.5 * IN).mu(shooter::MU).tread(20000.0),
        Wheel::new("shooter 2in 80A Stealth", t.small, [0.0; 3], [0.0, 0.0, 1.0], shooter::SMALL_R, 3.0 * IN).mu(shooter::MU).tread(40000.0),
    ];
    place_shooter(&mut w, e, 0.0, -0.127);
    w
}
/// js `placeShooter([lw, sw], e, yaw, turretX)`
pub fn place_shooter(w: &mut [Wheel], e: f64, yaw: f64, turret_x: f64) {
    let (c, s) = (m::cos(yaw), m::sin(yaw));
    let at = |x: f64, y: f64, z: f64| [turret_x + x * c + z * s, y, -x * s + z * c];
    let [sx, sy] = shooter_small_at(m::max(shooter::ELEVATION[0], m::min(shooter::ELEVATION[1], e)));
    w[0].center = at(shooter::LARGE_X, shooter::LARGE_Y, 0.0);
    w[1].center = at(sx, sy, 0.0);
    w[0].axis = [s, 0.0, c];
    w[1].axis = [s, 0.0, c];
}
/// js SHOT_TABLE: in play, what leaves for the flywheel's speed (rotations/s -> m/s)
pub const SHOT_TABLE: [[f64; 2]; 10] = [[0.0, 0.0], [15.0, 2.7], [20.0, 3.51], [30.0, 4.91], [40.0, 6.25], [50.0, 7.58], [60.0, 8.66], [70.0, 10.0], [80.0, 11.43], [90.0, 12.06]];
fn interp(t: &[[f64; 2]], x: f64, i: usize, j: usize) -> f64 {
    for k in 1..t.len() {
        if x <= t[k][i] { return t[k - 1][j] + ((t[k][j] - t[k - 1][j]) * (x - t[k - 1][i])) / (t[k][i] - t[k - 1][i]); }
    }
    t[t.len() - 1][j]
}
/// js `exitSpeed(rps)`, `rpsFor(v)`
pub fn exit_speed(rps: f64) -> f64 { interp(&SHOT_TABLE, m::max(0.0, rps), 0, 1) }
pub fn rps_for(v: f64) -> f64 { interp(&SHOT_TABLE, m::max(0.0, v), 1, 0) }

/// Surfaces: their indices in path_surfaces8793's list
pub const S_BODY: usize = 0;
pub const S_ARM: usize = 1;
pub const S_TURRET: usize = 2;
pub const S_BUMPER: usize = 3;
pub const S_GUIDES: usize = 4;

#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
struct Path { body: PathMesh, arm: PathMesh, turret: PathMesh }

/// js `pathSurfaces(PATH)` (turretX -0.127, the arm's pivot, mu 0.3): body, arm, turret, bumper,
/// guides, in that order; the arm on the pivot's shaft
pub fn path_surfaces8793(t: &Trains) -> Vec<Surface> {
    static PATH: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    let p = PATH.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path8793.json")).expect("path8793.json"));
    let s = |name: &str, mesh: &PathMesh, pos: [f64; 3]| {
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{name} (CAD)"), &v, &mesh.i, 0.3, pos, 0.0, 0.0, 0.08, f64::INFINITY)
    };
    let [px, py] = ARM_PIVOT;
    vec![
        s("body", &p.body, [0.0, 0.0, 0.0]),
        s("arm", &p.arm, [px, py, 0.0]).on_shaft(t.pivot, [px, py, 0.0], [0.0, 0.0, 1.0]),
        s("turret", &p.turret, [-0.127, 0.0, 0.0]),
        bumper_surface(),
        guide_surface(),
    ]
}
/// js `guideSurface()`
fn guide_surface() -> Surface {
    let (mut v, mut ix): (Vec<f64>, Vec<u32>) = (Vec::new(), Vec::new());
    let mut quad = |a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]| {
        let o = (v.len() / 3) as u32;
        for p in [a, b, c, d] { v.extend(p); }
        ix.extend([o, o + 1, o + 2, o, o + 2, o + 3]);
    };
    let (fx, fz, fy) = (0.10, 0.125, 0.215);
    for zs in [1.0, -1.0] {
        let z = 0.085 * zs;
        quad([0.0, 0.05, z], [-0.33, 0.05, z], [-0.33, 0.27, z], [0.0, 0.27, z]);
        quad([fx, 0.05, fz * zs], [0.0, 0.05, z], [0.0, fy, z], [fx, fy, fz * zs]);
        quad([0.15, 0.178, 0.165 * zs], [0.335, 0.178, 0.165 * zs], [0.335, 0.178, 0.335 * zs], [0.15, 0.178, 0.335 * zs]);
    }
    quad([0.03, 0.17, 0.175], [0.16, 0.17, 0.175], [0.16, 0.34, 0.175], [0.03, 0.34, 0.175]);
    Surface::mesh("guides", &v, &ix, 0.3, [0.0; 3], 0.0, 0.0, 0.08, f64::INFINITY)
}
/// js `bumperSurface()`: a solid block, its face where the arm's tip roller squeezes FUEL 1-1/8in
fn bumper_surface() -> Surface {
    let (half_l, half_w, squeeze) = (13.75 * IN, 13.75 * IN + BUMPER_T, (9.0 / 8.0) * IN);
    let (tx, ty) = (ARM_PIVOT[0] + ARM_ROLLERS[3][0], ARM_PIVOT[1] + ARM_ROLLERS[3][1]);
    let reach = 1.0 * IN + FUEL_R * 2.0 - squeeze;
    let x1 = m::min(half_l + BUMPER_T, tx - m::sqrt(reach * reach - m::pow(BUMPER_Y1 - ty, 2.0)));
    Surface::block("bumper", half_l, x1, BUMPER_Y0, BUMPER_Y1, half_w, 0.003, BUMPER_MU, 20000.0)
}
