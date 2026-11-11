//! js/mech/r1706.js: 1706's Mirage as physical mechanisms, from their CAD (Onshape "Mirage Public
//! Release", through a STEP translation) and their CAD release thread. Two streams from one
//! intake: the arm swings out over the bumper and pushes the hopper box out ahead of it; under the
//! hopper two spindexers carry the FUEL round to ramps under the two turrets, whose kickers push
//! them up into each turret's accelerators and hood. The browser game's file has the why of each
//! number; here are the numbers. The CAD's surfaces come from assets/mech/path1706.json. Robot
//! frame: x forward (the intake end), y up, z to its right.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{NetSlack, Surface, Wheel, BUMPER_MU};
use jsmath as m;

const DEG: f64 = m::PI / 180.0;

/// js MU, TREAD
pub mod mu {
    pub const URETHANE: f64 = 1.4;
    pub const POLYCARB: f64 = 0.6;
    pub const DISC: f64 = 0.45;
    pub const NET: f64 = 0.25;
}
pub mod tread {
    pub const URETHANE: f64 = 25000.0;
    pub const POLYCARB: f64 = 30000.0;
    pub const NET: f64 = 10000.0;
}
/// js HALF_L, HALF_W (25.2 x 29.5in), PIVOT (the intake arm's)
pub const HALF_L: f64 = 12.6 * IN;
pub const HALF_W: f64 = 14.75 * IN;
pub const PIVOT: [f64; 2] = [0.262, 0.168];
/// js INTAKE: its rollers and wheels as exported ([x, y], r, [z0, z1]), its swing, its arm's loop
pub mod intake {
    pub const TOP: ([f64; 2], f64, [f64; 2]) = ([0.0715, 0.3805], 0.0254, [-0.327, 0.336]);
    pub const BOTTOM: ([f64; 2], f64, [f64; 2]) = ([0.1500, 0.4485], 0.0254, [-0.334, 0.314]);
    pub const WHEELS: ([f64; 2], f64, [f64; 2]) = ([0.2955, 0.3440], 0.0127, [-0.351, 0.335]);
    pub const OUT: f64 = 113.4;
    pub const MAX: f64 = 113.4;
    pub const TOL: f64 = 20.0;
    pub const MM_V: f64 = 9.0;
    pub const MM_A: f64 = 60.0;
    pub const KP: f64 = 2.0;
    pub const KV: f64 = 0.12;
    pub const MASS: f64 = 4.5;
    pub const COM: [f64; 2] = [-0.12, 0.185];
    pub const J: f64 = 0.25;
}
/// js BOX: the hopper box
pub const BOX_OUT: f64 = 0.3048;
pub const BOX_SIDE: f64 = 0.374;
pub const BOX_FRONT: f64 = 0.318;
/// js SPIN: the spindexers (right, left)
pub const SPIN_AXES: [[f64; 2]; 2] = [[0.0889, -0.1778], [0.0889, 0.1778]];
pub const SPIN_TOP: f64 = 0.1611;
pub const SPIN_RATIO: f64 = 72.0 / 10.0;
pub const SPIN_J: f64 = 2.2e-3;
/// js SPIN_DIR: which way each turns to feed, seen from above
pub const SPIN_DIR: [f64; 2] = [1.0, -1.0];
/// js TURRET: the turrets' axes (right, left)
pub const TURRET: [[f64; 2]; 2] = [[-0.1651, -0.2220], [-0.1651, 0.2220]];
/// js KICKER
pub mod kicker {
    pub const FRONT: [f64; 3] = [-0.0718, 0.30, -0.2502];
    pub const BACK: [f64; 3] = [-0.2583, 0.30, -0.1943];
    pub const AXIS: [f64; 3] = [-0.2973, 0.0, -0.9548];
    pub const R: f64 = 0.0254;
    pub const W: f64 = 0.0508;
    pub const RATIO: f64 = 24.0 / 16.0;
}
/// js SHOOTER (each turret's, in its own frame: x along the shot, y up)
pub mod shooter {
    pub const ACCEL_AT: [f64; 2] = [0.1053, 0.4607];
    pub const ACCEL_R: f64 = 0.0381;
    pub const ACCEL_W: f64 = 0.0508;
    pub const HOOD: [([f64; 2], f64, f64); 2] = [([-0.0770, 0.4928], 0.0127, 0.0508), ([-0.0787, 0.4404], 0.0127, 0.0508)];
    pub const K_ACCEL: f64 = 22.0 / 24.0;
    pub const K_HOOD: f64 = 2.0;
    pub const J_FLY: f64 = 4.28e-4;
    pub const J_ACCEL: f64 = 1.90e-4;
    pub const J_HOOD: f64 = 9.5e-6;
    pub const HOOD_MAX: f64 = 45.0;
    pub const HOOD_V: f64 = 300.0;
    pub const HOOD_A: f64 = 3000.0;
    pub const KP: f64 = 0.3;
    pub const TOL_V: f64 = 0.3;
    pub const TOL_HOOD: f64 = 1.5;
    pub const TOL_YAW: f64 = 3.0;
}
/// js PRELOAD: four on each spindexer
pub fn preload() -> Vec<[f64; 3]> {
    [0, 1].iter().flat_map(|&s| (0..4).map(move |k| {
        let [x, z] = SPIN_AXES[s];
        let a = (k as f64 / 4.0) * 2.0 * m::PI + 0.4;
        [x + 0.112 * m::cos(a), SPIN_TOP + 0.08, z + 0.112 * m::sin(a)]
    })).collect()
}
/// js NET
pub mod net {
    pub const X0: f64 = -0.01;
    pub const Y: f64 = 0.548;
    pub const HW: f64 = 0.37;
    pub const BULGE: f64 = 0.15;
}

/// a turret's kicker (its train, its front and back stacks) and shooter (its train, accelerators,
/// hood wheels)
#[derive(Clone, Copy)]
pub struct Kicker { pub train: usize, pub front: usize, pub back: usize }
#[derive(Clone, Copy)]
pub struct Shooter { pub train: usize, pub accel: usize, pub hood: usize }
/// js build1706()'s answer (drives indices; and the hopper box's cam, geared off the arm's pivot)
pub struct Trains {
    pub intake: usize, pub top: usize, pub bottom: usize, pub wheels: usize, pub pivot: usize,
    pub spin: [usize; 2], pub kicker: [Kicker; 2], pub shooter: [Shooter; 2], pub cam: usize,
}
/// js `build1706()`
pub fn build1706(d: &mut Drives) -> Trains {
    let intake = d.train("intake rollers", Motor::new("krakenX60", 2.0, 1.0, 0.9, 80.0, 40.0));
    let top = d.add(intake, Shaft::new("intake top roller", None, 1.2e-4, 2e-4, 0.0), -10.0 / 14.0);
    let bottom = d.add(intake, Shaft::new("intake bottom roller", None, 1.2e-4, 2e-4, 0.0), -(10.0 / 14.0) * (14.0 / 24.0));
    let wheels = d.add(intake, Shaft::new("intake 1in wheels", None, 4e-5, 2e-4, 0.0), (10.0 / 14.0) * (14.0 / 24.0) * (18.0 / 16.0) * (24.0 / 20.0));
    let pivot = d.shaft(Shaft::new("intake arm", Some(Motor::new("krakenX60", 1.0, 16.0 * (32.0 / 22.0), 0.9, 80.0, 40.0)), intake::J, 0.3, 0.0));
    let spin = [0, 1].map(|i| d.shaft(Shaft::new(&format!("spindexer {}", if i == 1 { "left" } else { "right" }), Some(Motor::new("krakenX44", 1.0, SPIN_RATIO, 0.9, 80.0, 40.0)), SPIN_J, 0.01, 0.0)));
    let kicker = [0, 1].map(|i| {
        let train = d.train(&format!("kicker {}", if i == 1 { "left" } else { "right" }), Motor::new("krakenX44", 1.0, kicker::RATIO, 0.9, 80.0, 40.0));
        let front = d.add(train, Shaft::new("kicker front wheels", None, 3e-5, 2e-4, 0.0), 1.0);
        let back = d.add(train, Shaft::new("kicker back wheels", None, 3e-5, 2e-4, 0.0), 1.0);
        Kicker { train, front, back }
    });
    let shooter = [0, 1].map(|i| {
        let train = d.train(&format!("shooter {}", if i == 1 { "left" } else { "right" }), Motor::new("krakenX60", 1.0, 1.0, 0.9, 80.0, 40.0));
        d.add(train, Shaft::new("5in flywheel", None, shooter::J_FLY, 2e-4, 0.0), shooter::K_ACCEL);
        let accel = d.add(train, Shaft::new("3in accelerator wheels", None, shooter::J_ACCEL, 2e-4, 0.0), shooter::K_ACCEL);
        let hood = d.add(train, Shaft::new("hood 1in wheels", None, shooter::J_HOOD, 2e-4, 0.0), shooter::K_ACCEL * shooter::K_HOOD);
        Shooter { train, accel, hood }
    });
    // (js pathSurfaces1706's cam: the box slides boxRate m per radian of the arm)
    let mut c = Shaft::new("hopper box cam", None, f64::INFINITY, 0.0, 0.0);
    c.cam = Some((pivot, 0.0));
    let cam = d.shaft(c);
    Trains { intake, top, bottom, wheels, pivot, spin, kicker, shooter, cam }
}

/// js wheels1706()'s answer: the intake's (on the arm's pivot) and their homes, the kickers'
pub struct Wheels { pub intake: Vec<Wheel>, pub intake_home: Vec<[f64; 3]>, pub kicker: Vec<Wheel> }
/// js `wheels1706(t)`
pub fn wheels1706(t: &Trains) -> Wheels {
    let z = [0.0, 0.0, 1.0];
    let piv = [PIVOT[0], PIVOT[1], 0.0];
    let w = |name: &str, shaft: usize, (at, r, zz): ([f64; 2], f64, [f64; 2]), mu: f64, tread: f64| {
        Wheel::new(name, shaft, [at[0], at[1], (zz[0] + zz[1]) / 2.0], z, r, zz[1] - zz[0]).mu(mu).tread(tread).on_pivot(t.pivot, piv, [0.0, 0.0, -1.0])
    };
    let intake = vec![
        w("intake top roller (polycarbonate)", t.top, intake::TOP, mu::POLYCARB, tread::POLYCARB),
        w("intake bottom roller (polycarbonate)", t.bottom, intake::BOTTOM, mu::POLYCARB, tread::POLYCARB),
        w("intake 1in urethane wheels", t.wheels, intake::WHEELS, mu::URETHANE, tread::URETHANE),
    ];
    let intake_home = intake.iter().map(|x| x.center).collect();
    let mut kick = Vec::new();
    for i in 0..2 {
        let s = if i == 1 { -1.0 } else { 1.0 };
        let ax = [kicker::AXIS[0], 0.0, kicker::AXIS[2] * s];
        for (k, p) in [("front", kicker::FRONT), ("back", kicker::BACK)] {
            let c = [p[0], p[1], p[2] * s];
            let [tx, tz] = TURRET[i];
            let n = [tx - c[0], 0.0, tz - c[2]];
            let up = ax[2] * n[0] - ax[0] * n[2];
            let sg = m::sign(up);
            let shaft = if k == "front" { t.kicker[i].front } else { t.kicker[i].back };
            kick.push(Wheel::new(&format!("kicker {k} 2in urethane wheels"), shaft, c, ax.map(|v| v * sg), kicker::R, kicker::W).mu(mu::URETHANE).tread(tread::URETHANE));
        }
    }
    Wheels { intake, intake_home, kicker: kick }
}
/// js `shooterWheels(t)`: each turret's accelerators and the hood's two shafts' wheels
pub fn shooter_wheels(t: &Trains) -> [Vec<Wheel>; 2] {
    let z = [0.0, 0.0, 1.0];
    [0, 1].map(|i| {
        let mut v = vec![Wheel::new("3in accelerator wheels (urethane)", t.shooter[i].accel, [0.0; 3], z, shooter::ACCEL_R, shooter::ACCEL_W).mu(mu::URETHANE).tread(tread::URETHANE)];
        for (k, (_, r, w)) in shooter::HOOD.iter().enumerate() {
            v.push(Wheel::new(&format!("hood {} 1in wheels (urethane)", if k == 1 { "lower" } else { "upper" }), t.shooter[i].hood, [0.0; 3], z, *r, *w).mu(mu::URETHANE).tread(tread::URETHANE));
        }
        v
    })
}

/// js `rot([x, y], [cx, cy], a)`
fn rot([x, y]: [f64; 2], [cx, cy]: [f64; 2], a: f64) -> [f64; 2] {
    let (c, s, dx, dy) = (m::cos(a), m::sin(a), x - cx, y - cy);
    [cx + dx * c - dy * s, cy + dx * s + dy * c]
}
/// js `armRot(deg)`: the arm's turn from the CAD's pose (rad) at its swing deg
pub fn arm_rot(deg: f64) -> f64 { -deg * DEG }
/// js `placeIntake(wheels, deg)`
pub fn place_intake(wheels: &mut [Wheel], home: &[[f64; 3]], deg: f64) {
    let a = arm_rot(deg);
    for (w, h) in wheels.iter_mut().zip(home) { let [x, y] = rot([h[0], h[1]], PIVOT, a); w.center = [x, y, h[2]]; }
}
/// js `boxExt(ext, deg)`: how far out the hopper box is at the arm's swing deg
pub fn box_ext(ext: &[f64], deg: f64) -> f64 {
    let n = ext.len() as f64;
    let u = m::max(0.0, m::min(n - 1.0, deg));
    let i = m::min(n - 2.0, m::floor(u));
    let f = u - i;
    let i = i as usize;
    m::min(BOX_OUT, ext[i] + (ext[i + 1] - ext[i]) * f)
}
/// js `boxRate(ext, deg)`: m of box per radian of the arm
pub fn box_rate(ext: &[f64], deg: f64) -> f64 { ((box_ext(ext, deg + 0.5) - box_ext(ext, deg - 0.5)) * 180.0) / m::PI }
/// js `turretPoint(i, [x, y, z], yaw)`
pub fn turret_point(i: usize, [x, y, z]: [f64; 3], yaw: f64) -> [f64; 3] {
    let (c, s) = (m::cos(yaw), m::sin(yaw));
    let [tx, tz] = TURRET[i];
    [tx + x * c + z * s, y, tz - x * s + z * c]
}
/// js `hoodWheelAt(k, h)`
pub fn hood_wheel_at(k: usize, h: f64) -> [f64; 2] { rot(shooter::HOOD[k].0, shooter::ACCEL_AT, -h * DEG) }
/// js `placeShooter(i, [accel, upper, lower], yaw, h)`
pub fn place_shooter(i: usize, w: &mut [Wheel], yaw: f64, h: f64) {
    let a = shooter::ACCEL_AT;
    w[0].center = turret_point(i, [a[0], a[1], 0.0], yaw);
    let u = hood_wheel_at(0, h);
    w[1].center = turret_point(i, [u[0], u[1], 0.0], yaw);
    let l = hood_wheel_at(1, h);
    w[2].center = turret_point(i, [l[0], l[1], 0.0], yaw);
    let (s, c) = (m::sin(yaw), m::cos(yaw));
    let ax = [s, 0.0, c];
    w[0].axis = ax.map(|v| -v);
    w[1].axis = ax;
    w[2].axis = ax;
}

/// Surfaces: their indices in path_surfaces1706's list (js Object.values order, flattened)
pub const S_BODY: usize = 0;
pub const S_BOWL: usize = 1;
pub const S_INTAKE: usize = 2;
pub const S_HOPPER: usize = 3;
pub const S_SPIN: [usize; 2] = [4, 5];
pub const S_TURRET: [usize; 2] = [6, 7];
pub const S_HOOD: [usize; 2] = [8, 9];
pub const S_BUMPER: usize = 10;
pub const S_NET: usize = 11;

#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Path { body: PathMesh, bowl: PathMesh, intake: PathMesh, hopper: PathMesh, spin_a: PathMesh, spin_b: PathMesh, turret: PathMesh, hood: PathMesh, pub ext: Vec<f64> }
/// js path1706.js (assets/mech/path1706.json)
pub fn path() -> &'static Path {
    static P: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    P.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path1706.json")).expect("path1706.json"))
}
/// js `pathSurfaces1706(mesh, t)`: the body, the bowls, the intake's arm (on its pivot), the hopper
/// box (on its cam), the spindexers (on their shafts), the turrets and hoods, the bumper, the net
pub fn path_surfaces1706(t: &Trains) -> Vec<Surface> {
    let p = path();
    let (mu0, mu_sh) = (0.25, 0.3);
    let s = |k: &str, mesh: &PathMesh, pos: [f64; 3], mu: f64, tread: f64| {
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{k} (CAD)"), &v, &mesh.i, mu, pos, 0.0, 0.0, 0.08, tread)
    };
    let piv = [PIVOT[0], PIVOT[1], 0.0];
    let mut hopper = s("hopper", &p.hopper, [0.0; 3], mu0, f64::INFINITY).on_slide(t.cam, [1.0, 0.0, 0.0]);
    hopper.set_slide_k(1.0);
    let spin = |i: usize, mesh: &PathMesh| {
        let a = [SPIN_AXES[i][0], 0.0, SPIN_AXES[i][1]];
        let mut sf = s(if i == 1 { "spinB" } else { "spinA" }, mesh, a, mu::DISC, f64::INFINITY).on_shaft(t.spin[i], a, [0.0, SPIN_DIR[i], 0.0]);
        sf.name = format!("spindexer {} (CAD)", if i == 1 { "left" } else { "right" });
        sf
    };
    let tp = |i: usize| [TURRET[i][0], 0.0, TURRET[i][1]];
    vec![
        s("body", &p.body, [0.0; 3], mu0, f64::INFINITY),
        // (the bowls: TREAD.bowl, rigid)
        s("bowl", &p.bowl, [0.0; 3], mu0, f64::INFINITY),
        s("intake", &p.intake, piv, mu0, f64::INFINITY).on_shaft(t.pivot, piv, [0.0, 0.0, -1.0]),
        hopper,
        spin(0, &p.spin_a),
        spin(1, &p.spin_b),
        s("turret", &p.turret, tp(0), mu_sh, f64::INFINITY),
        s("turret", &p.turret, tp(1), mu_sh, f64::INFINITY),
        s("hood", &p.hood, tp(0), mu_sh, f64::INFINITY),
        s("hood", &p.hood, tp(1), mu_sh, f64::INFINITY),
        Surface::block("bumper", 0.321, 0.398, 0.033, 0.154, 0.457, 0.02, BUMPER_MU, 20000.0),
        Surface::slack("net", mu::NET, tread::NET, NetSlack { x0: net::X0, x1: BOX_FRONT, y: net::Y, hw: net::HW, bulge: net::BULGE, ext: 0.0 }),
    ]
}
/// js `placeBox(sf, ext, deg)`: and the cam's ratio
pub fn place_box(sf: &mut [Surface], d: &mut Drives, cam: usize, deg: f64) {
    let ext = &path().ext;
    sf[S_HOPPER].set_pose([box_ext(ext, deg), 0.0, 0.0], 0.0, 0.0);
    if let Some(c) = &mut d.shafts[cam].cam { c.1 = box_rate(ext, deg); }
}
/// js `placeSpin(sf, i, phi)`
pub fn place_spin(sf: &mut [Surface], i: usize, phi: f64) { sf[S_SPIN[i]].set_pose([SPIN_AXES[i][0], 0.0, SPIN_AXES[i][1]], SPIN_DIR[i] * phi, 0.0); }
/// js `placeTurret(sf, i, yaw, h)`
pub fn place_turret(sf: &mut [Surface], i: usize, yaw: f64, h: f64) {
    sf[S_TURRET[i]].set_pose([TURRET[i][0], 0.0, TURRET[i][1]], yaw, 0.0);
    let a = shooter::ACCEL_AT;
    sf[S_HOOD[i]].set_pose(turret_point(i, [a[0], a[1], 0.0], yaw), yaw, -h * DEG);
}
