//! js/mech/r1114.js: 1114's Simbot Tim as physical mechanisms, from their CAD and their code
//! (Simbotics/2026-Simbot-Tim). Robot frame: x forward (the intake), y up, z to its right.
//!
//! The hopper box slides out on racks carrying the intake on its front (a 2in silicone roller, the
//! kicker bar hanging in front of the bumper); inside, a floor of nine rollers slopes to a valley
//! in front of the tower and up its back under the squish wheels; side rollers and passive rollers
//! funnel the load in; the ball elevator's two columns lift FUEL to the 3in flywheel, which carries
//! them up round its back under the hood's three rollers and flings them forward. The roller wall
//! in front of the elevator carries the load leaning on it down to the elevator's foot.
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{NetSheet, Panel, Surface, Wheel, BUMPER_MU};
use jsmath as m;

/// js MU (estimates)
pub mod mu {
    pub const GRIP_TAPE: f64 = 1.3;
    pub const SILICONE: f64 = 1.0;
    pub const COMPLIANT: f64 = 1.6;
}
/// js FLOOR: the floor's nine 1.25in rollers, front to back
pub mod floor {
    pub const ROLLERS: [[f64; 2]; 9] = [[0.2857, 0.1668], [0.2276, 0.1457], [0.1696, 0.1246], [0.1116, 0.1034], [0.0536, 0.0823], [-0.0326, 0.0719], [-0.1144, 0.1007], [-0.1751, 0.1626], [-0.2022, 0.2451]];
    pub const R: f64 = 0.01665;
    pub const WIDTH: f64 = 0.3937;
}
/// js SQUISH: the squish wheels at the tower's foot
pub mod squish {
    pub const AT: [f64; 2] = [-0.0118, 0.2619];
    pub fn r() -> f64 { 1.5 * 0.0254 }
    pub const W: f64 = 0.0254;
    pub const Z: [f64; 7] = [-0.1524, -0.1016, -0.0508, 0.0, 0.0508, 0.1016, 0.1524];
}
/// js SIDE: the 2in side rollers
pub mod side {
    pub const X: f64 = 0.1129;
    pub const Y: f64 = 0.2246;
    pub const Z: f64 = 0.2725;
    pub const R: f64 = 0.0254;
    pub const LEN: f64 = 0.171;
}
/// js PASSIVE: the free 5/8in rollers either side of the tower's front
pub mod passive {
    pub const AT: [[f64; 2]; 2] = [[0.0483, 0.2883], [0.08, 0.3382]];
    pub const Y: f64 = 0.4286;
    pub const R: f64 = 0.008;
    pub const LEN: f64 = 0.153;
}
/// js ELEV: the elevator's columns and the roller wall
pub mod elev {
    pub const BACK: [[f64; 2]; 4] = [[-0.2096, 0.3224], [-0.2096, 0.3816], [-0.2096, 0.4408], [-0.2096, 0.5001]];
    pub const FRONT: [[f64; 2]; 4] = [[-0.0343, 0.3224], [-0.0343, 0.3816], [-0.0343, 0.4408], [-0.0343, 0.5001]];
    pub const WALL: [[f64; 2]; 3] = [[0.0134, 0.3266], [0.0195, 0.4134], [0.0255, 0.5002]];
    pub const R: f64 = 0.01665;
    pub const WIDTH: f64 = 0.3937;
}
/// js FLY: the flywheel's axis
pub const FLY: [f64; 2] = [-0.0176, 0.5715];
/// js SHOOTER
pub mod shooter {
    use jsmath as m;
    pub const FLY_R: f64 = 0.0402;
    pub const FLY_WIDTH: f64 = 0.4064;
    pub const HOOD: [[f64; 2]; 3] = [[-0.2024, 0.6137], [-0.1841, 0.6621], [-0.1534, 0.7037]];
    pub const HOOD_R: f64 = 0.01975;
    pub const HOOD_WIDTH: f64 = 0.4064;
    pub fn fly_per_motor() -> f64 { (36.0 / 24.0) * (24.0 / 48.0) }
    pub fn brass_per_motor() -> f64 { (36.0 / 24.0) * (36.0 / 15.0) }
    pub const HOOD_PER_FLY: f64 = 36.0 / 24.0;
    pub const FLY_J: f64 = 1.36e-3;
    pub const BRASS_J: f64 = 1.171e-3;
    pub const JACK_J: f64 = 5.06e-5;
    pub const MOTOR_J: f64 = 1.42e-4;
    pub const HOOD_J: f64 = 4.27e-4;
    pub const JACK_PER_MOTOR: f64 = 36.0 / 24.0;
    pub const KV: f64 = 0.14;
    pub const KP: f64 = 0.25;
    pub const IDLE: f64 = 1000.0;
    pub const STATOR: f64 = 50.0;
    pub const SUPPLY: f64 = 70.0;
    pub const HOOD_CAD: f64 = 45.6;
    pub const HOOD_HOME: f64 = 74.484;
    pub const HOOD_MIN: f64 = 42.0;
    pub const HOOD_MAX: f64 = 74.0;
    pub fn hood_deg_per_rot() -> f64 { (10.0 / 36.0) * (15.0 / 36.0) * (m::PI / 53.87831) * 360.0 }
    pub const HOOD_KP: f64 = 8.0;
}
/// js BALLEVATOR: the elevator's velocity loop and shot speed
pub mod ballevator {
    pub const KV: f64 = 0.13;
    pub const KP: f64 = 0.3;
    pub const SHOOT: f64 = 4500.0;
}
/// js INTAKE: the roller, as exported (the box all the way out)
pub mod intake {
    pub const AT: [f64; 2] = [0.5457, 0.1663];
    pub const R: f64 = 0.0262;
    pub const WIDTH: f64 = 0.635;
}
/// js SLIDE: the box's racks
pub mod slide {
    use jsmath as m;
    pub const OUT: f64 = 0.298;
    pub const DIR: [f64; 2] = [-0.988, 0.152];
    pub const PINION: f64 = 0.0127;
    pub const RATIO: f64 = 3.6;
    pub const MASS: f64 = 6.0;
    pub fn per_rot() -> f64 { (m::PI * 0.0254) / 3.6 }
    pub const PULL: f64 = -3.5;
}
/// js SLIDE_POS: their slide positions (motor turns)
pub mod slide_pos {
    pub const DROP: f64 = 13.55;
    pub const OUT: f64 = 13.4;
    pub const REDEPLOY: f64 = 13.0;
    pub const ROLLERS_OFF: f64 = 10.8;
    pub const AGITATE_MIN: f64 = 6.3;
    pub const OUT_CHECK: f64 = 9.0;
    pub fn tol() -> f64 { 0.025 / 0.0254 }
}
/// js `slideTurns(ext)`
pub fn slide_turns(ext: f64) -> f64 { ext / slide::per_rot() }
/// js KICKER
pub mod kicker {
    use jsmath as m;
    pub const PIN: [f64; 2] = [0.4513, 0.196];
    pub const LEN: f64 = 0.153;
    pub const BAR_R: f64 = 0.016;
    pub const MAX: f64 = m::PI / 2.0;
}
/// js BUMPER
pub mod bumper {
    pub const X0: f64 = 0.3294;
    pub fn x1() -> f64 { 0.3294 + 2.5 * 0.0254 }
    pub const Y0: f64 = 0.0476;
    pub const Y1: f64 = 0.1721;
    pub fn hw() -> f64 { 0.399 + 2.5 * 0.0254 }
}
/// js PANEL: the box's front panel
pub mod panel {
    pub const X0: f64 = 0.6044;
    pub const X1: f64 = 0.606;
    pub const Y0: f64 = 0.2692;
    pub const Y1: f64 = 0.701;
    pub const HW: f64 = 0.3766;
}
/// js BLANKET: the plush over the box's top
pub mod blanket {
    pub const X0: f64 = 0.115;
    pub const X1: f64 = 0.598;
    pub const Y: f64 = 0.701;
    pub const HW: f64 = 0.37;
    pub const BULGE: f64 = 0.05;
    pub const TREAD: f64 = 6000.0;
}
/// js RANGE: their hood CANrange
pub mod range {
    pub const AT: [f64; 2] = [-0.122, 0.33];
    pub const HALF: f64 = 0.06;
}
/// js PRELOAD
pub const PRELOAD: [[f64; 3]; 8] = [
    [-0.122, 0.4, -0.1], [-0.122, 0.4, 0.1],
    [0.082, 0.193, -0.153], [0.082, 0.193, 0.0], [0.082, 0.193, 0.153],
    [0.192, 0.3, -0.15], [0.192, 0.3, 0.0], [0.192, 0.3, 0.15],
];

/// js SIM_HUB, SIM_PASS: distance -> motor RPM, hood
pub const SIM_HUB: [[f64; 3]; 25] = [[1.583, 2347.0, 74.4], [1.713, 2487.0, 74.4], [1.915, 2585.0, 74.4], [2.115, 2591.0, 72.5], [2.29, 2687.0, 72.5], [2.353, 2597.0, 70.5], [2.53, 2688.0, 70.5],
    [2.73, 2756.0, 69.5], [2.93, 2749.0, 69.0], [3.02, 2854.0, 69.0], [3.22, 2872.0, 68.5], [3.35, 2885.0, 67.5], [3.5, 2974.0, 67.5], [3.67, 3026.0, 67.5], [3.8, 2963.0, 65.5], [3.9, 2987.0, 65.5],
    [4.02, 3076.0, 65.5], [4.07, 3089.0, 65.5], [4.11, 3113.0, 65.5], [4.25, 3189.0, 65.5], [4.46, 3307.0, 65.5], [4.8, 3390.0, 65.0], [4.95, 3447.0, 65.0], [5.15, 3533.0, 65.0], [5.35, 3640.0, 65.0]];
pub const SIM_PASS: [[f64; 3]; 7] = [[3.539, 2187.0, 55.0], [4.714, 2585.0, 55.0], [6.37, 3088.0, 55.0], [7.7, 3524.0, 55.0], [9.25, 4044.0, 55.0], [12.28, 5850.0, 50.0], [14.02, 5850.0, 50.0]];
fn interp(t: &[[f64; 3]], x: f64, j: usize) -> f64 {
    if x <= t[0][0] { return t[0][j]; }
    for k in 1..t.len() {
        if x <= t[k][0] { return t[k - 1][j] + ((t[k][j] - t[k - 1][j]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); }
    }
    t[t.len() - 1][j]
}
/// js `shotFor(kind, d)`: (rpm, hood)
pub fn shot_for(pass: bool, d: f64) -> (f64, f64) {
    let t: &[[f64; 3]] = if pass { &SIM_PASS } else { &SIM_HUB };
    let (rpm, hood) = (interp(t, d, 1), interp(t, d, 2));
    (rpm - 50.0, m::max(shooter::HOOD_MIN, m::min(shooter::HOOD_MAX, hood)))
}

/// js build1114()'s answer (indices into Drives)
pub struct Trains {
    pub intake: usize, pub roller: usize, pub floor: usize, pub rollers: Vec<usize>, pub squish: usize,
    pub side_t: usize, pub side: [usize; 2], pub elev: usize, pub front: Vec<usize>, pub back: Vec<usize>, pub wall: usize,
    pub passive: Vec<usize>, pub shooter: usize, pub fly: usize, pub hood: usize, pub slide: usize,
}
/// js `build1114()`
pub fn build1114(d: &mut Drives) -> Trains {
    const SUPPLY: f64 = 70.0;
    let sh = |name: &str, j: f64| Shaft::new(name, None, j, 2e-4, 0.0);
    let intake = d.train("intake rollers", Motor::new("krakenX60", 2.0, 1.0, 0.9, 85.0, SUPPLY));
    let roller = d.add(intake, sh("intake 2in roller (silicone)", 3.2e-4), -12.0 / 30.0);
    let floor = d.train("floor (hopper)", Motor::new("krakenX60", 2.0, 1.0, 0.9, 70.0, SUPPLY));
    let rollers = (0..floor::ROLLERS.len()).map(|i| d.add(floor, sh(&format!("floor roller {}", i + 1), 5e-5), 12.0 / 24.0)).collect();
    let squish = d.add(floor, sh("squish wheels (3in 45A)", 3.5e-4), -(12.0 / 24.0) * (18.0 / 30.0) * (20.0 / 44.0));
    let side_t = d.train("side rollers", Motor::new("krakenX44", 1.0, 1.0, 0.9, 70.0, SUPPLY));
    let side = [d.add(side_t, sh("left side roller", 7e-5), 12.0 / 64.0), d.add(side_t, sh("right side roller", 7e-5), -12.0 / 64.0)];
    let elev = d.train("ball elevator", Motor::new("krakenX60", 1.0, 1.0, 0.9, 70.0, SUPPLY));
    let front = (0..elev::FRONT.len()).map(|i| d.add(elev, sh(&format!("elevator front roller {}", i + 1), 5e-5), -12.0 / 24.0)).collect();
    let back = (0..elev::BACK.len()).map(|i| d.add(elev, sh(&format!("elevator back roller {}", i + 1), 5e-5), (12.0 / 24.0) * (30.0 / 26.0))).collect();
    let wall = d.add(elev, sh("roller wall", 1.5e-4), -12.0 / 24.0 / 2.0);
    let shooter = d.train("shooter", Motor::new("krakenX60Trap", 4.0, 1.0, 0.9, shooter::STATOR, shooter::SUPPLY));
    let fly = d.add(shooter, sh("flywheel (3in, grip tape)", shooter::FLY_J), -shooter::fly_per_motor());
    d.add(shooter, sh("4in brass flywheel", shooter::BRASS_J), shooter::brass_per_motor());
    d.add(shooter, Shaft::new("jackshaft (36t, 24ts)", None, shooter::JACK_J, 0.0, 0.0), shooter::JACK_PER_MOTOR);
    d.add(shooter, Shaft::new("motors' 50t gears and 36t", None, shooter::MOTOR_J, 0.0, 0.0), 1.0);
    let hood = d.add(shooter, sh("hood rollers", shooter::HOOD_J), shooter::fly_per_motor() * shooter::HOOD_PER_FLY);
    let mut s = Shaft::new("slide pinions", Some(Motor::new("krakenX44", 1.0, slide::RATIO, 0.9, 60.0, SUPPLY)), slide::MASS * m::pow(slide::PINION, 2.0), 0.05, 0.15);
    s.racks = true;
    let slide = d.shaft(s);
    let passive = (0..passive::AT.len() * 2).map(|i| d.shaft(Shaft::new(&format!("passive side roller {}", i + 1), None, 4e-6, 2e-5, 0.002))).collect();
    Trains { intake, roller, floor, rollers, squish, side_t, side, elev, front, back, wall, passive, shooter, fly, hood, slide }
}
/// js `slideStops(shaft, ext)`
pub fn slide_stops(d: &mut Drives, s: usize, ext: f64) {
    let sh = &mut d.shafts[s];
    sh.lo = if ext <= 1e-9 { 0.0 } else { f64::NEG_INFINITY };
    sh.hi = if ext >= slide::OUT - 1e-9 { 0.0 } else { f64::INFINITY };
}

/// js wheels1114()'s answer
pub struct Wheels {
    pub floor: Vec<Wheel>, pub squish: Vec<Wheel>, pub side: Vec<Wheel>, pub passive: Vec<Wheel>, pub elev: Vec<Wheel>, pub wall: Vec<Wheel>,
    pub intake: Vec<Wheel>, pub intake_home: Vec<[f64; 3]>, pub fly: Wheel, pub hood: Vec<Wheel>, pub hood_home: Vec<[f64; 3]>,
}
/// js `wheels1114(t)`
pub fn wheels1114(t: &Trains, d: &Drives) -> Wheels {
    let (zx, xx) = ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let name = |s: usize| d.shafts[s].name.clone();
    let floor = floor::ROLLERS.iter().enumerate().map(|(i, [x, y])| Wheel::new(&format!("{} (grip tape)", name(t.rollers[i])), t.rollers[i], [*x, *y, 0.0], zx, floor::R, floor::WIDTH).mu(mu::GRIP_TAPE).tread(40000.0)).collect();
    let squish = squish::Z.iter().map(|&z| Wheel::new("squish wheel (3in 45A)", t.squish, [squish::AT[0], squish::AT[1], z], zx, squish::r(), squish::W).mu(mu::COMPLIANT).tread(5000.0)).collect();
    let side = [-1.0, 1.0].iter().enumerate().map(|(i, s)| Wheel::new(&format!("{} (cat tongue)", name(t.side[i])), t.side[i], [side::X, side::Y, s * side::Z], xx, side::R, side::LEN).mu(mu::GRIP_TAPE).tread(40000.0)).collect();
    let col = |pts: &[[f64; 2]], shafts: &[usize]| -> Vec<Wheel> { pts.iter().enumerate().map(|(i, [x, y])| Wheel::new(&format!("{} (grip tape)", name(shafts[i])), shafts[i], [*x, *y, 0.0], zx, elev::R, elev::WIDTH).mu(mu::GRIP_TAPE).tread(40000.0)).collect() };
    let mut el = col(&elev::BACK, &t.back);
    el.extend(col(&elev::FRONT, &t.front));
    let wall = elev::WALL.iter().enumerate().map(|(i, [x, y])| Wheel::new(&format!("roller wall {} (cat tongue)", i + 1), t.wall, [*x, *y, 0.0], zx, elev::R, elev::WIDTH).mu(mu::GRIP_TAPE).tread(40000.0)).collect();
    let pts: Vec<[f64; 2]> = passive::AT.iter().flat_map(|[x, z]| [-1.0, 1.0].map(|sg| [*x, sg * z])).collect();
    let passive = pts.iter().enumerate().map(|(i, [x, z])| Wheel::new(&format!("{} (5/8in)", name(t.passive[i])), t.passive[i], [*x, passive::Y, *z], [0.0, 1.0, 0.0], passive::R, passive::LEN).mu(0.5).tread(40000.0)).collect();
    let intake = vec![Wheel::new("intake 2in roller (silicone)", t.roller, [intake::AT[0], intake::AT[1], 0.0], zx, intake::R, intake::WIDTH).mu(mu::SILICONE).tread(20000.0)];
    let intake_home = intake.iter().map(|w| w.center).collect();
    let fly = Wheel::new("flywheel (3in, grip tape)", t.fly, [FLY[0], FLY[1], 0.0], zx, shooter::FLY_R, shooter::FLY_WIDTH).mu(mu::GRIP_TAPE).tread(40000.0);
    let hood: Vec<Wheel> = shooter::HOOD.iter().enumerate().map(|(i, h)| Wheel::new(&format!("hood 1.5in roller {} (grip tape)", i + 1), t.hood, [h[0], h[1], 0.0], zx, shooter::HOOD_R, shooter::HOOD_WIDTH).mu(mu::GRIP_TAPE).tread(40000.0)).collect();
    let hood_home = hood.iter().map(|w| w.center).collect();
    Wheels { floor, squish, side, passive, elev: el, wall, intake, intake_home, fly, hood, hood_home }
}
/// js `rot(p, c, a)`
fn rot([x, y]: [f64; 2], [cx, cy]: [f64; 2], a: f64) -> [f64; 2] {
    let (c, s, dx, dy) = (m::cos(a), m::sin(a), x - cx, y - cy);
    [cx + dx * c - dy * s, cy + dx * s + dy * c]
}
/// js `slideOffset(ext)`
pub fn slide_offset(ext: f64) -> [f64; 2] { let d = slide::OUT - ext; [slide::DIR[0] * d, slide::DIR[1] * d] }
/// js `placeIntake(wheels, ext)`
pub fn place_intake(wheels: &mut [Wheel], home: &[[f64; 3]], ext: f64) {
    let [dx, dy] = slide_offset(ext);
    for (w, h) in wheels.iter_mut().zip(home) { w.center = [h[0] + dx, h[1] + dy, h[2]]; }
}
/// js `kickerRot(ext)`
pub fn kicker_rot(ext: f64) -> f64 {
    let [dx, dy] = slide_offset(ext);
    let (px, py, k) = (kicker::PIN[0] + dx, kicker::PIN[1] + dy, kicker::LEN);
    let (xb, yc) = (bumper::x1() + kicker::BAR_R, bumper::Y1 + kicker::BAR_R);
    if px >= xb { return 0.0; }
    let a1 = m::asin(m::min(1.0, (xb - px) / k));
    let a2 = m::acos(m::max(-1.0, m::min(1.0, (py - yc) / k)));
    m::min(m::min(kicker::MAX, a1), a2)
}
/// js `hoodRot(deg)`
pub fn hood_rot(deg: f64) -> f64 { ((deg - shooter::HOOD_CAD) * m::PI) / 180.0 }
/// js `placeHood(wheels, deg)`
pub fn place_hood(wheels: &mut [Wheel], home: &[[f64; 3]], deg: f64) {
    let a = hood_rot(deg);
    for (w, h) in wheels.iter_mut().zip(home) { let [x, y] = rot([h[0], h[1]], FLY, a); w.center = [x, y, 0.0]; }
}
/// js `hoodElevation(deg)`
pub fn hood_elevation(deg: f64) -> f64 {
    let [hx, hy] = shooter::HOOD[2];
    (m::atan2(hy - FLY[1], hx - FLY[0]) * 180.0) / m::PI - 90.0 + (hood_rot(deg) * 180.0) / m::PI
}

#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
struct Path { body: PathMesh, slide: PathMesh, kicker: PathMesh, hood: PathMesh }

/// pathSurfaces1114's surfaces, in its order (path: all of them; shooter: the hood)
pub const S_BODY: usize = 0;
pub const S_SLIDE: usize = 1;
pub const S_PANEL: usize = 2;
pub const S_KICKER: usize = 3;
pub const S_HOOD: usize = 4;
pub const S_BLANKET: usize = 5;
pub const S_BUMPER: usize = 6;

/// js `pathSurfaces1114(mesh, t)`
pub fn path_surfaces1114(t: &Trains) -> Vec<Surface> {
    static P: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    let p = P.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path1114.json")).expect("path1114.json"));
    const MU: f64 = 0.25;
    let s = |k: &str, mesh: &PathMesh, pos: [f64; 3], mu: f64, keep: &dyn Fn(&[f64], &[u32], usize) -> bool| {
        let n = mesh.i.len() / 3;
        let mut idx = Vec::new();
        for j in 0..n { if keep(&mesh.v, &mesh.i, j) { idx.extend_from_slice(&mesh.i[j * 3..j * 3 + 3]); } }
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{k} (CAD)"), &v, &idx, mu, pos, 0.0, 0.0, 0.08, f64::INFINITY)
    };
    let all = |_: &[f64], _: &[u32], _: usize| true;
    // (the mesh's panel, two sheets 1 cm apart, left out: FrontPanel is the panel)
    let panel_tri = |v: &[f64], i: &[u32], t: usize| (0..3).all(|k| { let (x, y) = (v[i[t * 3 + k] as usize * 3], v[i[t * 3 + k] as usize * 3 + 1]); (600.0..=610.0).contains(&x) && y >= 270.0 });
    let dir = [-slide::DIR[0], -slide::DIR[1], 0.0];
    let mut sl = s("slide", &p.slide, [0.0; 3], MU, &|v, i, j| !panel_tri(v, i, j)).on_slide(t.slide, dir);
    sl.set_slide_k(slide::PINION);
    let mut pn = Surface::panel("front panel (CAD)", MU, Panel::new(panel::X0, panel::X1, panel::Y0, panel::Y1, panel::HW)).on_slide(t.slide, dir);
    pn.set_slide_k(slide::PINION);
    let mut bl = Surface::sheet("blanket", 0.5, blanket::TREAD, NetSheet { x0: 0.0, y0: 0.0, xs: 0.0, x1: 0.0, yf: 0.0, hw: blanket::HW, bulge: blanket::BULGE });
    place_blanket(&mut bl, 0.0, 0.0);
    vec![
        s("body", &p.body, [0.0; 3], MU, &all),
        sl,
        pn,
        s("kicker", &p.kicker, [kicker::PIN[0], kicker::PIN[1], 0.0], 1.0, &all),
        s("hood", &p.hood, [FLY[0], FLY[1], 0.0], MU, &all),
        bl,
        Surface::block("bumper", bumper::X0, bumper::x1(), bumper::Y0, bumper::Y1, bumper::hw(), 0.02, BUMPER_MU, 20000.0),
    ]
}
/// js Blanket `place(dx, dy)`: level across the box's top (a NetSheet level to its front edge)
fn place_blanket(s: &mut Surface, dx: f64, dy: f64) {
    let b = s.sheet_mut().unwrap();
    b.x0 = blanket::X0 + dx;
    b.x1 = blanket::X1 + dx;
    b.xs = b.x1;
    b.y0 = blanket::Y + dy;
    b.yf = b.y0;
}
/// js `placeSlide(s, ext)`
pub fn place_slide(sf: &mut [Surface], ext: f64) {
    let [dx, dy] = slide_offset(ext);
    sf[S_SLIDE].set_pose([dx, dy, 0.0], 0.0, 0.0);
    sf[S_PANEL].set_pose([dx, dy, 0.0], 0.0, 0.0);
    sf[S_KICKER].set_pose([kicker::PIN[0] + dx, kicker::PIN[1] + dy, 0.0], 0.0, kicker_rot(ext));
    place_blanket(&mut sf[S_BLANKET], dx, dy);
}
