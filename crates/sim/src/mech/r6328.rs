//! js/mech/r6328.js: 6328's Darwin as physical mechanisms, from their CAD (the AdvantageScope model
//! their code ships) and their code (Mechanical-Advantage/RobotCode2026Public). Robot frame: x
//! forward (the intake), y up, z to its right.
//!
//! The slap-down intake's 2in roller pulls FUEL in and up its ramp plate into the hopper's front,
//! which slides out with the intake (a pin on its side plate in a slot in the walls); the
//! Superdexer's floor carries them back to the launcher: the back kicker's rollers up the tunnel's
//! back, the front kicker roller pinching them up into the 4in drum, the hood wrapping them up over
//! its front and out over the back. A bungee net over the hopper stretches as the load lifts it.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{Bungee, Surface, Wheel, BUMPER_MU};
use jsmath as m;

const DEG: f64 = m::PI / 180.0;
/// js MU (estimates)
pub mod mu {
    pub const SILICONE: f64 = 1.0;
    pub const POLYCARB: f64 = 0.5;
    pub const STEEL: f64 = 0.45;
    pub const NET: f64 = 0.25;
}
const SIL_TREAD: f64 = 20000.0;

/// js PIVOT, DRUM, LAUNCHER_X
pub const PIVOT: [f64; 2] = [0.317, 0.1855];
pub const DRUM: [f64; 2] = [-0.2639, 0.4861];
pub const LAUNCHER_X: f64 = -0.151;
/// js INTAKE
pub mod intake {
    pub const CAD: f64 = 147.5;
    pub const DOWN: f64 = 17.4;
    pub const UP: f64 = 145.8;
    pub const ROLLER: ([f64; 2], f64, f64) = ([0.6055, 0.1675], 0.0275, 0.547);
    pub const BASH: ([f64; 2], f64, f64) = ([0.635, 0.2145], 0.0159, 0.629);
    pub const TOE: ([f64; 2], f64, f64) = ([0.49, 0.048], 0.00635, 0.632);
    pub const RAMP_TOP: [f64; 2] = [0.405, 0.2];
    pub const RAMP_HW: f64 = 0.316;
    pub const MASS: f64 = 3.2;
    pub const COM: f64 = 0.222;
    pub const COM_DEG: f64 = -20.0;
    pub const J: f64 = 0.18;
}
/// js INTAKE[k].at: a part given down, in the arm's frame as exported (turned back up about the pivot)
pub fn at_exported(down: [f64; 2]) -> [f64; 2] {
    let a = (intake::CAD - intake::DOWN) * DEG;
    let [x, y] = down;
    let (dx, dy) = (x - PIVOT[0], y - PIVOT[1]);
    [PIVOT[0] + dx * m::cos(a) - dy * m::sin(a), PIVOT[1] + dx * m::sin(a) + dy * m::cos(a)]
}
/// js SLIDE_MASS
pub const SLIDE_MASS: f64 = 2.1;
/// js `hopX(deg)`: the hopper's sliding front for the pin at deg
pub fn hop_x(deg: f64) -> f64 {
    let t = deg * DEG;
    (12.48 + 7.204 * m::cos(t) + (7.204 * m::sin(t) - 2.393) / m::tan(70.0 * DEG) + 6.789) * IN
}
/// js `slideExt(deg)`, `slideRate(deg)`
pub fn slide_ext(deg: f64) -> f64 { hop_x(deg) - hop_x(intake::CAD) }
pub fn slide_rate(deg: f64) -> f64 { ((slide_ext(deg + 0.5) - slide_ext(deg - 0.5)) * 180.0) / m::PI }
/// js FLOOR, FLOOR_R, FLOOR_W, FLOOR_RATIO
pub const FLOOR: [[f64; 2]; 9] = [[-0.1587, 0.1905], [-0.1114, 0.1779], [-0.0620, 0.1780], [-0.0140, 0.1840], [0.0350, 0.1900], [0.0835, 0.1960], [0.1320, 0.2020], [0.1810, 0.2080], [0.2290, 0.2130]];
pub const FLOOR_R: f64 = 0.02;
pub const FLOOR_W: f64 = 0.538;
pub const FLOOR_RATIO: f64 = 1.5;
/// js BACK, BACK_R, BACK_W; KICK
pub const BACK: [[f64; 2]; 3] = [[-0.2059, 0.2186], [-0.2510, 0.2816], [-0.2664, 0.3703]];
pub fn back_r() -> f64 { 0.79 * IN }
pub const BACK_W: f64 = 0.51;
pub const KICK_AT: [f64; 2] = [-0.0789, 0.3572];
pub fn kick_r() -> f64 { 1.25 * IN }
pub const KICK_W: f64 = 0.559;
/// js SHOOTER
pub mod shooter {
    pub const DRUM_R: f64 = 0.0508;
    pub const DRUM_W: f64 = 0.533;
    pub const ROLLER_AT: [f64; 2] = [-0.0638, 0.5214];
    pub const ROLLER_R: f64 = 0.0254;
    pub const ROLLER_W: f64 = 0.544;
    pub const HOOD_MIN: f64 = 10.0;
    pub const HOOD_MAX: f64 = 41.2;
    pub const HOOD_CAD: f64 = 10.0;
    pub const DRUM_J: f64 = 4.0e-3;
    pub const FLY_J: f64 = 3.3e-3;
    pub const FLY_PER_DRUM: f64 = 30.0 / 15.0;
    pub const HOOD_J: f64 = 1.0e-4;
    pub const KS: f64 = 0.28;
    pub const KV: f64 = 0.0192;
    pub const KA: f64 = 0.007;
    pub const KP: f64 = 0.6;
    pub const MAX_ACCEL: f64 = 350.0;
    pub const BANG_BANG: f64 = 2.0;
    pub const BANG_MIN: f64 = 3.25;
    pub const PID_OFFSET: f64 = 4.0;
    pub const SUPPLY_TELEOP: f64 = 160.0;
    pub const SUPPLY_AUTO: f64 = 60.0;
    pub const EFF: f64 = 0.8;
    pub const IDLE: f64 = 80.0;
}
/// js KICKER: the kickers' velocity loops (kP, kS, kV)
pub const KICKER_FRONT: [f64; 3] = [1.5, 0.48, 0.056];
pub const KICKER_BACK: [f64; 3] = [3.0, 0.4, 0.0575];

fn interp(t: &[[f64; 2]], x: f64) -> f64 {
    if x <= t[0][0] { return t[0][1]; }
    for k in 1..t.len() {
        if x <= t[k][0] { return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); }
    }
    t[t.len() - 1][1]
}
/// js HUB_MAP, PASS_MAP
const HUB_HOOD: [[f64; 2]; 10] = [[1.12, 10.0], [1.52, 20.0], [2.01, 23.0], [2.73, 26.0], [3.51, 29.0], [3.93, 32.0], [4.40, 34.0], [4.68, 36.0], [4.90, 38.0], [5.01, 38.0]];
const HUB_FLY: [[f64; 2]; 10] = [[1.12, 145.0], [1.52, 145.0], [2.01, 153.0], [2.73, 160.0], [3.51, 175.0], [3.93, 188.0], [4.40, 192.0], [4.68, 195.0], [4.90, 195.0], [5.01, 195.0]];
const HUB_KICKER: [[f64; 2]; 4] = [[0.0, 6.0], [3.0, 6.0], [4.0, 3.0], [5.0, 3.0]];
const HUB_TOF: [[f64; 2]; 5] = [[1.63, 1.017], [2.40, 0.967], [3.25, 1.19], [4.15, 1.18], [4.875, 1.25]];
const PASS_HOOD: [[f64; 2]; 2] = [[5.46, 38.0], [17.16, 38.0]];
const PASS_FLY: [[f64; 2]; 4] = [[5.46, 160.0], [6.62, 180.0], [7.80, 200.0], [17.16, 400.0]];
const PASS_KICKER: [[f64; 2]; 1] = [[0.0, 6.0]];
const PASS_TOF: [[f64; 2]; 6] = [[5.46, 1.27], [6.62, 1.39], [7.8, 1.49], [11.0, 1.75], [13.0, 1.76], [17.16, 2.5]];
/// js RANGE
pub const RANGE_HUB: [f64; 2] = [0.6, 6.0];
pub const RANGE_PASS: [f64; 2] = [3.0, 17.16];
/// js SIM_FLY
const SIM_FLY_HUB: [f64; 10] = [256.0, 165.0, 175.0, 189.0, 199.0, 205.0, 214.0, 220.0, 224.0, 228.0];
const SIM_FLY_PASS: [f64; 4] = [205.0, 243.0, 279.0, 500.0];
/// js shotFor's answer
#[derive(Clone, Copy, Debug)]
pub struct Want { pub fly: f64, pub hood: f64, pub kicker: f64, pub tof: f64 }
/// js `shotFor(kind, d, { theirs })`
pub fn shot_for(pass: bool, d: f64, theirs: bool) -> Want {
    let (fly_t, hood_t, kick_t, tof_t): (&[[f64; 2]], &[[f64; 2]], &[[f64; 2]], &[[f64; 2]]) = if pass { (&PASS_FLY, &PASS_HOOD, &PASS_KICKER, &PASS_TOF) } else { (&HUB_FLY, &HUB_HOOD, &HUB_KICKER, &HUB_TOF) };
    let sim: &[f64] = if pass { &SIM_FLY_PASS } else { &SIM_FLY_HUB };
    let fly = if !theirs { interp(&fly_t.iter().enumerate().map(|(i, r)| [r[0], sim[i]]).collect::<Vec<_>>(), d) } else { interp(fly_t, d) };
    Want { fly, hood: interp(hood_t, d), kicker: interp(kick_t, d), tof: interp(tof_t, d) }
}
/// js PRELOAD
pub const PRELOAD: [[f64; 3]; 8] = [
    [0.19, 0.30, -0.225], [0.19, 0.30, -0.075], [0.19, 0.30, 0.075], [0.19, 0.30, 0.225],
    [0.03, 0.28, -0.225], [0.03, 0.28, -0.075], [0.03, 0.28, 0.075], [0.03, 0.28, 0.225],
];

/// js build6328()'s answer
pub struct Trains {
    pub intake: usize, pub roller: usize, pub locked: usize, pub floor: usize, pub rollers: Vec<usize>,
    pub back_t: usize, pub back: Vec<usize>, pub front_t: usize, pub kick: usize, pub flywheel: usize, pub drum: usize, pub hood: usize, pub pivot: usize,
}
/// js `build6328()`
pub fn build6328(d: &mut Drives) -> Trains {
    let sh = |name: &str, j: f64| Shaft::new(name, None, j, 2e-4, 0.0);
    let x60 = |count: f64, supply: f64| Motor::new("krakenX60", count, 1.0, 0.9, f64::INFINITY, supply);
    let intake = d.train("intake roller", x60(2.0, 70.0));
    let roller = d.add(intake, sh("intake 2in roller", 1.8e-4), -12.0 / 24.0);
    let locked = d.shaft(sh("intake fixed tubes", f64::INFINITY));
    let floor = d.train("Superdexer floor", x60(2.0, 40.0));
    let rollers = (0..FLOOR.len()).map(|i| d.add(floor, sh(&format!("floor roller {}", i + 1), 3e-5), 1.0 / FLOOR_RATIO)).collect();
    let back_t = d.train("back kicker", x60(1.0, 30.0));
    let back = (0..BACK.len()).map(|i| d.add(back_t, sh(&format!("back kicker roller {}", i + 1), 3e-5), 1.0 / 3.0)).collect();
    let front_t = d.train("front kicker", x60(1.0, 30.0));
    let kick = d.add(front_t, sh("front kicker 2.5in roller", 2.5e-4), -1.0 / 3.0);
    let flywheel = d.train("flywheel", x60(4.0, 40.0));
    let drum = d.add(flywheel, sh("drum", shooter::DRUM_J), 1.0);
    d.add(flywheel, Shaft::new("6in steel inertia wheel (30T:15T)", None, shooter::FLY_J, 0.0, 0.0), shooter::FLY_PER_DRUM);
    let hood = d.add(flywheel, sh("hood 2in roller", shooter::HOOD_J), -1.0);
    let pivot = d.shaft(Shaft::new("slam pivot", Some(Motor::new("krakenX60", 1.0, 52.5, 0.9, 70.0, 50.0)), intake::J, 0.5, 0.0));
    Trains { intake, roller, locked, floor, rollers, back_t, back, front_t, kick, flywheel, drum, hood, pivot }
}

/// js wheels6328()'s answer
pub struct Wheels { pub floor: Vec<Wheel>, pub kick: Vec<Wheel>, pub intake: Vec<Wheel>, pub intake_home: Vec<[f64; 3]>, pub drum: Wheel, pub hood: Vec<Wheel>, pub hood_home: Vec<[f64; 3]> }
/// js `wheels6328(t)`
pub fn wheels6328(t: &Trains, d: &Drives) -> Wheels {
    let z = [0.0, 0.0, 1.0];
    let name = |s: usize| d.shafts[s].name.clone();
    let floor = FLOOR.iter().enumerate().map(|(i, [x, y])| Wheel::new(&name(t.rollers[i]), t.rollers[i], [*x, *y, 0.0], z, FLOOR_R, FLOOR_W).mu(mu::SILICONE).tread(SIL_TREAD)).collect();
    let mut kick: Vec<Wheel> = BACK.iter().enumerate().map(|(i, [x, y])| Wheel::new(&name(t.back[i]), t.back[i], [*x, *y, 0.0], z, back_r(), BACK_W).mu(mu::SILICONE).tread(SIL_TREAD)).collect();
    kick.push(Wheel::new(&name(t.kick), t.kick, [KICK_AT[0], KICK_AT[1], 0.0], z, kick_r(), KICK_W).mu(mu::SILICONE).tread(SIL_TREAD));
    let (ro, ba, to) = (intake::ROLLER, intake::BASH, intake::TOE);
    let (ra, bat, tat) = (at_exported(ro.0), at_exported(ba.0), at_exported(to.0));
    let intake = vec![
        Wheel::new("intake 2in roller (silicone)", t.roller, [ra[0], ra[1], 0.0], z, ro.1, ro.2).mu(mu::SILICONE).tread(SIL_TREAD),
        Wheel::new("intake bash bar", t.locked, [bat[0], bat[1], 0.0], z, ba.1, ba.2).mu(mu::STEEL),
        Wheel::new("intake static bottom roller", t.locked, [tat[0], tat[1], 0.0], z, to.1, to.2).mu(mu::STEEL),
    ];
    let intake_home = intake.iter().map(|w| w.center).collect();
    let drum = Wheel::new("drum", t.drum, [DRUM[0], DRUM[1], 0.0], z, shooter::DRUM_R, shooter::DRUM_W).mu(mu::SILICONE).tread(SIL_TREAD);
    let hood = vec![Wheel::new("hood 2in roller", t.hood, [shooter::ROLLER_AT[0], shooter::ROLLER_AT[1], 0.0], z, shooter::ROLLER_R, shooter::ROLLER_W).mu(mu::SILICONE).tread(SIL_TREAD)];
    let hood_home = hood.iter().map(|w| w.center).collect();
    Wheels { floor, kick, intake, intake_home, drum, hood, hood_home }
}
/// js `rot(p, c, a)`
fn rot([x, y]: [f64; 2], [cx, cy]: [f64; 2], a: f64) -> [f64; 2] {
    let (c, s, dx, dy) = (m::cos(a), m::sin(a), x - cx, y - cy);
    [cx + dx * c - dy * s, cy + dx * s + dy * c]
}
/// js `intakeRot(deg)`
pub fn intake_rot(deg: f64) -> f64 { (deg - intake::CAD) * DEG }
/// js `placeIntake(wheels, deg)`
pub fn place_intake(wheels: &mut [Wheel], home: &[[f64; 3]], deg: f64) {
    let a = intake_rot(deg);
    for (w, h) in wheels.iter_mut().zip(home) { let [x, y] = rot([h[0], h[1]], PIVOT, a); w.center = [x, y, 0.0]; }
}
/// js `underRamp(x, y, deg)`: a FUEL caught under the ramp plate with the arm at deg
pub fn under_ramp(x: f64, y: f64, deg: f64) -> bool {
    if deg > 60.0 { return false; }
    let a = intake_rot(deg);
    let [tx, ty] = rot(at_exported(intake::TOE.0), PIVOT, a);
    let [ux, uy] = rot(at_exported(intake::RAMP_TOP), PIVOT, a);
    if x < HALF_L || y > uy { return false; }
    (ux - tx) * (y - ty) - (uy - ty) * (x - tx) > 0.0
}
/// js `hoodRot(deg)`, `placeHood(wheels, deg)`, `hoodElevation(deg)`
pub fn hood_rot(deg: f64) -> f64 { (deg - shooter::HOOD_CAD) * DEG }
pub fn place_hood(wheels: &mut [Wheel], home: &[[f64; 3]], deg: f64) {
    let a = hood_rot(deg);
    for (w, h) in wheels.iter_mut().zip(home) { let [x, y] = rot([h[0], h[1]], DRUM, a); w.center = [x, y, 0.0]; }
}
pub fn hood_elevation(deg: f64) -> f64 {
    let [hx, hy] = shooter::ROLLER_AT;
    let a = m::atan2(hy - DRUM[1], hx - DRUM[0]) / DEG + (deg - shooter::HOOD_CAD);
    90.0 - a + 0.0
}

/// js WALLS
pub mod walls {
    pub const X0: f64 = -0.035;
    pub const X1: f64 = 0.35;
    pub const Y0: f64 = 0.2;
    pub const Y1: f64 = 0.556;
    pub const HW: f64 = 0.334;
}
/// js NET
pub fn net() -> Bungee { Bungee::new(0.0, 0.555, 0.346, 0.51, 0.334, 0.22, 250.0, [1.0, 0.5], 0.7) }
pub const NET_TREAD: f64 = 8000.0;
/// js HALF_L, HALF_W
pub const HALF_L: f64 = 26.5 * IN / 2.0;
pub const HALF_W: f64 = 27.5 * IN / 2.0;

#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
struct Path { body: PathMesh, intake: PathMesh, slide: PathMesh, hood: PathMesh }

/// pathSurfaces6328's surfaces, in its order
pub const S_BODY: usize = 0;
pub const S_INTAKE: usize = 1;
pub const S_RAMP: usize = 2;
pub const S_SLIDE: usize = 3;
pub const S_HOOD: usize = 4;
pub const S_WALLS: usize = 5;
pub const S_NET: usize = 6;
pub const S_BUMPER: usize = 7;

/// js `pathSurfaces6328(mesh, t)`
pub fn path_surfaces6328(t: &Trains) -> Vec<Surface> {
    static P: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    let p = P.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path6328.json")).expect("path6328.json"));
    let s = |k: &str, mesh: &PathMesh, pos: [f64; 3]| {
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{k} (CAD)"), &v, &mesh.i, 0.35, pos, 0.0, 0.0, 0.08, f64::INFINITY)
    };
    let piv = [PIVOT[0], PIVOT[1], 0.0];
    let z = [0.0, 0.0, 1.0];
    let [tx, ty] = at_exported(intake::TOE.0);
    let [ux, uy] = at_exported(intake::RAMP_TOP);
    let (hw, p0) = (intake::RAMP_HW, PIVOT);
    let rv = [tx - p0[0], ty - p0[1], -hw, tx - p0[0], ty - p0[1], hw, ux - p0[0], uy - p0[1], hw, ux - p0[0], uy - p0[1], -hw];
    let ramp = Surface::mesh("intake ramp plate", &rv, &[0, 1, 2, 0, 2, 3], 0.35, piv, 0.0, 0.0, 0.08, f64::INFINITY).on_shaft(t.pivot, piv, z);
    let mut wv = Vec::new();
    for zz in [-walls::HW, walls::HW] { wv.extend_from_slice(&[walls::X0, walls::Y0, zz, walls::X1, walls::Y0, zz, walls::X1, walls::Y1, zz, walls::X0, walls::Y1, zz]); }
    let wl = Surface::mesh("static walls (polycarbonate)", &wv, &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7], mu::POLYCARB, [0.0; 3], 0.0, 0.0, 0.08, f64::INFINITY);
    vec![
        s("body", &p.body, [0.0; 3]),
        s("intake", &p.intake, piv).on_shaft(t.pivot, piv, z),
        ramp,
        s("slide", &p.slide, [0.0; 3]).on_slide(t.pivot, [1.0, 0.0, 0.0]),
        s("hood", &p.hood, [DRUM[0], DRUM[1], 0.0]),
        wl,
        Surface::bungee("net", mu::NET, NET_TREAD, net()),
        Surface::block("bumper", HALF_L + 0.75 * IN, HALF_L + 3.25 * IN, 0.032, 0.16, HALF_W + 3.25 * IN, 0.02, BUMPER_MU, 20000.0),
    ]
}
