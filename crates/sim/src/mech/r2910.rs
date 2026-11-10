//! 2910's Re•Blitz as physical mechanisms (js/mech/r2910.js), from their CAD and their code: the
//! layout, the drives, the wheels, the CAD's surfaces, how the intake and hood move them, and their
//! shot tables. Robot frame: x forward (the intake), y up, z to its right. (The why of each number
//! is in the JS file; this is the same numbers, operation for operation.)
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{Surface, Wheel, BUMPER_MU};
use jsmath as m;

/// js MU: what each grips FUEL with
pub mod mu { pub const COMPLIANT: f64 = 1.6; pub const GRIP_TAPE: f64 = 1.5; pub const SILICONE: f64 = 1.0; pub const ALUMINUM: f64 = 0.6; }

/// js PIVOT, DRUM
pub const PIVOT: [f64; 2] = [0.273, 0.17];
pub const DRUM: [f64; 2] = [-0.283, 0.47];

/// js INTAKE
pub mod intake {
    pub const MAIN_AT: [f64; 2] = [0.610, 0.161];
    pub const MAIN_D: f64 = 0.054;
    pub const LOWER: [([f64; 2], f64); 2] = [([0.448, 0.103], 0.0349), ([0.483, 0.042], 0.0349)];
    pub const HALF_WIDTH: f64 = 0.29;
    pub const DEPLOY: f64 = 114.3;
    pub const RETRACT: f64 = 10.0;
    pub const CAD_DEG: f64 = -1.5;
    pub const MASS: f64 = 8.1;
    pub const COM: f64 = 0.24;
}
pub use intake::{DEPLOY, RETRACT};

#[derive(Clone, Copy, PartialEq)]
pub enum Grip { Silicone, Aluminum }
/// js FLOOR: [x, y, diameter, grip]
pub const FLOOR: [(f64, f64, f64, Grip); 10] = [
    (0.256, 0.195, 0.0349, Grip::Silicone), (0.194, 0.179, 0.0349, Grip::Silicone), (0.132, 0.162, 0.0349, Grip::Silicone),
    (0.071, 0.146, 0.0349, Grip::Silicone), (0.009, 0.129, 0.0349, Grip::Silicone),
    (-0.053, 0.113, 0.0349, Grip::Aluminum), (-0.161, 0.113, 0.0349, Grip::Aluminum), (-0.251, 0.174, 0.0349, Grip::Aluminum),
    (-0.289, 0.261, 0.0349, Grip::Silicone), (-0.278, 0.356, 0.0349, Grip::Silicone),
];
fn grip_mu(g: Grip) -> f64 { match g { Grip::Silicone => mu::SILICONE, Grip::Aluminum => mu::ALUMINUM } }

/// js INDEXER
pub const INDEXER_AT: [f64; 2] = [-0.107, 0.289];
pub fn indexer_d() -> f64 { 2.25 * IN }
pub const INDEXER_Z: [f64; 9] = [-0.254, -0.191, -0.127, -0.064, 0.0, 0.063, 0.127, 0.19, 0.254];

/// js SHOOTER
pub mod shooter {
    pub const DRUM_D: f64 = 0.1048;
    pub const DRUM_WIDTH: f64 = 0.508;
    pub const HOOD: [([f64; 2], f64); 2] = [([-0.081, 0.461], 0.0413), ([-0.084, 0.508], 0.0413)];
    pub const HOOD_WIDTH: f64 = 0.603;
    pub const HOOD_MAX: f64 = 37.0;
    pub fn drum_per_motor() -> f64 { 24.0 / 38.0 }
    pub fn flywheel_per_drum() -> f64 { 67.0 / 10.0 }
    pub fn hood_roller_per_drum() -> f64 { 38.0 / 21.0 }
    pub const DRUM_J: f64 = 3.99e-3;
    pub const FLYWHEEL_J: f64 = 2.48e-3;
    pub const HOOD_J: f64 = 3.43e-4;
    pub const MOTOR_J: f64 = 2.3e-5;
    pub fn kv() -> f64 { 12.0 / (5800.0 / 60.0) }
    pub const KP: f64 = 0.35;
}

/// js SHOT_TABLE: distance to the HUB's center (m) -> motor RPM and hood angle
pub const SHOT_TABLE: [[f64; 3]; 5] = [[1.3, 2100.0, 0.0], [2.0, 2250.0, 8.0], [3.0, 2400.0, 17.0], [4.0, 2600.0, 23.0], [5.0, 2900.0, 28.0]];
/// js SIM_PASS_TABLE (PASS_TABLE without THEIR_PASS)
pub const PASS_TABLE: [[f64; 3]; 3] = [[2.0, 1230.0, 0.0], [6.0, 2640.0, 0.0], [10.0, 3625.0, 0.0]];
/// js PASS_TOF
pub const PASS_TOF: [[f64; 3]; 3] = [[0.5, 0.5, 0.0], [6.0, 0.83, 0.0], [10.0, 1.1, 0.0]];

/// js `interp(t, x, j)`
fn interp(t: &[[f64; 3]], x: f64, j: usize) -> f64 {
    if x <= t[0][0] { return t[0][j]; }
    for k in 1..t.len() {
        if x <= t[k][0] { return t[k - 1][j] + ((t[k][j] - t[k - 1][j]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); }
    }
    t[t.len() - 1][j]
}
/// js `shotFor(d)`: (rpm, hood)
pub fn shot_for(d: f64) -> (f64, f64) { (interp(&SHOT_TABLE, d, 1), interp(&SHOT_TABLE, d, 2)) }
/// js `passFor(d)`
pub fn pass_for(d: f64) -> (f64, f64) { (interp(&PASS_TABLE, d, 1), shooter::HOOD_MAX) }
/// js `passTof(d)`
pub fn pass_tof(d: f64) -> f64 { interp(&PASS_TOF, d, 1) }

/// js RANGES: their CANranges (at, z), tripped within reach
pub const RANGE_HOPPER: ([f64; 2], f64) = ([-0.012, 0.399], -0.327);
pub const RANGE_FEEDER: ([f64; 2], f64) = ([-0.229, 0.380], -0.327);
pub const RANGE_REACH: f64 = 0.57;

/// js PRELOAD (robot frame)
pub const PRELOAD: [[f64; 3]; 8] = [
    [-0.17, 0.205, -0.19], [-0.17, 0.205, -0.063], [-0.17, 0.205, 0.063], [-0.17, 0.205, 0.19],
    [-0.02, 0.225, -0.19], [-0.02, 0.225, -0.063], [-0.02, 0.225, 0.063], [-0.02, 0.225, 0.19],
];

/// js INTAKE_LIMITS: (statorLimit, supplyLimit)
pub const INTAKE_LIMITS_AUTO: (f64, f64) = (120.0, 100.0);
pub const INTAKE_LIMITS_TELEOP: (f64, f64) = (100.0, 40.0);

/// js build2910's result: the drives (trains and shafts) by name
pub struct Trains {
    pub intake: usize, pub main: usize, pub lower: [usize; 2],
    pub floor: usize, pub rollers: Vec<usize>, pub indexer: usize,
    pub shooter: usize, pub drum: usize, pub hood: usize,
    pub pivot: usize,
}

/// js `intakeLimits(t, teleop)`
pub fn intake_limits(d: &mut Drives, t: &Trains, teleop: bool) {
    let (s, u) = if teleop { INTAKE_LIMITS_TELEOP } else { INTAKE_LIMITS_AUTO };
    let mo = &mut d.trains[t.intake].motor;
    mo.stator_limit = s;
    mo.supply_limit = u;
}

/// js `build2910()`
pub fn build2910(d: &mut Drives) -> Trains {
    let sh = |name: &str, inertia: f64| Shaft::new(name, None, inertia, 2e-4, 0.0);
    let intake = d.train("intake rollers", Motor::new("krakenX60", 2.0, 1.0, 0.9, INTAKE_LIMITS_AUTO.0, INTAKE_LIMITS_AUTO.1));
    let main = d.add(intake, sh("intake 2in roller", 3e-4), -11.0 / 30.0);
    let lower = [0, 1].map(|i| d.add(intake, sh(&format!("intake 1in roller {}", i + 1), 5e-5), (24.0 / 48.0) * (12.0 / 12.0)));
    let floor = d.train("hopper", Motor::new("krakenX60", 4.0, 1.0, 0.9, 80.0, 40.0));
    let rollers = (0..FLOOR.len()).map(|i| d.add(floor, sh(&format!("hopper roller {}", i + 1), 5e-5), 1.0)).collect();
    let indexer = d.add(floor, sh("indexer compliant wheels", 2e-4), -16.0 / 28.0);
    let shooter_t = d.train("shooter", Motor::new("krakenX60", 4.0, 1.0, 0.9, 60.0, 35.0));
    let drum = d.add(shooter_t, sh("drum", shooter::DRUM_J), shooter::drum_per_motor());
    d.add(shooter_t, sh("brass flywheel", shooter::FLYWHEEL_J), shooter::drum_per_motor() * shooter::flywheel_per_drum());
    let hood = d.add(shooter_t, sh("hood rollers", shooter::HOOD_J), -shooter::drum_per_motor() * shooter::hood_roller_per_drum());
    d.add(shooter_t, Shaft::new("motors' 24t pulleys", None, shooter::MOTOR_J, 0.0, 0.0), 1.0);
    let pivot = d.shaft(Shaft::new("intake pivot", Some(Motor::new("krakenX60", 1.0, 40.0, 0.9, 75.0, 30.0)), 2.0 * intake::MASS * intake::COM * intake::COM, 0.5, 0.0));
    Trains { intake, main, lower, floor, rollers, indexer, shooter: shooter_t, drum, hood, pivot }
}

/// js wheels2910's result
pub struct Wheels { pub floor: Vec<Wheel>, pub indexer: Vec<Wheel>, pub intake: Vec<Wheel>, pub drum: Wheel, pub hood: Vec<Wheel> }

/// js `wheels2910(t)`
pub fn wheels2910(d: &Drives, t: &Trains) -> Wheels {
    let z = [0.0, 0.0, 1.0];
    let floor = FLOOR.iter().enumerate().map(|(i, &(x, y, dd, g))| Wheel::new(&d.shafts[t.rollers[i]].name, t.rollers[i], [x, y, 0.0], z, dd / 2.0, 0.559).mu(grip_mu(g)).tread(20000.0)).collect();
    let indexer = INDEXER_Z.iter().map(|&zz| Wheel::new("indexer 2.25in 35A compliant", t.indexer, [INDEXER_AT[0], INDEXER_AT[1], zz], z, indexer_d() / 2.0, 1.0 * IN).mu(mu::COMPLIANT).tread(3000.0)).collect();
    let mut intake = vec![Wheel::new("intake 2in roller", t.main, [intake::MAIN_AT[0], intake::MAIN_AT[1], 0.0], z, intake::MAIN_D / 2.0, 0.575).mu(mu::GRIP_TAPE).tread(40000.0)];
    for (i, (at, dd)) in intake::LOWER.iter().enumerate() {
        intake.push(Wheel::new(&d.shafts[t.lower[i]].name, t.lower[i], [at[0], at[1], 0.0], z, dd / 2.0, 0.586).mu(mu::SILICONE).tread(20000.0));
    }
    let drum = Wheel::new("drum (grip tape)", t.drum, [DRUM[0], DRUM[1], 0.0], z, shooter::DRUM_D / 2.0, shooter::DRUM_WIDTH).mu(mu::GRIP_TAPE).tread(40000.0);
    let hood = shooter::HOOD.iter().enumerate().map(|(i, (at, dd))| Wheel::new(&format!("hood roller {} (grip tape)", i + 1), t.hood, [at[0], at[1], 0.0], z, dd / 2.0, shooter::HOOD_WIDTH).mu(mu::GRIP_TAPE).tread(40000.0)).collect();
    Wheels { floor, indexer, intake, drum, hood }
}

/// js `bumper2910()`: the front bumper, a solid block of foam
pub fn bumper2910() -> Surface {
    Surface::block("bumper", 13.5 * IN + 0.75 * IN, 13.5 * IN + BUMPER_T, BUMPER_Y0, BUMPER_Y1, 13.75 * IN + BUMPER_T, 0.02, BUMPER_MU, f64::INFINITY)
}

/// js `rot([x, y], [cx, cy], a)`
fn rot(p: [f64; 2], c: [f64; 2], a: f64) -> [f64; 2] {
    let (co, s) = (m::cos(a), m::sin(a));
    let (dx, dy) = (p[0] - c[0], p[1] - c[1]);
    [c[0] + dx * co - dy * s, c[1] + dx * s + dy * co]
}
/// js `intakeRot(deg)`
pub fn intake_rot(deg: f64) -> f64 { ((DEPLOY - deg) * m::PI) / 180.0 }
/// js `placeIntake(wheels, deg)`, on wheels whose homes (w.home) are `homes`
pub fn place_intake(wheels: &mut [Wheel], homes: &[[f64; 2]], deg: f64) {
    let a = intake_rot(deg);
    for (w, h) in wheels.iter_mut().zip(homes) { let [x, y] = rot(*h, PIVOT, a); w.center = [x, y, 0.0]; }
}
/// js `placeHood(wheels, deg)`
pub fn place_hood(wheels: &mut [Wheel], homes: &[[f64; 2]], deg: f64) {
    let a = (deg * m::PI) / 180.0;
    for (w, h) in wheels.iter_mut().zip(homes) { let [x, y] = rot(*h, DRUM, a); w.center = [x, y, 0.0]; }
}
/// js `hoodElevation(deg)`
pub fn hood_elevation(deg: f64) -> f64 {
    let [hx, hy] = shooter::HOOD[1].0;
    90.0 - (m::atan2(hy - DRUM[1], hx - DRUM[0]) * 180.0) / m::PI - deg
}

/// js path2910.js: the ball path's surfaces from their CAD (mm), as assets/mech/path2910.json has them
#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
struct Path { body: PathMesh, intake: PathMesh, hood: PathMesh }

/// js `pathSurfaces2910(mesh, t)`: [body, intake (on the pivot's shaft), hood, bumper], in that order
pub fn path_surfaces2910(t: &Trains) -> Vec<Surface> {
    static PATH: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    let p = PATH.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path2910.json")).expect("path2910.json"));
    let mu = 0.25;
    let s = |name: &str, mesh: &PathMesh, pos: [f64; 3]| {
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{name} (CAD)"), &v, &mesh.i, mu, pos, 0.0, 0.0, 0.08, f64::INFINITY)
    };
    let body = s("body", &p.body, [0.0, 0.0, 0.0]);
    let intake = s("intake", &p.intake, [PIVOT[0], PIVOT[1], 0.0]).on_shaft(t.pivot, [PIVOT[0], PIVOT[1], 0.0], [0.0, 0.0, 1.0]);
    let hood = s("hood", &p.hood, [DRUM[0], DRUM[1], 0.0]);
    vec![body, intake, hood, bumper2910()]
}
