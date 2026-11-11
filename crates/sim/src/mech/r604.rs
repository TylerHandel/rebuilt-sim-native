//! js/mech/r604.js: 604's Toploader as physical mechanisms, from their CAD and their code
//! (frc604/2026-public). Robot frame: x forward (the intake), y up, z to its right.
//!
//! Single file from a full-width intake: the intake's two rollers bring FUEL up over the bumper into
//! the hopper box (it slides out as the arm comes down) over the Dynamic Point of Capture
//! serializer: a rotor whose hook sweeps the bucket, steering FUEL to the capture point, where the
//! infeed's omnis push them up its ramp under the up-feed roller into the tube, each new one pushing
//! the column up into the turret on the same axis; there the main roller flings them out between
//! itself and the hood's roller.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{Kind, Surface, Wheel, BUMPER_MU};
use jsmath as m;

/// js MU (estimates)
pub mod mu {
    pub const SILICONE: f64 = 1.6;
    pub const COMPLIANT: f64 = 1.6;
    pub const OMNI: f64 = 0.6;
    pub const PRINTED: f64 = 0.8;
}
/// js HALF: the frame's half length and width
pub fn half() -> f64 { 13.5 * IN }
/// js AXIS: the rotor's and the turret's axis; PIVOT: the intake arm's
pub const AXIS: [f64; 2] = [-0.0254, 0.0];
pub const PIVOT: [f64; 2] = [0.3048, 0.2064];
/// js INTAKE
pub mod intake {
    pub const ROLLERS: [[f64; 2]; 2] = [[0.5826, 0.1524], [0.5553, 0.2381]];
    pub const R: f64 = 0.027;
    pub const WIDTH: f64 = 0.533;
    pub const DEPLOY: f64 = -5.0;
    pub const STANDBY: f64 = 10.0;
    pub const STOW: f64 = 80.0;
    pub const START: f64 = 98.0;
    pub const MIN: f64 = -5.0;
    pub const MAX: f64 = 98.0;
    pub const MM_V: f64 = 4.0;
    pub const MM_SLOW: f64 = 2.0;
    pub const MM_A: f64 = 45.0;
    pub const KP: f64 = 1.0;
    pub const KV: f64 = 0.12;
    pub const TOL: f64 = 30.0;
    pub const MASS: f64 = 2.0;
    pub const COM: f64 = 0.2;
    pub const J: f64 = 0.118;
}
/// js HOPPER.out
pub const HOPPER_OUT: f64 = 0.2921;
/// js HOPPER.ext
pub const HOPPER_EXT: [f64; 104] = [0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.292, 0.29, 0.289, 0.288, 0.286, 0.285, 0.283, 0.282, 0.28, 0.278, 0.276, 0.274, 0.272, 0.27, 0.268, 0.266, 0.263, 0.261, 0.258, 0.256, 0.253, 0.25, 0.248, 0.245, 0.242, 0.239, 0.236, 0.233, 0.229, 0.226, 0.223, 0.219, 0.216, 0.212, 0.209, 0.205, 0.202, 0.198, 0.197, 0.196, 0.195, 0.194, 0.193, 0.192, 0.19, 0.189, 0.188, 0.186, 0.185, 0.183, 0.182, 0.18, 0.178, 0.177, 0.175, 0.173, 0.171, 0.169, 0.167, 0.165, 0.163, 0.161, 0.159, 0.156, 0.154, 0.152, 0.149, 0.147, 0.144, 0.142, 0.139, 0.137, 0.134, 0.131, 0.129];
/// js LAUNCH
pub const LAUNCH: [[f64; 3]; 70] = [[1.1, 5.647, 75.0], [1.2, 5.604, 74.65], [1.3, 5.57, 73.676], [1.4, 5.591, 71.896], [1.5, 5.634, 70.297], [1.6, 5.671, 69.58], [1.7, 5.728, 68.361], [1.8, 5.79, 67.543], [1.9, 5.894, 65.093], [2.0, 5.958, 64.39], [2.1, 6.035, 63.812], [2.2, 6.112, 63.277], [2.3, 6.17, 62.604], [2.4, 6.228, 61.951], [2.5, 6.291, 61.358], [2.6, 6.367, 60.949], [2.7, 6.445, 60.58], [2.8, 6.513, 60.115], [2.9, 6.62, 60.1], [3.0, 6.688, 60.0], [3.1, 6.732, 60.306], [3.2, 6.788, 60.435], [3.3, 6.855, 60.21], [3.4, 6.942, 60.107], [3.5, 7.034, 60.05], [3.6, 7.093, 60.0], [3.7, 7.165, 60.0], [3.8, 7.233, 60.0], [3.9, 7.289, 60.02], [4.0, 7.361, 60.02], [4.1, 7.446, 60.0], [4.2, 7.5, 60.0], [4.3, 7.572, 60.0], [4.4, 7.636, 60.0], [4.5, 7.699, 60.0], [4.6, 7.763, 60.0], [4.7, 7.833, 60.0], [4.8, 7.882, 60.0], [4.9, 7.957, 60.0], [5.0, 8.034, 60.02], [5.1, 8.098, 60.0], [5.2, 8.136, 60.0], [5.3, 8.246, 60.0], [5.4, 8.279, 60.0], [5.5, 8.359, 60.0], [5.6, 8.428, 60.0], [5.7, 8.5, 60.0], [5.8, 8.573, 60.0], [5.9, 8.642, 60.0], [6.0, 8.706, 60.0], [6.1, 8.726, 60.0], [6.2, 8.775, 60.0], [6.3, 8.841, 60.0], [6.4, 8.9, 60.0], [6.5, 8.96, 60.0], [6.6, 9.036, 60.0], [6.7, 9.116, 60.05], [6.8, 9.165, 60.0], [6.9, 9.226, 60.02], [7.0, 9.295, 60.02], [7.1, 9.336, 60.01], [7.2, 9.39, 60.0], [7.3, 9.45, 60.0], [7.4, 9.504, 60.0], [7.5, 9.573, 60.02], [7.6, 9.627, 60.0], [7.7, 9.68, 60.0], [7.8, 9.763, 60.0], [7.9, 9.82, 60.03], [8.0, 9.873, 60.02]];
/// js ROTOR: the infeed's omnis and the up-feed roller (as exported, rotor at 0)
pub mod rotor {
    pub const OMNI_AT: [f64; 2] = [-0.1150, -0.0913];
    pub const OMNI_YS: [f64; 3] = [0.2051, 0.2326, 0.2601];
    pub const OMNI_R: f64 = 0.025;
    pub const OMNI_W: f64 = 0.0275;
    pub const UP_AT: [f64; 2] = [-0.1159, 0.3159];
    pub const FLEX: [f64; 3] = [0.004, 0.0255, 0.0257];
    pub const PRINTED: [[f64; 2]; 2] = [[-0.0341, 0.051], [0.0231, 0.0129]];
    pub const PR: f64 = 0.0206;
    pub const J: f64 = 0.03;
    pub fn eff_r() -> f64 { 7.8 * crate::constants::IN }
}
/// js SHOOTER
pub mod shooter {
    pub const MAIN_AT: [f64; 2] = [0.0744, 0.5969];
    pub const MAIN_R: f64 = 0.027;
    pub const MAIN_W: f64 = 0.114;
    pub const HOOD_AT: [f64; 2] = [-0.0686, 0.6971];
    pub const HOOD_R: f64 = 0.01745;
    pub const HOOD_W: f64 = 0.115;
    pub const HOOD_PER_MAIN: f64 = 28.0 / 20.0;
    pub const J: f64 = 2.456e-3;
    pub const KP: f64 = 0.2;
    pub const KV: f64 = 0.12;
    pub const HOOD_MIN: f64 = 15.0;
    pub const HOOD_MAX: f64 = 35.0;
    pub const HOOD_CAD: f64 = 35.0;
    pub const HOOD_V: f64 = 2.0;
    pub const HOOD_A: f64 = 20.0;
    pub const TOL_V: f64 = 0.8;
    pub const TOL_ANGLE: f64 = 3.0;
    pub const TOL_YAW: f64 = 5.0;
    pub const FEED_TOL_V: f64 = 2.0;
    pub const FEED_TOL_ANGLE: f64 = 15.0;
    pub const FEED_TOL_YAW: f64 = 40.0;
    pub const YAW_START: f64 = 180.0;
    pub const YAW_V: f64 = 10.0;
}
/// js VELOCITY_TO_RAD, VELOCITY_MULTIPLIER, MAX_RAD, FEED
pub const VELOCITY_TO_RAD: [[f64; 2]; 6] = [[0.0, 0.0], [5.54, 220.0], [7.88, 360.0], [9.0, 480.0], [10.0, 510.0], [12.0, 600.0]];
pub const VELOCITY_MULTIPLIER: f64 = 1.0;
pub const MAX_RAD: f64 = 600.0;
pub const FEED_ANGLE: f64 = 55.0;
pub fn feed_height() -> f64 { 27.5 * IN }
fn interp3(t: &[[f64; 3]], x: f64, j: usize) -> f64 {
    if x <= t[0][0] { return t[0][j]; }
    for k in 1..t.len() {
        if x <= t[k][0] { return t[k - 1][j] + ((t[k][j] - t[k - 1][j]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); }
    }
    t[t.len() - 1][j]
}
fn interp(t: &[[f64; 2]], x: f64) -> f64 {
    if x <= t[0][0] { return t[0][1]; }
    for k in 1..t.len() {
        if x <= t[k][0] { return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); }
    }
    t[t.len() - 1][1]
}
/// js `launchFor(d)`: (v, angle)
pub fn launch_for(d: f64) -> (f64, f64) { (interp3(&LAUNCH, d, 1), interp3(&LAUNCH, d, 2)) }
/// js `feedSpeed(d)`
pub fn feed_speed(d: f64) -> f64 {
    let th = (FEED_ANGLE * m::PI) / 180.0;
    let den = 2.0 * m::pow(m::cos(th), 2.0) * (d * m::tan(th) + feed_height());
    if den > 0.0 { m::sqrt((9.81 * d * d) / den) } else { 0.0 }
}
/// js `radFor(v)`, `velocityAt(rad)`
pub fn rad_for(v: f64) -> f64 { interp(&VELOCITY_TO_RAD, v) * VELOCITY_MULTIPLIER }
pub fn velocity_at(rad: f64) -> f64 { interp(&VELOCITY_TO_RAD.map(|[a, b]| [b, a]), rad) }
/// js SERIALIZER
pub mod serializer {
    pub fn roller_r() -> f64 { 0.5 * 1.625 * crate::constants::IN }
    pub const BALL_V: f64 = 3.6;
    pub const UNJAM_V: f64 = -0.5;
    pub const ROTOR_PER_ROLLER: f64 = 1.2;
    pub const KP: f64 = 0.1;
    pub const KV: f64 = 0.125;
}
/// js `rollerRad(v)`
pub fn roller_rad(v: f64) -> f64 { v / (0.5 * serializer::roller_r()) }
/// js PRELOAD
pub const PRELOAD: [[f64; 3]; 8] = [
    [0.166, 0.23, 0.059], [0.068, 0.23, 0.177], [-0.085, 0.23, 0.191], [-0.099, 0.23, -0.197], [0.054, 0.23, -0.191],
    [0.16, 0.275, -0.09], [0.18, 0.335, 0.17], [-0.02, 0.355, -0.24],
];

/// js build604()'s answer (and the hopper's cam: a shaft geared off the arm's pivot)
pub struct Trains { pub intake: usize, pub front: usize, pub back: usize, pub pivot: usize, pub rotor: usize, pub roller: usize, pub infeed: usize, pub upfeed: usize, pub launcher: usize, pub main: usize, pub hood_roller: usize, pub cam: usize }
/// js `build604()`
pub fn build604(d: &mut Drives) -> Trains {
    let sh = |name: &str, j: f64| Shaft::new(name, None, j, 2e-4, 0.0);
    let intake = d.train("intake rollers", Motor::new("krakenX60", 2.0, 1.0, 0.9, 80.0, 40.0));
    let k = (12.0 / 24.0) * (24.0 / 24.0);
    let front = d.add(intake, sh("intake front roller", 1.1e-4), -k);
    let back = d.add(intake, sh("intake second roller", 1.1e-4), -k);
    let pivot = d.shaft(Shaft::new("intake arm", Some(Motor::new("krakenX44", 2.0, (38.0 / 8.0) * (38.0 / 18.0) * (36.0 / 12.0), 0.9, 30.0, 20.0)), intake::J, 0.3, 0.0));
    let rotor = d.shaft(Shaft::new("DPC rotor", Some(Motor::new("krakenX60", 1.0, 212.0 / 10.0, 0.9, 80.0, 40.0)), rotor::J, 0.02, 0.0));
    let roller = d.train("infeed and up-feed", Motor::new("krakenX60", 1.0, 1.0, 0.9, 100.0, 60.0));
    let infeed = d.add(roller, sh("infeed omnis", 3e-5), 10.0 / 16.0);
    let upfeed = d.add(roller, sh("up-feed roller", 2e-5), 10.0 / 16.0);
    let launcher = d.train("launcher", Motor::new("krakenX60", 2.0, 15.0 / 18.0, 0.9, 80.0, 40.0));
    let main = d.add(launcher, sh("main roller and brass flywheels", shooter::J), -1.0);
    let hood_roller = d.add(launcher, sh("hood roller", 2.17e-5), shooter::HOOD_PER_MAIN);
    let mut c = Shaft::new("hopper cam", None, f64::INFINITY, 0.0, 0.0);
    c.cam = Some((pivot, 0.0));
    let cam = d.shaft(c);
    Trains { intake, front, back, pivot, rotor, roller, infeed, upfeed, launcher, main, hood_roller, cam }
}

/// js wheels604()'s answer
pub struct Wheels { pub intake: Vec<Wheel>, pub intake_home: Vec<[f64; 3]>, pub rotor: Vec<Wheel>, pub rotor_home: Vec<([f64; 3], [f64; 3])> }
/// js `wheels604(t)`
pub fn wheels604(t: &Trains) -> Wheels {
    let (z, y) = ([0.0, 0.0, 1.0], [0.0, 1.0, 0.0]);
    let piv = [PIVOT[0], PIVOT[1], 0.0];
    let intake: Vec<Wheel> = intake::ROLLERS.iter().enumerate().map(|(i, [x, yy])| {
        Wheel::new(if i > 0 { "intake second roller (silicone)" } else { "intake front roller (silicone)" }, if i > 0 { t.back } else { t.front }, [*x, *yy, 0.0], z, intake::R, intake::WIDTH)
            .mu(mu::SILICONE).tread(20000.0).on_pivot(t.pivot, piv, z)
    }).collect();
    let intake_home = intake.iter().map(|w| w.center).collect();
    let ax = [AXIS[0], 0.0, AXIS[1]];
    let mut rot: Vec<Wheel> = rotor::OMNI_YS.iter().enumerate().map(|(i, yy)| Wheel::new(&format!("infeed 2in omni {}", i + 1), t.infeed, [rotor::OMNI_AT[0], *yy, rotor::OMNI_AT[1]], y, rotor::OMNI_R, rotor::OMNI_W).kind(Kind::Omni).mu(mu::OMNI).tread(30000.0)).collect();
    let [fz, fr, fw] = rotor::FLEX;
    rot.push(Wheel::new("up-feed 2in flex wheel", t.upfeed, [rotor::UP_AT[0], rotor::UP_AT[1], fz], z, fr, fw).mu(mu::COMPLIANT).tread(4000.0));
    for [pz, pw] in rotor::PRINTED { rot.push(Wheel::new("up-feed 1.625in printed roller", t.upfeed, [rotor::UP_AT[0], rotor::UP_AT[1], pz], z, rotor::PR, pw).mu(mu::PRINTED).tread(20000.0)); }
    let rot: Vec<Wheel> = rot.into_iter().map(|w| w.on_pivot(t.rotor, ax, [0.0, -1.0, 0.0])).collect();
    let rotor_home = rot.iter().map(|w| (w.center, w.axis)).collect();
    Wheels { intake, intake_home, rotor: rot, rotor_home }
}
/// js `shooterWheels(t)`
pub fn shooter_wheels(t: &Trains) -> Vec<Wheel> {
    let z = [0.0, 0.0, 1.0];
    let mut w = vec![
        Wheel::new("main roller (silicone)", t.main, [0.0; 3], z, shooter::MAIN_R, shooter::MAIN_W).mu(mu::SILICONE).tread(20000.0),
        Wheel::new("hood roller (silicone)", t.hood_roller, [0.0; 3], z, shooter::HOOD_R, shooter::HOOD_W).mu(mu::SILICONE).tread(20000.0),
    ];
    place_shooter(&mut w, 0.0, shooter::HOOD_CAD);
    w
}
/// js `rot(p, c, a)`
fn rot([x, y]: [f64; 2], [cx, cy]: [f64; 2], a: f64) -> [f64; 2] {
    let (c, s, dx, dy) = (m::cos(a), m::sin(a), x - cx, y - cy);
    [cx + dx * c - dy * s, cy + dx * s + dy * c]
}
/// js `armRot(deg)`
pub fn arm_rot(deg: f64) -> f64 { (deg * m::PI) / 180.0 }
/// js `placeIntake(wheels, deg)`
pub fn place_intake(wheels: &mut [Wheel], home: &[[f64; 3]], deg: f64) {
    let a = arm_rot(deg);
    for (w, h) in wheels.iter_mut().zip(home) { let [x, y] = rot([h[0], h[1]], PIVOT, a); w.center = [x, y, 0.0]; }
}
/// js `hopperExt(deg)`, `hopperRate(deg)`
pub fn hopper_ext(deg: f64) -> f64 {
    let n = HOPPER_EXT.len();
    let u = m::max(0.0, m::min((n - 1) as f64, deg + 5.0));
    let i = m::min((n - 2) as f64, m::floor(u));
    let f = u - i;
    let i = i as usize;
    m::min(HOPPER_OUT, HOPPER_EXT[i] + (HOPPER_EXT[i + 1] - HOPPER_EXT[i]) * f)
}
pub fn hopper_rate(deg: f64) -> f64 { let (e0, e1) = (hopper_ext(deg - 0.5), hopper_ext(deg + 0.5)); ((e1 - e0) / 1.0) * (180.0 / m::PI) }
/// js `rotorPoint(p, phi)`, `placeRotor(wheels, phi)`
pub fn rotor_point([x, y, z]: [f64; 3], phi: f64) -> [f64; 3] {
    let (dx, dz, c, s) = (x - AXIS[0], z - AXIS[1], m::cos(phi), m::sin(phi));
    [AXIS[0] + dx * c - dz * s, y, AXIS[1] + dx * s + dz * c]
}
pub fn place_rotor(wheels: &mut [Wheel], home: &[([f64; 3], [f64; 3])], phi: f64) {
    let (c, s) = (m::cos(phi), m::sin(phi));
    for (w, (hc, ha)) in wheels.iter_mut().zip(home) {
        w.center = rotor_point(*hc, phi);
        let [ax, ay, az] = *ha;
        w.axis = [ax * c - az * s, ay, ax * s + az * c];
    }
}
/// js `hoodRot(deg)`, `hoodElevation(deg)`
pub fn hood_rot(deg: f64) -> f64 { ((shooter::HOOD_CAD - m::max(shooter::HOOD_MIN, m::min(shooter::HOOD_MAX, deg))) * m::PI) / 180.0 }
pub fn hood_elevation(deg: f64) -> f64 {
    let ([mx, my], [hx, hy]) = (shooter::MAIN_AT, shooter::HOOD_AT);
    ((m::atan2(mx - hx, hy - my) + hood_rot(deg)) * 180.0) / m::PI
}
/// js `placeShooter([main, hood], yaw, hoodDeg)`
pub fn place_shooter(w: &mut [Wheel], yaw: f64, hood_deg: f64) {
    let (c, s) = (m::cos(yaw), m::sin(yaw));
    let at = |[x, y]: [f64; 2]| [AXIS[0] + x * c, y, AXIS[1] - x * s];
    w[0].center = at(shooter::MAIN_AT);
    let [hx, hy] = rot(shooter::HOOD_AT, shooter::MAIN_AT, hood_rot(hood_deg));
    w[1].center = at([hx, hy]);
    for x in w.iter_mut() { x.axis = [s, 0.0, c]; }
}

#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
struct Path { body: PathMesh, intake: PathMesh, hopper: PathMesh, rotor: PathMesh, turret: PathMesh, hood: PathMesh }

/// pathSurfaces604's surfaces, in its order
pub const S_BODY: usize = 0;
pub const S_INTAKE: usize = 1;
pub const S_HOPPER: usize = 2;
pub const S_ROTOR: usize = 3;
pub const S_TURRET: usize = 4;
pub const S_HOOD: usize = 5;
pub const S_BUMPER: usize = 6;

/// js `pathSurfaces604(mesh, t)`
pub fn path_surfaces604(t: &Trains) -> Vec<Surface> {
    static P: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    let p = P.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path604.json")).expect("path604.json"));
    let s = |k: &str, mesh: &PathMesh, pos: [f64; 3], mu: f64| {
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{k} (CAD)"), &v, &mesh.i, mu, pos, 0.0, 0.0, 0.08, f64::INFINITY)
    };
    let piv = [PIVOT[0], PIVOT[1], 0.0];
    let ax = [AXIS[0], 0.0, AXIS[1]];
    let mut hopper = s("hopper", &p.hopper, [0.0; 3], 0.3).on_slide(t.cam, [1.0, 0.0, 0.0]);
    hopper.set_slide_k(1.0);
    vec![
        s("body", &p.body, [0.0; 3], 0.25),
        s("intake", &p.intake, piv, 0.25).on_shaft(t.pivot, piv, [0.0, 0.0, 1.0]),
        hopper,
        s("rotor", &p.rotor, ax, 0.25).on_shaft(t.rotor, ax, [0.0, -1.0, 0.0]),
        s("turret", &p.turret, ax, 0.35),
        s("hood", &p.hood, [AXIS[0] + shooter::MAIN_AT[0], shooter::MAIN_AT[1], AXIS[1]], 0.35),
        Surface::block("bumper", 0.3699, 0.4334, 0.0381, 0.1651, 0.87 / 2.0, 0.02, BUMPER_MU, 20000.0),
    ]
}
/// js `placeHopper(sf, deg)`: and the cam's ratio (m of slide per rad of the arm)
pub fn place_hopper(sf: &mut [Surface], d: &mut Drives, cam: usize, deg: f64) {
    sf[S_HOPPER].set_pose([hopper_ext(deg) - HOPPER_OUT, 0.0, 0.0], 0.0, 0.0);
    if let Some(c) = &mut d.shafts[cam].cam { c.1 = hopper_rate(deg); }
}
/// js `placeRotorSurface(sf, phi)`
pub fn place_rotor_surface(sf: &mut [Surface], phi: f64) { sf[S_ROTOR].set_pose([AXIS[0], 0.0, AXIS[1]], -phi, 0.0); }
/// js `placeTurretSurfaces({ turret, hood }, yaw, hoodDeg)`
pub fn place_turret_surfaces(sf: &mut [Surface], yaw: f64, hood_deg: f64) {
    sf[S_TURRET].set_pose([AXIS[0], 0.0, AXIS[1]], yaw, 0.0);
    let (c, s, mx) = (m::cos(yaw), m::sin(yaw), shooter::MAIN_AT[0]);
    sf[S_HOOD].set_pose([AXIS[0] + mx * c, shooter::MAIN_AT[1], AXIS[1] - mx * s], yaw, hood_rot(hood_deg));
}
