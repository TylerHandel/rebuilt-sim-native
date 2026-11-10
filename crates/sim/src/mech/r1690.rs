//! js/mech/r1690.js: 1690's Kepler as physical mechanisms, from their CAD (the X_T release) and
//! what the team posted. The browser game's file has every drive traced and the why of each
//! number; here are the numbers. The surfaces its code builds (the belts' runs, the kicker bar's
//! tube, the funnel's lid) come as data (tools/export-mech.mjs: assets/mech/gen1690.json), its CAD's
//! from assets/mech/path1690.json. Robot frame: x forward (the intake end), y up, z to its right.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{Plate, Surface, Wheel, BUMPER_MU};
use jsmath as m;

const DEG: f64 = m::PI / 180.0;

/// js MU, TREAD
pub mod mu {
    pub const SILICONE: f64 = 1.2;
    pub const COMPLIANT: f64 = 1.3;
    pub const TREAD: f64 = 1.3;
    pub const ALU: f64 = 0.4;
    pub const PTFE: f64 = 0.05;
    pub const PLA: f64 = 0.3;
    pub const WALL: f64 = 0.25;
}
pub mod tread {
    pub const SILICONE: f64 = 20000.0;
    pub const COMPLIANT: f64 = 12000.0;
    pub const TREAD: f64 = 30000.0;
    pub const PLATE: f64 = 100000.0;
}
/// js HALF_L, HALF_W (its 25 x 29in frame)
pub const HALF_L: f64 = 12.5 * IN;
pub const HALF_W: f64 = 14.5 * IN;
/// js WALL.top
pub const WALL_TOP: f64 = 0.551;
/// js FLOOR
pub const FLOOR_R: f64 = 0.0165;
pub const FLOOR_AT: [[f64; 2]; 9] = [[0.2810, 0.1817], [0.2335, 0.1749], [0.1860, 0.1681], [0.1422, 0.1618], [0.0984, 0.1556], [0.0546, 0.1493], [0.0071, 0.1425], [-0.0409, 0.1425], [-0.0874, 0.1425]];
pub const FLOOR_Z0: [f64; 9] = [-0.322, -0.322, -0.322, -0.322, -0.322, -0.322, -0.132, -0.132, -0.132];
pub const FLOOR_Z1: [f64; 9] = [0.31, 0.298, 0.298, 0.298, 0.298, 0.298, 0.298, 0.276, 0.276];
pub const FLOOR_RATIO: f64 = 2.0;
pub fn floor_k() -> [f64; 9] {
    let a = 28.0 / 30.0 * m::pow(29.0 / 32.0, 3.0);
    [a, a, a, 28.0 / 30.0 * m::pow(29.0 / 32.0, 2.0), 28.0 / 30.0 * 29.0 / 32.0, 28.0 / 30.0, 28.0 / 30.0, 28.0 / 30.0, 1.0]
}
/// js TOP: the two rollers over the floor's back
pub const TOP_AT: [[f64; 2]; 2] = [[-0.0409, 0.3191], [-0.0874, 0.3183]];
pub const TOP_R: f64 = 0.016;
pub const TOP_Z: [f64; 2] = [-0.066, 0.291];
pub const TOP_K: [f64; 2] = [-28.0 / 30.0, -1.0];
/// js ROLL.rr: the foam's rolling resistance on the floor's rollers
pub const ROLL_RR: f64 = 0.08;
/// js BELT
pub const BELT_RATIO: f64 = 1.5;
pub fn belt_pitch() -> f64 { (24.0 * 0.005) / (2.0 * m::PI) }
/// js BACK.box (the electronics box's face)
pub const BACK_BOX: f64 = -0.043;
/// js VKICK
pub const VKICK_AT: [f64; 2] = [-0.0155, -0.1525];
pub const VKICK_Y: [f64; 2] = [0.088, 0.367];
pub const VKICK_R: f64 = 0.0125;
pub const VKICK_RATIO: f64 = 40.0 / 16.0;
/// js FEED
pub const FEED_X: f64 = -0.134;
pub const FEED_W: f64 = 0.05;
pub const FEED_WHEELS: [[f64; 2]; 3] = [[-0.1806, 0.1600], [-0.2358, 0.2031], [-0.2721, 0.2629]];
pub const FEED_R: f64 = 0.0286;
pub const FEED_TOP: [[f64; 2]; 2] = [[-0.2837, 0.3319], [-0.0909, 0.3319]];
pub const FEED_TOP_R: f64 = 0.0254;
pub const FEED_RATIO: f64 = 28.0 / 18.0;
/// js TURRET
pub const TURRET_AT: [f64; 2] = [-0.1340, -0.1872];
pub const TURRET_BORE_Y0: f64 = 0.36;
/// js HOOD0
pub const HOOD0: f64 = 172.47;
/// js SHOOTER
pub mod shooter {
    pub const AXLE: [f64; 2] = [0.070, 0.464];
    pub const R: f64 = 0.0502;
    pub const W: f64 = 0.05;
    pub const FEEDER_R_ARM: f64 = 0.2068;
    pub const FEEDER_R: f64 = 0.0254;
    pub const FEEDER_W: f64 = 0.05;
    pub const FEEDER_LOWER: f64 = 18.82;
    pub fn feeder_k() -> f64 { (42.0 / 30.0) * (21.0 / 19.0) }
    pub const EL_MIN: f64 = 35.0;
    pub fn el_max() -> f64 { super::HOOD0 - 90.0 }
    pub const RATIO: f64 = 1.5;
    pub const J: f64 = 5.43e-4;
    pub const HOOD_J: [f64; 2] = [4.18e-5, 7.09e-5];
    pub const ARM_J: f64 = 9.0e-6;
    pub const HOOD_V: f64 = 350.0;
    pub const HOOD_A: f64 = 3500.0;
    pub const KP: f64 = 0.25;
    pub const TOL_V: f64 = 0.03;
    pub const TOL_HOOD: f64 = 1.5;
}
/// js INTAKE
pub mod intake {
    pub const SLIDE: [f64; 2] = [0.3120, -0.0691];
    pub const WIDTH: f64 = 0.647;
    pub const ROLLERS: [[f64; 3]; 2] = [[0.2473, 0.2301, 0.021], [0.1993, 0.3290, 0.021]];
    pub const RATIO: f64 = 28.0 / 14.0;
    pub const FRAME_FRONT: f64 = 0.617;
    pub const FRAME_SIDES: f64 = 0.362;
    pub const FRAME_TOP: f64 = 0.482;
    pub const KICK_PIVOT: [f64; 2] = [0.2723, 0.1703];
    pub const KICK_TUBE: [f64; 3] = [0.1499, 0.3283, 0.0127];
    pub const KICK_FALL: f64 = -163.0;
    pub const KICK_MASS: f64 = 0.44;
    pub const KICK_COM: f64 = 0.173;
    pub const KICK_J: f64 = 0.015;
    pub const TUBING_PRELOAD: f64 = 1.4;
    pub const TUBING_RATE: f64 = 1.2;
}
/// js SLIDE: the carriage's drive
pub mod slide {
    use crate::constants::IN;
    use jsmath as m;
    pub const PINION: f64 = 0.7 * IN;
    pub const RATIO: f64 = (4.0 * 28.0) / 18.0;
    pub const MASS: f64 = 4.0;
    pub fn travel() -> f64 { m::hypot2(super::intake::SLIDE[0], super::intake::SLIDE[1]) }
    pub fn dir() -> [f64; 3] { let h = travel(); [super::intake::SLIDE[0] / h, super::intake::SLIDE[1] / h, 0.0] }
}
/// js PRELOAD
pub fn preload() -> Vec<[f64; 3]> {
    let mut p: Vec<[f64; 3]> = [-0.06, 0.09, 0.24].iter().map(|&z| [-0.135, 0.23, z]).collect();
    p.extend([-0.07, 0.08, 0.23].iter().map(|&z| [0.05, 0.27, z]));
    p.extend([0.03, 0.18].iter().map(|&z| [0.06, 0.42, z]));
    p
}
/// js NET, PLATE
pub mod plate {
    pub const HW: f64 = 0.358;
    pub const BULGE: f64 = 0.0;
    pub const K: f64 = 100.0;
    pub const K3: f64 = 6e5;
    pub const MAX: f64 = 0.08;
    pub const TAU: f64 = 0.05;
    pub const PRESS: f64 = 0.02;
    pub const BACK: f64 = -0.284;
    pub const OPEN_Z: f64 = 0.09;
}

/// js build1690()'s answer (drives indices)
pub struct Trains {
    pub intake: usize, pub rollers: [usize; 2], pub kick_arm: usize, pub slide: usize,
    pub floor: usize, pub floor_shafts: Vec<usize>, pub top: [usize; 2],
    pub belts: [usize; 2], pub vkick: usize, pub feed: usize, pub feed_shafts: Vec<usize>,
    pub shooter: usize, pub wheel: usize, pub hood_wheels: [usize; 2],
}
/// js `build1690()`
pub fn build1690(d: &mut Drives) -> Trains {
    let lim = |kind: &str, ratio: f64| Motor::new(kind, 1.0, ratio, 0.9, 80.0, 40.0);
    let intake = d.train("intake rollers", lim("krakenX60", intake::RATIO));
    let rollers = [0, 1].map(|i| d.add(intake, Shaft::new(&format!("intake roller {}", if i == 1 { "top" } else { "bottom" }), None, 1.5e-4, 2e-4, 0.0), -1.0));
    let kick_arm = d.shaft(Shaft::new("kicker bar arms (pin)", None, intake::KICK_J, 0.02, 0.0));
    let slide = d.shaft(Shaft::new("carriage racks (pinions)", Some(Motor::new("krakenX44", 1.0, slide::RATIO, 0.9, 40.0, 30.0)), slide::MASS * m::pow(slide::PINION, 2.0), 0.005, 0.05));
    let floor = d.train("roller floor", lim("krakenX60", FLOOR_RATIO));
    let fk = floor_k();
    let floor_shafts = (0..FLOOR_AT.len()).map(|i| d.add(floor, Shaft::new(&format!("floor roller {}", i + 1), None, 1.2e-4, 2e-4, 0.0), fk[i])).collect();
    let top = [0, 1].map(|i| d.add(floor, Shaft::new(&format!("top roller {}", i + 1), None, 1.2e-4, 2e-4, 0.0), TOP_K[i]));
    let belts = ["bottom", "top"].map(|k| d.shaft(Shaft::new(&format!("{k} belt"), Some(lim("krakenX44", BELT_RATIO)), 5e-5, 4e-4, 0.0)));
    let vkick = d.shaft(Shaft::new("vertical kicker", Some(lim("krakenX44", VKICK_RATIO)), 6e-5, 2e-4, 0.0));
    let feed = d.train("bottom and top kicker", lim("krakenX60", FEED_RATIO));
    let feed_shafts = (0..5).map(|i| {
        let name = if i < 3 { format!("bottom kicker {}", i + 1) } else { format!("top kicker {}", i - 2) };
        d.add(feed, Shaft::new(&name, None, 6e-5, 2e-4, 0.0), 1.0)
    }).collect();
    let shooter = d.train("shooter", Motor::new("krakenX60", 2.0, shooter::RATIO, 0.9, 80.0, 30.0));
    let wheel = d.add(shooter, Shaft::new("4in shooter wheel", None, shooter::J, 2e-4, 0.0), 1.0);
    let hood_wheels = [0, 1].map(|i| d.add(shooter, Shaft::new(&format!("hood wheel {}", if i == 0 { "upper" } else { "lower" }), None, shooter::HOOD_J[i], 1e-4, 0.0), shooter::feeder_k()));
    d.add(shooter, Shaft::new("hood 30T/21T shaft", None, shooter::ARM_J, 2e-4, 0.0), 42.0 / 30.0);
    Trains { intake, rollers, kick_arm, slide, floor, floor_shafts, top, belts, vkick, feed, feed_shafts, shooter, wheel, hood_wheels }
}

/// js wheels1690()'s answer: the intake's (and their homes), the floor's and the rollers over it,
/// the vertical kicker, the feeder wheels
pub struct Wheels { pub intake: Vec<Wheel>, pub intake_home: Vec<[f64; 3]>, pub floor: Vec<Wheel>, pub vkick: Wheel, pub feed: Vec<Wheel> }
/// js `wheels1690(t)`
pub fn wheels1690(t: &Trains) -> Wheels {
    let z = [0.0, 0.0, 1.0];
    let intake: Vec<Wheel> = intake::ROLLERS.iter().enumerate().map(|(i, [x, y, r])| Wheel::new(&format!("intake roller {} (silicone)", if i == 1 { "top" } else { "bottom" }), t.rollers[i], [*x, *y, 0.0], z, *r, intake::WIDTH).mu(mu::SILICONE).tread(tread::SILICONE)).collect();
    let intake_home = intake.iter().map(|w| w.center).collect();
    let mut floor: Vec<Wheel> = FLOOR_AT.iter().enumerate().map(|(i, [x, y])| Wheel::new(&format!("floor roller {} (silicone)", i + 1), t.floor_shafts[i], [*x, *y, (FLOOR_Z0[i] + FLOOR_Z1[i]) / 2.0], z, FLOOR_R, FLOOR_Z1[i] - FLOOR_Z0[i]).mu(mu::SILICONE).tread(tread::SILICONE)).collect();
    floor.extend(TOP_AT.iter().enumerate().map(|(i, [x, y])| Wheel::new(&format!("top roller {} (silicone)", i + 1), t.top[i], [*x, *y, (TOP_Z[0] + TOP_Z[1]) / 2.0], z, TOP_R, TOP_Z[1] - TOP_Z[0]).mu(mu::SILICONE).tread(tread::SILICONE)));
    let vkick = Wheel::new("vertical kicker (aluminum)", t.vkick, [VKICK_AT[0], (VKICK_Y[0] + VKICK_Y[1]) / 2.0, VKICK_AT[1]], [0.0, -1.0, 0.0], VKICK_R, VKICK_Y[1] - VKICK_Y[0]).mu(mu::ALU).tread(30000.0);
    let mut feed: Vec<Wheel> = FEED_WHEELS.iter().enumerate().map(|(i, [zz, y])| Wheel::new(&format!("bottom kicker {} (2.25in compliant)", i + 1), t.feed_shafts[i], [FEED_X, *y, *zz], [-1.0, 0.0, 0.0], FEED_R, FEED_W).mu(mu::COMPLIANT).tread(tread::COMPLIANT)).collect();
    feed.extend(FEED_TOP.iter().enumerate().map(|(i, [zz, y])| Wheel::new(&format!("top kicker {} (2in Stealth)", if i == 1 { "right" } else { "left" }), t.feed_shafts[3 + i], [FEED_X, *y, *zz], [if i == 1 { 1.0 } else { -1.0 }, 0.0, 0.0], FEED_TOP_R, FEED_W).mu(mu::COMPLIANT).tread(tread::COMPLIANT)));
    Wheels { intake, intake_home, floor, vkick, feed }
}
/// js `shooterWheel(t)`, `feederWheels(t)`
pub fn shooter_wheels(t: &Trains) -> Vec<Wheel> {
    let z = [0.0, 0.0, 1.0];
    let mut w = vec![Wheel::new("4in shooter wheel (swerve tread)", t.wheel, [0.0; 3], z, shooter::R, shooter::W).mu(mu::TREAD).tread(tread::TREAD)];
    for i in 0..2 {
        w.push(Wheel::new(if i == 1 { "hood wheel lower (2in roller)" } else { "hood wheel upper (2in Stealth)" }, t.hood_wheels[i], [0.0; 3], z, shooter::FEEDER_R, shooter::FEEDER_W).mu(mu::COMPLIANT).tread(tread::COMPLIANT));
    }
    w
}
/// js `turretPoint([s, y, z], yaw)`
pub fn turret_point([s, y, z]: [f64; 3], yaw: f64) -> [f64; 3] {
    let (c, sn) = (m::cos(yaw), m::sin(yaw));
    let [tx, tz] = TURRET_AT;
    [tx + s * c + z * sn, y, tz - s * sn + z * c]
}
/// js `kickAt(ext)`: how far the kicker bar has fallen (deg) with the carriage ext of the way out
pub fn kick_at(ext: f64) -> f64 { intake::KICK_FALL * m::min(1.0, m::max(0.0, (ext - 0.2) / 0.25)) }
/// js `kickLoad(deg)`: the kicker bar's arms' load (N m about the pin, + lifting them)
pub fn kick_load(deg: f64) -> f64 {
    let a0 = m::atan2(intake::KICK_TUBE[1] - intake::KICK_PIVOT[1], intake::KICK_TUBE[0] - intake::KICK_PIVOT[0]);
    let a = a0 + deg * DEG;
    let gravity = -intake::KICK_MASS * 9.81 * intake::KICK_COM * m::cos(a);
    let tubing = -(intake::TUBING_PRELOAD + intake::TUBING_RATE * m::max(0.0, (deg - intake::KICK_FALL) * DEG));
    gravity + tubing
}
/// js `intakeFront(ext)`: the U-frame's front, the carriage ext of the way out
pub fn intake_front(ext: f64) -> f64 { intake::FRAME_FRONT - intake::SLIDE[0] * (1.0 - ext) }

/// Surfaces: their indices in path_surfaces1690's list
pub const S_BODY: usize = 0;
pub const S_BODY_FINE: usize = 1;
pub const S_PTFE: usize = 2;
pub const S_BOTTOM: usize = 3;
pub const S_TOP: usize = 4;
pub const S_CARRIAGE: usize = 5;
pub const S_KICK: usize = 6;
pub const S_KICK_TUBE: usize = 7;
pub const S_FUNNEL_LID: usize = 8;
pub const S_TURRET: usize = 9;
pub const S_HOOD: usize = 10;
pub const S_BUMPER: usize = 11;
pub const S_NET: usize = 12;

#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct Path { body: PathMesh, body_fine: PathMesh, carriage: PathMesh, ptfe: PathMesh, kick: PathMesh, turret: PathMesh, hood: PathMesh }
#[derive(serde::Deserialize)]
struct GenSurface { key: String, name: String, tri: Vec<f64>, mu: f64, tread: Option<f64>, cell: f64, pos: [f64; 3], axes: [[f64; 3]; 3] }
#[derive(serde::Deserialize)]
struct Gen { surfaces: Vec<GenSurface> }

/// js `pathSurfaces1690(t)`: body, bodyFine, ptfe, the bottom and top belts (sliding along -z on
/// their shafts), the carriage (on its racks), the kicker bar's arms and tube (on their pin), the
/// funnel's lid, the turret, the hood, the bumper, the top plate and net
pub fn path_surfaces1690(t: &Trains) -> Vec<Surface> {
    static P: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    static G: std::sync::OnceLock<Gen> = std::sync::OnceLock::new();
    let p = P.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path1690.json")).expect("path1690.json"));
    let g = G.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/gen1690.json")).expect("gen1690.json"));
    assert_eq!(g.surfaces.iter().map(|s| s.key.as_str()).collect::<Vec<_>>(), ["bottom", "top", "kickTube", "funnelLid"], "gen1690.json's surfaces");
    let cad = |k: &str, mesh: &PathMesh, mu: f64, pos: [f64; 3]| {
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{k} (CAD)"), &v, &mesh.i, mu, pos, 0.0, 0.0, 0.08, f64::INFINITY)
    };
    let from_gen = |i: usize| { let s = &g.surfaces[i]; Surface::from_tri(&s.name, &s.tri, s.mu, s.tread.unwrap_or(f64::INFINITY), s.cell, s.pos, s.axes, None) };
    let piv = [intake::KICK_PIVOT[0], intake::KICK_PIVOT[1], 0.0];
    let mut bottom = from_gen(0).on_slide(t.belts[0], [0.0, 0.0, -1.0]);
    bottom.set_slide_k(belt_pitch());
    let mut top = from_gen(1).on_slide(t.belts[1], [0.0, 0.0, -1.0]);
    top.set_slide_k(belt_pitch());
    let mut carriage = cad("carriage", &p.carriage, mu::WALL, [0.0; 3]).on_slide(t.slide, slide::dir());
    carriage.set_slide_k(slide::PINION);
    vec![
        cad("body", &p.body, mu::WALL, [0.0; 3]),
        cad("bodyFine", &p.body_fine, mu::WALL, [0.0; 3]),
        cad("ptfe", &p.ptfe, mu::PTFE, [0.0; 3]),
        bottom,
        top,
        carriage,
        cad("kick", &p.kick, mu::WALL, piv).on_shaft(t.kick_arm, piv, [0.0, 0.0, 1.0]),
        from_gen(2).on_shaft(t.kick_arm, piv, [0.0, 0.0, 1.0]),
        from_gen(3),
        cad("turret", &p.turret, mu::PLA, [TURRET_AT[0], 0.0, TURRET_AT[1]]),
        cad("hood", &p.hood, mu::PLA, [0.0; 3]),
        Surface::block("bumper", HALF_L + 0.75 * IN, HALF_L + 3.25 * IN, 0.032, 0.16, HALF_W + 3.25 * IN, 0.02, BUMPER_MU, 20000.0),
        Surface::plate("top plate (polycarbonate) and net", mu::WALL, tread::PLATE, Plate::new(BACK_BOX, intake::FRAME_FRONT, WALL_TOP, plate::HW, plate::BULGE, HALF_L, intake::FRAME_TOP, plate::BACK, plate::OPEN_Z, plate::K, plate::K3, plate::TAU, plate::MAX)),
    ]
}
/// js `placeIntake(sf, wheels, ext, kickDeg)`
pub fn place_intake(sf: &mut [Surface], wheels: &mut [Wheel], home: &[[f64; 3]], ext: f64, kick_deg: f64) {
    let [dx, dy] = intake::SLIDE;
    let a = kick_deg * DEG;
    let piv = [intake::KICK_PIVOT[0], intake::KICK_PIVOT[1], 0.0];
    sf[S_CARRIAGE].set_pose([dx * ext, dy * ext, 0.0], 0.0, 0.0);
    sf[S_KICK].set_pose(piv, 0.0, a);
    sf[S_KICK_TUBE].set_pose(piv, 0.0, a);
    for (w, h) in wheels.iter_mut().zip(home) { w.center = [h[0] + dx * ext, h[1] + dy * ext, h[2]]; }
}
/// js `placeTurret(sf, wheels, yaw, hoodDeg)`: wheels, the shooter wheel then the hood's two
pub fn place_turret(sf: &mut [Surface], wheels: &mut [Wheel], yaw: f64, hood_deg: f64) {
    let [ax, ay] = shooter::AXLE;
    sf[S_TURRET].set_pose([TURRET_AT[0], 0.0, TURRET_AT[1]], yaw, 0.0);
    sf[S_HOOD].set_pose(turret_point([ax, ay, 0.0], yaw), yaw, (hood_deg - HOOD0) * DEG);
    let lat = [m::sin(yaw), 0.0, m::cos(yaw)];
    wheels[0].center = turret_point([ax, ay, 0.0], yaw);
    wheels[0].axis = [-lat[0], -lat[1], -lat[2]];
    for i in 0..2 {
        let a = (hood_deg + if i == 1 { shooter::FEEDER_LOWER } else { 0.0 }) * DEG;
        wheels[1 + i].center = turret_point([ax + shooter::FEEDER_R_ARM * m::cos(a), ay + shooter::FEEDER_R_ARM * m::sin(a), 0.0], yaw);
        wheels[1 + i].axis = lat;
    }
}
