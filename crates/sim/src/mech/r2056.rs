//! js/mech/r2056.js: 2056 OP Robotics' "Desperate Measures" as physical mechanisms, from their
//! technical binder and drawing package (no 3D CAD released: the ball path's surfaces are built in
//! code, measured off the drawings). The browser game's file has where each number was read; here
//! are the numbers. The surfaces come as data (tools/export-mech.mjs: assets/mech/gen2056.json).
//! Robot frame: x forward (the intake end), y up, z to its right.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{Plate, Surface, Wheel, BUMPER_MU};
use jsmath as m;

const DEG: f64 = m::PI / 180.0;
const R: f64 = FUEL::radius;

/// js MU, TREAD
pub mod mu {
    pub const SILICONE: f64 = 1.2;
    pub const STEALTH: f64 = 1.3;
    pub const URETHANE: f64 = 1.2;
    pub const NET: f64 = 0.25;
}
pub mod tread {
    pub const SILICONE: f64 = 20000.0;
    pub const STEALTH: f64 = 12000.0;
    pub const URETHANE: f64 = 30000.0;
    pub const NET: f64 = 100000.0;
}
/// js HALF_L, HALF_W (25.5 x 29.5in)
pub const HALF_L: f64 = 12.75 * IN;
pub const HALF_W: f64 = 14.75 * IN;
/// js WALL
pub const WALL_RIGHT: f64 = 0.346;
pub const WALL_TOP: f64 = 0.533;
/// js FLOOR
pub const FLOOR_R: f64 = (1.625 / 2.0) * IN;
pub const FLOOR_AT: [[f64; 2]; 9] = [[0.2952, 0.1830], [0.2426, 0.1830], [0.1900, 0.1830], [0.1403, 0.1786], [0.0970, 0.1661], [0.0537, 0.1536], [0.0107, 0.1414], [-0.0342, 0.1414], [-0.0792, 0.1414]];
pub const FLOOR_Z0: f64 = -0.324;
pub const FLOOR_Z1: [f64; 9] = [0.324, 0.324, 0.324, 0.324, 0.324, 0.324, 0.123, 0.123, 0.123];
pub const FLOOR_RATIO: f64 = (36.0 / 14.0) * (25.0 / 18.0);
/// js TOP: the two rollers over the floor's back two
pub const TOP_AT: [[f64; 2]; 2] = [[-0.0342, 0.3253], [-0.0792, 0.3253]];
pub const TOP_R: f64 = (1.625 / 2.0) * IN;
pub const TOP_Z: [f64; 2] = [-0.324, 0.07];
pub const TOP_K: [f64; 2] = [-1.0, -1.0];
/// js ROLL.rr
pub const ROLL_RR: f64 = 0.08;
/// js BELT
pub const BELT_RATIO: f64 = 36.0 / 12.0;
pub fn belt_pitch() -> f64 { (24.0 * 0.005) / (2.0 * m::PI) }
/// js BACK
pub const BACK_X: f64 = -0.2105;
pub const BACK_FRONT: f64 = -0.056;
/// js TURRET
pub const TURRET_AT: [f64; 2] = [-5.25 * IN, 7.25 * IN];
pub const TURRET_DECK: f64 = 0.3932;
/// js VKICK
pub const VKICK_AT: [f64; 2] = [-0.0415, 0.10];
pub const VKICK_Y: [f64; 2] = [0.2283 - 0.875 * IN, 0.2283 + 0.875 * IN];
pub const VKICK_R: f64 = (1.625 / 2.0) * IN;
pub const VKICK_RATIO: f64 = 24.0 / 18.0;
/// js PIPE.c
pub const PIPE_C: [f64; 2] = [0.088, 0.3203];
/// js FEED
pub const FEED_X: f64 = -0.134;
pub const FEED_W: f64 = 0.025;
pub const FEED_R: f64 = 1.0 * IN;
pub fn feed_wheels() -> [[f64; 2]; 2] { [[PIPE_C[0], PIPE_C[1]], [TURRET_AT[1] + (R + 1.0 * IN - 0.0044), 0.3393]] }
pub const FEED_RATIO: f64 = 28.0 / 16.0;
/// js SHOOTER
pub mod shooter {
    use crate::constants::IN;
    pub fn axle() -> [f64; 2] { [3.5 * IN, super::TURRET_DECK + 2.83 * IN] }
    pub const R: f64 = 2.0 * IN;
    pub const W: f64 = 1.92 * IN;
    pub const KICK_R_ARM: f64 = 8.16 * IN;
    pub const KICK_R: f64 = 1.0 * IN;
    pub const KICK_W: f64 = 0.05;
    pub const KICK_RATIO: f64 = 1.0;
    pub const EL_MIN: f64 = 39.5;
    pub const EL_MAX: f64 = 73.5;
    pub const RATIO: f64 = 34.0 / 28.0;
    pub const J: f64 = 6.75e-4;
    pub const KICK_J: f64 = 4e-5;
    pub const HOOD_V: f64 = 350.0;
    pub const HOOD_A: f64 = 3500.0;
    pub const KP: f64 = 0.25;
    pub const TOL_V: f64 = 0.03;
    pub const TOL_HOOD: f64 = 1.5;
}
/// js INTAKE: its travel along the racks (TRAVEL: the drawing's 12.25in), the drawn pose
pub mod intake {
    use super::DEG;
    use crate::constants::IN;
    use jsmath as m;
    fn tan() -> f64 { m::tan(15.2 * DEG) }
    fn travel() -> f64 { 12.25 * IN }
    /// js SLIDE_OUT (INTAKE.slide), DRAWN
    pub fn slide() -> [f64; 2] { [travel(), -travel() * tan()] }
    fn drawn() -> [f64; 2] { [12.25 * IN, -12.25 * IN * tan()] }
    /// js OUT(x, y), EXT(x, y)
    fn out(x: f64, y: f64) -> [f64; 2] { let d = drawn(); [x - d[0], y - d[1]] }
    fn ext(x: f64, y: f64) -> [f64; 2] { let (d, s) = (drawn(), slide()); [x - d[0] + s[0], y - d[1] + s[1]] }
    pub const WIDTH: f64 = 23.5 * IN;
    pub fn rollers() -> [[f64; 3]; 2] { [[0.5638, 0.1693], [0.5157, 0.2707]].map(|[x, y]| { let [a, b] = out(x, y); [a, b, (2.125 / 2.0) * IN] }) }
    pub const RATIO: f64 = 30.0 / 12.0;
    pub fn frame_front() -> f64 { ext(0.6202, 0.0)[0] }
    pub const FRAME_SIDES: f64 = 0.352;
    pub fn frame_top() -> f64 { ext(0.0, 0.467)[1] }
    /// the kicker bar: its arms' pin, its two tubes (the arms' frame), folded at A0, falling to FALL
    pub fn kick_pivot() -> [f64; 2] { out(0.4530, 0.1996) }
    pub const KICK_TUBES: [[f64; 2]; 2] = [[0.1617, 0.0], [0.1411, -0.0345]];
    pub const KICK_A0: f64 = 0.0;
    pub const KICK_FALL: f64 = -90.0;
    pub const KICK_R: f64 = 0.375 * IN;
    pub const KICK_MASS: f64 = 0.25;
    pub const KICK_J: f64 = 0.008;
    pub const TUBING_PRELOAD: f64 = 1.0;
    pub const TUBING_RATE: f64 = 1.0;
}
/// js SLIDE: the racks' drive
pub mod slide {
    use crate::constants::IN;
    use jsmath as m;
    pub const PINION: f64 = 0.75 * IN;
    pub const RATIO: f64 = 40.0 / 8.0;
    pub const MASS: f64 = 4.5;
    pub fn travel() -> f64 { let s = super::intake::slide(); m::hypot2(s[0], s[1]) }
    pub fn dir() -> [f64; 3] { let s = super::intake::slide(); [s[0] / m::hypot2(s[0], s[1]), s[1] / m::hypot2(s[0], s[1]), 0.0] }
}
/// js PRELOAD
pub fn preload() -> Vec<[f64; 3]> {
    let mut p: Vec<[f64; 3]> = [-0.22, -0.07, 0.02].iter().map(|&z| [-0.135, 0.24, z]).collect();
    p.extend([-0.24, -0.09, 0.06].iter().map(|&z| [0.13, 0.26, z]));
    p.extend([-0.17, -0.02].iter().map(|&z| [0.12, 0.41, z]));
    p
}
/// js GUIDE.to's z (the short rollers' end), GUARD.r
pub const GUIDE_TO_Z: f64 = 0.123;
pub const GUARD_R: f64 = 7.5 * IN + 0.01;
/// js PLATE
pub mod plate {
    pub const K: f64 = 40.0;
    pub const K3: f64 = 2e5;
    pub const MAX: f64 = 0.12;
    pub const TAU: f64 = 0.05;
    pub const PRESS: f64 = 0.02;
}

/// js build2056()'s answer (drives indices)
pub struct Trains {
    pub intake: usize, pub rollers: [usize; 2], pub kick_arm: usize, pub slide: usize,
    pub floor: usize, pub floor_shafts: Vec<usize>, pub top: [usize; 2],
    pub belts: [usize; 2], pub vkick: usize, pub feed: usize, pub feed_shafts: [usize; 2],
    pub shooter: usize, pub wheel: usize, pub kicker: usize,
}
/// js `build2056()`
pub fn build2056(d: &mut Drives) -> Trains {
    let lim = |kind: &str, ratio: f64| Motor::new(kind, 1.0, ratio, 0.9, 80.0, 40.0);
    let intake = d.train("intake rollers", lim("krakenX60", intake::RATIO));
    let rollers = [0, 1].map(|i| d.add(intake, Shaft::new(&format!("intake roller {}", if i == 1 { "top" } else { "bottom" }), None, 2e-4, 2e-4, 0.0), -1.0));
    let kick_arm = d.shaft(Shaft::new("kicker bar arms (pin)", None, intake::KICK_J, 0.02, 0.0));
    let slide = d.shaft(Shaft::new("intake racks (pinions)", Some(Motor::new("krakenX44", 1.0, slide::RATIO, 0.9, 20.0, 20.0)), slide::MASS * m::pow(slide::PINION, 2.0), 0.005, 0.05));
    let floor = d.train("roller floor", lim("krakenX60", FLOOR_RATIO));
    let floor_shafts = (0..FLOOR_AT.len()).map(|i| d.add(floor, Shaft::new(&format!("floor roller {}", i + 1), None, 1.5e-4, 2e-4, 0.0), 1.0)).collect();
    let top = [0, 1].map(|i| d.add(floor, Shaft::new(&format!("top roller {}", i + 1), None, 1.5e-4, 2e-4, 0.0), TOP_K[i]));
    let belts = ["bottom", "top"].map(|k| d.shaft(Shaft::new(&format!("{k} belt"), Some(lim("krakenX44", BELT_RATIO)), 5e-5, 4e-4, 0.0)));
    let vkick = d.shaft(Shaft::new("side kicker", Some(lim("krakenX44", VKICK_RATIO)), 3e-5, 2e-4, 0.0));
    let feed = d.train("shooter feed (Stealth wheels)", lim("krakenX60", FEED_RATIO));
    let feed_shafts = [0, 1].map(|i| d.add(feed, Shaft::new(&format!("shooter feed {}", if i == 1 { "outer" } else { "inner" }), None, 4e-5, 2e-4, 0.0), 1.0));
    let shooter = d.train("shooter", Motor::new("krakenX60", 2.0, shooter::RATIO, 0.9, 80.0, 40.0));
    let wheel = d.add(shooter, Shaft::new("4in shooter wheel", None, shooter::J, 2e-4, 0.0), 1.0);
    let kicker = d.shaft(Shaft::new("backspin kicker", Some(Motor::new("krakenX44", 1.0, shooter::KICK_RATIO, 0.9, 60.0, 30.0)), shooter::KICK_J, 1e-4, 0.0));
    Trains { intake, rollers, kick_arm, slide, floor, floor_shafts, top, belts, vkick, feed, feed_shafts, shooter, wheel, kicker }
}

/// js wheels2056()'s answer: the intake's (and their homes), the floor's and the rollers over it,
/// the side kicker, the feed wheels
pub struct Wheels { pub intake: Vec<Wheel>, pub intake_home: Vec<[f64; 3]>, pub floor: Vec<Wheel>, pub vkick: Wheel, pub feed: Vec<Wheel> }
/// js `wheels2056(t)`
pub fn wheels2056(t: &Trains) -> Wheels {
    let z = [0.0, 0.0, 1.0];
    let intake: Vec<Wheel> = intake::rollers().iter().enumerate().map(|(i, [x, y, r])| Wheel::new(&format!("intake roller {} (silicone)", if i == 1 { "top" } else { "bottom" }), t.rollers[i], [*x, *y, 0.0], z, *r, intake::WIDTH).mu(mu::SILICONE).tread(tread::SILICONE)).collect();
    let intake_home = intake.iter().map(|w| w.center).collect();
    let mut floor: Vec<Wheel> = FLOOR_AT.iter().enumerate().map(|(i, [x, y])| Wheel::new(&format!("floor roller {} (silicone)", i + 1), t.floor_shafts[i], [*x, *y, (FLOOR_Z0 + FLOOR_Z1[i]) / 2.0], z, FLOOR_R, FLOOR_Z1[i] - FLOOR_Z0).mu(mu::SILICONE).tread(tread::SILICONE)).collect();
    floor.extend(TOP_AT.iter().enumerate().map(|(i, [x, y])| Wheel::new(&format!("top roller {} (silicone)", i + 1), t.top[i], [*x, *y, (TOP_Z[0] + TOP_Z[1]) / 2.0], z, TOP_R, TOP_Z[1] - TOP_Z[0]).mu(mu::SILICONE).tread(tread::SILICONE)));
    let vkick = Wheel::new("side kicker (polyurethane)", t.vkick, [VKICK_AT[0], (VKICK_Y[0] + VKICK_Y[1]) / 2.0, VKICK_AT[1]], [0.0, 1.0, 0.0], VKICK_R, VKICK_Y[1] - VKICK_Y[0]).mu(mu::URETHANE).tread(30000.0);
    let feed = feed_wheels().iter().enumerate().map(|(i, [zz, y])| Wheel::new(&format!("shooter feed {} (2in Stealth)", if i == 1 { "outer" } else { "inner" }), t.feed_shafts[i], [FEED_X, *y, *zz], [if i == 1 { 1.0 } else { -1.0 }, 0.0, 0.0], FEED_R, FEED_W).mu(mu::STEALTH).tread(tread::STEALTH)).collect();
    Wheels { intake, intake_home, floor, vkick, feed }
}
/// js `shooterWheel(t)`, `kickerWheel(t)`
pub fn shooter_wheels(t: &Trains) -> Vec<Wheel> {
    let z = [0.0, 0.0, 1.0];
    vec![
        Wheel::new("4in shooter wheel (60A urethane)", t.wheel, [0.0; 3], z, shooter::R, shooter::W).mu(mu::URETHANE).tread(tread::URETHANE),
        Wheel::new("backspin kicker (2in Stealth)", t.kicker, [0.0; 3], z, shooter::KICK_R, shooter::KICK_W).mu(mu::STEALTH).tread(tread::STEALTH),
    ]
}
/// js `turretPoint([s, y, z], yaw)`
pub fn turret_point([s, y, z]: [f64; 3], yaw: f64) -> [f64; 3] {
    let (c, sn) = (m::cos(yaw), m::sin(yaw));
    let [tx, tz] = TURRET_AT;
    [tx + s * c + z * sn, y, tz - s * sn + z * c]
}
/// js `kickAt(ext)`: how far the kicker bar can fall (deg from folded) with the intake ext of the
/// way out: once its tubes clear the bumper's foam and, inside the frame, the floor under it
pub fn kick_at(ext: f64) -> f64 {
    let (piv, s) = (intake::kick_pivot(), intake::slide());
    let (px, py) = (piv[0] + s[0] * ext, piv[1] + s[1] * ext);
    let (bx1, by1) = (HALF_L + 3.25 * IN + intake::KICK_R, 0.16 + intake::KICK_R);
    let mut d = intake::KICK_FALL;
    while d < 0.0 {
        let a = (intake::KICK_A0 + d) * DEG;
        let (c, sn) = (m::cos(a), m::sin(a));
        if intake::KICK_TUBES.iter().all(|[tx, ty]| {
            let (x, y) = (px + tx * c - ty * sn, py + tx * sn + ty * c);
            !((x < bx1 && y < by1) || (x < HALF_L + 0.01 && y < 0.215))
        }) { return d; }
        d += 0.5;
    }
    0.0
}
/// js `kickLoad(deg)`: the arms' load about their pin: their weight, the tubing toward their stop
pub fn kick_load(deg: f64) -> f64 {
    let a = (intake::KICK_A0 + deg) * DEG;
    let s = m::sign(intake::KICK_FALL);
    -intake::KICK_MASS * 9.81 * 0.12 * m::cos(a) + s * (intake::TUBING_PRELOAD + intake::TUBING_RATE * m::max(0.0, (intake::KICK_FALL - deg) * s * DEG))
}
/// js `intakeFront(ext)`
pub fn intake_front(ext: f64) -> f64 { intake::frame_front() - intake::slide()[0] * (1.0 - ext) }

/// Surfaces: their indices in path_surfaces2056's list (js Object.values order)
pub const S_BODY: usize = 0;
pub const S_BORE: usize = 1;
pub const S_BOTTOM: usize = 2;
pub const S_TOP: usize = 3;
pub const S_CARRIAGE: usize = 4;
pub const S_KICK_TUBE: usize = 5;
pub const S_TURRET: usize = 6;
pub const S_HOOD: usize = 7;
pub const S_BUMPER: usize = 8;
pub const S_NET: usize = 9;

#[derive(serde::Deserialize)]
struct GenSurface { key: String, name: String, tri: Vec<f64>, mu: f64, tread: Option<f64>, cell: f64, pos: [f64; 3], axes: [[f64; 3]; 3] }
#[derive(serde::Deserialize)]
struct Gen { surfaces: Vec<GenSurface> }

/// js `pathSurfaces2056(t)`: the body, the bore, the bottom and top belts (sliding along +z), the
/// intake (on its racks), the kicker bar's tubes (on their pin), the turret, the hood's guide, the
/// bumper, the net
pub fn path_surfaces2056(t: &Trains) -> Vec<Surface> {
    static G: std::sync::OnceLock<Gen> = std::sync::OnceLock::new();
    let g = G.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/gen2056.json")).expect("gen2056.json"));
    assert_eq!(g.surfaces.iter().map(|s| s.key.as_str()).collect::<Vec<_>>(), ["body", "bore", "bottom", "top", "carriage", "kickTube", "turret", "hood"], "gen2056.json's surfaces");
    let from_gen = |i: usize| { let s = &g.surfaces[i]; Surface::from_tri(&s.name, &s.tri, s.mu, s.tread.unwrap_or(f64::INFINITY), s.cell, s.pos, s.axes, None) };
    let mut bottom = from_gen(2).on_slide(t.belts[0], [0.0, 0.0, 1.0]);
    bottom.set_slide_k(belt_pitch());
    let mut top = from_gen(3).on_slide(t.belts[1], [0.0, 0.0, 1.0]);
    top.set_slide_k(belt_pitch());
    let mut carriage = from_gen(4).on_slide(t.slide, slide::dir());
    carriage.set_slide_k(slide::PINION);
    let piv = intake::kick_pivot();
    vec![
        from_gen(0),
        from_gen(1),
        bottom,
        top,
        carriage,
        from_gen(5).on_shaft(t.kick_arm, [piv[0], piv[1], 0.0], [0.0, 0.0, 1.0]),
        from_gen(6),
        from_gen(7),
        Surface::block("bumper", HALF_L + 0.75 * IN, HALF_L + 3.25 * IN, 0.032, 0.16, HALF_W + 3.25 * IN, 0.02, BUMPER_MU, 20000.0),
        Surface::plate("net", mu::NET, tread::NET, Plate::new2056(BACK_X, intake::frame_front(), WALL_TOP, WALL_RIGHT, 0.0, HALF_L, intake::frame_top(), BACK_FRONT, GUIDE_TO_Z, plate::K, plate::K3, plate::TAU, plate::MAX)),
    ]
}
/// js `placeIntake(sf, wheels, ext, kickDeg)`
pub fn place_intake(sf: &mut [Surface], wheels: &mut [Wheel], home: &[[f64; 3]], ext: f64, kick_deg: f64) {
    let [dx, dy] = intake::slide();
    let k = intake::kick_pivot();
    let piv = [k[0] + dx * ext, k[1] + dy * ext, 0.0];
    sf[S_CARRIAGE].set_pose([dx * ext, dy * ext, 0.0], 0.0, 0.0);
    sf[S_KICK_TUBE].set_pose(piv, 0.0, (intake::KICK_A0 + kick_deg) * DEG);
    sf[S_KICK_TUBE].set_pivot(piv);
    for (w, h) in wheels.iter_mut().zip(home) { w.center = [h[0] + dx * ext, h[1] + dy * ext, h[2]]; }
}
/// js `placeTurret(sf, wheels, yaw, hoodDeg)`: wheels, the shooter wheel then the backspin kicker
pub fn place_turret(sf: &mut [Surface], wheels: &mut [Wheel], yaw: f64, hood_deg: f64) {
    let [ax, ay] = shooter::axle();
    sf[S_TURRET].set_pose([TURRET_AT[0], 0.0, TURRET_AT[1]], yaw, 0.0);
    let lat = [m::sin(yaw), 0.0, m::cos(yaw)];
    wheels[0].center = turret_point([ax, ay, 0.0], yaw);
    wheels[0].axis = [-lat[0], -lat[1], -lat[2]];
    let a = hood_deg * DEG;
    sf[S_HOOD].set_pose(turret_point([ax, ay, 0.0], yaw), yaw, a);
    wheels[1].center = turret_point([ax + shooter::KICK_R_ARM * m::cos(a), ay + shooter::KICK_R_ARM * m::sin(a), 0.0], yaw);
    wheels[1].axis = lat;
}
