//! 4414's RIPCURRENT as physical mechanisms (js/mech/r4414.js): the Dye Rotor (a hook sweeping the
//! hopper's bowl, a stack of TTB wheels feeding the channel to the bend and up the hub's bore), the
//! box on its racks with the intake's rollers, and the turret's flywheel, hood roller and sushi
//! feeder. Robot frame: x forward, y up, z to its right. Its surfaces are built in the browser game's
//! code (no CAD): their triangles come over as data (tools/export-mech.mjs, assets/mech/gen4414.json),
//! with the hook's and ramp's roller places that come from its curves; the rest is here, the same
//! numbers, operation for operation (the why of each is in the JS file).
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{Kind, Surface, Wheel};
use jsmath as m;

const R: f64 = FUEL::radius;

/// js MU
pub mod mu {
    pub const URETHANE: f64 = 1.1; pub const STEALTH: f64 = 1.6; pub const SUSHI: f64 = 1.0; pub const POLYCARB: f64 = 0.5;
    pub const OMNI: f64 = 0.6; pub const PLATE: f64 = 0.3;
}

pub fn half_l() -> f64 { 12.5 * IN }
pub fn half_w() -> f64 { 16.0 * IN }
/// js AXIS: the rotor's (and the turret's) axis
pub const AXIS: [f64; 2] = [-0.02, 0.0];
pub const FLOOR: f64 = 0.105;
/// js BAY.x0, BAY.hw
pub fn bay_x0() -> f64 { -(half_l() + IN) }
pub fn bay_hw() -> f64 { half_w() + IN }

/// js FEED
pub mod feed {
    use crate::constants::IN;
    pub const W: f64 = 0.078;
    pub const TTB_U: f64 = 0.09;
    pub fn ttb_r() -> f64 { 1.5 * IN }
    pub const TTB_YS: [f64; 2] = [0.16, 0.197];
    pub const TTB_OMNI_Y: f64 = 0.234;
    pub const TTB_WID: f64 = 0.024;
    pub fn ttb_squeeze() -> f64 { 0.8 * IN }
    pub fn omni_r() -> f64 { 0.625 * IN }
    pub const OMNI_SQUEEZE: f64 = 0.003;
    pub const OMNI_WID: f64 = 0.15;
}
/// js YB, BEND, BORE.r
pub fn yb() -> f64 { FLOOR + R + 0.002 }
pub fn bend() -> [f64; 2] { [R + feed::omni_r() - feed::OMNI_SQUEEZE, yb() + R + feed::omni_r() - feed::OMNI_SQUEEZE] }
pub fn bore_r() -> f64 { R + 0.004 }
/// js RAMP.rollers, HOOK
pub mod ramp { pub const R: f64 = 0.0125; pub const J: f64 = 2e-6; pub const FRICTION: f64 = 0.01; }
pub mod hook {
    use super::FLOOR;
    pub const R: f64 = 0.011; pub const J: f64 = 1e-6; pub const FRICTION: f64 = 0.01;
    pub fn y0() -> f64 { FLOOR + 0.006 }
    pub fn y1() -> f64 { FLOOR + 0.1 }
}
/// js CH: the channel's direction in the rotor (rad from +x toward +z)
pub fn ch() -> f64 { m::PI / 2.0 }

/// js FLY
pub mod fly { use crate::constants::IN; pub fn r() -> f64 { 1.5 * IN } pub const Y: f64 = 0.49; pub const W: f64 = 0.12; }
/// js SQ
mod sq { use crate::constants::IN; pub fn low() -> f64 { 0.9 * IN } pub fn hood() -> f64 { 0.8 * IN } pub fn roller() -> f64 { 1.0 * IN } pub fn feeder() -> f64 { 0.5 * IN } }
/// js SHOOTER
pub mod shooter {
    use super::{fly, sq, R};
    use crate::constants::IN;
    use jsmath as m;
    pub fn path() -> f64 { fly::r() + R - sq::low() / 2.0 }
    pub const ENTRY: f64 = 210.0;
    pub fn x() -> f64 { -path() * m::cos((ENTRY * m::PI) / 180.0) }
    pub fn hood_r() -> f64 { fly::r() + 2.0 * R - sq::hood() }
    pub fn roller_r() -> f64 { 0.5 * IN }
    pub const ROLLER_N: usize = 3;
    pub fn roller_seg() -> f64 { 1.4 * IN }
    pub const ROLLER_AT: f64 = 0.0;
    pub const ROLLER_K: f64 = 1.5;
    pub const ROLLER_J: f64 = 9.8e-6;
    pub fn roller_radius() -> f64 { hood_r() + roller_r() - (sq::roller() - sq::hood()) }
    pub fn feeder_r() -> f64 { 0.5 * IN }
    pub const FEEDER_YS: [f64; 3] = [0.32, 0.347, 0.374];
    pub const FEEDER_W: f64 = 0.06;
    pub const FEEDER_MU: [f64; 3] = [1.2, 1.2, 0.9];
    pub const FEEDER_TREAD: [f64; 3] = [10000.0, 10000.0, 25000.0];
    pub const J: f64 = 1.01e-3;
    pub const HOOD_MIN: f64 = 125.0;
    pub const HOOD_MAX: f64 = 160.0;
    pub const HOOD_STOW: f64 = 175.0;
    pub const HOOD_V: f64 = 7.0;
    pub const HOOD_A: f64 = 60.0;
}
/// js BOX
pub mod boxx {
    use super::{half_l, half_w};
    use crate::constants::IN;
    pub const EXT: f64 = 0.27;
    pub fn front() -> f64 { half_l() + 3.25 * IN + 0.2224 }
    pub fn side() -> f64 { half_w() + IN + 0.005 }
}
/// js RACK
pub mod rack { use crate::constants::IN; pub fn pinion() -> f64 { 0.5 * IN } pub const RATIO: f64 = 3.33; pub const MASS: f64 = 5.0; }
/// js INTAKE
pub mod intake {
    use crate::constants::IN;
    pub const PRIMARY_DX: f64 = -0.056;
    pub const PRIMARY_Y: f64 = 0.172;
    pub fn primary_r() -> f64 { 1.5 * IN }
    pub fn bar_r() -> f64 { 0.75 * IN }
    pub const BAR_MU: f64 = 1.0;
    pub const BAR_TREAD: f64 = 20000.0;
    pub fn bar_dx() -> f64 { -0.2224 - bar_r() }
    pub fn bar_y() -> f64 { 0.16 + bar_r() + 0.001 }
    pub const KICK_GAP: f64 = 0.010;
    pub const KICK_Y: f64 = 0.040;
    pub fn kick_r() -> f64 { 0.75 * IN }
    pub const KICK_MU: f64 = 1.0;
    pub const KICK_TREAD: f64 = 20000.0;
    pub fn kick_dx() -> f64 { kick_r() + KICK_GAP - 0.2224 }
    pub const GUARDS: [[f64; 2]; 2] = [[-0.066, 0.242], [-0.015, 0.27]];
    pub fn guard_r() -> f64 { 0.5 * IN }
    pub const WIDTH: f64 = 0.8;
}

/// js PRELOAD (robot frame)
pub const PRELOAD: [[f64; 3]; 8] = [
    [0.012, 0.186, -0.181], [-0.194, 0.186, -0.062], [-0.233, 0.25, 0.072],
    [-0.13, 0.243, -0.191], [-0.263, 0.302, -0.142], [-0.103, 0.25, 0.209], [-0.26, 0.38, -0.01], [-0.025, 0.295, -0.29],
];

/// js build4414's result
pub struct Trains { pub primary: usize, pub bar: usize, pub rotor: usize, pub rollers: usize, pub ttb: usize, pub feeder: usize, pub bend: usize, pub launcher: usize, pub fly: usize, pub hood_roller: usize, pub slide: usize }

/// js `build4414()`
pub fn build4414(d: &mut Drives) -> Trains {
    let primary = d.shaft(Shaft::new("intake primary 3in roller", Some(Motor::new("krakenX60", 2.0, 3.45, 0.9, 60.0, 40.0)), 3e-4, 2e-4, 0.0));
    let bar = d.shaft(Shaft::new("intake kick wheel and black roller", Some(Motor::new("krakenX44", 1.0, 1.5, 0.9, 80.0, 40.0)), 8e-5, 2e-4, 0.0));
    let rotor = d.shaft(Shaft::new("Dye Rotor", Some(Motor::new("krakenX60", 1.0, 36.4, 0.9, 80.0, 60.0)), 0.04, 0.02, 0.0));
    let rollers = d.train("Dye Rotor rollers", Motor::new("krakenX60", 2.0, 2.5, 0.9, 60.0, 40.0));
    let ttb = d.add(rollers, Shaft::new("TTB feeder wheels", None, 2.4e-4, 2e-4, 0.0), 1.0);
    let bend = d.add(rollers, Shaft::new("1.25in omnis", None, 1e-5, 1e-5, 0.0), 3.0 / 1.25);
    let launcher = d.train("launcher", Motor::new("krakenX44", 4.0, 1.0, 0.9, 60.0, 40.0));
    let fly = d.add(launcher, Shaft::new("3in flywheel and copper", None, shooter::J, 2e-4, 0.0), 1.0);
    let hood_roller = d.add(launcher, Shaft::new("hood sushi roller (belted up the hood)", None, shooter::ROLLER_J, 2e-4, 0.0), shooter::ROLLER_K);
    let feeder = d.add(launcher, Shaft::new("shooter sushi feeder", None, 2.9e-6, 2e-4, 0.0), 16.0 / 14.0);
    let slide = d.shaft(Shaft::new("box racks (pinions)", Some(Motor::new("krakenX44", 2.0, rack::RATIO, 0.9, 40.0, 30.0)), rack::MASS * m::pow(rack::pinion(), 2.0), 0.01, 0.05));
    Trains { primary, bar, rotor, rollers, ttb, feeder, bend, launcher, fly, hood_roller, slide }
}

/// js `chanL(u, y, w)`: the channel's frame -> the rotor's
pub fn chan_l(u: f64, y: f64, w: f64) -> [f64; 3] { let (c, s) = (m::cos(ch()), m::sin(ch())); [u * c - w * s, y, u * s + w * c] }
/// js `chan(u, y, w)`: and the robot's
pub fn chan(u: f64, y: f64, w: f64) -> [f64; 3] { let p = chan_l(u, y, w); [AXIS[0] + p[0], y, AXIS[1] + p[2]] }
fn chan_dir(u: f64, w: f64) -> [f64; 3] { let (c, s) = (m::cos(ch()), m::sin(ch())); [u * c - w * s, 0.0, u * s + w * c] }

/// The generated data (assets/mech/gen4414.json)
#[derive(serde::Deserialize)]
pub struct GenSurface { pub key: String, pub name: String, pub tri: Vec<f64>, pub mu: f64, pub tread: Option<f64>, pub cell: f64, pub pos: [f64; 3], pub axes: [[f64; 3]; 3], pub block: Option<GenBlock> }
#[derive(serde::Deserialize)]
pub struct GenBlock { pub c: [f64; 3], pub h: [f64; 3], pub r: f64 }
#[derive(serde::Deserialize)]
pub struct GenChain { pub y0: f64, pub n: usize, pub arm: Vec<f64>, pub pitch: f64, pub spring: f64, #[serde(rename = "rhoT")] pub rho_t: f64, #[serde(rename = "rhoC")] pub rho_c: f64, pub perim: f64, pub rc: f64 }
#[derive(serde::Deserialize)]
pub struct Fit { pub v: [f64; 7], pub el: [f64; 7], pub x: [f64; 7], pub y: [f64; 7] }
#[derive(serde::Deserialize)]
pub struct Gen {
    pub surfaces: Vec<GenSurface>,
    #[serde(rename = "hookRollers")] pub hook_rollers: Vec<[f64; 3]>,
    #[serde(rename = "rampRollers")] pub ramp_rollers: Vec<[f64; 3]>,
    pub chain: GenChain,
    pub fit: Fit,
    pub launch: Vec<[f64; 5]>,
}
pub fn gen4414() -> &'static Gen {
    static G: std::sync::OnceLock<Gen> = std::sync::OnceLock::new();
    G.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/gen4414.json")).expect("gen4414.json: run tools/sync-assets.mjs"))
}

/// js wheels4414's result: the rotor's (feed) and the box's (intake), each with its home
pub struct Wheels { pub feed: Vec<Wheel>, pub feed_home: Vec<([f64; 3], [f64; 3])>, pub intake: Vec<Wheel> }

/// js `wheels4414(t)` (its own shafts for each free roller added to the drives)
pub fn wheels4414(d: &mut Drives, t: &Trains) -> Wheels {
    let g = gen4414();
    let tw = R + feed::ttb_r() - feed::ttb_squeeze();
    let piv = |w: Wheel| w.on_pivot(t.rotor, [AXIS[0], 0.0, AXIS[1]], [0.0, -1.0, 0.0]);
    let mut fd = Vec::new();
    for (i, y) in feed::TTB_YS.iter().enumerate() {
        fd.push(Wheel::new(&format!("TTB 3in urethane {}", i + 1), t.ttb, chan(feed::TTB_U, *y, tw), [0.0, 1.0, 0.0], feed::ttb_r(), feed::TTB_WID).mu(mu::URETHANE).tread(15000.0));
    }
    fd.push(Wheel::new("TTB 3in omni", t.ttb, chan(feed::TTB_U, feed::TTB_OMNI_Y, tw), [0.0, 1.0, 0.0], feed::ttb_r(), feed::TTB_WID).kind(Kind::Omni).mu(mu::OMNI).tread(30000.0));
    for (i, c) in g.hook_rollers.iter().enumerate() {
        let s = d.shaft(Shaft::new(&format!("hook roller {}", i + 1), None, hook::J, 1e-6, hook::FRICTION));
        fd.push(Wheel::new("hook rollers", s, *c, [0.0, 1.0, 0.0], hook::R, hook::y1() - hook::y0() - 0.006).mu(mu::PLATE));
    }
    for (i, c) in g.ramp_rollers.iter().enumerate() {
        let s = d.shaft(Shaft::new(&format!("ramp roller {}", i + 1), None, ramp::J, 1e-6, ramp::FRICTION));
        fd.push(Wheel::new("ramp rollers", s, *c, chan_dir(0.0, 1.0), ramp::R, 2.0 * feed::W - 0.01).mu(mu::PLATE));
    }
    let b = bend();
    fd.push(Wheel::new("1.25in omnis", t.bend, chan(b[0], b[1], 0.0), chan_dir(0.0, -1.0), feed::omni_r(), feed::OMNI_WID).kind(Kind::Omni).mu(mu::OMNI).tread(30000.0));
    let fd: Vec<Wheel> = fd.into_iter().map(piv).collect();
    let feed_home = fd.iter().map(|w| (w.center, w.axis)).collect();
    let z = [0.0, 0.0, 1.0];
    let mut it = vec![
        Wheel::new("intake black roller", t.bar, [0.0; 3], z, intake::bar_r(), intake::WIDTH).mu(intake::BAR_MU).tread(intake::BAR_TREAD),
        Wheel::new("intake primary 3in roller", t.primary, [0.0; 3], [0.0, 0.0, -1.0], intake::primary_r(), intake::WIDTH).mu(mu::POLYCARB).tread(60000.0),
        Wheel::new("intake kick wheel", t.bar, [0.0; 3], z, intake::kick_r(), intake::WIDTH).mu(intake::KICK_MU).tread(intake::KICK_TREAD),
    ];
    for i in 0..intake::GUARDS.len() {
        let s = d.shaft(Shaft::new(&format!("guard tube {}", i + 1), None, 2e-5, 1e-6, 0.0));
        it.push(Wheel::new(&format!("intake guard tube {}", i + 1), s, [0.0; 3], z, intake::guard_r(), intake::WIDTH).mu(mu::PLATE));
    }
    place_intake(&mut it, boxx::EXT);
    Wheels { feed: fd, feed_home, intake: it }
}

/// js `rotorPoint(p, phi)`
pub fn rotor_point(p: [f64; 3], phi: f64) -> [f64; 3] {
    let (dx, dz, c, s) = (p[0] - AXIS[0], p[2] - AXIS[1], m::cos(phi), m::sin(phi));
    [AXIS[0] + dx * c - dz * s, p[1], AXIS[1] + dx * s + dz * c]
}
/// js `placeRotor(wheels, phi)`
pub fn place_rotor(wheels: &mut [Wheel], home: &[([f64; 3], [f64; 3])], phi: f64) {
    let (c, s) = (m::cos(phi), m::sin(phi));
    for (w, (hc, ha)) in wheels.iter_mut().zip(home) {
        w.center = rotor_point(*hc, phi);
        let [ax, ay, az] = *ha;
        w.axis = [ax * c - az * s, ay, ax * s + az * c];
    }
}
/// js `placeIntake([bar, primary, kick, ...guards], ext)`
pub fn place_intake(w: &mut [Wheel], ext: f64) {
    let xf = boxx::front() - boxx::EXT + ext;
    w[0].center = [xf + intake::bar_dx(), intake::bar_y(), 0.0];
    w[1].center = [xf + intake::PRIMARY_DX, intake::PRIMARY_Y, 0.0];
    w[2].center = [xf + intake::kick_dx(), intake::KICK_Y, 0.0];
    for (i, g) in intake::GUARDS.iter().enumerate() { w[3 + i].center = [xf + g[0], g[1], 0.0]; }
}

/// js `shooterWheels(t)`: the flywheel, the hood roller's sushi wheels, the feeder's; their homes
pub fn shooter_wheels(t: &Trains) -> (Vec<Wheel>, Vec<[f64; 3]>) {
    let z = [0.0, 0.0, 1.0];
    let mut w = vec![Wheel::new("3in flywheel (stealth wheels)", t.fly, [0.0; 3], [0.0, 0.0, -1.0], fly::r(), fly::W).mu(mu::STEALTH).tread(30000.0)];
    for i in 0..shooter::ROLLER_N {
        w.push(Wheel::new(&format!("hood 1in sushi roller {}", i + 1), t.hood_roller, [0.0; 3], z, shooter::roller_r(), shooter::roller_seg() - 0.002).mu(mu::SUSHI).tread(15000.0));
    }
    for (i, y) in shooter::FEEDER_YS.iter().enumerate() {
        w.push(Wheel::new(&format!("1in sushi feeder {}", i + 1), t.feeder, [0.0, *y, 0.0], [0.0, 0.0, -1.0], shooter::feeder_r(), shooter::FEEDER_W).mu(shooter::FEEDER_MU[i]).tread(shooter::FEEDER_TREAD[i]));
    }
    let home = w.iter().map(|x| x.center).collect::<Vec<_>>();
    place_shooter(&mut w, &home, 0.0, shooter::HOOD_STOW);
    (w, home)
}
/// js `hoodElevation(deg)`: where the hood lets FUEL go -> the shot's elevation
pub fn hood_elevation(deg: f64) -> f64 { deg - 90.0 }
/// js `placeShooter([fly, ...rest], yaw, hoodDeg)`
pub fn place_shooter(w: &mut [Wheel], home: &[[f64; 3]], yaw: f64, hood_deg: f64) {
    let (c, s, n) = (m::cos(yaw), m::sin(yaw), shooter::ROLLER_N);
    let at = |x: f64, y: f64, z: f64| [AXIS[0] + x * c + z * s, y, AXIS[1] - x * s + z * c];
    w[0].center = at(shooter::x(), fly::Y, 0.0);
    let b = ((hood_deg + shooter::ROLLER_AT) * m::PI) / 180.0;
    let rr = shooter::roller_radius();
    for i in 0..n {
        w[1 + i].center = at(shooter::x() + rr * m::cos(b), fly::Y + rr * m::sin(b), (i as f64 - (n as f64 - 1.0) / 2.0) * shooter::roller_seg());
    }
    let fx = -bore_r() + 2.0 * R - sq::feeder() + shooter::feeder_r();
    for i in 1 + n..w.len() { w[i].center = at(fx, home[i][1], 0.0); }
    for (i, wh) in w.iter_mut().enumerate() {
        let k = if (1..1 + n).contains(&i) { 1.0 } else { -1.0 };
        wh.axis = [k * s, 0.0, k * c];
    }
}

/// The surfaces' places in the robot's arena (js pathSurfaces4414's keys, in order)
pub const S_BODY: usize = 0;
pub const S_HOOK: usize = 2;
pub const S_FEED: usize = 3;
pub const S_BORE: usize = 4;
pub const S_COLUMN: usize = 5;
pub const S_BOX: usize = 6;
pub const S_TURRET: usize = 7;
pub const S_HOOD: usize = 8;

/// js `pathSurfaces4414(t)`: as the browser game builds them (gen4414.json), on their shafts
pub fn path_surfaces4414(t: &Trains) -> Vec<Surface> {
    let a = [AXIS[0], 0.0, AXIS[1]];
    let keys = ["body", "net", "hook", "feed", "bore", "column", "box", "turret", "hood", "bumper", "bumperBack", "bumperL", "bumperR"];
    let g = gen4414();
    assert_eq!(g.surfaces.iter().map(|s| s.key.as_str()).collect::<Vec<_>>(), keys, "gen4414.json's surfaces");
    g.surfaces.iter().enumerate().map(|(i, s)| {
        let sf = Surface::from_tri(&s.name, &s.tri, s.mu, s.tread.unwrap_or(f64::INFINITY), s.cell, s.pos, s.axes, s.block.as_ref().map(|b| (b.c, b.h, b.r)));
        match i {
            S_HOOK | S_FEED | S_BORE | S_COLUMN => sf.on_shaft(t.rotor, a, [0.0, -1.0, 0.0]),
            S_BOX => { let mut sf = sf.on_slide(t.slide, [1.0, 0.0, 0.0]); sf.set_slide_k(rack::pinion()); sf }
            _ => sf,
        }
    }).collect()
}
/// js `placeRotorSurfaces(s, phi)`
pub fn place_rotor_surfaces(s: &mut [Surface], phi: f64) {
    for i in [S_HOOK, S_FEED, S_BORE, S_COLUMN] { s[i].set_pose([AXIS[0], 0.0, AXIS[1]], -phi, 0.0); }
}
/// js `placeBox(sf, ext)`
pub fn place_box(s: &mut Surface, ext: f64) { s.set_pose([ext - boxx::EXT, 0.0, 0.0], 0.0, 0.0); }
/// js `placeTurretSurfaces(s, yaw, hoodDeg)`
pub fn place_turret_surfaces(s: &mut [Surface], yaw: f64, hood_deg: f64) {
    s[S_TURRET].set_pose([AXIS[0], 0.0, AXIS[1]], yaw, 0.0);
    let (c, sn) = (m::cos(yaw), m::sin(yaw));
    s[S_HOOD].set_pose([AXIS[0] + shooter::x() * c, fly::Y, AXIS[1] - shooter::x() * sn], yaw, (hood_deg * m::PI) / 180.0);
}
