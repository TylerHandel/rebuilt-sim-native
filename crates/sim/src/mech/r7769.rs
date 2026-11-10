//! js/mech/r7769.js: 7769 The CREW's CHUNK as physical mechanisms, from their CAD and their Chief
//! Delphi thread (no public code: the controls, speeds and some sizes are estimates; the JS says
//! which). Robot frame: x forward (the intake), y up, z to its right.
//!
//! The intake slides out on two polycarbonate racks, its 3in roller (cat-tongue grip tape) pulling
//! FUEL in under it and up over the kick bar and the bumper; the floor is ten timing belts sloping
//! back to a static curved ramp; the feeder's shaft (compliant, ION and vectored wheels) rolls them
//! up it into two stacks of 4in Stealth wheels, which throw them up round their front under the
//! fixed and adjustable hoods and out over the back.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{NetSheet, Surface, Wheel, BUMPER_MU};
use jsmath as m;

const R: f64 = FUEL::radius;
const DEG: f64 = m::PI / 180.0;

/// js MU: what each grips FUEL with (estimates)
pub mod mu {
    pub const GRIP_TAPE: f64 = 1.4;
    pub const COMPLIANT: f64 = 1.6;
    pub const ION: f64 = 1.5;
    pub const STEALTH: f64 = 1.3;
    pub const BELT: f64 = 0.8;
    pub const PETG: f64 = 0.5;
    pub const WALL: f64 = 0.25;
    pub const HOOD: f64 = 0.3;
}
/// js TREAD: how far each gives where a FUEL presses it (N/m; estimates)
pub mod tread {
    pub const GRIP_TAPE: f64 = 40000.0;
    pub const COMPLIANT: f64 = 3000.0;
    pub const ION: f64 = 4000.0;
    pub const STEALTH: f64 = 12000.0;
    pub const BELT: f64 = 20000.0;
    pub const PETG: f64 = 40000.0;
}

/// js HALF_L, HALF_W: the frame perimeter
pub const HALF_L: f64 = 0.3239;
pub const HALF_W: f64 = 0.3747;
/// js BUMPER
pub mod bumper {
    pub fn x0() -> f64 { super::HALF_L + 0.75 * super::IN }
    pub const X1: f64 = 0.409;
    pub const Y0: f64 = 0.038;
    pub const Y1: f64 = 0.152;
    pub const HW: f64 = 0.46;
}
/// js INTAKE: the 3in roller, as exported (all the way out)
pub mod intake {
    pub const AT: [f64; 2] = [0.5779, 0.1746];
    pub const R: f64 = 0.039;
    pub const WIDTH: f64 = 0.566;
}
/// js SLIDE: the intake's racks, sloping 9.2 deg down toward the front; out: its travel
pub mod slide {
    use jsmath as m;
    pub fn slope() -> f64 { 9.2 * super::DEG }
    pub const OUT: f64 = 0.305;
    pub fn dir() -> [f64; 2] { [-m::cos(slope()), m::sin(slope())] }
    pub fn pinion() -> f64 { 0.5 * super::IN }
    pub const RATIO: f64 = 42.0 / 12.0;
    pub const MASS: f64 = 4.5;
    /// js SLIDE_CURRENT (A, stator)
    pub const DEPLOY: f64 = 45.0;
    pub const HELD: f64 = 5.0;
}
/// js KICK: the kick bar hanging on the intake's arms
pub mod kick {
    use jsmath as m;
    pub const PIN: [f64; 2] = [0.4873, 0.1915];
    pub const BAR: [f64; 2] = [0.455, 0.043];
    pub const R: f64 = 0.007;
    pub const MAX: f64 = m::PI / 2.0;
    pub fn len() -> f64 { m::hypot2(BAR[0] - PIN[0], BAR[1] - PIN[1]) }
    /// (from straight down, forward positive)
    pub fn b0() -> f64 { m::atan2(BAR[0] - PIN[0], PIN[1] - BAR[1]) }
}
/// js FLOOR: ten 105T belts on 18T pulleys
pub mod floor {
    use jsmath as m;
    pub const FRONT: [f64; 2] = [0.2045, 0.2030];
    pub const BACK: [f64; 2] = [-0.0075, 0.1560];
    pub fn pitch() -> f64 { (18.0 * 0.005) / (2.0 * m::PI) }
    pub const R: f64 = 0.016;
    pub const HW: f64 = 0.27;
    pub fn len() -> f64 { m::hypot2(FRONT[0] - BACK[0], FRONT[1] - BACK[1]) }
    /// (the run's way)
    pub fn dir() -> [f64; 3] { let l = len(); [(BACK[0] - FRONT[0]) / l, (BACK[1] - FRONT[1]) / l, 0.0] }
}
/// js FEED: the feeder's shaft and its wheels (r, z, w)
pub mod feed {
    use crate::constants::IN;
    pub const AT: [f64; 2] = [-0.1061, 0.3175];
    pub fn compliant() -> [f64; 3] { [2.25 * IN / 2.0, 0.076, 0.14] }
    pub fn ion() -> [f64; 3] { [IN, 0.165, 0.0127] }
    pub fn vector() -> [f64; 3] { [IN, 0.2395, 0.111] }
}
/// js AXLE: the shooter's wheels' shaft
pub const AXLE: [f64; 2] = [-0.2286, 0.4953];
/// js SHOOTER
pub mod shooter {
    use crate::constants::IN;
    pub fn wheel() -> [f64; 3] { [2.0 * IN, 0.0952, 0.152] }
    pub const J: f64 = 1.6e-3;
    pub const HOOD_MIN: f64 = 0.0;
    pub const HOOD_MAX: f64 = 35.0;
    pub const EXIT_DEG: f64 = 14.7;
    pub const STATOR: f64 = 60.0;
    pub const SUPPLY: f64 = 40.0;
    pub fn kv() -> f64 { 12.0 / (5800.0 / 60.0) }
    pub const KP: f64 = 0.4;
}
/// js HOOD: its motor's position loop
pub mod hood {
    pub const RATE: f64 = 400.0;
    pub const TOL: f64 = 1.5;
}
/// js NET: the net over the hopper
pub mod net {
    pub const X0: f64 = -0.02;
    pub const Y0: f64 = 0.552;
    pub const XS: f64 = 0.309;
    pub const Y_FRONT: f64 = 0.494;
    pub const HW: f64 = 0.35;
    pub const BULGE: f64 = 0.02;
    pub const TREAD: f64 = 3000.0;
}
/// js FRONT_X: the hopper front's inner face, as exported
pub const FRONT_X: f64 = 0.6224;

/// js PRELOAD: eight on the belts
pub fn preload() -> Vec<[f64; 3]> {
    let (f, b) = (floor::FRONT, floor::BACK);
    let floor_y = |x: f64| b[1] + ((x - b[0]) / (f[0] - b[0])) * (f[1] - b[1]) + floor::R + R + 0.004;
    [0.02, 0.17].iter().flat_map(|&x| [-0.235, -0.08, 0.08, 0.235].map(|z| [x, floor_y(x), z])).collect()
}

/// js build7769()'s answer (indices into Drives)
pub struct Trains { pub intake: usize, pub roller: usize, pub floor: usize, pub belts: usize, pub feed_t: usize, pub feed: usize, pub shooter: usize, pub wheels: usize, pub slide: usize }

/// js `build7769()`
pub fn build7769(d: &mut Drives) -> Trains {
    let x60 = |count: f64, ratio: f64, stator: f64, supply: f64| Motor::new("krakenX60", count, ratio, 0.9, stator, supply);
    let intake = d.train("intake rollers", x60(2.0, 1.0, 60.0, 40.0));
    let roller = d.add(intake, Shaft::new("intake 3in roller (cat tongue)", None, 6e-4, 2e-4, 0.0), -12.0 / 30.0);
    let floor = d.train("floor belts", x60(1.0, 3.0, 60.0, 40.0));
    let belts = d.add(floor, Shaft::new("floor belts", None, 2e-4, 2e-4, 0.0), 1.0);
    let feed_t = d.train("feeder", x60(1.0, 1.0, 60.0, 40.0));
    let feed = d.add(feed_t, Shaft::new("feeder wheels", None, 3e-4, 2e-4, 0.0), -18.0 / 42.0);
    let shooter = d.train("shooter", x60(4.0, 1.0, shooter::STATOR, shooter::SUPPLY));
    let wheels = d.add(shooter, Shaft::new("shooter wheels (4in Stealth)", None, shooter::J, 2e-4, 0.0), 1.0);
    let mut s = Shaft::new("slide pinions", Some(Motor::new("krakenX44", 1.0, slide::RATIO, 0.9, slide::DEPLOY, 40.0)), slide::MASS * m::pow(slide::pinion(), 2.0), 0.05, 0.1);
    s.racks = true;
    let slide = d.shaft(s);
    Trains { intake, roller, floor, belts, feed_t, feed, shooter, wheels, slide }
}
/// js `slideStops(shaft, ext)`
pub fn slide_stops(d: &mut Drives, s: usize, ext: f64) {
    let sh = &mut d.shafts[s];
    sh.lo = if ext <= 1e-9 { 0.0 } else { f64::NEG_INFINITY };
    sh.hi = if ext >= slide::OUT - 1e-9 { 0.0 } else { f64::INFINITY };
}

/// js wheels7769()'s answer
pub struct Wheels { pub intake: Vec<Wheel>, pub intake_home: Vec<[f64; 3]>, pub feed: Vec<Wheel>, pub shooter: Vec<Wheel> }
/// js `wheels7769(t, { vector = 45 })`
pub fn wheels7769(t: &Trains) -> Wheels {
    let z = [0.0, 0.0, 1.0];
    let vector = 45.0;
    let intake = vec![Wheel::new("intake 3in roller (cat tongue)", t.roller, [intake::AT[0], intake::AT[1], 0.0], z, intake::R, intake::WIDTH).mu(mu::GRIP_TAPE).tread(tread::GRIP_TAPE)];
    let intake_home = intake.iter().map(|w| w.center).collect();
    let at = feed::AT;
    let mut feed = Vec::new();
    let [cr, cz, cw] = feed::compliant();
    for s in [-1.0, 1.0] { feed.push(Wheel::new("2.25in 35A compliant wheels", t.feed, [at[0], at[1], s * cz], z, cr, cw).mu(mu::COMPLIANT).tread(tread::COMPLIANT)); }
    let [ir, iz, iw] = feed::ion();
    for s in [-1.0, 1.0] { feed.push(Wheel::new("2in 40A ION wheel", t.feed, [at[0], at[1], s * iz], z, ir, iw).mu(mu::ION).tread(tread::ION)); }
    let [vr, vz, vw] = feed::vector();
    for s in [-1.0f64, 1.0] {
        let name = format!("2in vectored wheels ({})", if s < 0.0 { "LH" } else { "RH" });
        feed.push(Wheel::new(&name, t.feed, [at[0], at[1], s * vz], z, vr, vw).mecanum(s * vector).mu(mu::PETG).tread(tread::PETG));
    }
    let [sr, sz, sw] = shooter::wheel();
    let shooter = [-1.0, 1.0].iter().map(|s| Wheel::new("4in 50A Stealth wheels", t.wheels, [AXLE[0], AXLE[1], s * sz], z, sr, sw).mu(mu::STEALTH).tread(tread::STEALTH)).collect();
    Wheels { intake, intake_home, feed, shooter }
}
/// js `slideOffset(ext)`: the intake's offset from the CAD's pose (all the way out)
pub fn slide_offset(ext: f64) -> [f64; 2] { let d = slide::OUT - ext; let dir = slide::dir(); [dir[0] * d, dir[1] * d] }
/// js `placeIntake(wheels, ext)`
pub fn place_intake(wheels: &mut [Wheel], home: &[[f64; 3]], ext: f64) {
    let [dx, dy] = slide_offset(ext);
    for (w, h) in wheels.iter_mut().zip(home) { w.center = [h[0] + dx, h[1] + dy, h[2]]; }
}
/// js `kickRot(ext)`: the kick bar's swing (rad, forward and up from hanging)
pub fn kick_rot(ext: f64) -> f64 {
    let [dx, dy] = slide_offset(ext);
    let (px, py, l) = (kick::PIN[0] + dx, kick::PIN[1] + dy, kick::len());
    let (xb, yc) = (bumper::X1 + kick::R, bumper::Y1 + kick::R);
    if px + l * m::sin(kick::b0()) >= xb { return 0.0; }
    let b1 = m::asin(m::max(-1.0, m::min(1.0, (xb - px) / l)));
    let b2 = m::acos(m::max(-1.0, m::min(1.0, (py - yc) / l)));
    m::min(kick::MAX, m::max(0.0, m::min(b1, b2) - kick::b0()))
}
/// js `hoodRot(deg)`
pub fn hood_rot(deg: f64) -> f64 { deg * DEG }
/// js `hoodElevation(deg)`
pub fn hood_elevation(deg: f64) -> f64 { 90.0 - shooter::EXIT_DEG - deg }

#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct Path { body: PathMesh, body_fine: PathMesh, fixed_hood: PathMesh, slide: PathMesh, kick: PathMesh, hood: PathMesh }

/// pathSurfaces7769's surfaces, in its order (path: up to S_BUMPER; shooter: the two hoods)
pub const S_BODY: usize = 0;
pub const S_FINE: usize = 1;
pub const S_SLIDE: usize = 2;
pub const S_KICK: usize = 3;
pub const S_BELTS: usize = 4;
pub const S_NET: usize = 5;
pub const S_BUMPER: usize = 6;
pub const S_FIXED_HOOD: usize = 7;
pub const S_HOOD: usize = 8;

/// js `pathSurfaces7769(PATH, t)`
pub fn path_surfaces7769(t: &Trains) -> Vec<Surface> {
    static P: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    let p = P.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path7769.json")).expect("path7769.json"));
    let s = |k: &str, mesh: &PathMesh, pos: [f64; 3], mu: f64| {
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{k} (CAD)"), &v, &mesh.i, mu, pos, 0.0, 0.0, 0.08, f64::INFINITY)
    };
    let dir = slide::dir();
    let mut sl = s("slide", &p.slide, [0.0; 3], mu::WALL).on_slide(t.slide, [-dir[0], -dir[1], 0.0]);
    sl.set_slide_k(slide::pinion());
    let mut net = Surface::sheet("net", 0.5, net::TREAD, NetSheet { x0: net::X0, y0: net::Y0, xs: net::XS, x1: 0.0, yf: 0.0, hw: net::HW, bulge: net::BULGE });
    place_net(&mut net, slide::OUT);
    vec![
        s("body", &p.body, [0.0; 3], mu::WALL),
        s("bodyFine", &p.body_fine, [0.0; 3], mu::HOOD),
        sl,
        s("kick", &p.kick, [kick::PIN[0], kick::PIN[1], 0.0], mu::WALL),
        belt_run(t),
        net,
        Surface::block("bumper", bumper::x0(), bumper::X1, bumper::Y0, bumper::Y1, bumper::HW, 0.02, BUMPER_MU, 20000.0),
        s("fixedHood", &p.fixed_hood, [0.0; 3], mu::HOOD),
        s("hood", &p.hood, [AXLE[0], AXLE[1], 0.0], mu::HOOD),
    ]
}
/// js `beltRun(t)`: the belts' top run, its ends rounded over the pulleys, sliding back at the
/// belts' pitch-line speed
fn belt_run(t: &Trains) -> Surface {
    const NZ: usize = 6;
    const NA: usize = 4;
    let ([fx, fy], [bx, by], d, r, hw) = (floor::FRONT, floor::BACK, floor::dir(), floor::R, floor::HW);
    let up = if d[0] < 0.0 { [-d[1], d[0]] } else { [d[1], -d[0]] };
    let mut prof = Vec::new();
    for a in (0..=NA).rev() {
        let th = (a as f64 / NA as f64) * (m::PI / 2.0);
        prof.push([fx - d[0] * r * m::sin(th) + up[0] * r * m::cos(th), fy - d[1] * r * m::sin(th) + up[1] * r * m::cos(th)]);
    }
    for a in 0..=NA {
        let th = (a as f64 / NA as f64) * (m::PI / 2.0);
        prof.push([bx + d[0] * r * m::sin(th) + up[0] * r * m::cos(th), by + d[1] * r * m::sin(th) + up[1] * r * m::cos(th)]);
    }
    let n = prof.len();
    let mut v = Vec::new();
    for k in 0..=NZ { for [x, y] in &prof { v.extend_from_slice(&[*x, *y, -hw + (2.0 * hw * k as f64) / NZ as f64]); } }
    let mut ix = Vec::new();
    for k in 0..NZ { for j in 0..n - 1 {
        let a = (k * n + j) as u32;
        let (b, c) = (a + 1, a + n as u32);
        let e = c + 1;
        ix.extend_from_slice(&[a, c, b, b, c, e]);
    } }
    let mut sf = Surface::mesh("floor belts", &v, &ix, mu::BELT, [0.0; 3], 0.0, 0.0, 0.08, tread::BELT).on_slide(t.belts, d);
    sf.set_slide_k(floor::pitch());
    sf
}
/// js Net `place(ext)`: its front edge where the intake has the hopper's front
fn place_net(s: &mut Surface, ext: f64) {
    let o = slide_offset(ext);
    let n = s.sheet_mut().unwrap();
    n.x1 = FRONT_X + o[0];
    n.yf = net::Y_FRONT + o[1];
}
/// js `placeSlide(s, ext)`
pub fn place_slide(sf: &mut [Surface], ext: f64) {
    let [dx, dy] = slide_offset(ext);
    sf[S_SLIDE].set_pose([dx, dy, 0.0], 0.0, 0.0);
    sf[S_KICK].set_pose([kick::PIN[0] + dx, kick::PIN[1] + dy, 0.0], 0.0, kick_rot(ext));
    place_net(&mut sf[S_NET], ext);
}
