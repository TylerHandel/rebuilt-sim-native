//! js/mech/r581.js: 581's robot as physical mechanisms, from their CAD and their code. Robot frame:
//! x forward (the intake), y up, z to its right.
//!
//! The intake slides out on its racks (its 3in and 2in rollers pulling FUEL in over the bumper);
//! the floor, a row of rollers on a frame pivoting on its powered back shaft, tilts up as the
//! intake comes in (a cam geared off the deploy pinions), carrying the load back to the tower's
//! rollers, which lift it into the drum; the hood's two rollers wrap them over it and out. A net
//! over the hopper leans down to the intake's front.
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{NetLean, Surface, Wheel};
use jsmath as m;

/// js MU (estimates)
pub mod mu {
    pub const GRIP_TAPE: f64 = 1.3;
    pub const FLOOR: f64 = 0.6;
    pub const NET: f64 = 0.25;
}
/// js DRUM, PIVOT, SLIDE
pub const DRUM: [f64; 2] = [-0.2883, 0.4788];
pub const PIVOT: [f64; 2] = [-0.0701, 0.1105];
pub mod slide {
    pub const OUT: f64 = 0.3061;
    pub const DIR: [f64; 2] = [-0.9954, 0.0960];
    pub fn pinion() -> f64 { 0.5 * crate::constants::IN }
    pub const MASS: f64 = 5.5;
}
/// js INTAKE: the 3in and 2in rollers, as exported (out)
pub mod intake {
    pub const BIG: ([f64; 2], f64, f64) = ([0.5794, 0.1781], 0.0397, 0.5525);
    pub const SMALL: ([f64; 2], f64, f64) = ([0.5307, 0.2345], 0.0286, 0.5588);
}
/// js FLOOR
pub mod floor {
    pub const ROLLERS: [[f64; 2]; 11] = [[-0.0701, 0.1105], [-0.0300, 0.1220], [0.0102, 0.1335], [0.0503, 0.1450], [0.0904, 0.1564], [0.1306, 0.1679],
        [0.1707, 0.1794], [0.2123, 0.1823], [0.2540, 0.1852], [0.2938, 0.1975], [0.3237, 0.1684]];
    pub fn r() -> f64 { 0.5 * crate::constants::IN }
    pub const WIDTH: f64 = 0.508;
    pub const CAM: [[f64; 2]; 24] = [[0.0, 0.0], [0.083, 0.0], [0.089, 0.8], [0.101, 1.8], [0.110, 2.4], [0.119, 3.6], [0.129, 5.4], [0.141, 7.2], [0.150, 8.9],
        [0.159, 10.6], [0.171, 12.2], [0.181, 13.8], [0.190, 15.4], [0.199, 16.9], [0.211, 18.4], [0.220, 19.7], [0.230, 21.1],
        [0.239, 22.4], [0.251, 23.7], [0.260, 25.0], [0.269, 26.3], [0.279, 27.6], [0.291, 28.8], [0.3061, 30.6]];
    pub const MASS: f64 = 2.5;
    pub const COM: f64 = 0.2;
}
/// js TOWER
pub mod tower {
    pub const ROLLERS: [[f64; 2]; 3] = [[-0.1142, 0.2789], [-0.1027, 0.3454], [-0.0911, 0.4119]];
    pub const R: f64 = 0.019;
    pub const WIDTH: f64 = 0.553;
}
/// js SHOOTER
pub mod shooter {
    pub const DRUM_R: f64 = 0.051;
    pub const DRUM_W: f64 = 0.553;
    pub const DRUM_Z: f64 = -0.0105;
    pub const HOOD: [[f64; 2]; 2] = [[-0.0820, 0.4615], [-0.0865, 0.5245]];
    pub const HOOD_R: f64 = 0.0254;
    pub const HOOD_W: f64 = 0.552;
    pub fn hood_per_drum() -> f64 { (18.0 / 15.0) * (30.0 / 22.0) }
    pub const FLYWHEEL_PER_DRUM: f64 = 45.0 / 15.0;
    pub const DRUM_J: f64 = 2.21e-3;
    pub const FLYWHEEL_J: f64 = 7.88e-4;
    pub const HOOD_J: f64 = 5.11e-4;
    pub const HOOD_CAD: f64 = 11.0;
    pub const HOOD_MIN: f64 = 13.0;
    pub const HOOD_MAX: f64 = 45.0;
    pub const HOOD_IDLE: f64 = 13.0;
    pub const HOOD_TURN: [[f64; 2]; 2] = [[11.0, 0.0], [45.0, 34.0]];
    pub const KS: f64 = 6.4;
    pub const KV: f64 = 0.13;
    pub const KP: f64 = 14.0;
    pub const IDLE: f64 = 750.0;
    pub const RPM_TOL: f64 = 50.0;
    pub const RPM_TOL_SHOOTING: f64 = 300.0;
    pub const RPM_TOL_FEEDING: f64 = 500.0;
}
fn interp(t: &[[f64; 2]], x: f64) -> f64 {
    if x <= t[0][0] { return t[0][1]; }
    for k in 1..t.len() {
        if x <= t[k][0] { return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); }
    }
    t[t.len() - 1][1]
}
/// js SCORE_HOOD, FEED_HOOD, FEEDER_FF, SIM_SCORE_RPM, SIM_FEED_RPM
const SCORE_HOOD: [[f64; 2]; 4] = [[1.42, 12.5], [2.38, 20.0], [3.46, 29.0], [4.92, 33.0]];
const FEED_HOOD: [[f64; 2]; 3] = [[6.0, 35.0], [8.71, 40.0], [13.6, 40.0]];
const FEEDER_FF: [[f64; 2]; 4] = [[0.0, 0.0], [20.0, 0.0], [30.0, 15.0], [50.0, 22.0]];
const SIM_SCORE_RPM: [[f64; 2]; 4] = [[1.42, 1240.0], [2.79, 1440.0], [3.46, 1510.0], [4.92, 1760.0]];
const SIM_FEED_RPM: [[f64; 2]; 3] = [[6.0, 1730.0], [8.71, 2500.0], [13.6, 2500.0]];
/// js `clampHood(deg)`
pub fn clamp_hood(deg: f64) -> f64 { m::max(shooter::HOOD_MIN, m::min(shooter::HOOD_MAX, deg)) }
/// js `shotFor(kind, d)`: (rpm, hood)
pub fn shot_for(feed: bool, d: f64) -> (f64, f64) {
    (m::min(6000.0, interp(if feed { &SIM_FEED_RPM } else { &SIM_SCORE_RPM }, d)), clamp_hood(interp(if feed { &FEED_HOOD } else { &SCORE_HOOD }, d)))
}
/// js `feederFF(amps)`
pub fn feeder_ff(amps: f64) -> f64 { interp(&FEEDER_FF, amps) }
/// js DEPLOY (their inches)
pub mod deploy {
    pub const INTAKE: f64 = 11.8;
    pub const SCORE_COMPACTION_WAITING: f64 = 11.8 - 3.5;
    pub const SCORE_COMPACTION: f64 = 1.0;
    pub const FEED_COMPACTION: f64 = 6.915;
    pub const BEAST_MODE_COMPACTION: f64 = 5.0;
    pub const MIN: f64 = 5.0;
    pub const MAX: f64 = 11.8;
    pub const KP: f64 = 8.0;
    pub const MM_V: f64 = 200.0;
    pub const MM_A: f64 = 50.0;
    pub const HOME_V: f64 = 4.0;
    pub const TOL: f64 = 0.25;
}
/// js RANGES
pub mod ranges {
    use crate::constants::IN;
    pub const HOPPER_AT: [f64; 2] = [-0.014, 0.418];
    pub const HOPPER_Z: f64 = 0.289;
    pub const HOPPER_REACH: f64 = 0.6;
    pub const HOPPER_HALF: f64 = 3.4;
    pub fn high() -> f64 { 6.0 * IN }
    pub fn medium() -> f64 { 8.0 * IN }
    pub const TOWER_AT: [f64; 2] = [-0.19, 0.31];
}
/// js PRELOAD
pub const PRELOAD: [[f64; 3]; 8] = [
    [0.0, 0.33, -0.225], [0.0, 0.33, -0.075], [0.0, 0.33, 0.075], [0.0, 0.33, 0.225],
    [0.16, 0.42, -0.225], [0.16, 0.42, -0.075], [0.16, 0.42, 0.075], [0.16, 0.42, 0.225],
];

/// js build581()'s answer (and the floor's cam: a shaft geared off the deploy pinions)
pub struct Trains { pub intake: usize, pub small: usize, pub big: usize, pub floor: usize, pub rollers: Vec<usize>, pub feeder_t: usize, pub tower: Vec<usize>, pub shooter: usize, pub drum: usize, pub hood: usize, pub deploy: usize, pub cam: usize }
/// js `build581()`
pub fn build581(d: &mut Drives) -> Trains {
    let sh = |name: &str, j: f64| Shaft::new(name, None, j, 2e-4, 0.0);
    let intake = d.train("intake rollers", Motor::new("krakenX60", 2.0, 1.0, 0.9, 75.0, 25.0));
    let small = d.add(intake, sh("intake 2in roller", 1.2e-4), -12.0 / 20.0);
    let big = d.add(intake, sh("intake 3in roller", 4.5e-4), -(12.0 / 20.0) * (20.0 / 30.0));
    let floor = d.train("conveyor (floor)", Motor::new("krakenX60", 2.0, 1.0, 0.9, 50.0, 10.0));
    let rollers = (0..floor::ROLLERS.len()).map(|i| d.add(floor, sh(&format!("floor roller {}", i + 1), 2.5e-5), 12.0 / 18.0)).collect();
    let feeder_t = d.train("feeder (tower)", Motor::new("krakenX60", 2.0, 1.0, 0.9, 50.0, 10.0));
    let tower = (0..tower::ROLLERS.len()).map(|i| d.add(feeder_t, sh(&format!("tower 1.5in roller {}", i + 1), 9e-5), -12.0 / 18.0)).collect();
    let shooter = d.train("shooter", Motor::new("krakenX60", 4.0, 1.0, 0.9, 150.0, 50.0));
    let drum = d.add(shooter, sh("drum", shooter::DRUM_J), 1.0);
    d.add(shooter, sh("inertia flywheel", shooter::FLYWHEEL_J), shooter::FLYWHEEL_PER_DRUM);
    let hood = d.add(shooter, sh("hood rollers", shooter::HOOD_J), -shooter::hood_per_drum());
    let deploy = d.shaft(Shaft::new("deploy pinions", Some(Motor::new("krakenX44", 2.0, 40.0 / 8.0, 0.9, 50.0, 18.0)), slide::MASS * m::pow(slide::pinion(), 2.0), 0.05, 0.0));
    let mut c = Shaft::new("floor cam", None, f64::INFINITY, 0.0, 0.0);
    c.cam = Some((deploy, 0.0));
    let cam = d.shaft(c);
    Trains { intake, small, big, floor, rollers, feeder_t, tower, shooter, drum, hood, deploy, cam }
}

/// js wheels581()'s answer
pub struct Wheels { pub floor: Vec<Wheel>, pub floor_home: Vec<[f64; 3]>, pub tower: Vec<Wheel>, pub intake: Vec<Wheel>, pub intake_home: Vec<[f64; 3]>, pub drum: Wheel, pub hood: Vec<Wheel>, pub hood_home: Vec<[f64; 3]> }
/// js `wheels581(t)`
pub fn wheels581(t: &Trains, d: &Drives) -> Wheels {
    let z = [0.0, 0.0, 1.0];
    let name = |s: usize| d.shafts[s].name.clone();
    let floor: Vec<Wheel> = floor::ROLLERS.iter().enumerate().map(|(i, [x, y])| Wheel::new(&name(t.rollers[i]), t.rollers[i], [*x, *y, -0.003], z, floor::r(), floor::WIDTH).mu(mu::FLOOR).tread(40000.0)).collect();
    let floor_home = floor.iter().map(|w| w.center).collect();
    let tower = tower::ROLLERS.iter().enumerate().map(|(i, [x, y])| Wheel::new(&name(t.tower[i]), t.tower[i], [*x, *y, -0.0025], z, tower::R, tower::WIDTH).mu(mu::GRIP_TAPE).tread(40000.0)).collect();
    let (b, s) = (intake::BIG, intake::SMALL);
    let intake = vec![
        Wheel::new("intake 3in roller (grip tape)", t.big, [b.0[0], b.0[1], -0.0025], z, b.1, b.2).mu(mu::GRIP_TAPE).tread(40000.0),
        Wheel::new("intake 2in roller (grip tape)", t.small, [s.0[0], s.0[1], -0.0025], z, s.1, s.2).mu(mu::GRIP_TAPE).tread(40000.0),
    ];
    let intake_home = intake.iter().map(|w| w.center).collect();
    let drum = Wheel::new("drum (grip tape)", t.drum, [DRUM[0], DRUM[1], shooter::DRUM_Z], z, shooter::DRUM_R, shooter::DRUM_W).mu(mu::GRIP_TAPE).tread(40000.0);
    let hood: Vec<Wheel> = shooter::HOOD.iter().enumerate().map(|(i, h)| Wheel::new(&format!("hood 2in roller {} (grip tape)", i + 1), t.hood, [h[0], h[1], -0.001], z, shooter::HOOD_R, shooter::HOOD_W).mu(mu::GRIP_TAPE).tread(40000.0)).collect();
    let hood_home = hood.iter().map(|w| w.center).collect();
    Wheels { floor, floor_home, tower, intake, intake_home, drum, hood, hood_home }
}
/// js `rot(p, c, a)`
fn rot([x, y]: [f64; 2], [cx, cy]: [f64; 2], a: f64) -> [f64; 2] {
    let (c, s, dx, dy) = (m::cos(a), m::sin(a), x - cx, y - cy);
    [cx + dx * c - dy * s, cy + dx * s + dy * c]
}
/// js `slideOffset(ext)`, `placeIntake(wheels, ext)`
pub fn slide_offset(ext: f64) -> [f64; 2] { let d = slide::OUT - ext; [slide::DIR[0] * d, slide::DIR[1] * d] }
pub fn place_intake(wheels: &mut [Wheel], home: &[[f64; 3]], ext: f64) {
    let [dx, dy] = slide_offset(ext);
    for (w, h) in wheels.iter_mut().zip(home) { w.center = [h[0] + dx, h[1] + dy, h[2]]; }
}
/// js `floorRot(ext)`, `floorRate(ext)`, `placeFloor(wheels, ext)`
pub fn floor_rot(ext: f64) -> f64 { (interp(&floor::CAM, slide::OUT - ext) * m::PI) / 180.0 }
pub fn floor_rate(ext: f64) -> f64 { (floor_rot(ext - 0.0025) - floor_rot(ext + 0.0025)) / 0.005 }
pub fn place_floor(wheels: &mut [Wheel], home: &[[f64; 3]], ext: f64) {
    let a = floor_rot(ext);
    for (w, h) in wheels.iter_mut().zip(home) { let [x, y] = rot([h[0], h[1]], PIVOT, a); w.center = [x, y, h[2]]; }
}
/// js `hoodRot(deg)`, `placeHood(wheels, deg)`, `hoodElevation(deg)`
pub fn hood_rot(deg: f64) -> f64 { (interp(&shooter::HOOD_TURN, deg) * m::PI) / 180.0 }
pub fn place_hood(wheels: &mut [Wheel], home: &[[f64; 3]], deg: f64) {
    let a = hood_rot(deg);
    for (w, h) in wheels.iter_mut().zip(home) { let [x, y] = rot([h[0], h[1]], DRUM, a); w.center = [x, y, h[2]]; }
}
pub fn hood_elevation(deg: f64) -> f64 {
    let [hx, hy] = shooter::HOOD[1];
    90.0 - (m::atan2(hy - DRUM[1], hx - DRUM[0]) * 180.0) / m::PI - interp(&shooter::HOOD_TURN, deg)
}

#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
struct Path { body: PathMesh, slide: PathMesh, floor: PathMesh, hood: PathMesh }

/// pathSurfaces581's surfaces, in its order
pub const S_BODY: usize = 0;
pub const S_SLIDE: usize = 1;
pub const S_FLOOR: usize = 2;
pub const S_HOOD: usize = 3;
pub const S_NET: usize = 4;
/// js NET
pub mod net {
    pub const TOP_X: f64 = 0.302;
    pub const TOP_Y: f64 = 0.55;
    pub const FRONT_X: f64 = 0.643;
    pub const FRONT_Y: f64 = 0.49;
    pub const HW: f64 = 0.34;
    pub const BULGE: f64 = 0.05;
    pub const TREAD: f64 = 8000.0;
}
/// js `netFront(ext)`
pub fn net_front(ext: f64) -> [f64; 2] { let [dx, dy] = slide_offset(ext); [net::FRONT_X + dx, net::FRONT_Y + dy] }

/// js `pathSurfaces581(mesh, t)`
pub fn path_surfaces581(t: &Trains) -> Vec<Surface> {
    static P: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    let p = P.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path581.json")).expect("path581.json"));
    let s = |k: &str, mesh: &PathMesh, pos: [f64; 3]| {
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{k} (CAD)"), &v, &mesh.i, 0.35, pos, 0.0, 0.0, 0.08, f64::INFINITY)
    };
    let piv = [PIVOT[0], PIVOT[1], 0.0];
    let mut sl = s("slide", &p.slide, [0.0; 3]).on_slide(t.deploy, [-slide::DIR[0], -slide::DIR[1], 0.0]);
    sl.set_slide_k(slide::pinion());
    let [fx, fy] = net_front(slide::OUT);
    vec![
        s("body", &p.body, [0.0; 3]),
        sl,
        s("floor", &p.floor, piv).on_shaft(t.cam, piv, [0.0, 0.0, 1.0]),
        s("hood", &p.hood, [DRUM[0], DRUM[1], 0.0]),
        Surface::lean("net", mu::NET, net::TREAD, NetLean { top_x: net::TOP_X, top_y: net::TOP_Y, fx, fy, hw: net::HW, bulge: net::BULGE }),
    ]
}
/// js `placeFloorSurface(sf, ext)`: the floor tilted, and its cam's ratio
pub fn place_floor_surface(sf: &mut [Surface], d: &mut Drives, cam: usize, ext: f64) {
    sf[S_FLOOR].set_pose([PIVOT[0], PIVOT[1], 0.0], 0.0, floor_rot(ext));
    if let Some(c) = &mut d.shafts[cam].cam { c.1 = -floor_rate(ext) * slide::pinion(); }
}
/// js `placeNet(net, ext)`
pub fn place_net(sf: &mut [Surface], ext: f64) {
    let [fx, fy] = net_front(ext);
    let n = sf[S_NET].lean_mut().unwrap();
    n.fx = fx; n.fy = fy;
}
