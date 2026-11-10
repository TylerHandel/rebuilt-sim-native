//! js/mech/r1678.js: 1678's Limestone as physical mechanisms, from their CAD and their code. The
//! browser game's file has the why of every number; here are the numbers. Robot frame: x forward
//! (the intake), y up, z to its right.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, Shaft};
use crate::mech::mechanism::{NetDome, Surface, Wheel};
use jsmath as m;

/// js MU
pub mod mu {
    pub const COMPLIANT: f64 = 1.6;
    pub const SILICONE: f64 = 1.0;
    pub const POLYCARB: f64 = 0.5;
}
/// js PIVOT (the intake's pivot tube), DRUM (the drum's axle, the hood turns about it)
pub const PIVOT: [f64; 2] = [0.305, 0.169];
pub const DRUM: [f64; 2] = [-0.288, 0.477];
/// js INTAKE
pub mod intake {
    pub const MAIN_AT: [f64; 2] = [0.143, 0.368];
    pub const MAIN_D: f64 = 0.0643;
    pub const MAIN_MU: f64 = 1.3;
    pub const MAIN_TREAD: f64 = 10000.0;
    pub const KICKER_AT: [f64; 2] = [0.3043, 0.3305];
    pub const KICKER_D: f64 = 0.0326;
    pub const IDLERS: [([f64; 2], f64); 2] = [([0.253, 0.247], 0.0188), ([0.131, 0.329], 0.0188)];
    pub const STOWED: f64 = 130.0;
    pub const CAD: f64 = 132.0;
    pub const DOWN: f64 = 0.0;
    pub const MASS: f64 = 4.2;
    pub const COM: f64 = 0.22;
    pub const COM_DEG: f64 = -1.0;
    pub const J: f64 = 0.222;
}
/// js FLOOR (1in polycarbonate), FLEX (16 2in 30A flex wheels each)
pub const FLOOR: [[f64; 2]; 5] = [[0.323, 0.215], [0.262, 0.208], [0.214, 0.202], [-0.015, 0.126], [-0.075, 0.115]];
pub const FLEX: [[f64; 2]; 3] = [[0.159, 0.179], [0.101, 0.160], [0.042, 0.142]];
/// js BACK (up the back of the tunnel, bottom to top), FRONT, FEEDER, TUNNEL_D
pub const BACK: [[f64; 2]; 6] = [[-0.147, 0.122], [-0.201, 0.154], [-0.247, 0.212], [-0.266, 0.284], [-0.266, 0.346], [-0.266, 0.408]];
pub const FRONT: [[f64; 2]; 2] = [[-0.090, 0.346], [-0.090, 0.408]];
pub const FEEDER_AT: [f64; 2] = [-0.082, 0.294];
pub const FEEDER_D: f64 = 0.0516;
pub const TUNNEL_D: f64 = 0.0302;
/// js SHOOTER
pub mod shooter {
    pub const DRUM_D: f64 = 0.092;
    pub const DRUM_WIDTH: f64 = 0.563;
    pub const HOOD: [[f64; 2]; 3] = [[-0.094, 0.448], [-0.093, 0.490], [-0.101, 0.536]];
    pub const HOOD_D: f64 = 0.0413;
    pub const HOOD_WIDTH: f64 = 0.619;
    pub const HOOD_MIN: f64 = 17.5;
    pub const HOOD_MAX: f64 = 52.5;
    pub const HOOD_TURN: [[f64; 2]; 3] = [[17.5, 0.0], [26.5, 20.0], [52.5, 37.0]];
    pub const DRUM_PER_MOTOR: f64 = 18.0 / 24.0;
    pub const DRUM_J: f64 = 4.85e-3;
    pub const FLYWHEEL_J: f64 = 2.30e-3;
    pub const FLYWHEEL_PER_DRUM: f64 = 50.0 / 15.0;
    pub const HOOD_J: f64 = 5.18e-4;
    pub const MOTOR_J: f64 = 7.9e-6;
    pub fn kv() -> f64 { (12.0 / 100.0) * (24.0 / 18.0) * 0.995 }
    pub const KP: f64 = 0.8;
    pub const KS: f64 = 0.32;
    pub const SLOW: f64 = 2100.0;
}
/// js TUNNEL
pub mod tunnel {
    use jsmath as m;
    pub fn back_per_motor(i: usize) -> f64 { (12.0 / 21.0) * m::pow(21.0 / 19.0, i as f64) }
    pub fn front_per_motor(i: usize) -> f64 { (12.0 / 21.0) * m::pow(21.0 / 19.0, (4 + i) as f64) }
    pub fn kv() -> f64 { (12.0 / 100.0) * ((21.0 / 12.0) * m::pow(19.0 / 21.0, 5.0)) * 1.02 }
    pub const KP: f64 = 1.75;
    pub const KS: f64 = 0.36;
    pub const PRELOAD: f64 = 250.0;
}
/// js `interp(t, x)`
fn interp(t: &[[f64; 2]], x: f64) -> f64 {
    if x <= t[0][0] { return t[0][1]; }
    for k in 1..t.len() {
        if x <= t[k][0] { return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); }
    }
    t[t.len() - 1][1]
}
/// What its shot is for (js REGRESSIONS' keys)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Kind { Hub, Ferry, Far }
/// js `shotFor(kind, d)`'s answer
#[derive(Clone, Copy, Debug)]
pub struct Want { pub rpm: f64, pub hood: f64, pub tunnel: f64 }
fn rows(d: &[f64], v: &[f64]) -> Vec<[f64; 2]> { d.iter().zip(v).map(|(&x, &y)| [x, y]).collect() }
/// js `shotFor(kind, d)` (their distances, hood angles and tunnel speeds; the sim's drum RPMs, SIM_PASS)
pub fn shot_for(kind: Kind, d: f64) -> Want {
    let (dd, hood, tun, sim): (&[f64], &[f64], &[f64], &[f64]) = match kind {
        Kind::Hub => (&[0.0, 1.7, 2.0, 2.215, 2.495, 2.888, 3.222, 3.57, 3.83, 4.15, 4.75],
            &[17.5, 17.5, 18.5, 19.0, 19.5, 20.5, 21.5, 22.5, 23.0, 24.5, 26.5],
            &[1400.0, 1425.0, 1439.6, 1450.0, 1500.0, 1550.0, 1650.0, 1750.0, 1800.0, 1950.0, 2150.0],
            &[1815.0, 1815.0, 1865.0, 1910.0, 1990.0, 2055.0, 2105.0, 2170.0, 2210.0, 2230.0, 2330.0]),
        Kind::Ferry => (&[0.0, 4.977, 5.976, 6.902, 7.912, 8.477, 9.029, 9.59, 10.061],
            &[52.5, 52.5, 51.5, 50.5, 49.5, 48.5, 46.5, 45.5, 43.5],
            &[1600.0, 1650.0, 1700.0, 1750.0, 1850.0, 1900.0, 2000.0, 2200.0, 2350.0],
            &[1935.0, 1935.0, 2175.0, 2385.0, 2600.0, 2720.0, 2825.0, 2940.0, 3055.0]),
        Kind::Far => (&[0.0, 12.46, 13.133], &[45.0, 45.0, 45.0], &[1400.0, 1400.0, 1300.0], &[4200.0, 4200.0, 4200.0]),
    };
    Want { rpm: interp(&rows(dd, sim), d), hood: interp(&rows(dd, hood), d), tunnel: interp(&rows(dd, tun), d) }
}
/// js RANGES: their CANranges
pub const RANGE_TUNNEL: ([f64; 2], f64, f64) = ([-0.178, 0.379], -0.327, 0.6);
pub const RANGE_HOPPER: ([f64; 2], f64, f64, f64) = ([-0.04, 0.45], -0.285, 0.3, 13.5);
/// js PRELOAD: eight on the floor
pub const PRELOAD: [[f64; 3]; 8] = [
    [0.21, 0.30, -0.225], [0.21, 0.30, -0.075], [0.21, 0.30, 0.075], [0.21, 0.30, 0.225],
    [0.05, 0.25, -0.225], [0.05, 0.25, -0.075], [0.05, 0.25, 0.075], [0.05, 0.25, 0.225],
];

/// js build1678()'s answer (drives indices)
pub struct Trains {
    pub intake: usize, pub main: usize, pub kicker: usize, pub idlers: [usize; 2],
    pub floor: usize, pub flex: Vec<usize>, pub rollers: Vec<usize>,
    pub tunnel: usize, pub back: Vec<usize>, pub front: Vec<usize>,
    pub feeder_t: usize, pub feeder: usize,
    pub shooter: usize, pub drum: usize, pub hood: usize,
    pub pivot: usize,
}
/// js `build1678()`
pub fn build1678(d: &mut Drives) -> Trains {
    let intake = d.train("intake roller", Motor::new("krakenX60", 2.0, 1.0, 0.9, 80.0, 50.0));
    let main = d.add(intake, Shaft::new("intake 2in roller", None, 1.6e-4, 2e-4, 0.0), -16.0 / 30.0);
    let kicker = d.shaft(Shaft::new("intake kicker bar (locked)", None, f64::INFINITY, 2e-4, 0.0));
    let idlers = [0, 1].map(|i| d.shaft(Shaft::new(&format!("intake free roller {}", i + 1), None, 1.5e-5, 2e-5, 0.0)));
    let floor = d.train("roller floor", Motor::new("krakenX60", 1.0, 1.0, 0.9, 90.0, 45.0));
    let flex = (0..FLEX.len()).map(|i| d.add(floor, Shaft::new(&format!("flex wheel shaft {}", i + 1), None, 8e-5, 2e-4, 0.0), 12.0 / 36.0)).collect();
    let rollers = (0..FLOOR.len()).map(|i| d.add(floor, Shaft::new(&format!("floor roller {}", i + 1), None, 2e-5, 2e-4, 0.0), (12.0 / 36.0) * (30.0 / 18.0))).collect();
    let tunnel = d.train("tunnel", Motor::new("krakenX60", 2.0, 1.0, 0.9, 60.0, 10.0));
    let back = (0..BACK.len()).map(|i| d.add(tunnel, Shaft::new(&format!("tunnel back roller {}", i + 1), None, 6e-5, 2e-4, 0.0), tunnel::back_per_motor(i))).collect();
    let front = (0..FRONT.len()).map(|i| d.add(tunnel, Shaft::new(&format!("tunnel front roller {}", i + 1), None, 6e-5, 2e-4, 0.0), -tunnel::front_per_motor(i))).collect();
    let feeder_t = d.train("feeder", Motor::new("krakenX60", 1.0, 1.0, 0.9, 50.0, 25.0));
    let feeder = d.add(feeder_t, Shaft::new("feeder 2in roller", None, 2.5e-4, 2e-4, 0.0), -12.0 / 30.0);
    let shooter = d.train("shooter", Motor::new("krakenX60", 4.0, 1.0, 0.9, 80.0, 30.0));
    let drum = d.add(shooter, Shaft::new("drum", None, shooter::DRUM_J, 2e-4, 0.0), shooter::DRUM_PER_MOTOR);
    d.add(shooter, Shaft::new("bronze inertia flywheel (50t:15t)", None, shooter::FLYWHEEL_J, 0.0, 0.0), shooter::DRUM_PER_MOTOR * shooter::FLYWHEEL_PER_DRUM);
    d.add(shooter, Shaft::new("motors' pulleys", None, shooter::MOTOR_J, 0.0, 0.0), 1.0);
    let hood = d.add(shooter, Shaft::new("hood rollers", None, shooter::HOOD_J, 2e-4, 0.0), -shooter::DRUM_PER_MOTOR);
    let pivot = d.shaft(Shaft::new("intake pivot", Some(Motor::new("krakenX60", 1.0, (62.0 / 8.0) * (68.0 / 18.0) * (15.0 / 9.0), 0.9, 60.0, 40.0)), intake::J, 0.5, 0.0));
    Trains { intake, main, kicker, idlers, floor, flex, rollers, tunnel, back, front, feeder_t, feeder, shooter, drum, hood, pivot }
}

/// js wheels1678()'s answer: the floor's, the tunnel's and the intake's (with each intake wheel's
/// home: where the CAD has it), the drum and the hood's rollers (and their homes)
pub struct Wheels { pub floor: Vec<Wheel>, pub tunnel: Vec<Wheel>, pub intake: Vec<Wheel>, pub intake_home: Vec<[f64; 3]>, pub drum: Wheel, pub hood: Vec<Wheel>, pub hood_home: Vec<[f64; 3]> }
/// js `wheels1678(t)`
pub fn wheels1678(d: &Drives, t: &Trains) -> Wheels {
    let z = [0.0, 0.0, 1.0];
    let name = |s: usize| d.shafts[s].name.clone();
    let mut floor: Vec<Wheel> = FLOOR.iter().enumerate().map(|(i, [x, y])| Wheel::new(&name(t.rollers[i]), t.rollers[i], [*x, *y, -0.013], z, 0.0125, 0.529).mu(mu::POLYCARB)).collect();
    floor.extend(FLEX.iter().enumerate().map(|(i, [x, y])| Wheel::new(&format!("{} (2in 30A flex wheels)", name(t.flex[i])), t.flex[i], [*x, *y, 0.0], z, 1.0 * IN, 0.514).mu(mu::COMPLIANT).tread(3000.0)));
    let mut tunnel: Vec<Wheel> = BACK.iter().enumerate().map(|(i, [x, y])| Wheel::new(&name(t.back[i]), t.back[i], [*x, *y, 0.0], z, TUNNEL_D / 2.0, 0.55).mu(mu::SILICONE).tread(20000.0)).collect();
    tunnel.extend(FRONT.iter().enumerate().map(|(i, [x, y])| Wheel::new(&name(t.front[i]), t.front[i], [*x, *y, 0.0], z, TUNNEL_D / 2.0, 0.544).mu(mu::SILICONE).tread(20000.0)));
    tunnel.push(Wheel::new(&name(t.feeder), t.feeder, [FEEDER_AT[0], FEEDER_AT[1], 0.012], z, FEEDER_D / 2.0, 0.564).mu(mu::SILICONE).tread(20000.0));
    let mut intake = vec![
        Wheel::new("intake 2in roller (silicone)", t.main, [intake::MAIN_AT[0], intake::MAIN_AT[1], 0.0], z, intake::MAIN_D / 2.0, 0.618).mu(intake::MAIN_MU).tread(intake::MAIN_TREAD),
        Wheel::new("intake kicker bar (locked)", t.kicker, [intake::KICKER_AT[0], intake::KICKER_AT[1], 0.0], z, intake::KICKER_D / 2.0, 0.66).mu(mu::POLYCARB),
    ];
    for (i, (at, dd)) in intake::IDLERS.iter().enumerate() { intake.push(Wheel::new(&name(t.idlers[i]), t.idlers[i], [at[0], at[1], 0.0], z, dd / 2.0, 0.66).mu(mu::POLYCARB)); }
    let intake_home = intake.iter().map(|w| w.center).collect();
    let drum = Wheel::new("drum (silicone)", t.drum, [DRUM[0], DRUM[1], -0.009], z, shooter::DRUM_D / 2.0, shooter::DRUM_WIDTH).mu(mu::SILICONE);
    let hood: Vec<Wheel> = shooter::HOOD.iter().enumerate().map(|(i, h)| Wheel::new(&format!("hood roller {} (silicone)", i + 1), t.hood, [h[0], h[1], 0.0], z, shooter::HOOD_D / 2.0, shooter::HOOD_WIDTH).mu(mu::SILICONE)).collect();
    let hood_home = hood.iter().map(|w| w.center).collect();
    Wheels { floor, tunnel, intake, intake_home, drum, hood, hood_home }
}
/// js `rot([x, y], [cx, cy], a)`
fn rot([x, y]: [f64; 2], [cx, cy]: [f64; 2], a: f64) -> [f64; 2] {
    let (c, s, dx, dy) = (m::cos(a), m::sin(a), x - cx, y - cy);
    [cx + dx * c - dy * s, cy + dx * s + dy * c]
}
/// js `intakeRot(deg)`: the arm's turn from the CAD's pose at their angle deg (0 down, 130 folded)
pub fn intake_rot(deg: f64) -> f64 { ((deg - intake::CAD) * m::PI) / 180.0 }
/// js `placeIntake(wheels, deg)`
pub fn place_intake(wheels: &mut [Wheel], home: &[[f64; 3]], deg: f64) {
    let a = intake_rot(deg);
    for (w, h) in wheels.iter_mut().zip(home) { let [x, y] = rot([h[0], h[1]], PIVOT, a); w.center = [x, y, 0.0]; }
}
/// js `hoodRot(deg)`
pub fn hood_rot(deg: f64) -> f64 { (interp(&shooter::HOOD_TURN, deg) * m::PI) / 180.0 }
/// js `placeHood(wheels, deg)`
pub fn place_hood(wheels: &mut [Wheel], home: &[[f64; 3]], deg: f64) {
    let a = hood_rot(deg);
    for (w, h) in wheels.iter_mut().zip(home) { let [x, y] = rot([h[0], h[1]], DRUM, a); w.center = [x, y, 0.0]; }
}
/// js `hoodElevation(deg)`
pub fn hood_elevation(deg: f64) -> f64 { 74.0 - interp(&shooter::HOOD_TURN, deg) }

/// Surfaces: their indices in path_surfaces1678's list
pub const S_BODY: usize = 0;
pub const S_INTAKE: usize = 1;
pub const S_HOOD: usize = 2;
pub const S_SLIDE: usize = 3;
pub const S_LID: usize = 4;
pub const S_NET: usize = 5;

#[derive(serde::Deserialize)]
struct PathMesh { v: Vec<f64>, i: Vec<u32> }
#[derive(serde::Deserialize)]
struct Path { body: PathMesh, intake: PathMesh, hood: PathMesh, slide: PathMesh, lid: PathMesh }

/// js NET
pub const NET_LID_X: f64 = 0.335;
pub const NET_LID_Y: f64 = 0.545;
pub const NET_FRONT_X: f64 = 0.335;
pub const NET_FRONT_Y: f64 = 0.505;
/// js `pathSurfaces1678(PATH, t)`: body, intake (on the pivot's shaft), hood, slide, lid, net
pub fn path_surfaces1678(t: &Trains) -> Vec<Surface> {
    static P: std::sync::OnceLock<Path> = std::sync::OnceLock::new();
    let p = P.get_or_init(|| serde_json::from_str(include_str!("../../../../assets/mech/path1678.json")).expect("path1678.json"));
    let s = |k: &str, mesh: &PathMesh, pos: [f64; 3]| {
        let v: Vec<f64> = mesh.v.iter().map(|x| x / 1000.0).collect();
        Surface::mesh(&format!("{k} (CAD)"), &v, &mesh.i, 0.35, pos, 0.0, 0.0, 0.08, f64::INFINITY)
    };
    let mut net = Surface::net("net", 0.3, 1500.0, NetDome { ext: 0.0, x0: 0.0, x1: 0.0, y0: 0.0, y1: 0.0, hw: 0.34, bulge: 0.12 });
    place_net(&mut net, 0.0, 0.0);
    vec![
        s("body", &p.body, [0.0; 3]),
        s("intake", &p.intake, [PIVOT[0], PIVOT[1], 0.0]).on_shaft(t.pivot, [PIVOT[0], PIVOT[1], 0.0], [0.0, 0.0, 1.0]),
        s("hood", &p.hood, [DRUM[0], DRUM[1], 0.0]),
        s("slide", &p.slide, [0.0; 3]),
        s("lid", &p.lid, [0.0; 3]),
        net,
    ]
}
/// js `placeNet(net, ext, lift)`
pub fn place_net(net: &mut Surface, ext: f64, lift: f64) {
    let d = net.net_mut().unwrap();
    d.ext = ext; d.x0 = NET_LID_X; d.x1 = NET_FRONT_X + ext; d.y0 = NET_LID_Y + lift; d.y1 = NET_FRONT_Y;
}
/// js SLOT: the pin on the arm, the slot in the extension's plate, the extension's mass
const SLOT_PIN: [f64; 2] = [0.158, 0.2597];
const SLOT_AT: [f64; 2] = [0.1543, 0.2697];
const SLOT_DIR: [f64; 2] = [-0.2079, 0.9782];
pub const SLOT_MASS: f64 = 2.5;
fn slot_pin(deg: f64) -> [f64; 2] {
    let [px, py] = SLOT_PIN;
    let r = m::hypot2(px - PIVOT[0], py - PIVOT[1]);
    let f = m::atan2(py - PIVOT[1], px - PIVOT[0]) + intake_rot(deg);
    [PIVOT[0] + r * m::cos(f), PIVOT[1] + r * m::sin(f)]
}
fn slot_x(y: f64) -> f64 { SLOT_AT[0] + ((y - SLOT_AT[1]) * SLOT_DIR[0]) / SLOT_DIR[1] }
/// js `slotExt(deg)`: how far out the extension is (m) with the arm at their angle deg
pub fn slot_ext(deg: f64) -> f64 {
    let slot0 = SLOT_PIN[0] - slot_x(SLOT_PIN[1]);
    let [x, y] = slot_pin(deg);
    m::max(0.0, x - slot_x(y) - slot0)
}
/// js `slotRate(deg)`: m of extension per radian of the arm (up)
pub fn slot_rate(deg: f64) -> f64 { ((slot_ext(deg + 0.5) - slot_ext(deg - 0.5)) * 180.0) / m::PI }
