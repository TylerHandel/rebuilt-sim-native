//! js/localize.js: what a robot's code knows of where it is, the pose estimate every aim works from.
//! From the simulated hardware, each step: odometry (the swerve modules' own wheel speeds and angles,
//! through a wheel radius and module offsets off by their calibration), a gyro (bias drift, random
//! walk, scale error, noise), each team's cameras seeing the field's AprilTags (in view, facing them,
//! big enough, not hidden behind a robot or a HUB) with pixel noise, blur, a rolling shutter's skew
//! and calibration error, solved the way that team's pipeline does (MegaTag2, PnP, 2910's tag
//! distance), reaching the robot code a latency later, and fused as its code does (WPILib's /
//! AdvantageKit's Kalman blend at the frame's timestamp, or 2910's reset). The aim reads it a loop
//! later. Each robot draws from its own seeded generator, never the match's.
//!
//! The why of each number is in the JS file; this is the same numbers, operation for operation.
use crate::constants::*;
use crate::drivetrain::SwerveDrive;
use crate::rng::Mulberry32;
use jsmath as m;

const DEG: f64 = m::PI / 180.0;
fn wrap(a: f64) -> f64 { m::atan2(m::sin(a), m::cos(a)) }
const LOOP: f64 = 0.02;
const AIR_T: f64 = 0.3;
const SPIN_T: f64 = 0.25;
const ACT_DELAY: usize = 2;

/// js `hash(...xs)`: FNV-1a over each value's String() form
pub fn hash(xs: &[&str]) -> u32 {
    let mut h: u32 = 2166136261;
    for x in xs { for c in x.encode_utf16() { h = (h ^ c as u32).wrapping_mul(16777619); } }
    h
}

#[derive(Clone, Copy, Debug)]
struct Tag { id: u32, x: f64, y: f64, h: f64, th: f64 }
fn tags() -> Vec<Tag> {
    APRILTAGS.iter().map(|t| Tag { id: t[0] as u32, x: t[1] - HALF_L, y: t[2] - HALF_W, h: t[3], th: t[4] * DEG }).collect()
}

/// js CAMS
#[derive(Clone, Copy, Debug)]
struct Cam { f: f64, hfov: f64, vfov: f64, fps: f64, lat: f64, px: f64, exp: f64, rs: f64, min_px: f64 }
fn cam(name: &str) -> Cam {
    match name {
        "ll4" => Cam { f: 736.0, hfov: 82.0, vfov: 56.2, fps: 50.0, lat: 0.028, px: 0.25, exp: 0.0012, rs: 0.0, min_px: 14.0 },
        "ll3" => Cam { f: 1050.0, hfov: 62.5, vfov: 48.9, fps: 22.0, lat: 0.045, px: 0.4, exp: 0.0015, rs: 0.03, min_px: 16.0 },
        "ns1800" => Cam { f: 1173.0, hfov: 75.0, vfov: 54.0, fps: 50.0, lat: 0.035, px: 0.2, exp: 0.0018, rs: 0.0, min_px: 14.0 },
        "ns1280" => Cam { f: 640.0, hfov: 90.0, vfov: 64.0, fps: 50.0, lat: 0.035, px: 0.2, exp: 0.0018, rs: 0.0, min_px: 14.0 },
        "ov9281" => Cam { f: 917.0, hfov: 70.0, vfov: 47.0, fps: 40.0, lat: 0.032, px: 0.25, exp: 0.0015, rs: 0.0, min_px: 14.0 },
        "ov800" => Cam { f: 460.0, hfov: 82.0, vfov: 66.0, fps: 40.0, lat: 0.032, px: 0.3, exp: 0.0015, rs: 0.0, min_px: 12.0 },
        _ => panic!("no camera {name}"),
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Mode { Mt2, Pnp, TagDist, Pnp1h }
#[derive(Clone, Copy, Debug, PartialEq)]
enum Fuse { Kalman, Reset }
/// js a profile's std(d, n): [xy, theta] for a vision pose
#[derive(Clone, Copy, Debug)]
enum Std { None, LlDocs, D2 { xy: f64, th: Option<f64> }, Linear(f64), Pow08(f64), Pb }

/// One camera on the robot (js profile cams[]): x forward, y left, h up, yaw, pitch (deg)
#[derive(Clone, Copy, Debug)]
struct CamMount { cam: &'static str, turret: Option<usize>, x: f64, y: f64, h: f64, yaw: f64, pitch: f64, mode: Option<Mode>, max_tag: Option<f64> }
fn mount(cam: &'static str, x: f64, y: f64, h: f64, yaw: f64, pitch: f64) -> CamMount { CamMount { cam, turret: None, x, y, h, yaw, pitch, mode: None, max_tag: None } }

#[derive(Clone, Debug)]
struct Profile {
    cams: Vec<CamMount>, mode: Mode, fuse: Fuse, q: [f64; 3], std: Std,
    odo_tilt: bool, max_amb: Option<f64>, min_tags: Option<usize>, pitch_gate: Option<f64>, max_omega: Option<f64>, max_dist: Option<f64>, ignore: Vec<u32>,
}
const CTRE_Q: [f64; 3] = [0.1, 0.1, 0.1];
const AK_Q: [f64; 3] = [0.003, 0.003, 0.002];

impl Profile {
    fn new(cams: Vec<CamMount>, mode: Mode, fuse: Fuse, q: [f64; 3], std: Std) -> Self {
        Profile { cams, mode, fuse, q, std, odo_tilt: false, max_amb: None, min_tags: None, pitch_gate: None, max_omega: None, max_dist: None, ignore: Vec::new() }
    }
    fn std(&self, d: f64, n: usize) -> [f64; 2] {
        let nf = n as f64;
        match self.std {
            Std::None => [f64::INFINITY, f64::INFINITY],
            Std::LlDocs => [0.7, 9999999.0],
            Std::D2 { xy, th } => [xy * d * d / (nf * nf), match th { Some(t) if n > 1 => t * d * d / (nf * nf), _ => f64::INFINITY }],
            Std::Linear(k) => [k * d, f64::INFINITY],
            Std::Pow08(k) => [k * m::pow(d, 0.8), f64::INFINITY],
            Std::Pb => { let k = 1.0 + d * d / 30.0; if n > 1 { [0.5 * k, 1.0 * k] } else { [4.0 * k, 8.0 * k] } }
        }
    }
}

/// js `profileFor(cfg)`: each team's localization (or the common 2026 setup)
fn profile_for(cfg: &serde_json::Value) -> Profile {
    let key = cfg.get("physical").and_then(|x| x.as_str()).or_else(|| cfg["key"].as_str()).unwrap_or("");
    let t = |c: CamMount, i: usize| CamMount { turret: Some(i), ..c };
    match key {
        "2910" => Profile::new(vec![mount("ll3", -0.34, 0.0, 0.39, 180.0, 15.0)], Mode::TagDist, Fuse::Reset, CTRE_Q, Std::None),
        "6328" => {
            let mut p = Profile::new(vec![mount("ns1800", -0.324, 0.0, 0.351, 180.0, 22.5), mount("ns1280", -0.171, 0.317, 0.435, 70.0, 12.0), mount("ns1280", -0.158, -0.317, 0.435, -70.0, 12.0)],
                Mode::Pnp, Fuse::Kalman, AK_Q, Std::D2 { xy: 0.01, th: Some(0.03) });
            p.odo_tilt = true; p.max_amb = Some(0.4); p
        }
        "341" => { let mut p = Profile::new(vec![t(mount("ll4", 0.21, 0.0, 0.62, 0.0, 28.0), 0)], Mode::Pnp, Fuse::Kalman, AK_Q, Std::D2 { xy: 0.025, th: None }); p.min_tags = Some(2); p }
        "1678" => {
            let mut c0 = mount("ll4", -0.328, 0.0, 0.36, 180.0, 20.0); c0.mode = Some(Mode::Mt2);
            let mut c1 = mount("ov800", -0.064, 0.342, 0.334, 90.0, 0.0); c1.mode = Some(Mode::Pnp1h); c1.max_tag = Some(5.5);
            let mut c2 = mount("ov800", -0.064, -0.342, 0.334, -90.0, 0.0); c2.mode = Some(Mode::Pnp1h); c2.max_tag = Some(5.5);
            let mut p = Profile::new(vec![c0, c1, c2], Mode::Mt2, Fuse::Kalman, CTRE_Q, Std::Linear(0.3)); p.pitch_gate = Some(3.0); p
        }
        "8793" => {
            let mut p = Profile::new(vec![mount("ov9281", -0.305, 0.178, 0.340, 135.0, 11.0), mount("ov9281", -0.305, -0.178, 0.340, -135.0, 11.0), mount("ov9281", 0.165, -0.314, 0.381, -45.0, 11.0), mount("ov9281", 0.165, 0.314, 0.381, 45.0, 11.0)],
                Mode::Pnp, Fuse::Kalman, CTRE_Q, Std::Pb);
            p.max_amb = Some(0.2); p.max_dist = Some(6.0); p.ignore = vec![13, 14, 15, 16, 29, 30, 31, 32]; p
        }
        "581" => { let mut p = Profile::new(vec![mount("ll4", -0.321, 0.0, 0.410, 180.0, 20.0), mount("ll4", -0.191, 0.353, 0.422, 90.0, 15.0), mount("ll4", -0.191, -0.353, 0.422, -90.0, 15.0)], Mode::Mt2, Fuse::Kalman, CTRE_Q, Std::Pow08(0.01)); p.max_omega = Some(100.0); p }
        "604" => {
            let mut p = Profile::new(vec![mount("ov9281", -0.216, 0.289, 0.660, 90.0, 0.0), mount("ov9281", -0.330, 0.025, 0.648, 150.0, 0.0), mount("ov9281", -0.330, -0.025, 0.648, -150.0, 0.0), mount("ov9281", -0.216, -0.289, 0.660, -90.0, 0.0)],
                Mode::Pnp, Fuse::Kalman, AK_Q, Std::D2 { xy: 0.01, th: Some(0.03) });
            p.max_amb = Some(0.3); p
        }
        "4414" => Profile::new(vec![t(mount("ll4", 0.15, 0.0, 0.62, 0.0, 15.0), 0)], Mode::Mt2, Fuse::Kalman, CTRE_Q, Std::LlDocs),
        "1690" => Profile::new(vec![t(mount("ll4", 0.1, 0.0, 0.75, 0.0, 17.0), 0)], Mode::Mt2, Fuse::Kalman, CTRE_Q, Std::LlDocs),
        "2056" => Profile::new(vec![t(mount("ll4", 0.1, 0.0, 0.85, 0.0, 14.0), 0)], Mode::Mt2, Fuse::Kalman, CTRE_Q, Std::LlDocs),
        _ => {
            let sh = &cfg["shooter"];
            let back = sh["type"].as_str() != Some("fixed") || sh["facing"].as_str() == Some("back");
            let l = cfg["frame"]["length"].as_f64().unwrap() / 2.0 - 0.05;
            Profile::new(vec![mount("ll4", if back { -l } else { l }, 0.0, 0.4, if back { 180.0 } else { 0.0 }, 20.0), mount("ll4", if back { l } else { -l }, 0.0, 0.4, if back { 0.0 } else { 180.0 }, 20.0)],
                Mode::Mt2, Fuse::Kalman, CTRE_Q, Std::LlDocs)
        }
    }
}

/// js SIGMA
mod sigma {
    use super::DEG;
    pub fn mount_yaw() -> f64 { 0.25 * DEG }
    pub const SCALE: f64 = 0.005;
    pub const TAG: f64 = 0.005;
    pub fn turret_ang() -> f64 { 0.3 * DEG }
    pub const TURRET_T: f64 = 0.003;
    pub const TS_JITTER: f64 = 0.002;
    pub const LAT_JITTER: f64 = 0.004;
    pub const WHEEL_R: f64 = 0.005;
    pub fn mod_angle() -> f64 { 0.3 * DEG }
    pub const WHEEL_V: f64 = 0.02;
    pub fn gyro_bias() -> f64 { (0.1 * DEG) / 60.0 }
    pub fn gyro_walk() -> f64 { (0.05 * DEG) / jsmath::sqrt(60.0) }
    pub const GYRO_SCALE: f64 = 0.001;
    pub fn gyro_noise() -> f64 { 0.01 * DEG }
    pub const START: f64 = 0.02;
    pub fn start_yaw() -> f64 { 0.5 * DEG }
}

/// A 2D pose (js { X, Y, th }: X world x, Y = -world z, headings CCW from +X)
#[derive(Clone, Copy, Debug, Default)]
pub struct P2 { pub x: f64, pub y: f64, pub th: f64 }
fn compose(a: P2, b: P2) -> P2 {
    P2 { x: a.x + m::cos(a.th) * b.x - m::sin(a.th) * b.y, y: a.y + m::sin(a.th) * b.x + m::cos(a.th) * b.y, th: wrap(a.th + b.th) }
}
fn between(a: P2, b: P2) -> P2 {
    let (dx, dy, c, s) = (b.x - a.x, b.y - a.y, m::cos(a.th), m::sin(a.th));
    P2 { x: c * dx + s * dy, y: -s * dx + c * dy, th: wrap(b.th - a.th) }
}

#[derive(Clone, Copy, Debug)]
struct CamCal { yaw: f64, scale: f64, turret: f64, phase: f64 }
#[derive(Clone, Debug)]
struct Cal { cams: Vec<CamCal>, tags: Vec<[f64; 2]>, wheel_r: f64, md: Vec<f64>, gyro_bias: f64, gyro_scale: f64 }

#[derive(Clone, Copy, Debug)]
struct Obs { k: usize, d: f64, b: f64, r: f64, sb: f64, sr: f64, phi: f64, sphi: f64, amb: f64 }
#[derive(Clone, Copy, Debug)]
struct Meas { stamp: f64, due: f64, pose: P2, n: usize, d: f64 }
#[derive(Clone, Copy, Debug)]
struct Hist { x: f64, y: f64, th: f64, f: f64, l: f64, w: f64 }

/// What a step reads of its robot (js this.r) and the others (for hiding tags)
pub struct RobotView<'a> {
    pub pos: [f64; 3],
    pub quat: [f64; 4],
    pub yaw: f64,
    pub vel: [f64; 3],
    pub omega: f64,
    pub swerve: &'a SwerveDrive,
    /// js r.ready && r.wantShoot (2910's latch)
    pub ready_shoot: bool,
    /// the other robots: x, z, halfL, halfW
    pub others: &'a [[f64; 4]],
    /// its turrets (x, z: robot frame) and their yaws (rad)
    pub turrets: &'a [[f64; 2]],
    pub turret_yaws: &'a [f64],
}

/// js `Localizer`
#[derive(Clone, Debug)]
pub struct Localizer {
    pub truth: bool,
    prof: Profile,
    pub ready: bool,
    rng: Option<Mulberry32>,
    tags: Vec<Tag>,
    cal: Option<Cal>,
    est: P2,
    odo: P2,
    gyro_err: f64,
    last_yaw: f64,
    gyro_prev: Option<f64>,
    stall_t: Option<f64>,
    buf: Vec<(f64, P2)>,
    hist: Vec<Hist>,
    pending: Vec<Meas>,
    cam_t: Vec<f64>,
    v: [f64; 3],
    latched: bool,
    air: Vec<f64>,
    last_turret: Vec<f64>,
    t: Option<f64>,
    pub frames: u32,
    pub used: u32,
}

impl Localizer {
    /// js `new Localizer(robot)`: aim_truth (AIMTRUTH=1): the aims see the truth
    pub fn new(cfg: &serde_json::Value, aim_truth: bool) -> Self {
        Localizer {
            truth: aim_truth, prof: profile_for(cfg), ready: false, rng: None, tags: tags(), cal: None,
            est: P2::default(), odo: P2::default(), gyro_err: 0.0, last_yaw: 0.0, gyro_prev: None, stall_t: None,
            buf: Vec::new(), hist: Vec::new(), pending: Vec::new(), cam_t: Vec::new(), v: [0.0; 3], latched: false, air: Vec::new(), last_turret: Vec::new(), t: None,
            frames: 0, used: 0,
        }
    }
    fn rnd(&mut self) -> f64 { self.rng.as_mut().unwrap().random() }
    /// js `gauss()` (its own: one draw for v)
    fn gauss(&mut self) -> f64 {
        let mut u = 0.0;
        while u == 0.0 { u = self.rnd(); }
        let v = self.rnd();
        m::sqrt(-2.0 * m::log(u)) * m::cos(2.0 * m::PI * v)
    }

    /// js `init(robots, slot)`: its seeded generator (the match's seed, its slot, team and ALLIANCE)
    /// and its calibration; then reset at r's pose
    pub fn init(&mut self, seed: &str, slot: usize, key: &str, alliance: Alliance, r: &RobotView) {
        let a = if alliance == Alliance::Blue { "blue" } else { "red" };
        self.rng = Some(Mulberry32::new(hash(&[seed, &slot.to_string(), key, a])));
        let ncams = self.prof.cams.len();
        let mut cams = Vec::new();
        for _ in 0..ncams {
            let yaw = self.gauss() * sigma::mount_yaw();
            let scale = self.gauss() * sigma::SCALE;
            let turret = self.gauss() * sigma::turret_ang();
            let phase = self.rnd();
            cams.push(CamCal { yaw, scale, turret, phase });
        }
        let mut tg = Vec::new();
        for _ in 0..self.tags.len() { let x = self.gauss() * sigma::TAG; let y = self.gauss() * sigma::TAG; tg.push([x, y]); }
        let wheel_r = 1.0 + self.gauss() * sigma::WHEEL_R;
        let md = (0..r.swerve.modules.len()).map(|_| self.gauss() * sigma::mod_angle()).collect();
        let gyro_bias = self.gauss() * sigma::gyro_bias();
        let gyro_scale = self.gauss() * sigma::GYRO_SCALE;
        let _loop = self.rnd() * LOOP;
        self.cal = Some(Cal { cams, tags: tg, wheel_r, md, gyro_bias, gyro_scale });
        self.reset(r);
        self.ready = true;
    }

    fn truth_of(r: &RobotView) -> P2 { P2 { x: r.pos[0], y: -r.pos[2], th: r.yaw } }

    /// js `reset()`: told its start pose, to its placement's tolerance
    pub fn reset(&mut self, r: &RobotView) {
        if self.rng.is_none() { return; }
        let t = Self::truth_of(r);
        let (gx, gy, gt) = (self.gauss(), self.gauss(), self.gauss());
        self.est = P2 { x: t.x + gx * sigma::START, y: t.y + gy * sigma::START, th: wrap(t.th + gt * sigma::start_yaw()) };
        self.odo = self.est;
        self.gyro_err = self.est.th - t.th;
        self.last_yaw = t.th;
        self.buf.clear();
        self.hist.clear();
        self.pending.clear();
        let cal = self.cal.as_ref().unwrap();
        self.cam_t = self.prof.cams.iter().enumerate().map(|(i, c)| cal.cams[i].phase / cam(c.cam).fps).collect();
        self.v = [0.0; 3];
        self.latched = false;
        self.air = vec![0.0; r.swerve.modules.len()];
        self.last_turret = r.turret_yaws.to_vec();
        self.t = None;
        self.frames = 0;
        self.used = 0;
    }

    fn tilt(q: [f64; 4]) -> f64 { m::acos(m::min(1.0, 1.0 - 2.0 * (q[0] * q[0] + q[2] * q[2]))) }

    /// js `step(dt, t)`: each physics step, after the robot's pose is synced
    pub fn step(&mut self, dt: f64, t: f64, r: &RobotView) {
        if !self.ready || self.truth { return; }
        if self.t.is_none() { self.t = Some(t); }
        let tr = Self::truth_of(r);
        // ---- gyro
        let d_yaw = wrap(tr.th - self.last_yaw);
        self.last_yaw = tr.th;
        let (gb, gs) = { let c = self.cal.as_ref().unwrap(); (c.gyro_bias, c.gyro_scale) };
        let g1 = self.gauss();
        self.gyro_err += gb * dt + gs * d_yaw + g1 * sigma::gyro_walk() * m::sqrt(dt);
        let g2 = self.gauss();
        let gyro = wrap(tr.th + self.gyro_err + g2 * sigma::gyro_noise());
        let dth = wrap(gyro - self.gyro_prev.unwrap_or(gyro));
        self.gyro_prev = Some(gyro);
        // ---- odometry
        let sw = r.swerve;
        let (mut a11, a12, mut a13, mut a22, mut a23, mut a33, mut b1, mut b2, mut b3) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let sp = m::hypot2(r.vel[0], r.vel[2]);
        let driven = sw.modules.iter().any(|md| m::abs(md.w * sw.d.rw) > 1.0);
        self.stall_t = Some(if sp < 0.3 && driven { self.stall_t.unwrap_or(0.0) + dt } else { 0.0 });
        let (cy, sy) = (m::cos(r.yaw), m::sin(r.yaw));
        let rvx = r.vel[0] * cy - r.vel[2] * sy;
        let rvz = r.vel[0] * sy + r.vel[2] * cy;
        for i in 0..sw.modules.len() {
            let md = &sw.modules[i];
            let [px, pz] = sw.pts[i];
            let mut w = md.w * sw.d.rw;
            let ground = (rvx + r.omega * pz) * m::cos(md.angle) + (rvz - r.omega * px) * m::sin(md.angle);
            self.air[i] = if !sw.down.is_empty() && !sw.down[i] { self.air[i] + dt } else { 0.0 };
            let air = self.air[i];
            if air > 0.0 { w = if air < AIR_T { sw.want[i] } else { ground }; }
            else if md.slip {
                let cap = 0.07 * m::abs(ground) + if self.stall_t.unwrap() < SPIN_T { 0.05 } else { 0.25 };
                w = ground + m::max(-cap, m::min(cap, w - ground));
            }
            let (ma, wr) = { let c = self.cal.as_ref().unwrap(); (c.md[i], c.wheel_r) };
            let a = md.angle + ma;
            let s = w * wr + self.gauss() * sigma::WHEEL_V;
            let (ux, uz) = (s * m::cos(a), s * m::sin(a));
            a11 += 1.0; a13 += pz; a22 += 1.0; a23 -= px; a33 += pz * pz + px * px;
            b1 += ux; b2 += uz; b3 += pz * ux - px * uz;
        }
        let [vx, vz, _] = solve3(a11, a12, a13, a22, a23, a33, b1, b2, b3);
        let (mut vf, mut vl) = (vx, -vz);
        self.v = [vf, vl, if dt > 0.0 { dth / dt } else { 0.0 }];
        if self.prof.odo_tilt { let k = 1.0 - m::min(1.0, Self::tilt(r.quat) / (25.0 * DEG)); vf *= k; vl *= k; }
        let step = |p: &mut P2| {
            let hm = p.th + dth / 2.0;
            p.x += (m::cos(hm) * vf - m::sin(hm) * vl) * dt;
            p.y += (m::sin(hm) * vf + m::cos(hm) * vl) * dt;
            p.th = wrap(p.th + dth);
        };
        step(&mut self.odo);
        step(&mut self.est);
        self.buf.push((t, self.odo));
        while !self.buf.is_empty() && self.buf[0].0 < t - 1.5 { self.buf.remove(0); }
        // ---- cameras
        for i in 0..self.prof.cams.len() {
            let c = cam(self.prof.cams[i].cam);
            self.cam_t[i] -= dt;
            if self.cam_t[i] > 0.0 { continue; }
            self.cam_t[i] += 1.0 / c.fps;
            if let Some(ms) = self.frame(i, t, tr, dt, r) { self.pending.push(ms); }
        }
        // ---- vision results reaching the robot code
        if !self.pending.is_empty() {
            // (Array.sort: stable)
            self.pending.sort_by(|a, b| { let d = a.due - b.due; if d < 0.0 { std::cmp::Ordering::Less } else if d > 0.0 { std::cmp::Ordering::Greater } else { std::cmp::Ordering::Equal } });
            while !self.pending.is_empty() && self.pending[0].due <= t { let ms = self.pending.remove(0); self.apply(ms); }
        }
        self.latched = r.ready_shoot;
        // ---- what the aim reads, ACT_DELAY steps late
        self.hist.push(Hist { x: self.est.x, y: self.est.y, th: self.est.th, f: self.v[0], l: self.v[1], w: self.v[2] });
        if self.hist.len() > ACT_DELAY + 1 { self.hist.remove(0); }
        self.last_turret = r.turret_yaws.to_vec();
    }

    /// js `_frame(cm, ci, t, T, dt)`: one camera frame, the pose its pipeline makes of the tags it sees
    fn frame(&mut self, ci: usize, t: f64, tr: P2, dt: f64, r: &RobotView) -> Option<Meas> {
        let cm = self.prof.cams[ci];
        let c = cam(cm.cam);
        let cal = self.cal.as_ref().unwrap().cams[ci];
        self.frames += 1;
        let (mut cx, mut cy, mut cyaw, mut t_rate) = (cm.x, cm.y, cm.yaw * DEG, 0.0);
        if let Some(ti) = cm.turret {
            // (on a turret, along its shot: turning with it)
            let tt = r.turrets[ti];
            let ty = r.turret_yaws.get(ti).copied().unwrap_or(0.0);
            t_rate = if dt > 0.0 { wrap(ty - self.last_turret.get(ti).copied().unwrap_or(ty)) / dt } else { 0.0 };
            cx = tt[0] + m::cos(ty) * cm.x - m::sin(ty) * cm.y;
            cy = -tt[1] + m::sin(ty) * cm.x + m::cos(ty) * cm.y;
            cyaw = ty + cm.yaw * DEG;
        }
        let cam_world = |p: P2, yaw_err: f64| P2 { x: p.x + m::cos(p.th) * cx - m::sin(p.th) * cy, y: p.y + m::sin(p.th) * cx + m::cos(p.th) * cy, th: p.th + cyaw + yaw_err };
        let camp = cam_world(tr, 0.0);
        let tilt = Self::tilt(r.quat);
        if let Some(g) = self.prof.pitch_gate { if tilt > g * DEG { return None; } }
        let w_cam = m::abs(r.omega + t_rate);
        let sp = m::hypot2(r.vel[0], r.vel[2]);
        let mut obs: Vec<Obs> = Vec::new();
        for k in 0..self.tags.len() {
            let tg = self.tags[k];
            if self.prof.ignore.contains(&tg.id) { continue; }
            let tc = self.cal.as_ref().unwrap().tags[k];
            let (tx, ty) = (tg.x + tc[0], tg.y + tc[1]);
            let (dx, dy) = (tx - camp.x, ty - camp.y);
            let d = m::hypot2(dx, dy);
            if d < 0.3 { continue; }
            let view = wrap(m::atan2(-dy, -dx) - tg.th);
            if m::abs(view) > 70.0 * DEG { continue; }
            let b = wrap(m::atan2(dy, dx) - camp.th);
            if m::abs(b) > (c.hfov / 2.0 - 2.0) * DEG { continue; }
            let el = m::atan2(tg.h - cm.h, d) - cm.pitch * DEG;
            if m::abs(el) > (c.vfov / 2.0 - 2.0) * DEG { continue; }
            let tag_px = (c.f * TAG_SIZE) / d;
            let edge_px = tag_px * m::sqrt(m::max(0.05, m::cos(view)));
            if edge_px < c.min_px { continue; }
            if let Some(mt) = cm.max_tag { if m::hypot2(tg.x - tr.x, tg.y - tr.y) > mt { continue; } }
            if Self::hidden(r.others, camp, cm.h, tg, tx, ty, d) { continue; }
            if self.rnd() > 0.97 { continue; }
            let blur = c.exp * c.f * (w_cam + sp / d);
            let px = m::hypot2(c.px, 0.3 * blur);
            let sb = px / c.f;
            let sr = (d * d * px) / (c.f * TAG_SIZE);
            let skew = if c.rs != 0.0 { -(r.omega + t_rate) * c.rs * (0.5 - (0.5 - el / (c.vfov * DEG))) } else { 0.0 };
            let bm = b + cal.yaw + skew + self.gauss() * sb;
            let dm = d * (1.0 + cal.scale) + self.gauss() * sr;
            let cue = edge_px * (0.15 + m::abs(m::sin(view)));
            let sphi = (2.0 * px) / cue;
            let phim = wrap(tg.th + m::PI - camp.th) + self.gauss() * sphi;
            obs.push(Obs { k, d, b: bm, r: dm, sb, sr, phi: phim, sphi, amb: m::min(1.0, (6.0 * px) / cue) });
        }
        if obs.is_empty() { return None; }
        // (the turret's angle as the code knows it at the frame's time: encoder, timing)
        let turret_err = if cm.turret.is_some() { cal.turret + t_rate * self.gauss() * sigma::TURRET_T } else { 0.0 };
        let mode = cm.mode.unwrap_or(self.prof.mode);
        if self.prof.min_tags.unwrap_or(1) > obs.len() { return None; }
        if let Some(mo) = self.prof.max_omega { if m::abs(r.omega) > mo * DEG { return None; } }
        let cam_nominal = |p: P2| cam_world(p, -turret_err);
        let mut single = obs.len() == 1;
        let mut pose;
        if mode == Mode::Mt2 || mode == Mode::TagDist || (mode == Mode::Pnp1h && single) {
            let use_: Vec<Obs> = if mode == Mode::TagDist { vec![obs.iter().copied().reduce(|a, o| if o.d < a.d { o } else { a }).unwrap()] } else { obs.clone() };
            let th = self.est.th;
            let cn = cam_nominal(P2 { x: 0.0, y: 0.0, th });
            let (mut i11, mut i12, mut i22, mut v1, mut v2) = (0.0, 0.0, 0.0, 0.0, 0.0);
            for o in &use_ {
                let tg = self.tags[o.k];
                let a = cn.th + o.b;
                let (ux, uy) = (m::cos(a), m::sin(a));
                let (px_, py_) = (tg.x - o.r * ux - cn.x, tg.y - o.r * uy - cn.y);
                let wa = 1.0 / (o.sr * o.sr);
                let wc = 1.0 / m::pow(o.r * o.sb, 2.0);
                let (j11, j12, j22) = (wa * ux * ux + wc * uy * uy, (wa - wc) * ux * uy, wa * uy * uy + wc * ux * ux);
                i11 += j11; i12 += j12; i22 += j22; v1 += j11 * px_ + j12 * py_; v2 += j12 * px_ + j22 * py_;
            }
            let det = i11 * i22 - i12 * i12;
            pose = P2 { x: (i22 * v1 - i12 * v2) / det, y: (i11 * v2 - i12 * v1) / det, th };
            single = use_.len() == 1;
        } else {
            if single && obs[0].amb > self.prof.max_amb.unwrap_or(0.3) { return None; }
            pose = self.pnp(&obs, &cam_nominal, tr);
        }
        if tilt > 1e-3 {
            let q = r.quat;
            let up_x = 2.0 * (q[0] * q[1] - q[3] * q[2]);
            let up_z = 2.0 * (q[1] * q[2] + q[3] * q[0]);
            pose.x += cm.h * up_x; pose.y -= cm.h * up_z;
        }
        let n = if single { 1 } else { obs.len() };
        let avg_d = obs.iter().fold(0.0, |s, o| s + o.d) / obs.len() as f64;
        if let Some(md) = self.prof.max_dist { if avg_d > md { return None; } }
        if pose.x < -HALF_L - 0.5 || pose.x > HALF_L + 0.5 || pose.y < -HALF_W - 0.5 || pose.y > HALF_W + 0.5 { return None; }
        let lat = c.lat + m::abs(self.gauss()) * sigma::LAT_JITTER;
        let stamp = t + self.gauss() * sigma::TS_JITTER;
        Some(Meas { stamp, due: t + lat, pose, n, d: avg_d })
    }

    /// js `_hidden(cam, ch, tg, tX, tY, d)`
    fn hidden(others: &[[f64; 4]], camp: P2, ch: f64, tg: Tag, tx: f64, ty: f64, d: f64) -> bool {
        let seg = |x: f64, y: f64, rad: f64, top: f64| {
            let (ex, ey) = (tx - camp.x, ty - camp.y);
            let u = m::max(0.0, m::min(1.0, ((x - camp.x) * ex + (y - camp.y) * ey) / (d * d)));
            if u <= 0.0 || u >= 0.98 { return false; }
            let (qx, qy) = (camp.x + u * ex - x, camp.y + u * ey - y);
            m::hypot2(qx, qy) < rad && ch + u * (tg.h - ch) < top
        };
        for o in others { if seg(o[0], -o[1], m::max(o[2], o[3]) * 0.9, 0.55) { return true; } }
        for a in [Alliance::Blue, Alliance::Red] {
            let hc = crate::field::hub_center(a);
            if m::hypot2(tg.x - hc.0, tg.y + hc.1) < HUB::size { continue; }
            if seg(hc.0, -hc.1, HUB::size / 2.0, 2.0) { return true; }
        }
        false
    }

    /// js `_pnp(obs, camNominal, T)`: Gauss-Newton from the truth
    fn pnp(&self, obs: &[Obs], cam_nominal: &dyn Fn(P2) -> P2, tr: P2) -> P2 {
        let mut p = tr;
        let res = |q: P2| -> Vec<f64> {
            let cn = cam_nominal(q);
            let mut out = Vec::with_capacity(obs.len() * 3);
            for o in obs {
                let tg = self.tags[o.k];
                let (dx, dy) = (tg.x - cn.x, tg.y - cn.y);
                out.push(wrap(m::atan2(dy, dx) - cn.th - o.b) / o.sb);
                out.push((m::hypot2(dx, dy) - o.r) / o.sr);
                out.push(wrap(wrap(tg.th + m::PI - cn.th) - o.phi) / o.sphi);
            }
            out
        };
        for _ in 0..3 {
            let r0 = res(p);
            let h = [1e-4, 1e-4, 1e-5];
            let mut jac: Vec<Vec<f64>> = Vec::new();
            for j in 0..3 {
                let mut q = p;
                match j { 0 => q.x += h[j], 1 => q.y += h[j], _ => q.th += h[j] }
                jac.push(res(q).iter().enumerate().map(|(i, v)| (v - r0[i]) / h[j]).collect());
            }
            let mut a = [[0.0; 3]; 3];
            let mut g = [0.0; 3];
            for i in 0..r0.len() { for ai in 0..3 { g[ai] += jac[ai][i] * r0[i]; for bi in 0..3 { a[ai][bi] += jac[ai][i] * jac[bi][i]; } } }
            let dx = solve3(a[0][0], a[0][1], a[0][2], a[1][1], a[1][2], a[2][2], -g[0], -g[1], -g[2]);
            p = P2 { x: p.x + dx[0], y: p.y + dx[1], th: wrap(p.th + dx[2]) };
        }
        p
    }

    /// js `_apply(m, t)`: a vision pose reaching the robot code
    fn apply(&mut self, ms: Meas) {
        self.used += 1;
        if self.prof.fuse == Fuse::Reset {
            if !self.latched { self.est.x = ms.pose.x; self.est.y = ms.pose.y; }
            return;
        }
        let [sxy, sth] = self.prof.std(ms.d, ms.n);
        let rr = [sxy * sxy, sxy * sxy, sth * sth];
        let k: Vec<f64> = (0..3).map(|i| { let q = self.prof.q[i] * self.prof.q[i]; if rr[i].is_finite() { q / (q + m::sqrt(q * rr[i])) } else { 0.0 } }).collect();
        let Some(at) = self.odo_at(ms.stamp) else { return };
        let est0 = compose(self.est, between(self.odo, at));
        let d = between(est0, ms.pose);
        let corr = P2 { x: k[0] * d.x, y: k[1] * d.y, th: k[2] * d.th };
        let est1 = compose(est0, corr);
        self.est = compose(est1, between(at, self.odo));
    }

    /// js `_odoAt(ts)`
    fn odo_at(&self, ts: f64) -> Option<P2> {
        let b = &self.buf;
        if b.is_empty() || ts < b[0].0 { return None; }
        let mut i = b.len() - 1;
        while i > 0 {
            if b[i - 1].0 <= ts {
                let (a, c) = (b[i - 1], b[i]);
                let u = if c.0 > a.0 { m::min(1.0, (ts - a.0) / (c.0 - a.0)) } else { 0.0 };
                return Some(P2 { x: a.1.x + u * (c.1.x - a.1.x), y: a.1.y + u * (c.1.y - a.1.y), th: wrap(a.1.th + u * wrap(c.1.th - a.1.th)) });
            }
            i -= 1;
        }
        Some(b[b.len() - 1].1)
    }

    fn aim_state(&self) -> Option<Hist> { if !self.truth && self.ready { self.hist.first().copied() } else { None } }

    /// js `aimPt(p)`: world point p (x, z) as the aim sees it, from the robot's true pose (pos, yaw)
    pub fn aim_pt(&self, p: [f64; 2], pos: [f64; 3], yaw: f64) -> [f64; 2] {
        let Some(s) = self.aim_state() else { return p };
        let tr = P2 { x: pos[0], y: -pos[2], th: yaw };
        let rel = between(P2 { x: s.x, y: s.y, th: s.th }, P2 { x: p[0], y: -p[1], th: 0.0 });
        let w = compose(tr, rel);
        [w.x, -w.y]
    }
    /// js `vel()`: the chassis velocity the aim uses (world x, z), or None (the truth)
    pub fn vel(&self, yaw: f64) -> Option<[f64; 2]> {
        let s = self.aim_state()?;
        let (c, sn) = (m::cos(yaw), m::sin(yaw));
        let (x, y) = (c * s.f - sn * s.l, sn * s.f + c * s.l);
        Some([x, -y])
    }
    /// (for comparing with native-trace --loc: est, odo, v, vision poses used)
    pub fn log_line(&self) -> String {
        format!("est {} {} {} odo {} {} v {} {} {} used {}", self.est.x, self.est.y, self.est.th, self.odo.x, self.odo.y, self.v[0], self.v[1], self.v[2], self.used)
    }
    /// js `omega()`
    pub fn omega(&self) -> Option<f64> { self.aim_state().map(|s| s.w) }
}

/// js `solve3`: a symmetric 3x3 solve
fn solve3(a11: f64, a12: f64, a13: f64, a22: f64, a23: f64, a33: f64, b1: f64, b2: f64, b3: f64) -> [f64; 3] {
    let c11 = a22 * a33 - a23 * a23;
    let c12 = a13 * a23 - a12 * a33;
    let c13 = a12 * a23 - a13 * a22;
    let det = a11 * c11 + a12 * c12 + a13 * c13;
    if m::abs(det) < 1e-12 { return [0.0; 3]; }
    let c22 = a11 * a33 - a13 * a13;
    let c23 = a12 * a13 - a11 * a23;
    let c33 = a11 * a22 - a12 * a12;
    [(c11 * b1 + c12 * b2 + c13 * b3) / det, (c12 * b1 + c22 * b2 + c23 * b3) / det, (c13 * b1 + c23 * b2 + c33 * b3) / det]
}
