//! js/drivetrain.js: a robot's motors, gearing, wheels, current limits, battery and tires
//! (Drivetrain), and the four swerve modules that drive it in play (SwerveDrive).
use crate::constants::*;
use jsmath as m;
use rapier3d::prelude::*;

/// A motor's datasheet numbers (js MOTORS)
#[derive(Debug, Clone, Copy)]
pub struct MotorSpec { pub free: f64, pub stall_torque: f64, pub stall_current: f64, pub free_current: f64, pub v: f64, pub rotor_inertia: f64 }

/// js `MOTORS[name]`
pub fn motor_spec(name: &str) -> MotorSpec {
    let rpm = |r: f64| (r / 60.0) * 2.0 * m::PI;
    match name {
        "krakenX60" => MotorSpec { free: rpm(5800.0), stall_torque: 9.37, stall_current: 483.0, free_current: 2.0, v: 12.0, rotor_inertia: 6.5e-5 },
        "krakenX60Trap" => MotorSpec { free: rpm(6000.0), stall_torque: 7.09, stall_current: 366.0, free_current: 2.0, v: 12.0, rotor_inertia: 6.5e-5 },
        "krakenX44" => MotorSpec { free: rpm(7530.0), stall_torque: 4.1, stall_current: 280.0, free_current: 1.5, v: 12.0, rotor_inertia: 2.5e-5 },
        "falcon500" => MotorSpec { free: rpm(6380.0), stall_torque: 4.69, stall_current: 257.0, free_current: 1.5, v: 12.0, rotor_inertia: 5e-5 },
        _ => panic!("no motor {name}"),
    }
}

const V_REST: f64 = 12.6;
const R_BATTERY: f64 = 0.02;
const OTHER_LOAD: f64 = 25.0;
const ETA: f64 = 0.93;
const TOP_FRACTION: f64 = 0.88;

/// cfg.drive, as robots.json has it (fields left out take the JS DEFAULTS)
#[derive(Debug, Clone, serde::Deserialize)]
#[serde(default)]
pub struct DriveCfg { pub motor: String, pub motors: f64, pub wheel: f64, pub stator: f64, pub supply: f64, pub mu: f64, pub ratio: f64, pub teleop: Option<Limits> }
impl Default for DriveCfg {
    fn default() -> Self { DriveCfg { motor: "krakenX60".into(), motors: 4.0, wheel: 4.0 * IN, stator: 80.0, supply: 70.0, mu: 1.3, ratio: 0.0, teleop: None } }
}
#[derive(Debug, Clone, Copy, serde::Deserialize)]
pub struct Limits { pub stator: Option<f64>, pub supply: Option<f64> }

#[derive(Debug, Clone)]
pub struct Drivetrain {
    pub motors: f64, pub wheel: f64, pub stator: f64, pub supply: f64, pub mu: f64, pub ratio: f64,
    pub teleop: Option<Limits>, base: (f64, f64),
    pub m: f64, pub r: f64, pub mo: MotorSpec,
    pub rm: f64, pub kt: f64, pub kv: f64, pub rw: f64,
}

impl Drivetrain {
    /// js `new Drivetrain(d, m, L, W, r)` (only what play uses: the stated numbers aren't needed)
    pub fn new(d: &DriveCfg, mass: f64, r: f64) -> Self {
        let mo = motor_spec(&d.motor);
        let rm = mo.v / mo.stall_current;
        let kt = mo.stall_torque / mo.stall_current;
        let kv = mo.free / (mo.v - mo.free_current * rm);
        Drivetrain { motors: d.motors, wheel: d.wheel, stator: d.stator, supply: d.supply, mu: d.mu, ratio: d.ratio, teleop: d.teleop,
            base: (d.stator, d.supply), m: mass, r, mo, rm, kt, kv, rw: d.wheel / 2.0 }
    }
    /// js `limitsFor(teleop)`
    pub fn limits_for(&mut self, teleop: bool) {
        if let Some(t) = self.teleop {
            self.stator = self.base.0; self.supply = self.base.1;
            if teleop {
                if let Some(s) = t.stator { self.stator = s; }
                if let Some(s) = t.supply { self.supply = s; }
            }
        }
    }
}

const STEER_RATIO: f64 = 150.0 / 7.0;
const STEER_ACCEL: f64 = 1500.0;
const STEER_ROLL: f64 = 0.015;
const STEER_FRICTION: f64 = 2.0;

#[derive(Debug, Clone, Copy, Default)]
pub struct Module { pub angle: f64, pub rate: f64, pub w: f64, pub i: f64, pub slip: bool }

#[derive(Debug, Clone)]
pub struct SwerveDrive {
    pub d: Drivetrain,
    pub pts: Vec<[f64; 2]>,
    pub steer_rate: f64,
    pub modules: Vec<Module>,
    pub v: f64,
    pub current: f64,
    pub want: Vec<f64>,
    pub down: Vec<bool>,
}

fn wrap(a: f64) -> f64 { m::atan2(m::sin(a), m::cos(a)) }

struct K { u: [f64; 2], ru: f64, m: f64, lam: f64 }
struct Ct { i: usize, r: [f64; 2], k: [K; 2], want: f64, lo: f64, hi: f64, lim: f64, slip: bool }

impl SwerveDrive {
    pub fn new(d: Drivetrain, pts: Vec<[f64; 2]>) -> Self {
        let steer_rate = 0.85 * (d.mo.free / STEER_RATIO);
        let n = pts.len();
        SwerveDrive { d, pts, steer_rate, modules: vec![Module::default(); n], v: 12.4, current: 0.0, want: Vec::new(), down: Vec::new() }
    }
    pub fn reset(&mut self) { for m in &mut self.modules { *m = Module::default(); } }

    fn flat_out(&self, w: f64, dir: f64, v: f64) -> f64 {
        let d = &self.d;
        let eb = (w * d.ratio) / d.kv;
        let i = (dir * v - eb) / d.rm;
        let e = m::sign(i) * eb;
        let cap = m::min(d.stator, (-e + m::sqrt(e * e + 4.0 * d.rm * d.supply * v)) / (2.0 * d.rm));
        d.kt * (if m::abs(i) > cap { m::sign(i) * cap } else { i }) * d.ratio * ETA
    }
    fn supply_current(&self, w: f64, tau: f64, v: f64) -> f64 {
        let d = &self.d;
        let ic = tau / (d.kt * d.ratio * ETA);
        let eb = (w * d.ratio) / d.kv;
        (ic * (eb + ic * d.rm)) / v
    }

    /// js `step(body, yaw, vx, vz, omega, down, dt)`
    pub fn step(&mut self, body: &mut RigidBody, yaw: f64, vx: f64, vz: f64, omega: f64, down: Vec<bool>, dt: f64) {
        let rw = self.d.rw;
        let n = self.pts.len();
        let (c, s) = (m::cos(yaw), m::sin(yaw));
        let to_field = |x: f64, z: f64| [x * c + z * s, -x * s + z * c];
        let to_robot = |x: f64, z: f64| [x * c - z * s, x * s + z * c];
        let [rvx, rvz] = to_robot(vx, vz);
        let mut want = Vec::with_capacity(n);
        for i in 0..n {
            let [px, pz] = self.pts[i];
            let steer_rate = self.steer_rate;
            let md = &mut self.modules[i];
            let ux = rvx + omega * pz;
            let uz = rvz - omega * px;
            let mut sp = m::hypot2(ux, uz);
            md.angle = m::atan2(m::sin(md.angle), m::cos(md.angle));
            let target = if sp > 0.02 { m::atan2(uz, ux) } else { md.angle };
            let mut err = wrap(target - md.angle);
            if m::abs(err) > m::PI / 2.0 { err = wrap(err + m::PI); sp = -sp; }
            let goal = md.angle + err;
            let stop_rate = m::sqrt(2.0 * STEER_ACCEL * m::abs(err)) * m::sign(err);
            let rate = m::max(-steer_rate, m::min(steer_rate, stop_rate));
            md.rate += m::max(-STEER_ACCEL * dt, m::min(STEER_ACCEL * dt, rate - md.rate));
            let turn = md.rate * dt;
            if m::abs(turn) >= m::abs(err) { md.angle = goal; md.rate = 0.0; } else { md.angle += turn; }
            want.push(sp * m::max(0.0, m::cos(goal - md.angle)));
        }
        let lv = body.linvel();
        let av = body.angvel();
        let com = body.center_of_mass();
        let p = body.translation();
        let mass = body.mass() as f64;
        let iz = body.mass_properties().local_mprops.principal_inertia().y as f64;
        let (lvx, lvz, avy) = (lv.x as f64, lv.z as f64, av.y as f64);
        let mut v = [lvx, lvz];
        let mut wy = avy;
        let n_down = down.iter().filter(|x| **x).count() as f64;
        let vb = self.v;
        let mut cts: Vec<Ct> = Vec::new();
        for i in 0..n {
            if !down[i] { self.modules[i].slip = false; self.modules[i].i = 0.0; continue; }
            let [px, pz] = self.pts[i];
            let [fx, fz] = to_field(px, pz);
            let r = [p.x as f64 + fx - com.x as f64, p.z as f64 + fz - com.z as f64];
            let ang = self.modules[i].angle;
            let [lx, lz] = to_field(m::cos(ang), m::sin(ang));
            let mk = |ux: f64, uz: f64| { let ru = r[1] * ux - r[0] * uz; K { u: [ux, uz], ru, m: 1.0 / (1.0 / mass + (ru * ru) / iz), lam: 0.0 } };
            let k = [mk(lx, lz), mk(-lz, lx)];
            let w = self.modules[i].w;
            let fwd = (self.flat_out(w, 1.0, vb) * dt) / rw;
            let back = (self.flat_out(w, -1.0, vb) * dt) / rw;
            let drag = ((STEER_ROLL * self.d.m * GRAVITY + STEER_FRICTION * m::abs(w * rw)) / n as f64) * dt;
            cts.push(Ct { i, r, k, want: want[i], lo: back - drag, hi: fwd + drag, lim: (self.d.mu * mass * GRAVITY * dt) / n_down, slip: false });
        }
        for _ in 0..10 {
            for ct in &mut cts {
                let vpx = v[0] + wy * ct.r[1];
                let vpz = v[1] - wy * ct.r[0];
                let (a, b) = (&ct.k[0], &ct.k[1]);
                let mut l1 = a.lam - (vpx * a.u[0] + vpz * a.u[1] - ct.want) * a.m;
                let mut l2 = b.lam - (vpx * b.u[0] + vpz * b.u[1]) * b.m;
                l1 = m::max(ct.lo, m::min(ct.hi, l1));
                let mag = m::hypot2(l1, l2);
                ct.slip = mag > ct.lim;
                if ct.slip { l1 *= ct.lim / mag; l2 *= ct.lim / mag; }
                for (kk, l) in ct.k.iter_mut().zip([l1, l2]) {
                    let dl = l - kk.lam;
                    kk.lam = l;
                    v = [v[0] + (kk.u[0] * dl) / mass, v[1] + (kk.u[1] * dl) / mass];
                    wy += (kk.ru * dl) / iz;
                }
            }
        }
        let mut isum = 0.0;
        for ct in &cts {
            let a = &ct.k[0];
            let ground = (v[0] + wy * ct.r[1]) * a.u[0] + (v[1] - wy * ct.r[0]) * a.u[1];
            let tau = (a.lam * rw) / dt;
            let w = (if ct.slip && m::abs(ct.want) > m::abs(ground) { ground + 0.5 * (ct.want - ground) } else { ground }) / rw;
            let i_ = self.supply_current(w, tau, vb);
            let md = &mut self.modules[ct.i];
            md.w = w;
            md.slip = ct.slip;
            md.i = i_;
            isum += i_;
        }
        self.current = isum;
        self.v += 0.5 * (m::max(6.0, V_REST - R_BATTERY * (isum + OTHER_LOAD)) - self.v);
        self.want = want;
        self.down = down;
        body.reset_forces(true);
        body.reset_torques(true);
        body.add_force(Vector::new((((v[0] - lvx) * mass) / dt) as f32, 0.0, (((v[1] - lvz) * mass) / dt) as f32), true);
        body.add_torque(Vector::new(0.0, (((wy - avy) * iz) / dt) as f32, 0.0), true);
    }
}
