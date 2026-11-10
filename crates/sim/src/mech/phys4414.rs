//! 4414's RIPCURRENT in play (js/mech/phys4414.js), built of its physical mechanisms (r4414.rs; the
//! shared part: physical.rs). Their code isn't public; its controls follow their binder and what the
//! team said (the JS file has it all): the box out on its racks once the match starts, stroking in
//! and out while it shoots and isn't intaking; the Dye Rotor at 2.25 rps feeding, turning back
//! slowly otherwise (faster while intaking), backing out of a stall; the aim from their
//! precomputed trajectories (shot4414's table and fits), the robot's velocity taken out in 3D, tilt
//! compensated; the turret on its motor against its energy chain.
use crate::ballistics::sim2d;
use crate::constants::*;
use crate::mech::drives::{Drives, VelLoop};
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, Physical, Stats, Where};
use crate::mech::r4414::*;
use crate::mech::turret_drive::{drive4414, TurretDrive};
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const TAU: f64 = 2.0 * m::PI;
const R: f64 = FUEL::radius;
const ROTOR_RPS: f64 = 2.25;
const AGITATE_RPS: f64 = -0.15;
/// js INTAKE_RPS (INTAKE_ROTOR unset)
const INTAKE_RPS: f64 = -0.5;
const LATENCY: f64 = 0.03;
const PASS_HOOD: f64 = 135.0;
/// js TILT
fn tilt_min() -> f64 { 2.0 * DEG }
fn tilt_lock() -> f64 { 5.0 * DEG }
const TILT_LOCK_SPEED: f64 = 0.5;
/// js BOX_KP, STROKE
const BOX_KP: f64 = 4.0;
mod stroke { pub const IN: f64 = 0.14; pub const VOLTS: f64 = 2.0; pub const OUT_VOLTS: f64 = 3.0; pub const TOL: f64 = 0.008; pub const SLOW: f64 = 0.05; pub const STALL: f64 = 0.08; }
fn turns_per_m() -> f64 { rack::RATIO / (2.0 * m::PI * rack::pinion()) }
const RADS: [f64; 2] = [200.0, 700.0];

fn clamp(v: f64, lo: f64, hi: f64) -> f64 { if v < lo { lo } else if v > hi { hi } else { v } }

/// js phys604 `profile(p, v, goal, vmax, amax, dt)`: a trapezoidal profile's next setpoint
pub fn profile604(p: f64, v: f64, goal: f64, vmax: f64, amax: f64, dt: f64) -> [f64; 2] {
    let d = goal - p;
    let dir = m::sign(d);
    if m::abs(d) < 1e-6 && m::abs(v) < amax * dt { return [goal, 0.0]; }
    let want = dir * m::min(vmax, m::sqrt(2.0 * amax * m::abs(d)));
    let nv = v + clamp(want - v, -amax * dt, amax * dt);
    let np = p + nv * dt;
    if (goal - np) * dir < 0.0 { return [goal, 0.0]; }
    [np, nv]
}
/// js phys604 `velocityDuty(motor, w, target, kV, kP)`
pub fn velocity_duty(ratio: f64, w: f64, target: f64, kv: f64, kp: f64) -> f64 {
    let tgt = (target * ratio) / TAU;
    let rps = (w * ratio) / TAU;
    clamp((kv * tgt + kp * (tgt - rps)) / 12.0, -1.0, 1.0)
}

/// js `fitOf(k, rad, hood)`: their fits, over [1, r, r^2, r^3, h, h r, h^2]
fn fit_of(f: &[f64; 7], rad: f64, hood: f64) -> f64 {
    let (r, h) = (rad / 100.0, (hood - 140.0) / 10.0);
    let feats = [1.0, r, r * r, r * r * r, h, h * r, h * h];
    feats.iter().enumerate().fold(0.0, |a, (i, x)| a + x * f[i])
}
fn bisect(f: &dyn Fn(f64) -> f64, mut lo: f64, mut hi: f64, want: f64) -> f64 {
    for _ in 0..30 { let mid = (lo + hi) / 2.0; if f(mid) < want { lo = mid; } else { hi = mid; } }
    (lo + hi) / 2.0
}
/// js `shooterFor(v, el)`: the flywheel speed and hood that make a shot at v, el
pub fn shooter_for(v: f64, el: f64) -> (f64, f64) {
    let g = gen4414();
    let (mut rad, mut hood) = (450.0, 145.0);
    for _ in 0..4 {
        hood = bisect(&|h| fit_of(&g.fit.el, rad, h), shooter::HOOD_MIN, shooter::HOOD_MAX, el);
        rad = bisect(&|r| fit_of(&g.fit.v, r, hood), RADS[0], RADS[1], v);
    }
    (rad, hood)
}
/// js `launchFor(d)`: the table's shot for a distance: (exit speed, elevation)
pub fn launch_for(d: f64) -> (f64, f64) {
    let t = &gen4414().launch;
    if d <= t[0][0] { return (t[0][3], t[0][4]); }
    for k in 1..t.len() {
        if d <= t[k][0] { let f = (d - t[k - 1][0]) / (t[k][0] - t[k - 1][0]); return (t[k - 1][3] + f * (t[k][3] - t[k - 1][3]), t[k - 1][4] + f * (t[k][4] - t[k - 1][4])); }
    }
    let e = t[t.len() - 1];
    (e[3], e[4])
}
/// js `qrot(q, v)`, `qrotInv(q, v)` (q: x, y, z, w)
fn qrot(q: [f64; 4], [vx, vy, vz]: [f64; 3]) -> [f64; 3] {
    let (qx, qy, qz, qw) = (q[0], q[1], q[2], q[3]);
    let (tx, ty, tz) = (2.0 * (qy * vz - qz * vy), 2.0 * (qz * vx - qx * vz), 2.0 * (qx * vy - qy * vx));
    [vx + qw * tx + (qy * tz - qz * ty), vy + qw * ty + (qz * tx - qx * tz), vz + qw * tz + (qx * ty - qy * tx)]
}
fn qrot_inv(q: [f64; 4], [vx, vy, vz]: [f64; 3]) -> [f64; 3] {
    let (x, y, z, w) = (-q[0], -q[1], -q[2], q[3]);
    let ix = w * vx + y * vz - z * vy;
    let iy = w * vy + z * vx - x * vz;
    let iz = w * vz + x * vy - y * vx;
    let iw = -x * vx - y * vy - z * vz;
    [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x]
}
/// js `rotorLoop(shaft, rps)`: its velocity loop at rps (None: coast)
fn rotor_loop(d: &mut Drives, s: usize, rps: Option<f64>) {
    d.shafts[s].vel = rps.map(|r| VelLoop { target: r * TAU, kv: 0.12, kp: 0.2, ks: 0.0, lo: -12.0, hi: 12.0, per: None, rate: 1000.0 });
    d.shafts[s].cmd = rps.map(|_| 0.0);
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Stroke { In, Out }

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool, pub intake_deploy: f64, pub turret_yaw: f64 }
/// What its postStep reads of the robot (js postStep's arguments, and the robot's)
pub struct PostIn<'a> {
    pub on: bool, pub feeding: bool, pub outtake: bool, pub setpoint: f64, pub pass: bool, pub intake: bool,
    pub has_shot: bool, pub target: Option<[f64; 2]>, pub aim_hub: [f64; 2], pub aim_err: Option<f64>,
    pub est_vel: [f64; 2], pub est_omega: f64, pub yaw: f64, pub spin: f64, pub wheel_pts: &'a [[f64; 2]], pub legal: bool,
}
pub use crate::mech::phys2910::PostOut;

pub struct Phys4414 {
    pub p: Physical,
    pub t: Trains,
    feed_home: Vec<([f64; 3], [f64; 3])>,
    shooter_home: Vec<[f64; 3]>,
    n_feed: usize,
    pub ext: f64,
    pub deployed: bool,
    stroke: Stroke,
    stroke_t: Option<f64>,
    slow_t: Option<f64>,
    pub strokes: u32,
    agitating: bool,
    pub tilt: f64,
    pub tilt_lock: bool,
    pub tilt_comped: bool,
    tilt_comp: bool,
    tilt_pos: bool,
    pub hood_deg: f64,
    hood_set: [f64; 2],
    pub hood_goal: f64,
    pub phi: f64,
    rotor_jam: f64, rotor_back: f64, roller_jam: f64, roller_back: f64,
    pub target_rad: f64,
    launching: bool,
    pub ready: bool,
    pub feeding_now: bool,
    intaking_now: bool,
    va: Option<[f64; 4]>,
    last_deploy: Option<f64>,
    not_ready: Option<f64>,
    pub aim_dist: f64,
    /// js aimPsi: where it wants the turret (None until it has aimed)
    pub aim_psi: Option<f64>,
    pub launch_angle: f64,
    pub launch_v: f64,
    pub tilt_dh: f64,
    turret: Option<TurretDrive>,
    xs: Vec<f32>, ys: Vec<f32>,
}

impl Phys4414 {
    /// js `new Physical4414(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = Drives::default();
        let t = build4414(&mut d);
        let w = wheels4414(&mut d, &t);
        let surfaces = path_surfaces4414(&t);
        let (sw, shooter_home) = shooter_wheels(&t);
        let n_feed = w.feed.len();
        let mut path_wheels = w.feed;
        path_wheels.extend(w.intake);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.push(t.rotor);
        shafts.push(t.slide);
        let path_surfs: Vec<usize> = (0..surfaces.len()).filter(|&i| i != S_TURRET && i != S_HOOD).collect();
        let path = Mechanism::new(Some(body), shafts, path_wheels, path_surfs, 6, 1);
        let shooter = Mechanism::new(Some(body), vec![t.fly, t.hood_roller, t.feeder], sw, vec![S_TURRET, S_HOOD], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), PRELOAD.to_vec(), None);
        let mut me = Phys4414 {
            p, t, feed_home: w.feed_home, shooter_home, n_feed, ext: 0.0, deployed: false, stroke: Stroke::Out, stroke_t: None, slow_t: None, strokes: 0, agitating: false,
            tilt: 0.0, tilt_lock: false, tilt_comped: false, tilt_comp: true, tilt_pos: true, hood_deg: 0.0, hood_set: [0.0; 2], hood_goal: 0.0, phi: 0.0,
            rotor_jam: 0.0, rotor_back: 0.0, roller_jam: 0.0, roller_back: 0.0, target_rad: 0.0, launching: false, ready: false, feeding_now: false, intaking_now: false,
            va: None, last_deploy: None, not_ready: None, aim_dist: 0.0, aim_psi: None, launch_angle: 0.0, launch_v: 0.0, tilt_dh: 0.0, turret: None,
            xs: vec![0f32; 1802], ys: vec![0f32; 1802],
        };
        me.reset();
        me
    }

    /// js `reset()`
    pub fn reset(&mut self) {
        self.p.reset();
        self.ext = 0.0; self.deployed = false; self.stroke = Stroke::Out; self.strokes = 0; self.agitating = false;
        let d = &mut self.p.drives;
        d.shafts[self.t.slide].omega_ = 0.0; d.shafts[self.t.slide].cmd = None;
        self.tilt = 0.0; self.tilt_lock = false; self.tilt_comped = false;
        self.hood_deg = shooter::HOOD_STOW; self.hood_set = [shooter::HOOD_STOW * DEG, 0.0]; self.hood_goal = shooter::HOOD_STOW;
        self.phi = 0.0;
        self.rotor_jam = 0.0; self.rotor_back = 0.0; self.roller_jam = 0.0; self.roller_back = 0.0;
        self.target_rad = 0.0; self.launching = false; self.ready = false; self.feeding_now = false;
        self.va = None;
        for tr in [self.t.rollers, self.t.launcher] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = None; }
        for sh in [self.t.rotor, self.t.primary, self.t.bar] { d.shafts[sh].omega_ = 0.0; d.shafts[sh].cmd = Some(0.0); }
        d.shafts[self.t.rotor].vel = None;
    }

    /// js `_front()`: the box's front now
    fn front(ext: f64) -> f64 { boxx::front() - boxx::EXT + ext }
    /// js `where(x, y, z)`
    pub fn where_(ext: f64, [x, y, z]: [f64; 3]) -> Where {
        let xf = Self::front(ext);
        let in_frame = x > bay_x0() - 0.02 && x < half_l() + 0.01 && m::abs(z) < bay_hw() + 0.01 && y > 0.1 && y < 0.8;
        let in_box = ext > 0.05 && x >= half_l() && x < xf + 0.01 && m::abs(z) < boxx::side() + 0.01 && y < 0.8;
        let at_lip = ext > boxx::EXT - 0.02 && x >= xf && x < xf + 0.09 && m::abs(z) < boxx::side() && y < 0.2;
        Where {
            own: in_frame || in_box || at_lip,
            held: in_frame || (in_box && y > 0.12 && x < xf - 0.03),
            near: x > bay_x0() - 0.1 && x < xf + 0.15 && m::abs(z) < boxx::side() + 0.1 && y < 0.9,
        }
    }
    /// js `shooterBalls(near)`'s test: up the hub's bore and in the turret
    pub fn shooter_ball([x, y, z]: [f64; 3]) -> bool { y > 0.2 && m::hypot2(x - AXIS[0], z - AXIS[1]) < 0.3 }

    /// js `driveBox(st, slide, dt, { on, agitate })`
    fn drive_box(&mut self, dt: f64, on: bool, agitate: bool) {
        let s = self.t.slide;
        let d = &mut self.p.drives;
        if self.ext >= boxx::EXT - 1e-6 { d.stop(s, f64::NEG_INFINITY, 0.0); } else if self.ext <= 1e-6 { d.stop(s, 0.0, f64::INFINITY); } else { d.stop(s, f64::NEG_INFINITY, f64::INFINITY); }
        if !on { d.shafts[s].cmd = None; self.stroke = Stroke::Out; return; }
        let (mut target, mut vmax) = (boxx::EXT, 12.0);
        if agitate {
            self.stroke_t = Some(self.stroke_t.unwrap_or(0.0) + dt);
            self.slow_t = Some(if m::abs(d.omega(s) * rack::pinion()) < stroke::SLOW { self.slow_t.unwrap_or(0.0) + dt } else { 0.0 });
            if self.stroke != Stroke::In && self.ext >= boxx::EXT - stroke::TOL { self.stroke = Stroke::In; self.stroke_t = Some(0.0); self.slow_t = Some(0.0); self.strokes += 1; }
            else if self.stroke == Stroke::In && (self.ext <= stroke::IN + stroke::TOL || (self.stroke_t.unwrap() > stroke::STALL && self.slow_t.unwrap() > stroke::STALL)) { self.stroke = Stroke::Out; self.stroke_t = Some(0.0); self.slow_t = Some(0.0); }
            if self.stroke == Stroke::In { target = stroke::IN; vmax = stroke::VOLTS; } else { vmax = stroke::OUT_VOLTS; }
        } else { self.stroke = Stroke::Out; }
        d.shafts[s].cmd = Some(clamp(BOX_KP * (target - self.ext) * turns_per_m(), -vmax, vmax) / 12.0);
    }
    /// js `moveBox(st, slide, dt)`
    fn move_box(&mut self, dt: f64) {
        let s = self.t.slide;
        let d = &mut self.p.drives;
        self.ext += d.omega(s) * rack::pinion() * dt;
        if self.ext < 0.0 { self.ext = 0.0; if d.shafts[s].omega_ < 0.0 { d.shafts[s].omega_ = 0.0; } }
        if self.ext > boxx::EXT { self.ext = boxx::EXT; if d.shafts[s].omega_ > 0.0 { d.shafts[s].omega_ = 0.0; } }
    }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, dt: f64, r: &PreIn) {
        if let Some(ld) = self.last_deploy {
            if m::abs(r.intake_deploy - ld) > 1e-6 { self.ext = boxx::EXT * r.intake_deploy; self.deployed = r.intake_deploy > 0.5; self.p.drives.shafts[self.t.slide].omega_ = 0.0; }
        }
        if r.enabled { self.deployed = true; }
        let (on, ag) = (r.enabled && self.deployed, self.agitating);
        self.drive_box(dt, on, ag);
        self.hood_set = profile604(self.hood_set[0], self.hood_set[1], self.hood_goal * DEG, shooter::HOOD_V, shooter::HOOD_A, dt);
        self.hood_deg = self.hood_set[0] / DEG;
        let nf = self.n_feed;
        place_intake(&mut self.p.path.wheels[nf..], self.ext);
        place_box(&mut self.p.surfaces[S_BOX], self.ext);
        place_rotor_surfaces(&mut self.p.surfaces, self.phi);
        place_rotor(&mut self.p.path.wheels[..nf], &self.feed_home, self.phi);
        let yaw = r.turret_yaw;
        place_turret_surfaces(&mut self.p.surfaces, yaw, self.hood_deg);
        let sh = self.p.shooter.as_mut().unwrap();
        place_shooter(&mut sh.wheels, &self.shooter_home, yaw, self.hood_deg);
    }
    /// js `PhysicalRobot.stepMechanisms` (at its yield)
    pub fn mechanisms(&mut self, c: &mut Ctx, dt: f64, t: f64) {
        let ext = self.ext;
        self.p.mech_prepare(c, t, &|l| Self::where_(ext, l), &Self::shooter_ball);
        self.p.mech_run(dt, &mut c.phys.bodies);
        self.p.mech_finish(c.fuel);
    }
    /// js `preSteps` after its yield; returns the robot's intakeDeploy
    pub fn post_mech(&mut self, dt: f64) -> f64 {
        self.move_box(dt);
        self.phi = (self.phi + self.p.drives.omega(self.t.rotor) * dt) % TAU;
        let d = self.ext / boxx::EXT;
        self.last_deploy = Some(d);
        d
    }

    /// js `_aim(dt, feed)`: the shot's heading (aimPsi), speed and elevation; the flywheel speed and
    /// hood that make it
    fn aim(&mut self, c: &Ctx, dt: f64, feed: bool, r: &PostIn) -> (f64, f64) {
        let b = &c.phys.bodies[c.body];
        let (v, w) = (r.est_vel, r.est_omega);
        let k = 1.0 - m::pow(0.5, dt / 0.02);
        let va = self.va.get_or_insert([v[0], v[1], 0.0, 0.0]);
        va[2] += k * ((v[0] - va[0]) / dt - va[2]);
        va[3] += k * ((v[1] - va[1]) / dt - va[3]);
        va[0] = v[0]; va[1] = v[1];
        let (ax, az) = (va[2], va[3]);
        let l = LATENCY;
        let ct = b.translation();
        let (cx, cz) = (ct.x as f64, ct.z as f64);
        let yaw = r.yaw + w * l;
        let rq = b.rotation();
        let q = [rq.x as f64, rq.y as f64, rq.z as f64, rq.w as f64];
        let up_y = 1.0 - 2.0 * (q[0] * q[0] + q[2] * q[2]);
        self.tilt = m::acos(clamp(up_y, -1.0, 1.0));
        self.tilt_lock = self.tilt > tilt_lock() && m::hypot2(v[0], v[1]) > TILT_LOCK_SPEED;
        self.tilt_comped = self.tilt_comp && !self.tilt_lock && self.tilt > tilt_min();
        let px = cx + v[0] * l + 0.5 * ax * l * l;
        let pz = cz + v[1] * l + 0.5 * az * l * l;
        let ox = AXIS[0] * m::cos(yaw) + AXIS[1] * m::sin(yaw);
        let oz = -AXIS[0] * m::sin(yaw) + AXIS[1] * m::cos(yaw);
        let (mut tx, mut tz) = (px + ox, pz + oz);
        let mut dh = 0.0;
        if self.tilt_comped && self.tilt_pos {
            let flyp = qrot(q, [AXIS[0], fly::Y, AXIS[1]]);
            let (c0, s0) = (m::cos(r.yaw), m::sin(r.yaw));
            let low = r.wheel_pts.iter().map(|p| qrot(q, [p[0], 0.0, p[1]])[1]).fold(f64::INFINITY, m::min);
            tx += flyp[0] - (AXIS[0] * c0 + AXIS[1] * s0);
            tz += flyp[2] - (-AXIS[0] * s0 + AXIS[1] * c0);
            dh = flyp[1] - low - fly::Y;
        }
        self.tilt_dh = dh;
        let vtx = v[0] + ax * l + w * oz;
        let vtz = v[1] + az * l - w * ox;
        let tgt = if feed { r.target.unwrap_or(r.aim_hub) } else { r.aim_hub };
        let (dx, dz) = (tgt[0] - tx, tgt[1] - tz);
        let d = m::hypot2(dx, dz);
        let g = gen4414();
        let (v0, el);
        if feed {
            el = fit_of(&g.fit.el, 450.0, PASS_HOOD);
            let y0 = fit_of(&g.fit.y, 450.0, PASS_HOOD) + dh;
            let x0 = fit_of(&g.fit.x, 450.0, PASS_HOOD);
            let s = r.spin;
            let (xs, ys) = (&mut self.xs, &mut self.ys);
            let xs = std::cell::RefCell::new((xs, ys));
            let reach = |sp: f64| { let mut b = xs.borrow_mut(); let (xs, ys) = &mut *b; sim2d(y0, sp, el * DEG, R, 0.0, (s * sp) / R, xs, ys).map_or(0.0, |f| f.d) + x0 };
            let vmax = fit_of(&g.fit.v, RADS[1], PASS_HOOD);
            v0 = if reach(vmax) < d { vmax } else { bisect(&reach, 3.0, 12.0, d) };
        } else {
            let (mut vv, mut ee) = launch_for(d);
            if dh != 0.0 {
                for _ in 0..2 {
                    let (rad, hood) = shooter_for(vv, ee);
                    let y0 = fit_of(&g.fit.y, rad, hood);
                    let w0 = (r.spin * vv) / R;
                    let a = sim2d(y0, vv, ee * DEG, HUB::rimFront, 0.0, w0, &mut self.xs, &mut self.ys).map(|f| f.d);
                    let z = sim2d(y0 + dh, vv, ee * DEG, HUB::rimFront, 0.0, w0, &mut self.xs, &mut self.ys).map(|f| f.d);
                    let (Some(a), Some(z)) = (a, z) else { break };
                    let de = d - (z - a);
                    (vv, ee) = launch_for(de);
                }
            }
            v0 = vv; el = ee;
        }
        let dd = if d == 0.0 { 1.0 } else { d };
        let (ux, uz) = (dx / dd, dz / dd);
        let sx = v0 * m::cos(el * DEG) * ux - vtx;
        let sz = v0 * m::cos(el * DEG) * uz - vtz;
        let sy = v0 * m::sin(el * DEG);
        let vr = m::hypot3(sx, sy, sz);
        self.aim_dist = d;
        self.aim_psi = Some(m::atan2(-sz, sx));
        self.launch_angle = if vr > 0.0 { m::asin(sy / vr) / DEG } else { 90.0 };
        self.launch_v = vr;
        if self.tilt_comped && vr > 0.0 {
            let [bx, by, bz] = qrot_inv(q, [sx, sy, sz]);
            self.aim_psi = Some(r.yaw + m::atan2(-bz, bx));
            self.launch_angle = m::asin(clamp(by / vr, -1.0, 1.0)) / DEG;
        }
        shooter_for(vr, self.launch_angle)
    }

    /// js `postStep(dt, t, { on, feeding, outtake, setpoint, mode })`; returns the robot's stored
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut) -> Vec<usize> {
        let stored = self.p.account(c, t, i.pass, i.legal, stats);
        let out_ = self.ext > boxx::EXT - 0.02;
        let intaking = i.on && i.intake && out_;
        let unjam = i.on && i.outtake;
        let feed = i.pass;
        let launch = i.on && i.feeding && i.setpoint > 0.0 && i.has_shot && !unjam;
        let cmd = if !i.on { 0.0 } else if unjam { -0.6 } else if intaking || launch { 1.0 } else { 0.0 };
        self.p.drives.shafts[self.t.primary].cmd = Some(cmd);
        self.p.drives.shafts[self.t.bar].cmd = Some(cmd);
        let (rad, hood) = self.aim(c, dt, feed, i);
        self.launching = launch;
        if launch { self.target_rad = rad; self.hood_goal = clamp(hood, shooter::HOOD_MIN, shooter::HOOD_MAX); } else { self.target_rad = 0.0; self.hood_goal = shooter::HOOD_STOW; }
        let d = &mut self.p.drives;
        let lm = &d.trains[self.t.launcher].motor;
        let kv = 12.0 / (lm.m.free / TAU);
        let ratio = lm.ratio;
        let fly_w = d.omega(self.t.fly);
        d.trains[self.t.launcher].cmd = if self.target_rad > 0.0 { Some(velocity_duty(ratio, fly_w, self.target_rad, kv, 0.3)) } else { None };
        let at_v = self.target_rad > 0.0 && m::abs(fly_w - self.target_rad) < (if feed { 0.06 } else { 0.02 }) * self.target_rad;
        let at_hood = m::abs(self.hood_deg - self.hood_goal) < 1.5;
        let tol = if feed { 6.0 * DEG } else { m::max(0.8 * DEG, m::atan2(0.15, if self.aim_dist == 0.0 { 1.0 } else { self.aim_dist })) };
        let at_yaw = m::abs(i.aim_err.unwrap_or(f64::INFINITY)) < tol;
        self.ready = launch && at_v && at_hood && at_yaw && !self.tilt_lock;
        self.not_ready = Some(if self.ready { 0.0 } else { self.not_ready.unwrap_or(1.0) + dt });
        self.feeding_now = i.on && launch && (self.ready || (self.feeding_now && self.not_ready.unwrap() < 0.3));
        self.agitating = launch && !i.intake;
        self.intaking_now = intaking;
        self.rotor(dt, i.on, unjam, stats);
        let pc = self.p.drives.shafts[self.t.primary].cmd.unwrap_or(0.0);
        *out = PostOut {
            hood_deg: hood_elevation(self.hood_deg),
            flywheel: self.exit_speed(),
            ready: self.ready,
            feeding: if self.feeding_now { 1.0 } else { 0.0 },
            intake_speed: if pc > 0.0 { 1.0 } else if pc < 0.0 { -1.0 } else { 0.0 },
        };
        stored
    }

    /// js `_rotor(dt, on, unjam)`: the Dye Rotor and its rollers
    fn rotor(&mut self, dt: f64, on: bool, unjam: bool, stats: &mut Stats) {
        let (rs, rt) = (self.t.rotor, self.t.rollers);
        let d = &mut self.p.drives;
        if !on { rotor_loop(d, rs, None); d.shafts[rs].cmd = Some(0.0); d.trains[rt].cmd = Some(0.0); return; }
        let cur = d.shafts[rs].motor.as_ref().unwrap().current;
        if self.feeding_now && m::abs(cur) > 70.0 && m::abs(d.omega(rs)) < 2.0 { self.rotor_jam += dt; } else { self.rotor_jam = 0.0; }
        if self.rotor_jam > 0.3 { self.rotor_back = 0.25; self.rotor_jam = 0.0; stats.unjams += 1; }
        let rm = &d.trains[rt].motor;
        let free = rm.free_speed();
        if self.feeding_now && m::abs(rm.current) / rm.count > 50.0 && d.trains[rt].omega < 0.2 * free { self.roller_jam += dt; } else { self.roller_jam = 0.0; }
        if self.roller_jam > 0.3 { self.roller_back = 0.15; self.roller_jam = 0.0; stats.unjams += 1; }
        self.rotor_back = m::max(0.0, self.rotor_back - dt);
        self.roller_back = m::max(0.0, self.roller_back - dt);
        if unjam { rotor_loop(d, rs, Some(-0.5)); d.trains[rt].cmd = Some(-0.5); return; }
        let rps = if self.rotor_back > 0.0 { -1.0 } else if self.feeding_now { ROTOR_RPS } else if self.intaking_now { INTAKE_RPS } else { AGITATE_RPS };
        rotor_loop(d, rs, Some(rps));
        d.trains[rt].cmd = if self.roller_back > 0.0 { Some(-0.6) } else if self.feeding_now { Some(1.0) } else { None };
    }

    /// js `extNow()`
    pub fn ext_now(&self) -> f64 { self.ext }
    /// js `rotorTurn()`, `hoodTurn()`
    pub fn rotor_turn(&self) -> f64 { self.phi }
    pub fn hood_turn(&self) -> f64 { (self.hood_deg - shooter::HOOD_STOW) * DEG }
    /// js `exitSpeed()`
    pub fn exit_speed(&self) -> f64 {
        let w = self.p.drives.omega(self.t.fly);
        if w > 100.0 { fit_of(&gen4414().fit.v, clamp(w, RADS[0], RADS[1]), self.hood_deg) } else { 0.0 }
    }
    /// js `loadTop()`: the top of its load
    pub fn load_top(&mut self, phys: &PhysicsWorld, fuel: &crate::fuel::Fuel, body: RigidBodyHandle) -> f64 {
        let mut top = 0.0;
        for k in 0..self.p.held.len() {
            let b = self.p.held[k];
            let y = self.p.local(phys, body, fuel.balls[b].pos)[1];
            if y + R > top { top = y + R; }
        }
        top
    }
    /// js `capacity()`
    pub fn capacity(&self) -> usize { 62 }
    /// js `turretStep(cur, goal, dt, t)`: the turret on its motor
    pub fn turret_step(&mut self, cur: f64, goal: f64, dt: f64, t: f64) -> f64 {
        self.turret.get_or_insert_with(drive4414).step(cur, goal, dt, Some(t))
    }
}
