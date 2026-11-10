//! js/mech/phys1690.js: 1690's Kepler in play, built of its physical mechanisms (r1690.rs). Their
//! code isn't public: its controls are what the team said about how it runs, the rest estimates.
//! The carriage slides out on its racks and agitates while it shoots; the kicker bar falls on its pin;
//! the turret aims itself (on its motor against its swivel chain), the wheel's and indexer's
//! velocity loops feed once it's there; the top plate bows under the load.
use crate::ballistics::{solve_moving_shot, ShotTable};
use crate::constants::*;
use crate::mech::drives::{Drives, VelLoop};
use crate::mech::mechanism::{Mechanism, Toucher, Touch};
use crate::mech::physical::{Ctx, FlatOut, Physical, Stats, Where};
use crate::mech::r1690::*;
use crate::mech::r1690::{HALF_L, HALF_W};
use crate::mech::turret_drive::{drive1690, TurretDrive};
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const TAU: f64 = 2.0 * m::PI;
const R: f64 = FUEL::radius;
/// js RPS: the indexer's speeds (their motors' RPS), scoring and passing
const RPS_FLOOR: [f64; 2] = [75.0, 85.0];
const RPS_BELTS: [f64; 2] = [95.0, 100.0];
const RPS_VKICK: f64 = 60.0;
/// js FEED_SHARE, FEED_MIN, LATENCY
const FEED_SHARE: f64 = 0.6;
const FEED_MIN: f64 = 5.0;
const LATENCY: f64 = 0.05;
/// js EXIT, EL_MAP, EXIT_MAP
const EXIT_S: f64 = 0.03;
const EXIT_Y: f64 = 0.66;
const EL_MAP: [[f64; 2]; 13] = [[125.0, 37.4], [144.7, 57.6], [145.2, 57.9], [146.5, 59.0], [147.6, 60.0], [148.7, 61.0], [151.1, 63.0], [152.1, 63.9], [154.5, 65.9], [157.3, 68.6], [161.0, 72.0], [164.0, 75.0], [172.0, 84.9]];
const EXIT_MAP: [[f64; 2]; 16] = [[0.0, 0.0], [4.0, 4.84], [5.95, 7.22], [6.04, 7.34], [6.4, 7.85], [6.77, 8.45], [7.17, 9.16], [7.55, 9.81], [7.87, 10.35], [8.25, 11.04], [8.57, 11.67], [8.95, 12.33], [9.44, 15.75], [9.72, 17.22], [10.65, 20.0], [10.7, 24.0]];
fn interp(t: &[[f64; 2]], x: f64, a: usize, b: usize) -> f64 {
    if x <= t[0][a] { return t[0][b]; }
    for k in 1..t.len() {
        if x <= t[k][a] { return t[k - 1][b] + ((t[k][b] - t[k - 1][b]) * (x - t[k - 1][a])) / (t[k][a] - t[k - 1][a]); }
    }
    t[t.len() - 1][b]
}
fn clamp(x: f64, a: f64, b: f64) -> f64 { m::max(a, m::min(b, x)) }
fn hood_range() -> [f64; 2] { [shooter::EL_MIN + 90.0, shooter::el_max() + 90.0] }
/// js `hoodFor(el)`: the hood angle (deg about the axle) for an elevation (EL_MAP is in order of both)
pub fn hood_for(el: f64) -> f64 { let [a, b] = hood_range(); clamp(interp(&EL_MAP, el, 1, 0), a, b) }
/// js `elevationAt(h)`, `surfFor(v)`, `exitAt(surf)`
pub fn elevation_at(h: f64) -> f64 { interp(&EL_MAP, h, 0, 1) }
pub fn surf_for(v: f64) -> f64 { interp(&EXIT_MAP, v, 0, 1) }
pub fn exit_at(surf: f64) -> f64 { interp(&EXIT_MAP, surf, 1, 0) }
/// js HOOD_STOW: its steep end
pub fn hood_stow() -> f64 { hood_range()[1] }
/// js TURNS_PER_M, SLIDE_KP, STROKE, HOLD_V, FLOOR_INTAKE
fn turns_per_m() -> f64 { slide::RATIO / (2.0 * m::PI * slide::PINION) }
const SLIDE_KP: f64 = 4.0;
const STROKE_HZ: f64 = 4.0;
const STROKE_DEPTH: f64 = 0.08;
const STROKE_VOLTS: f64 = 12.0;
const FLOOR_INTAKE: f64 = 75.0;
const HOLD_V: f64 = 0.5;

/// js phys1706.js `profile(p, v, goal, vmax, amax, dt)`
fn profile(p: f64, v: f64, goal: f64, vmax: f64, amax: f64, dt: f64) -> [f64; 2] {
    let d = goal - p;
    let dir = m::sign(d);
    if m::abs(d) < 1e-6 && m::abs(v) < amax * dt { return [goal, 0.0]; }
    let want = dir * m::min(vmax, m::sqrt(2.0 * amax * m::abs(d)));
    let nv = v + clamp(want - v, -amax * dt, amax * dt);
    let np = p + nv * dt;
    if (goal - np) * dir < 0.0 { return [goal, 0.0]; }
    [np, nv]
}
/// What a speed loop drives: a shaft or a train
#[derive(Clone, Copy)]
enum Drv { S(usize), T(usize) }
/// js phys1706.js `speedLoop(a, target, kP, kV)` (kV: the motor's own, 12 V over its free speed)
fn speed_loop(d: &mut Drives, a: Drv, target: Option<f64>, kp: f64) {
    let free = match a { Drv::S(s) => d.shafts[s].motor.as_ref().unwrap().m.free, Drv::T(t) => d.trains[t].motor.m.free };
    let kv = (12.0 * 2.0 * m::PI) / free;
    let vel = target.map(|target| VelLoop { target, kv, kp, ks: 0.0, lo: -12.0, hi: 12.0, per: None, rate: 1000.0 });
    let cmd = target.map(|_| 0.0);
    match a { Drv::S(s) => { d.shafts[s].vel = vel; d.shafts[s].cmd = cmd; } Drv::T(t) => { d.trains[t].vel = vel; d.trains[t].cmd = cmd; } }
}
fn ratio(d: &Drives, a: Drv) -> f64 { match a { Drv::S(s) => d.shafts[s].motor.as_ref().unwrap().ratio, Drv::T(t) => d.trains[t].motor.ratio } }

/// js `floorRoll(contacts, dt)`: the foam's rolling resistance on the floor's rollers, against each
/// FUEL's spin about their axis
fn floor_roll(contacts: &[Touch], wheels: &[crate::mech::mechanism::Wheel], bodies: &mut RigidBodySet, dt: f64) {
    for c in contacts {
        let Toucher::Wheel(i) = c.wheel else { continue };
        let n = &wheels[i].name;
        if !(n.starts_with("floor roller") || n.starts_with("top roller")) || !(c.force > 0.0) { continue; }
        let b = &mut bodies[c.ball];
        let w = b.angvel();
        let dw = (ROLL_RR * c.force * R * dt) / (0.4 * FUEL::mass * R * R);
        let wz = m::sign(w.z as f64) * m::max(0.0, m::abs(w.z as f64) - dw);
        b.set_angvel(Vector::new(w.x, w.y, wz as f32), true);
    }
}

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool, pub intake_deploy: f64, pub turret_yaw: f64 }
/// What its postStep reads of the robot
pub struct PostIn<'a> {
    pub on: bool, pub feeding: bool, pub outtake: bool, pub pass: bool, pub legal: bool, pub intake: bool,
    /// its shot's target (None: no shot), and where it aims at the HUB
    pub target: Option<[f64; 2]>, pub aim_hub: [f64; 2],
    pub table: &'a ShotTable, pub est_vel: [f64; 2], pub est_omega: f64, pub yaw: f64, pub pos: [f64; 3], pub aim_err: Option<f64>,
}
pub use crate::mech::phys2910::PostOut;

pub struct Phys1690 {
    pub p: Physical,
    pub t: Trains,
    intake_home: Vec<[f64; 3]>,
    n_floor: (usize, usize),
    pub deploy: f64,
    pub pos: f64,
    pub kick: f64,
    kick_max: Option<f64>,
    deployed: bool,
    agitating: bool,
    stroke_t: Option<f64>,
    strokes: u32,
    pub hood: f64,
    hood_set: [f64; 2],
    hood_goal: f64,
    feeding_now: bool,
    slow_t: f64, jam_t: f64, back_t: f64,
    pub ready: bool,
    /// js aimPsi: where it wants the turret (None: no shot)
    pub aim_psi: Option<f64>,
    last_deploy: Option<f64>,
    flat: Option<FlatOut>,
    turret: Option<TurretDrive>,
}

impl Phys1690 {
    /// js `new Physical1690(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = Drives::default();
        let t = build1690(&mut d);
        let w = wheels1690(&t);
        let surfaces = path_surfaces1690(&t);
        let sw = shooter_wheels(&t);
        let n_floor = (w.intake.len(), w.intake.len() + w.floor.len());
        let mut path_wheels = w.intake;
        path_wheels.extend(w.floor);
        path_wheels.push(w.vkick);
        path_wheels.extend(w.feed);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.extend([t.belts[0], t.belts[1], t.slide, t.kick_arm]);
        let path_surfs = vec![S_BODY, S_BODY_FINE, S_PTFE, S_BOTTOM, S_TOP, S_CARRIAGE, S_KICK, S_KICK_TUBE, S_FUNNEL_LID, S_BUMPER, S_NET];
        let path = Mechanism::new(Some(body), shafts, path_wheels, path_surfs, 6, 1);
        let shooter = Mechanism::new(Some(body), vec![t.wheel, t.hood_wheels[0], t.hood_wheels[1]], sw, vec![S_TURRET, S_HOOD], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), preload(), None);
        let mut me = Phys1690 {
            p, t, intake_home: w.intake_home, n_floor, deploy: 0.0, pos: 0.0, kick: 0.0, kick_max: None, deployed: false, agitating: false, stroke_t: None, strokes: 0,
            hood: 0.0, hood_set: [0.0; 2], hood_goal: 0.0, feeding_now: false, slow_t: 0.0, jam_t: 0.0, back_t: 0.0, ready: false, aim_psi: None,
            last_deploy: None, flat: None, turret: None,
        };
        me.reset();
        me
    }

    /// js `reset()`
    pub fn reset(&mut self) {
        self.p.reset();
        self.deploy = 0.0; self.pos = 0.0; self.kick = 0.0; self.deployed = false; self.agitating = false; self.stroke_t = None;
        let hs = hood_stow();
        self.hood = hs; self.hood_set = [hs, 0.0]; self.hood_goal = hs;
        self.feeding_now = false; self.slow_t = 0.0; self.jam_t = 0.0; self.back_t = 0.0; self.ready = false;
        self.aim_psi = None;
        let t = &self.t;
        let d = &mut self.p.drives;
        for tr in [t.intake, t.floor, t.feed, t.shooter] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = None; d.trains[tr].vel = None; }
        for sh in [t.kick_arm, t.vkick, t.slide, t.belts[0], t.belts[1]] { d.shafts[sh].omega_ = 0.0; d.shafts[sh].cmd = None; d.shafts[sh].vel = None; }
    }

    /// js `where(x, y, z)` (deploy: the carriage's, as it is now)
    pub fn where_(deploy: f64, [x, y, z]: [f64; 3]) -> Where {
        let in_frame = x > -HALF_L - 0.02 && x < HALF_L && m::abs(z) < HALF_W && y > 0.09 && y < 0.8;
        let out = deploy > 0.5;
        let in_front = out && x >= HALF_L && x < intake::FRAME_FRONT + 0.01 && m::abs(z) < intake::FRAME_SIDES && y < 0.62;
        Where {
            own: in_frame || in_front,
            held: in_frame || (in_front && y > 0.20 && x < intake::FRAME_FRONT - 0.04),
            near: x > -HALF_L - 0.1 && x < intake::FRAME_FRONT + 0.15 && m::abs(z) < HALF_W + 0.1 && y < 0.9,
        }
    }
    /// js `shooterBalls(near)`'s test: up in the turret
    pub fn shooter_ball([x, y, z]: [f64; 3]) -> bool { y > TURRET_BORE_Y0 && m::hypot2(x - TURRET_AT[0], z - TURRET_AT[1]) < 0.25 }
    /// js `_front()`
    fn front(&self) -> f64 { intake_front(self.deploy) }

    /// js `driveCarriage(st, slide, dt, { on, agitate })`
    fn drive_carriage(&mut self, dt: f64, on: bool, agitate: bool) {
        let tr = slide::travel();
        let s = self.t.slide;
        let d = &mut self.p.drives;
        if self.pos >= tr - 1e-6 { d.stop(s, f64::NEG_INFINITY, 0.0); } else if self.pos <= 1e-6 { d.stop(s, 0.0, f64::INFINITY); } else { d.stop(s, f64::NEG_INFINITY, f64::INFINITY); }
        if !on { d.shafts[s].cmd = None; self.stroke_t = None; return; }
        let mut target = tr;
        if agitate {
            let per = 1.0 / STROKE_HZ;
            let t0 = self.stroke_t.unwrap_or(0.0);
            let st = t0 + dt;
            self.stroke_t = Some(st);
            if m::floor(st / per) > m::floor(t0 / per) || t0 == 0.0 { self.strokes += 1; }
            if st % per < per / 2.0 { target = tr - STROKE_DEPTH; }
        } else { self.stroke_t = None; }
        let mut v = clamp(SLIDE_KP * (target - self.pos) * turns_per_m(), -STROKE_VOLTS, STROKE_VOLTS);
        if target >= tr { v = m::max(v, HOLD_V); }
        d.shafts[s].cmd = Some(v / 12.0);
    }
    /// js `moveCarriage(st, slide, dt)`
    fn move_carriage(&mut self, dt: f64) {
        let s = self.t.slide;
        let d = &mut self.p.drives;
        self.pos += d.omega(s) * slide::PINION * dt;
        let tr = slide::travel();
        if self.pos < 0.0 { self.pos = 0.0; if d.shafts[s].omega_ < 0.0 { d.shafts[s].omega_ = 0.0; } }
        if self.pos > tr { self.pos = tr; if d.shafts[s].omega_ > 0.0 { d.shafts[s].omega_ = 0.0; } }
    }
    /// js `driveKick(st, arm, ext)`
    fn drive_kick(&mut self) {
        let a = self.t.kick_arm;
        let lim = kick_at(self.deploy);
        let d = &mut self.p.drives;
        if self.kick < lim { self.kick = lim; }
        if self.kick <= lim + 1e-6 { d.stop(a, 0.0, f64::INFINITY); } else if self.kick >= 0.0 { d.stop(a, f64::NEG_INFINITY, 0.0); } else { d.stop(a, f64::NEG_INFINITY, f64::INFINITY); }
        d.shafts[a].load = kick_load(self.kick);
        d.shafts[a].cmd = None;
    }
    /// js `moveKick(st, arm, dt, ext)`
    fn move_kick(&mut self, dt: f64) {
        let a = self.t.kick_arm;
        let d = &mut self.p.drives;
        self.kick += (d.omega(a) * dt) / DEG;
        let lim = kick_at(self.deploy);
        if self.kick < lim { self.kick = lim; if d.shafts[a].omega_ < 0.0 { d.shafts[a].omega_ = 0.0; } }
        if self.kick > 0.0 { self.kick = 0.0; if d.shafts[a].omega_ > 0.0 { d.shafts[a].omega_ = 0.0; } }
        self.kick_max = Some(m::max(self.kick_max.unwrap_or(f64::NEG_INFINITY), self.kick - intake::KICK_FALL));
    }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, dt: f64, r: &PreIn) {
        if let Some(ld) = self.last_deploy {
            if m::abs(r.intake_deploy - ld) > 1e-6 {
                self.pos = slide::travel() * r.intake_deploy; self.kick = kick_at(r.intake_deploy); self.deployed = r.intake_deploy > 0.5;
                self.p.drives.shafts[self.t.slide].omega_ = 0.0; self.p.drives.shafts[self.t.kick_arm].omega_ = 0.0;
            }
        }
        if r.enabled { self.deployed = true; }
        let (on, ag) = (r.enabled && self.deployed, self.agitating);
        self.drive_carriage(dt, on, ag);
        self.deploy = self.pos / slide::travel();
        self.drive_kick();
        self.hood_set = profile(self.hood_set[0], self.hood_set[1], self.hood_goal, shooter::HOOD_V, shooter::HOOD_A, dt);
        self.hood = self.hood_set[0];
        self.place(r.turret_yaw);
    }
    /// js `_place(yaw)`
    fn place(&mut self, yaw: f64) {
        let (deploy, kick) = (self.deploy, self.kick);
        let n = self.intake_home.len();
        place_intake(&mut self.p.surfaces, &mut self.p.path.wheels[..n], &self.intake_home, deploy, kick);
        let x1 = m::max(HALF_L, self.front());
        self.p.surfaces[S_NET].plate_mut().unwrap().x1 = x1;
        place_turret(&mut self.p.surfaces, &mut self.p.shooter.as_mut().unwrap().wheels, yaw, self.hood);
    }
    /// js `PhysicalRobot.stepMechanisms` (at its yield)
    pub fn mechanisms(&mut self, c: &mut Ctx, dt: f64, t: f64) {
        let deploy = self.deploy;
        self.p.mech_prepare(c, t, &|l| Self::where_(deploy, l), &Self::shooter_ball);
        self.p.mech_run(dt, &mut c.phys.bodies);
        self.p.mech_finish(c.fuel);
    }
    /// js `preSteps` after its yield; returns the robot's intakeDeploy
    pub fn post_mech(&mut self, bodies: &mut RigidBodySet, dt: f64) -> f64 {
        let contacts = std::mem::take(&mut self.p.path.contacts);
        self.p.surfaces[S_NET].plate_mut().unwrap().flex(dt, &contacts, S_NET);
        floor_roll(&contacts, &self.p.path.wheels, bodies, dt);
        self.p.path.contacts = contacts;
        self.move_carriage(dt);
        self.deploy = self.pos / slide::travel();
        self.move_kick(dt);
        self.last_deploy = Some(self.deploy);
        self.deploy
    }

    /// js `_aim(tgt, table)`: the shot, from where the FUEL will leave LATENCY ahead: (psi, el, v, d)
    fn aim(&self, c: &Ctx, tgt: [f64; 2], i: &PostIn) -> Option<(f64, f64, f64, f64)> {
        let b = &c.phys.bodies[c.body];
        let (v, w, l) = (i.est_vel, i.est_omega, LATENCY);
        let ct = b.translation();
        let yaw = i.yaw + w * l;
        let (cy, sy) = (m::cos(yaw), m::sin(yaw));
        let [tx, tz] = TURRET_AT;
        let (ox, oz) = (tx * cy + tz * sy, -tx * sy + tz * cy);
        let (px, pz) = (ct.x as f64 + v[0] * l + ox, ct.z as f64 + v[1] * l + oz);
        let (vx, vz) = (v[0] + w * oz, v[1] - w * ox);
        let (dx, dz) = (tgt[0] - px, tgt[1] - pz);
        let dd = { let h = m::hypot2(dx, dz); if h == 0.0 { 1.0 } else { h } };
        let exit = [px + (dx / dd) * EXIT_S, EXIT_Y, pz + (dz / dd) * EXIT_S];
        solve_moving_shot(i.table, exit, [vx, vz], tgt).map(|s| (s.psi, s.theta / DEG, s.v, s.dist))
    }

    /// js `postStep(dt, t, { on, feeding, outtake, mode })`; returns the robot's stored
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut) -> Vec<usize> {
        let stored = self.p.account(c, t, i.pass, i.legal, stats);
        let on = i.on;
        let intaking = on && i.intake;
        let unjam = on && i.outtake;
        let tt = self.t.intake;
        self.p.drives.trains[tt].cmd = if !on { Some(0.0) } else if unjam { Some(-0.6) } else if intaking || self.feeding_now || self.agitating { Some(1.0) } else { None };
        let pass = i.pass && i.target.is_some();
        let point = i.target.map(|tg| if pass { tg } else { i.aim_hub });
        let sol = if on && !unjam { point.and_then(|p| self.aim(c, p, i)) } else { None };
        self.agitating = sol.is_some() && !intaking && !unjam;
        self.aim_psi = sol.map(|s| s.0);
        let surf = sol.map_or(0.0, |s| surf_for(s.2));
        let in_trench = crate::mech::phys8793::under_trench(i.pos[0], i.pos[2], 0.45);
        self.p.surfaces[S_NET].plate_mut().unwrap().cap = if in_trench { m::max(0.0, TRENCH::clearHeight - 0.003 - WALL_TOP) } else { plate::MAX };
        let d = &mut self.p.drives;
        if let Some(s) = sol { self.hood_goal = if in_trench { hood_stow() } else { hood_for(s.1) }; speed_loop(d, Drv::T(self.t.shooter), Some(surf / shooter::R), shooter::KP); }
        else { self.hood_goal = hood_stow(); speed_loop(d, Drv::T(self.t.shooter), None, 0.2); }
        let wheel_surf = m::abs(d.omega(self.t.wheel)) * shooter::R;
        let flat = self.flat.get_or_insert_with(FlatOut::default).step(dt, wheel_surf, if pass { surf } else { 0.0 });
        let at_v = sol.is_some() && (m::abs(wheel_surf - surf) < shooter::TOL_V * surf || flat);
        let at_hood = m::abs(self.hood - self.hood_goal) < shooter::TOL_HOOD;
        let tol = if pass { 4.0 * DEG } else { m::max(0.8 * DEG, m::atan2(0.14, match sol { Some(s) if s.3 != 0.0 => s.3, _ => 1.0 })) };
        let at_yaw = m::abs(i.aim_err.unwrap_or(f64::INFINITY)) < tol;
        self.ready = sol.is_some() && at_v && at_hood && at_yaw && !in_trench;
        self.feeding_now = on && i.feeding && sol.is_some() && !unjam && !in_trench && at_yaw && at_hood && (at_v || self.feeding_now);
        self.slow_t = if sol.is_some() && wheel_surf < 0.92 * surf { self.slow_t + dt } else { 0.0 };
        let slow = if self.slow_t > 0.1 { 0.6 } else { 1.0 };
        let k = if pass { 1 } else { 0 };
        let rps = |d: &Drives, n: f64, a: Drv| (n * TAU) / ratio(d, a);
        let vk = Drv::S(self.t.vkick);
        let vks = &d.shafts[self.t.vkick];
        self.jam_t = if self.feeding_now && m::abs(vks.motor.as_ref().unwrap().current) > 60.0 && m::abs(vks.omega_) < 0.2 * rps(d, RPS_VKICK, vk) { self.jam_t + dt } else { 0.0 };
        if self.jam_t > 0.1 { self.back_t = 0.15; self.jam_t = 0.0; stats.unjams += 1; }
        self.back_t = m::max(0.0, self.back_t - dt);
        let (fl, feed) = (Drv::T(self.t.floor), Drv::T(self.t.feed));
        let belts = [Drv::S(self.t.belts[0]), Drv::S(self.t.belts[1])];
        if unjam {
            let v = -rps(d, 30.0, fl); speed_loop(d, fl, Some(v), 0.2);
            for b in belts { let v = -rps(d, 40.0, b); speed_loop(d, b, Some(v), 0.2); }
            let v = -rps(d, 30.0, vk); speed_loop(d, vk, Some(v), 0.2);
            let v = -rps(d, 30.0, feed); speed_loop(d, feed, Some(v), 0.2);
        } else if self.feeding_now {
            let v = rps(d, RPS_FLOOR[k], fl) * slow; speed_loop(d, fl, Some(v), 0.2);
            for b in belts { let v = rps(d, RPS_BELTS[k], b) * slow; speed_loop(d, b, Some(v), 0.2); }
            let v = (if self.back_t > 0.0 { -1.0 } else { 1.0 }) * rps(d, RPS_VKICK, vk); speed_loop(d, vk, Some(v), 0.2);
            speed_loop(d, feed, Some((m::max(FEED_MIN, FEED_SHARE * surf) / FEED_TOP_R) * slow), 0.2);
        } else {
            for a in [fl, belts[0], belts[1], vk, feed] { speed_loop(d, a, None, 0.2); }
            if intaking { let v = rps(d, FLOOR_INTAKE, fl); speed_loop(d, fl, Some(v), 0.2); }
        }
        let ic = d.trains[tt].cmd;
        *out = PostOut {
            hood_deg: elevation_at(self.hood),
            flywheel: exit_at(wheel_surf),
            ready: self.ready,
            feeding: if self.feeding_now { 1.0 } else { 0.0 },
            intake_speed: match ic { Some(x) if x > 0.0 => 1.0, Some(x) if x < 0.0 => -1.0, _ => 0.0 },
        };
        stored
    }

    /// js `extNow()`: how far the U-frame is out past the frame (m)
    pub fn ext_now(&self) -> f64 { m::max(0.0, self.front() - HALF_L) }
    /// js `capacity()`
    pub fn capacity(&self) -> usize { 50 }
    /// the top plate's bow (m), for drawing it
    pub fn bow(&self) -> f64 { self.p.surfaces[S_NET].plate_ref().unwrap().bow }
    /// js `turretStep(cur, goal, dt, t)`: the turret on its motor
    pub fn turret_step(&mut self, cur: f64, goal: f64, dt: f64, t: f64) -> f64 {
        self.turret.get_or_insert_with(drive1690).step(cur, goal, dt, Some(t))
    }
    /// js `mechPose()`: its moving parts, for drawing them: [name, value] as its drawing's rig names
    /// them (the carriage out 0-1, the kicker bar's fall and the hood's angle in rad; the wheels',
    /// rollers' and belts' speeds, rad/s and m/s)
    pub fn mech_pose(&self) -> [(&'static str, f64); 11] {
        let (d, t) = (&self.p.drives, &self.t);
        [("mech.ext", self.deploy), ("mech.kick", self.kick * DEG), ("mech.hood", self.hood * DEG),
         ("mech.wheel", d.omega(t.wheel)), ("mech.feeder", d.omega(t.hood_wheels[0])), ("mech.floor", d.trains[t.floor].omega),
         ("mech.belts.0", d.omega(t.belts[0]) * belt_pitch()), ("mech.belts.1", d.omega(t.belts[1]) * belt_pitch()),
         ("mech.vkick", d.omega(t.vkick)), ("mech.feed", d.trains[t.feed].omega), ("mech.intakeRollers", d.trains[t.intake].omega)]
    }
    /// the roller floor's speed, for drawing
    pub fn floor_spin(&self) -> f64 { self.p.drives.trains[self.t.floor].omega }
    #[allow(dead_code)]
    fn n_floor(&self) -> (usize, usize) { self.n_floor }
}
