//! js/mech/phys604.js: 604's Toploader in play, built of its physical mechanisms (r604.rs). Its
//! controls are its code's (frc604/2026-public): the intake down on Motion Magic and its rollers
//! once there, up to standby let go (stowed in AUTO and once it's shooting); TrackAndLaunch's aim
//! (LaunchCalculator: the turret ahead by their latency, their table's speed and angle, a 55 deg feed
//! out of their zone, less the turret's own velocity); the serializer only while the turret's ready
//! and outside their zones, the rotor following the rollers' measured speed; their jam detection.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, VelLoop};
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, FlatOut, Physical, Stats, Where};
use crate::mech::r604::*;
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const TAU: f64 = 2.0 * m::PI;
fn clamp(x: f64, a: f64, b: f64) -> f64 { m::max(a, m::min(b, x)) }
/// js FEED_PT, LATENCY, FIELD_W_IN
fn feed_pt() -> [f64; 2] { [100.0 * IN, 60.0 * IN] }
const LATENCY: f64 = 0.05;
const FIELD_W_IN: f64 = 317.0;
/// js EXIT_PER_MAIN_SURFACE
const EXIT_PER_MAIN_SURFACE: f64 = 0.9;

/// js `profile(p, v, goal, vmax, amax, dt)`: their Motion Magic's next setpoint
pub fn profile(p: f64, v: f64, goal: f64, vmax: f64, amax: f64, dt: f64) -> [f64; 2] {
    let d = goal - p;
    let dir = m::sign(d);
    if m::abs(d) < 1e-6 && m::abs(v) < amax * dt { return [goal, 0.0]; }
    let want = dir * m::min(vmax, m::sqrt(2.0 * amax * m::abs(d)));
    let nv = v + clamp(want - v, -amax * dt, amax * dt);
    let np = p + nv * dt;
    if (goal - np) * dir < 0.0 { return [goal, 0.0]; }
    [np, nv]
}
/// js `velocityDuty(motor, w, target, kV, kP)`
pub fn velocity_duty(mo: &Motor, w: f64, target: f64, kv: f64, kp: f64) -> f64 {
    let (tgt, rps) = ((target * mo.ratio) / TAU, (w * mo.ratio) / TAU);
    clamp((kv * tgt + kp * (tgt - rps)) / 12.0, -1.0, 1.0)
}

/// js ZONE (their FieldZones, blue-origin inches: [x0, x1, y0, y1])
fn zone(name: &str) -> [f64; 4] {
    match name {
        "TOWER" => [-1.0 / IN, 45.0, 110.0, 180.0],
        "BUMP" => [180.0, 210.0, 50.0, 317.0 - 50.0],
        _ => [182.11, 182.11 + 72.0, 317.0 / 2.0 - 40.0, 317.0 / 2.0 + 40.0],
    }
}
/// js `_fieldIn()`: the robot's pose in their field frame (inches)
fn field_in(a: Alliance, pos: [f64; 3]) -> [f64; 2] {
    let s = if a == Alliance::Blue { 1.0 } else { -1.0 };
    [(s * pos[0] + HALF_L) / IN, (HALF_W - s * pos[2]) / IN]
}
fn in_zone(a: Alliance, pos: [f64; 3], name: &str) -> bool {
    let [x, y] = field_in(a, pos);
    let [x0, x1, y0, y1] = zone(name);
    x >= x0 && x <= x1 && y >= y0 && y <= y1
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Jam { Normal, Jammed }

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool, pub intake_deploy: f64, pub turret_yaw: f64 }
/// What its postStep reads of the robot
pub struct PostIn {
    pub on: bool, pub feeding: bool, pub outtake: bool, pub setpoint: f64, pub pass: bool, pub legal: bool, pub intake: bool, pub auto: bool,
    pub has_shot: bool, pub aim_pass: [f64; 2], pub aim_hub: [f64; 2],
    pub est_vel: [f64; 2], pub est_omega: f64, pub yaw: f64, pub turret_yaw: f64, pub aim_err: Option<f64>, pub pos: [f64; 3],
}
/// What it tells the robot back, past PostOut: the turret where it put it (not launching), and its
/// drive's supply limit
pub struct PostOut604 { pub turret_yaw: Option<f64>, pub drive_supply: f64 }
pub use crate::mech::phys2910::PostOut;

pub struct Phys604 {
    pub p: Physical,
    pub t: Trains,
    intake_home: Vec<[f64; 3]>,
    rotor_home: Vec<([f64; 3], [f64; 3])>,
    n_rotor: (usize, usize),
    pub arm_deg: f64,
    arm_set: [f64; 2],
    arm_goal: f64,
    deployed_once: bool,
    pub hood_deg: f64,
    hood_set: [f64; 2],
    hood_goal: f64,
    pub phi: f64,
    target_roller: f64,
    roller_buf: Vec<f64>,
    target_rotor: f64,
    jam: Jam, jam_t: f64,
    compress: bool,
    pub ext: f64,
    pub ready: bool,
    va: Option<[f64; 4]>,
    flat: Option<FlatOut>,
    last_deploy: Option<f64>,
    aim_dist: f64,
    launch_angle: f64,
    launch_v: f64,
    /// js aimPsi: the turret's field heading to its shot (None before its first step)
    pub aim_psi: Option<f64>,
}

impl Phys604 {
    /// js `new Physical604(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = Drives::default();
        let t = build604(&mut d);
        let w = wheels604(&t);
        let surfaces = path_surfaces604(&t);
        let sw = shooter_wheels(&t);
        let mut path_wheels = w.intake;
        let n0 = path_wheels.len();
        let n_rotor = (n0, n0 + w.rotor.len());
        path_wheels.extend(w.rotor);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.extend([t.pivot, t.rotor]);
        let path = Mechanism::new(Some(body), shafts, path_wheels, vec![S_BODY, S_INTAKE, S_HOPPER, S_ROTOR, S_TURRET, S_HOOD, S_BUMPER], 6, 1);
        let shooter = Mechanism::new(Some(body), vec![t.main, t.hood_roller], sw, vec![S_TURRET, S_HOOD], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), PRELOAD.to_vec(), None);
        let mut me = Phys604 {
            p, t, intake_home: w.intake_home, rotor_home: w.rotor_home, n_rotor, arm_deg: 0.0, arm_set: [0.0; 2], arm_goal: 0.0, deployed_once: false,
            hood_deg: 0.0, hood_set: [0.0; 2], hood_goal: 0.0, phi: 0.0, target_roller: 0.0, roller_buf: Vec::new(), target_rotor: 0.0,
            jam: Jam::Normal, jam_t: 0.0, compress: false, ext: 0.0, ready: false, va: None, flat: None, last_deploy: None,
            aim_dist: 0.0, launch_angle: 0.0, launch_v: 0.0, aim_psi: None,
        };
        me.reset();
        me
    }

    /// js `reset()`: boots stowed, the hood on 15, the rotor at the CAD's pose
    pub fn reset(&mut self) {
        self.p.reset();
        self.arm_deg = intake::START; self.arm_set = [intake::START * DEG, 0.0]; self.arm_goal = intake::START; self.deployed_once = false;
        self.hood_deg = shooter::HOOD_MIN; self.hood_set = [shooter::HOOD_MIN * DEG, 0.0]; self.hood_goal = shooter::HOOD_MIN;
        self.phi = 0.0;
        self.target_roller = 0.0; self.roller_buf.clear(); self.target_rotor = 0.0;
        self.jam = Jam::Normal; self.jam_t = 0.0;
        self.compress = false;
        self.ready = false;
        self.va = None;
        let t = &self.t;
        let d = &mut self.p.drives;
        for tr in [t.intake, t.roller, t.launcher] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = None; }
        d.trains[t.roller].vel = None;
        for sh in [t.pivot, t.rotor] { d.shafts[sh].omega_ = 0.0; d.shafts[sh].cmd = Some(0.0); }
        self.ext = hopper_ext(self.arm_deg);
    }

    /// js `where(x, y, z)` (arm: its angle; ext: the box's)
    pub fn where_(arm: f64, ext: f64, [x, y, z]: [f64; 3]) -> Where {
        let h = half();
        let in_frame = m::abs(x) < h && m::abs(z) < h && y > 0.14 && y < 0.8;
        let in_box = x >= h && x < h + ext + 0.01 && m::abs(z) < 0.336 && y > 0.26 && y < 0.72;
        let on_arm = arm < 45.0 && x >= h && x < 0.68 && m::abs(z) < 0.3 && y > BUMPER_Y0 && y < 0.45;
        Where { own: in_frame || in_box || on_arm, held: in_frame || in_box || (on_arm && y > 0.24), near: x > -h - 0.1 && x < 0.72 && m::abs(z) < h + 0.1 && y < 0.85 }
    }
    /// js `shooterBalls(near)`'s test: up in the turret
    pub fn shooter_ball([x, y, z]: [f64; 3]) -> bool { y > 0.5 && m::hypot2(x - AXIS[0], z - AXIS[1]) < 0.16 }

    /// js `_arm(dt, on)`
    fn arm(&mut self, dt: f64, on: bool) {
        let pv = self.t.pivot;
        let slow = self.arm_goal == intake::STOW;
        self.arm_set = profile(self.arm_set[0], self.arm_set[1], self.arm_goal * DEG, if slow { intake::MM_SLOW } else { intake::MM_V }, intake::MM_A, dt);
        let d = &mut self.p.drives;
        if on {
            let ratio = d.shafts[pv].motor.as_ref().unwrap().ratio;
            let rot = |rad: f64| (rad * ratio) / TAU;
            let v = intake::KP * (rot(self.arm_set[0]) - rot(self.arm_deg * DEG)) + intake::KV * rot(self.arm_set[1]);
            d.shafts[pv].cmd = Some(clamp(v / 12.0, -1.0, 1.0));
        } else {
            d.shafts[pv].cmd = Some(0.0); d.shafts[pv].omega_ = 0.0; self.arm_set = [self.arm_deg * DEG, 0.0];
            return;
        }
        d.impulse(pv, -intake::MASS * 9.81 * intake::COM * m::cos(arm_rot(self.arm_deg) - 0.19) * dt);
    }
    /// js `_turnArm(dt)`
    fn turn_arm(&mut self, dt: f64) {
        let pv = self.t.pivot;
        let d = &mut self.p.drives;
        self.arm_deg += (d.omega(pv) * dt) / DEG;
        if self.arm_deg < intake::MIN - 1.0 { self.arm_deg = intake::MIN - 1.0; if d.shafts[pv].omega_ < 0.0 { d.shafts[pv].omega_ = 0.0; } }
        if self.arm_deg > intake::MAX + 1.0 { self.arm_deg = intake::MAX + 1.0; if d.shafts[pv].omega_ > 0.0 { d.shafts[pv].omega_ = 0.0; } }
    }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, dt: f64, r: &PreIn) {
        if let Some(ld) = self.last_deploy {
            if m::abs(r.intake_deploy - ld) > 1e-6 {
                self.arm_deg = intake::START + (intake::DEPLOY - intake::START) * r.intake_deploy;
                self.arm_set = [self.arm_deg * DEG, 0.0]; self.p.drives.shafts[self.t.pivot].omega_ = 0.0;
            }
        }
        self.arm(dt, r.enabled);
        self.hood_set = profile(self.hood_set[0], self.hood_set[1], self.hood_goal * DEG, shooter::HOOD_V, shooter::HOOD_A, dt);
        self.hood_deg = self.hood_set[0] / DEG;
        self.ext = hopper_ext(self.arm_deg);
        self.p.surfaces[S_INTAKE].set_pose([PIVOT[0], PIVOT[1], 0.0], 0.0, arm_rot(self.arm_deg));
        place_intake(&mut self.p.path.wheels[..self.n_rotor.0], &self.intake_home, self.arm_deg);
        place_hopper(&mut self.p.surfaces, &mut self.p.drives, self.t.cam, self.arm_deg);
        place_rotor_surface(&mut self.p.surfaces, self.phi);
        let (a, b) = self.n_rotor;
        place_rotor(&mut self.p.path.wheels[a..b], &self.rotor_home, self.phi);
        let yaw = r.turret_yaw;
        place_turret_surfaces(&mut self.p.surfaces, yaw, self.hood_deg);
        place_shooter(&mut self.p.shooter.as_mut().unwrap().wheels, yaw, self.hood_deg);
    }
    /// js `PhysicalRobot.stepMechanisms` (at its yield)
    pub fn mechanisms(&mut self, c: &mut Ctx, dt: f64, t: f64) {
        let (arm, ext) = (self.arm_deg, self.ext);
        self.p.mech_prepare(c, t, &|l| Self::where_(arm, ext, l), &Self::shooter_ball);
        self.p.mech_run(dt, &mut c.phys.bodies);
        self.p.mech_finish(c.fuel);
    }
    /// js `preSteps` after its yield; returns the robot's intakeDeploy
    pub fn post_mech(&mut self, dt: f64) -> f64 {
        self.turn_arm(dt);
        self.phi = (self.phi + self.p.drives.omega(self.t.rotor) * dt) % TAU;
        let d = clamp((intake::START - self.arm_deg) / (intake::START - intake::DEPLOY), 0.0, 1.0);
        self.last_deploy = Some(d);
        d
    }

    /// js `passTarget()`: their feed point on the side it's on
    pub fn pass_target(a: Alliance, pos: [f64; 3]) -> [f64; 2] {
        let left = field_in(a, pos)[1] > 317.0 / 2.0;
        let fp = feed_pt();
        let (x, z) = fw(fp[0], if left { FIELD_W_IN * IN - fp[1] } else { fp[1] }, a);
        [x, z]
    }

    /// js `_aim(dt, feed)`: their LaunchCalculator
    fn aim(&mut self, c: &Ctx, dt: f64, feed: bool, i: &PostIn) {
        let (v, w) = (i.est_vel, i.est_omega);
        let k = 1.0 - m::pow(0.5, dt / 0.02);
        let va = self.va.get_or_insert([v[0], v[1], 0.0, 0.0]);
        va[2] += k * ((v[0] - va[0]) / dt - va[2]);
        va[3] += k * ((v[1] - va[1]) / dt - va[3]);
        va[0] = v[0]; va[1] = v[1];
        let (ax, az) = (va[2], va[3]);
        let l = LATENCY;
        let ct = c.phys.bodies[c.body].translation();
        let yaw = i.yaw + w * l;
        let px = ct.x as f64 + v[0] * l + 0.5 * ax * l * l;
        let pz = ct.z as f64 + v[1] * l + 0.5 * az * l * l;
        let ox = AXIS[0] * m::cos(yaw) + AXIS[1] * m::sin(yaw);
        let oz = -AXIS[0] * m::sin(yaw) + AXIS[1] * m::cos(yaw);
        let (tx, tz) = (px + ox, pz + oz);
        let vtx = v[0] + ax * l + w * oz;
        let vtz = v[1] + az * l - w * ox;
        let tgt = if feed { i.aim_pass } else { i.aim_hub };
        let (dx, dz) = (tgt[0] - tx, tgt[1] - tz);
        let d = m::hypot2(dx, dz);
        let (v0, phi0) = if feed { (feed_speed(d), FEED_ANGLE * DEG) } else { let (vv, ang) = launch_for(d); (vv, ang * DEG) };
        let dd = if d == 0.0 { 1.0 } else { d };
        let (ux, uz) = (dx / dd, dz / dd);
        let sx = v0 * m::cos(phi0) * ux - vtx;
        let sz = v0 * m::cos(phi0) * uz - vtz;
        let sy = v0 * m::sin(phi0);
        let vr = m::hypot3(sx, sy, sz);
        self.aim_dist = d;
        self.aim_psi = Some(m::atan2(-sz, sx));
        self.launch_angle = if vr > 0.0 { m::asin(sy / vr) / DEG } else { 90.0 };
        self.launch_v = vr;
    }

    /// js `postStep(dt, t, { on, feeding, outtake, setpoint, mode })`; returns the robot's stored
    #[allow(clippy::too_many_arguments)]
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut, out4: &mut PostOut604) -> Vec<usize> {
        let stored = self.p.account(c, t, i.pass, i.legal, stats);
        let on = i.on;
        let intaking = on && i.intake;
        let unjam = on && i.outtake;
        if intaking || unjam { self.compress = false; } else if on && i.feeding { self.compress = true; }
        if intaking || unjam { self.arm_goal = intake::DEPLOY; self.deployed_once = true; }
        else if self.deployed_once { self.arm_goal = if i.auto || self.compress { intake::STOW } else { intake::STANDBY }; }
        let arm_ready = m::abs(self.arm_deg - self.arm_goal) < intake::TOL;
        let ti = self.t.intake;
        self.p.drives.trains[ti].cmd = Some(if !on { 0.0 } else if unjam { -0.3 } else if intaking && arm_ready { 1.0 } else { 0.0 });
        let launch = on && i.feeding && i.setpoint > 0.0 && i.has_shot && !unjam;
        let feed = i.pass;
        self.aim(c, dt, feed, i);
        out4.drive_supply = if launch { 20.0 } else { 40.0 };
        out4.turret_yaw = None;
        let mut target_rad = 0.0;
        if launch {
            target_rad = rad_for(self.launch_v);
            self.hood_goal = clamp(90.0 - self.launch_angle, shooter::HOOD_MIN, shooter::HOOD_MAX);
        } else {
            self.hood_goal = shooter::HOOD_MIN;
            let goal = shooter::YAW_START * DEG;
            let cur = i.turret_yaw;
            out4.turret_yaw = Some(cur + clamp(goal - cur, -shooter::YAW_V * dt, shooter::YAW_V * dt));
        }
        let (tl, tm) = (self.t.launcher, self.t.main);
        let main_w = m::abs(self.p.drives.omega(tm));
        let d = &mut self.p.drives;
        d.trains[tl].cmd = if target_rad > 0.0 { Some(velocity_duty(&d.trains[tl].motor, main_w, target_rad, shooter::KV, shooter::KP)) } else { None };
        let (yaw_tol, angle_tol, v_tol) = if feed { (shooter::FEED_TOL_YAW, shooter::FEED_TOL_ANGLE, shooter::FEED_TOL_V) } else { (shooter::TOL_YAW, shooter::TOL_ANGLE, shooter::TOL_V) };
        let at_yaw = m::abs(i.aim_err.unwrap_or(f64::INFINITY)) < yaw_tol * DEG;
        let at_angle = m::abs(self.hood_deg - self.hood_goal) < angle_tol;
        let flat_out = self.flat.get_or_insert_with(FlatOut::default).step(dt, main_w, if feed && launch { target_rad } else { 0.0 });
        let at_v = target_rad > 0.0 && ((target_rad < MAX_RAD && m::abs(velocity_at(target_rad) - velocity_at(main_w)) < v_tol) || flat_out);
        let in_launch_zone = !in_zone(c.alliance, i.pos, "TOWER") && !in_zone(c.alliance, i.pos, "BUMP") && !in_zone(c.alliance, i.pos, "BLOCKED");
        self.ready = launch && at_yaw && at_angle && at_v;
        let mut ball_v = if self.ready && in_launch_zone { serializer::BALL_V } else { 0.0 };
        if unjam { ball_v = serializer::UNJAM_V; }
        self.serializer(dt, if on { ball_v } else { 0.0 }, stats);
        let ic = self.p.drives.trains[ti].cmd.unwrap_or(0.0);
        *out = PostOut {
            hood_deg: hood_elevation(self.hood_deg),
            flywheel: self.exit_speed(),
            ready: self.ready && in_launch_zone,
            feeding: if self.target_roller > 0.0 { 1.0 } else { 0.0 },
            intake_speed: if ic > 0.0 { 1.0 } else if ic < 0.0 { -1.0 } else { 0.0 },
        };
        stored
    }

    /// js `_serializer(dt, ballV)`: SerializerSubsystem's setRollerVelocity and periodic
    fn serializer(&mut self, dt: f64, ball_v: f64, stats: &mut Stats) {
        let (roller, rotor, infeed) = (self.t.roller, self.t.rotor, self.t.infeed);
        let target = roller_rad(ball_v);
        let active = target != 0.0;
        self.target_roller = target;
        let d = &mut self.p.drives;
        let roller_w = d.omega(infeed);
        let ik = d.shafts[infeed].k;
        let lp = |d: &mut Drives, v: Option<f64>| {
            d.trains[roller].vel = v.map(|v| VelLoop { target: v / ik, kv: serializer::KV, kp: serializer::KP, ks: 0.0, lo: -12.0, hi: 12.0, per: None, rate: 1000.0 });
            d.trains[roller].cmd = v.map(|_| 0.0);
        };
        let rotor_jam = m::abs(d.shafts[rotor].motor.as_ref().unwrap().current) > 40.0 && d.omega(rotor) < 2.0;
        let rm = &d.trains[roller].motor;
        let roller_jam = m::abs(rm.current) / rm.count > 80.0 && roller_w < 0.1 * target;
        if self.jam == Jam::Normal {
            if active && (rotor_jam || roller_jam) {
                self.jam_t += dt;
                if self.jam_t >= 0.3 { self.jam = Jam::Jammed; self.jam_t = 0.0; stats.unjams += 1; }
            } else { self.jam_t = 0.0; }
        } else { self.jam_t += dt; if self.jam_t >= 0.1 { self.jam = Jam::Normal; self.jam_t = 0.0; } }
        if self.jam == Jam::Jammed {
            lp(d, Some(roller_rad(serializer::UNJAM_V)));
            d.shafts[rotor].cmd = Some(0.0); self.target_rotor = 0.0;
            return;
        }
        if !active { lp(d, None); self.roller_buf.clear(); d.shafts[rotor].cmd = None; self.target_rotor = 0.0; return; }
        lp(d, Some(target));
        self.roller_buf.push(roller_w * 0.5 * serializer::roller_r());
        if self.roller_buf.len() > 10 { self.roller_buf.remove(0); }
        let avg = self.roller_buf.iter().fold(0.0, |a, b| a + b) / self.roller_buf.len() as f64;
        self.target_rotor = m::max(0.0, avg / serializer::ROTOR_PER_ROLLER / rotor::eff_r());
        let w = d.omega(rotor);
        d.shafts[rotor].cmd = Some(velocity_duty(d.shafts[rotor].motor.as_ref().unwrap(), w, self.target_rotor, serializer::KV, serializer::KP));
    }

    /// js `extNow()`, `armTurn()`, `rotorTurn()`, `hoodTurn()`
    pub fn ext_now(&self) -> f64 { self.ext }
    pub fn arm_turn(&self) -> f64 { arm_rot(self.arm_deg) }
    pub fn rotor_turn(&self) -> f64 { self.phi }
    pub fn hood_turn(&self) -> f64 { hood_rot(self.hood_deg) }
    /// js `exitSpeed()`
    pub fn exit_speed(&self) -> f64 { EXIT_PER_MAIN_SURFACE * m::abs(self.p.drives.omega(self.t.main)) * shooter::MAIN_R }
    /// js `capacity()`
    pub fn capacity(&self) -> usize { 63 }
}
