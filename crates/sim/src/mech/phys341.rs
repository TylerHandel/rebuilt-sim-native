//! js/mech/phys341.js: 341's Miss Daisy XXIV in play, built of its physical mechanisms (r341.rs).
//! Its controls are its code's (Team341/FRC2026-Public): the intake down and held stiff; shooting,
//! the flywheel's velocity loop and the hood at their table's for the distance from the turret to
//! where it aims, then SHOOTING once at speed and the turret's on target: the intake's rollers and
//! the top rollers / uptake at 12 V, the feeder at 3 V once the hood's there; their unjam; and not
//! their code, the feeder backing off a moment when nothing's come out (FEED_BACKOFF).
use crate::constants::*;
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, Physical, Stats, Where};
use crate::mech::r341::*;
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
fn clamp(x: f64, a: f64, b: f64) -> f64 { m::max(a, m::min(b, x)) }
/// js FEEDER_V: their 3 V
const FEEDER_V: f64 = 3.0;
/// js HOOD_MM: the hood's profile
const HOOD_V: f64 = 300.0;
const HOOD_A: f64 = 3000.0;
/// js `profile(p, v, goal, vmax, amax, dt)`
fn profile(p: f64, v: f64, goal: f64, vmax: f64, amax: f64, dt: f64) -> [f64; 2] {
    let (d, stop) = (goal - p, (v * v) / (2.0 * amax));
    let mut want = m::sign(d) * m::min(vmax, m::sqrt(2.0 * amax * m::abs(d)));
    if m::abs(d) < 1e-3 && m::abs(v) < amax * dt { return [goal, 0.0]; }
    if m::abs(d) <= stop && m::sign(v) == m::sign(d) { want = m::sign(d) * m::sqrt(2.0 * amax * m::abs(d)); }
    let nv = v + clamp(want - v, -amax * dt, amax * dt);
    [p + nv * dt, nv]
}
/// js EXIT_PER_FLY_SURFACE (SUB341 '1'), EXIT_SLIP
const EXIT_PER_FLY_SURFACE: f64 = 0.67;
const EXIT_SLIP: [f64; 2] = [10.8, 0.13];
/// js LET_GO: the AI's driver letting go of the trigger when nothing comes out
const LET_GO_AFTER: f64 = 0.8;
const LET_GO_FOR: f64 = 0.4;
/// js FEED_BACKOFF
const BACKOFF_AFTER: f64 = 0.3;
const BACKOFF_FOR: f64 = 0.1;

/// Their shooter's states
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum State { Stopped, SpinningUp, Shooting, AtSpeed, Pulsing }

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool, pub turret_yaw: f64 }
/// What its postStep reads of the robot
pub struct PostIn {
    pub on: bool, pub feeding: bool, pub outtake: bool, pub setpoint: f64, pub pass: bool, pub legal: bool, pub intake: bool,
    /// its shot's target (js r.shot.target; None: no shot), and where it aims at the HUB
    pub shot: Option<[f64; 2]>, pub aim_hub: [f64; 2],
    pub est_vel: [f64; 2], pub est_omega: f64, pub pos: [f64; 3], pub alliance: Alliance, pub aim_err: Option<f64>,
}
pub use crate::mech::phys2910::PostOut;

pub struct Phys341 {
    pub p: Physical,
    pub t: Trains,
    arm_home: Vec<[f64; 3]>,
    n_arm: (usize, usize),
    pub arm_deg: f64,
    arm_goal: f64,
    pub hood_deg: f64,
    hood_set: [f64; 2],
    hood_goal: f64,
    want_hood: Option<f64>,
    pub state: State,
    target_rpm: f64,
    at_speed_t: f64, turret_t: f64, jam_t: f64, pulse_t: f64,
    last_shot_t: f64, shots_seen: u32, let_go_until: f64, now: f64, back_t: f64, back_from: f64,
    at_speed_now: bool,
    fv: Option<[f64; 2]>,
    aim_dist: f64,
    /// js aimPsi: where it wants the turret (field heading; None before its first step)
    pub aim_psi: Option<f64>,
}

impl Phys341 {
    /// js `new Physical341(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = crate::mech::drives::Drives::default();
        let t = build341(&mut d);
        let w = wheels341(&t, &d);
        let surfaces = path_surfaces341(&t);
        let sw = shooter_wheels(&t);
        let n0 = w.pivot_row.len();
        let n_arm = (n0, n0 + w.arm.len());
        let mut path_wheels = w.pivot_row;
        path_wheels.extend(w.arm);
        path_wheels.extend(w.table);
        path_wheels.extend(w.uptake);
        path_wheels.extend(w.omnis);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.push(t.pivot);
        let path = Mechanism::new(Some(body), shafts, path_wheels, vec![S_BODY, S_BUMPER, S_INTAKE, S_TURRET, S_HOOD, S_LID, S_WALL, S_OMNI], 6, 1);
        let shooter = Mechanism::new(Some(body), vec![t.fly, t.stealth, t.feed], sw, vec![S_TURRET, S_HOOD], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), PRELOAD.to_vec(), None);
        let mut me = Phys341 {
            p, t, arm_home: w.arm_home, n_arm, arm_deg: 0.0, arm_goal: 0.0, hood_deg: 0.0, hood_set: [0.0; 2], hood_goal: 0.0, want_hood: None,
            state: State::Stopped, target_rpm: 0.0, at_speed_t: 0.0, turret_t: 0.0, jam_t: 0.0, pulse_t: 0.0,
            last_shot_t: 0.0, shots_seen: 0, let_go_until: -1.0, now: 0.0, back_t: 0.0, back_from: 0.0, at_speed_now: false,
            fv: None, aim_dist: 0.0, aim_psi: None,
        };
        me.reset();
        me
    }

    /// js `reset()`: down from the start
    pub fn reset(&mut self) {
        self.p.reset();
        self.arm_deg = intake::DOWN; self.arm_goal = intake::DOWN;
        self.hood_deg = 0.0; self.hood_set = [0.0, 0.0]; self.hood_goal = 0.0;
        self.state = State::Stopped;
        self.target_rpm = 0.0; self.at_speed_t = 0.0; self.turret_t = 0.0; self.jam_t = 0.0; self.pulse_t = 0.0;
        self.last_shot_t = 0.0; self.shots_seen = 0; self.let_go_until = -1.0; self.now = 0.0; self.back_t = 0.0; self.back_from = 0.0;
        let t = &self.t;
        let d = &mut self.p.drives;
        for tr in [t.intake, t.serializer, t.feeder_t, t.shooter] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = Some(0.0); }
        d.shafts[t.pivot].omega_ = 0.0; d.shafts[t.pivot].cmd = Some(0.0);
    }

    /// js `where(x, y, z)` (arm: its angle now)
    pub fn where_(arm: f64, [x, y, z]: [f64; 3]) -> Where {
        let h = half();
        let in_frame = m::abs(x) < h && m::abs(z) < h && y > 0.13 && y < 0.8;
        let on_arm = arm > 90.0 && x >= h && x < h + 0.33 && m::abs(z) < 0.32 && y < 0.45;
        Where { own: in_frame || on_arm, held: in_frame || (on_arm && y > 0.12), near: x > -h - 0.1 && x < h + 0.4 && m::abs(z) < h + 0.08 && y < 0.85 }
    }
    /// js `shooterBalls(near)`'s test
    pub fn shooter_ball([_x, y, _z]: [f64; 3]) -> bool { y > 0.33 }

    /// js servoArm(T, deg, goal, dt)
    fn servo_arm(&mut self, goal: Option<f64>, dt: f64) {
        let a = intake_rot(self.arm_deg);
        let pv = self.t.pivot;
        let d = &mut self.p.drives;
        d.shafts[pv].cmd = Some(match goal {
            Some(g) => { let err = g - self.arm_deg; let k = if m::abs(err) > 30.0 { 0.25 } else { 3.0 }; clamp((k * err) / 12.0, -1.0, 1.0) }
            None => 0.0,
        });
        d.impulse(pv, intake::MASS * 9.81 * intake::COM * -m::sin(a) * dt);
    }
    /// js turnArm(T, deg, dt)
    fn turn_arm(&mut self, dt: f64) {
        let pv = self.t.pivot;
        let d = &mut self.p.drives;
        let mut deg = self.arm_deg + (d.omega(pv) * dt) / DEG;
        if deg > intake::DOWN + 1.0 { deg = intake::DOWN + 1.0; if d.shafts[pv].omega_ > 0.0 { d.shafts[pv].omega_ = 0.0; } }
        if deg < intake::STOWED { deg = intake::STOWED; if d.shafts[pv].omega_ < 0.0 { d.shafts[pv].omega_ = 0.0; } }
        self.arm_deg = deg;
    }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, dt: f64, r: &PreIn) {
        self.servo_arm(if r.enabled { Some(self.arm_goal) } else { None }, dt);
        self.hood_set = profile(self.hood_set[0], self.hood_set[1], self.hood_goal, HOOD_V, HOOD_A, dt);
        self.hood_deg = self.hood_set[0];
        self.p.surfaces[S_INTAKE].set_pose([PIVOT[0], PIVOT[1], 0.0], 0.0, intake_rot(self.arm_deg));
        let (a, b) = self.n_arm;
        place_intake(&mut self.p.path.wheels[a..b], &self.arm_home, self.arm_deg);
        place_turret_surfaces(&mut self.p.surfaces, r.turret_yaw, self.hood_deg);
        place_shooter(&mut self.p.shooter.as_mut().unwrap().wheels, r.turret_yaw, self.hood_deg);
    }
    /// js `PhysicalRobot.stepMechanisms` (at its yield)
    pub fn mechanisms(&mut self, c: &mut Ctx, dt: f64, t: f64) {
        let arm = self.arm_deg;
        self.p.mech_prepare(c, t, &|l| Self::where_(arm, l), &Self::shooter_ball);
        self.p.mech_run(dt, &mut c.phys.bodies);
        self.p.mech_finish(c.fuel);
    }
    /// js `preSteps` after its yield; returns the robot's intakeDeploy
    pub fn post_mech(&mut self, dt: f64) -> f64 {
        self.turn_arm(dt);
        clamp(self.arm_deg / intake::DOWN, 0.0, 1.0)
    }

    /// js `_aim(dt, pass)`: where the turret aims and how far that is
    fn aim(&mut self, c: &mut Ctx, dt: f64, pass: bool, i: &PostIn) {
        let tp = self.p.world(c.phys, c.body, [TURRET[0], 0.0, TURRET[1]]);
        let ct = c.phys.bodies[c.body].translation();
        let (v, w) = (i.est_vel, i.est_omega);
        let vx = v[0] + w * (tp[2] - ct.z as f64);
        let vz = v[1] - w * (tp[0] - ct.x as f64);
        let k = 1.0 - m::pow(0.75, dt / 0.02);
        let fv = self.fv.get_or_insert([vx, vz]);
        fv[0] += k * (vx - fv[0]);
        fv[1] += k * (vz - fv[1]);
        let tgt = match i.shot { Some(s) if pass => s, _ => i.aim_hub };
        let own = if i.alliance == Alliance::Blue { i.pos[0] + HALF_L } else { HALF_L - i.pos[0] };
        let tof = if !pass { 1.16 } else if own < HALF_L { 1.3 } else if own < 2.0 * HALF_L - ALLIANCE_ZONE_DEPTH { 1.6 } else { 1.8 };
        let (ax, az) = (tgt[0] - fv[0] * tof, tgt[1] - fv[1] * tof);
        self.aim_dist = m::hypot2(ax - tp[0], az - tp[2]);
        self.aim_psi = Some(m::atan2(-(az - tp[2]), ax - tp[0]));
    }

    /// js `postStep(dt, t, { on, feeding, outtake, setpoint, mode })`; returns the robot's stored
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut) -> Vec<usize> {
        let stored = self.p.account(c, t, i.pass, i.legal, stats);
        let (on, pass) = (i.on, i.pass);
        let want_shot = on && i.feeding && i.setpoint > 0.0 && i.shot.is_some();
        self.aim(c, dt, pass, i);
        if i.shot.is_some() {
            let (rpm, hood) = shot_for(pass, self.aim_dist);
            if want_shot || self.state != State::Stopped { self.target_rpm = rpm; }
            self.want_hood = Some(hood);
        }
        let tr = &self.t;
        let d = &mut self.p.drives;
        let rpm = (m::abs(d.omega(tr.fly)) * 60.0) / (2.0 * m::PI);
        self.at_speed_t = if m::abs(rpm - self.target_rpm) <= shooter::TOL { 0.1 } else { m::max(0.0, self.at_speed_t - dt) };
        let at_speed = self.at_speed_t > 0.0 && self.target_rpm > 0.0;
        self.turret_t = if m::abs(i.aim_err.unwrap_or(f64::INFINITY)) < 6.0 * DEG { 0.2 } else { m::max(0.0, self.turret_t - dt) };
        let turret_ready = self.turret_t > 0.0;
        self.at_speed_now = at_speed;
        let in_trench = crate::mech::phys8793::under_trench(i.pos[0], i.pos[2], 0.0);
        if !on { self.state = State::Stopped; }
        else if want_shot { self.state = if self.pulse_t > 0.0 { State::Pulsing } else if at_speed && turret_ready { State::Shooting } else { State::SpinningUp }; }
        else if self.state != State::Stopped { self.state = State::AtSpeed; }
        let shooting = matches!(self.state, State::Shooting | State::Pulsing);
        self.hood_goal = if matches!(self.state, State::Shooting | State::SpinningUp | State::Pulsing) && !in_trench { self.want_hood.unwrap_or(0.0) } else { 0.0 };
        self.now = t;
        let letting_go = t < self.let_go_until;
        let intaking = on && (shooting || (i.intake && !letting_go));
        if stats.shots != self.shots_seen { self.shots_seen = stats.shots; self.last_shot_t = t; }
        if self.state != State::Shooting { self.last_shot_t = t; }
        let stalled = |d: &crate::mech::drives::Drives, x: usize| {
            let t = &d.trains[x];
            (m::abs(t.omega) * 60.0) / (2.0 * m::PI) < 500.0 && m::abs(t.motor.current) / t.motor.count > 75.0
        };
        self.jam_t = if intaking && shooting && (stalled(d, tr.serializer) || stalled(d, tr.feeder_t)) { self.jam_t + dt } else { 0.0 };
        if self.jam_t > 0.2 && self.pulse_t <= 0.0 { self.pulse_t = 0.25; self.jam_t = 0.0; stats.unjams += 1; }
        self.pulse_t = m::max(0.0, self.pulse_t - dt);
        let pulsing = self.pulse_t > 0.0;
        self.back_from = m::max(self.back_from, self.last_shot_t);
        if BACKOFF_FOR > 0.0 && self.state == State::Shooting && !stored.is_empty() && t - self.back_from > BACKOFF_AFTER { self.back_t = BACKOFF_FOR; self.back_from = t + BACKOFF_FOR; }
        self.back_t = m::max(0.0, self.back_t - dt);
        let o = i.outtake;
        d.trains[tr.intake].cmd = Some(if !on { 0.0 } else if o || pulsing { -1.0 } else if intaking { 1.0 } else { 0.0 });
        d.trains[tr.serializer].cmd = Some(if !on { 0.0 } else if o || pulsing { -0.5 } else if shooting { 1.0 } else if intaking { 0.0 } else { -3.0 / 12.0 });
        let feeder = if !on { 0.0 } else if pulsing { -3.0 / 12.0 } else if self.back_t > 0.0 && self.state == State::Shooting { -FEEDER_V / 12.0 } else if self.state == State::Shooting && m::abs(self.hood_deg - self.hood_goal) <= 3.0 { FEEDER_V / 12.0 } else { 0.0 };
        d.trains[tr.feeder_t].cmd = Some(feeder);
        if self.state != State::Stopped && self.target_rpm > 0.0 {
            let rps = d.omega(tr.fly) / (2.0 * m::PI) * -1.0;
            let tgt = self.target_rpm / 60.0;
            d.trains[tr.shooter].cmd = Some(clamp((shooter::KV * tgt + shooter::KP * (tgt - rps)) / 12.0, 0.0, 1.0));
        } else { d.trains[tr.shooter].cmd = Some(0.0); }
        self.arm_goal = intake::DOWN;
        let ic = d.trains[tr.intake].cmd.unwrap_or(0.0);
        *out = PostOut {
            hood_deg: hood_elevation(self.hood_deg),
            flywheel: self.exit_speed(),
            ready: self.state == State::Shooting,
            feeding: if feeder > 0.0 { 1.0 } else { 0.0 },
            intake_speed: if ic > 0.0 { 1.0 } else if ic < 0.0 { -1.0 } else { 0.0 },
        };
        stored
    }

    /// js `letGo()`: the AI's driver lets go of the trigger now (shooting with FUEL in it and
    /// nothing out for a while, for a moment)
    pub fn let_go(&mut self, holding: bool, stats: &mut Stats) -> bool {
        let t = self.now;
        if t < self.let_go_until { return true; }
        if self.state == State::Shooting && holding && t - self.last_shot_t > LET_GO_AFTER {
            self.let_go_until = t + LET_GO_FOR;
            let _ = stats;
            return true;
        }
        false
    }
    /// js `shotStatus()`
    pub fn shot_status(&self) -> Option<&'static str> {
        match self.state {
            State::Pulsing => Some("Unjamming"),
            State::Shooting => Some(if m::abs(self.hood_deg - self.hood_goal) <= 3.0 { "READY" } else { "Hood" }),
            State::SpinningUp => Some(if !self.at_speed_now { "Spinning up" } else { "Aiming" }),
            _ => None,
        }
    }
    /// js `armTurn()`: the arm's turn from the CAD's pose (rad), for drawing it
    pub fn arm_turn(&self) -> f64 { intake_rot(self.arm_deg) }
    /// js `hoodTurn()`
    pub fn hood_turn(&self) -> f64 { hood_rot(self.hood_deg) }
    /// js `exitSpeed()`
    pub fn exit_speed(&self) -> f64 {
        let s = (m::abs(self.p.drives.omega(self.t.fly)) * shooter::fly_d()) / 2.0;
        m::min(EXIT_PER_FLY_SURFACE * s, EXIT_SLIP[0] + EXIT_SLIP[1] * s)
    }
    /// js `capacity()`
    pub fn capacity(&self) -> usize { 8 }
}
