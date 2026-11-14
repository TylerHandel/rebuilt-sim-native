//! js/mech/phys6328.js: 6328's Darwin in play, built of its physical mechanisms (r6328.rs). Its
//! controls are its code's (Mechanical-Advantage/RobotCode2026Public): the hood at 10 deg and the
//! flywheel idling at 80 rad/s; intake, the slam down and the roller at 13 V; launch, the flywheel
//! and hood tracking LaunchCalculator's maps (the setpoint profiled within its supply budget,
//! bang-bang far out), then once in tolerance the floor and the kickers' loops, and 0.35 s in the
//! compaction (the slam retracting slowly, the hopper's front pushing the load back).
use crate::constants::*;
use crate::drivetrain::motor_spec;
use crate::mech::drives::{Drives, VelLoop};
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, Physical, Stats, Where};
use crate::mech::r6328::*;
use crate::mech::r6328::HALF_L;
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const TAU: f64 = 2.0 * m::PI;
fn clamp(x: f64, a: f64, b: f64) -> f64 { m::max(a, m::min(b, x)) }
/// js HALF: the hopper's half width
const HALF: f64 = 0.343;
/// js SLAM
const SLAM_KP: f64 = 80.0;
const SLAM_RETRACT: f64 = 3.0;
const SLAM_SLOW: f64 = 0.88;
/// js HOOD
const HOOD_RATE: f64 = 250.0;
const HOOD_TOL: f64 = 1.5;
/// js TOL, COMPACT, KICK_BACK, CAUGHT, PASS_PT
fn tol_yaw() -> f64 { 10.0 * DEG }
const TOL_PASS_FLY: f64 = 40.0;
fn tol_tilt() -> f64 { 5.0 * DEG }
const COMPACT_DELAY: f64 = 0.35;
const COMPACT_ROLL: f64 = 1.2;
const KICK_BACK: f64 = 0.5;
const CAUGHT_T: f64 = 0.3;
const CAUGHT_DEG: f64 = 3.0;
const CAUGHT_CLEAR: f64 = 0.6;
fn pass_pt() -> [f64; 2] { [37.0 * IN, 65.0 * IN] }

/// js FLY: the flywheel's four X60s as WPILib's DCMotor sees them
struct FlyK { kv: f64, r: f64, kt: f64, moi: f64 }
fn fly_k() -> FlyK {
    let x = motor_spec("krakenX60");
    FlyK { kv: x.free / 12.0, r: 12.0 / (x.stall_current * 4.0), kt: x.stall_torque / x.stall_current, moi: 0.02 }
}
/// js Flywheel.runVelocity's state: { sp, acc, init, atGoal }
#[derive(Clone, Copy, Debug, Default)]
pub struct FlyState { pub sp: f64, pub acc: f64, pub init: bool, pub at_goal: bool }
/// js `flywheelVolts(st, goal, omega, dt, { bangBang, idle, supply })`
fn flywheel_volts(st: &mut FlyState, goal: f64, omega: f64, dt: f64, bang_bang: bool, idle: bool, supply: f64) -> f64 {
    let f = fly_k();
    let v_bus = 12.0;
    let p = supply * v_bus * shooter::EFF;
    let emf = st.sp / f.kv;
    let mut imax = (-emf + m::sqrt(emf * emf + 4.0 * f.r * p)) / (2.0 * f.r);
    imax = m::min(imax, m::max(0.0, (v_bus - emf) / f.r));
    let a_max = clamp((f.kt * imax - f.kt * (shooter::KS / f.r)) / f.moi, 0.0, shooter::MAX_ACCEL);
    let (step, err) = (a_max * dt, goal - st.sp);
    let raw;
    if m::abs(err) <= step { st.sp = goal; raw = err / dt; } else { st.sp += m::sign(err) * step; raw = m::sign(err) * a_max; }
    st.acc = if st.init { st.acc + (raw - st.acc) * dt / 0.05 } else { raw };
    st.init = true;
    st.at_goal = m::abs(st.sp - goal) <= 2.0;
    let mut v = m::sign(st.sp) * shooter::KS + st.sp * shooter::KV + st.acc * shooter::KA;
    if bang_bang { if omega < st.sp { v *= shooter::BANG_BANG; } } else if !idle { v += (shooter::KP * (st.sp + shooter::PID_OFFSET - omega)) / TAU; }
    clamp(v, -12.0, 12.0)
}
/// js `kickers(T, surf)`: the kickers' velocity loops at a surface speed (None: stopped, braked)
fn kickers(d: &mut Drives, t: &Trains, surf: Option<f64>) {
    let (kick_k, back_k) = (d.shafts[t.kick].k, d.shafts[t.back[0]].k);
    let mut set = |tr: usize, r: f64, g: [f64; 3], k: f64| match surf {
        None => { d.trains[tr].vel = None; d.trains[tr].cmd = Some(0.0); }
        Some(sf) => {
            let per = m::abs(k) / TAU;
            d.trains[tr].vel = Some(VelLoop { target: sf / r / m::abs(k), kv: g[2] * TAU, kp: g[0], ks: g[1], lo: -12.0, hi: 12.0, per: Some(per), rate: 1000.0 });
            d.trains[tr].cmd = Some(0.0);
        }
    };
    set(t.front_t, kick_r(), KICKER_FRONT, kick_k);
    set(t.back_t, back_r(), KICKER_BACK, back_k);
}

/// What the slam's asked for
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Slam { Deploy, Retract, RetractSlow }

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool }
/// What its postStep reads of the robot
pub struct PostIn {
    pub on: bool, pub feeding: bool, pub outtake: bool, pub pass: bool, pub legal: bool, pub intake: bool,
    /// js r.match.isAuto, r.humanDriven, r.cmd.prespin, r.cmd.stow
    pub auto: bool, pub human: bool, pub prespin: bool, pub stow: bool,
    /// whether it has a shot (js r.shot), and its aim point (js r.shot.aim)
    pub has_shot: bool, pub shot_aim: [f64; 2],
    pub aim_pass: [f64; 2], pub aim_hub: [f64; 2], pub pos: [f64; 3], pub yaw: f64, pub quat: [f64; 4], pub aim_err: Option<f64>, pub pass_clear: Option<bool>,
}
pub use crate::mech::phys2910::PostOut;

pub struct Phys6328 {
    pub p: Physical,
    pub t: Trains,
    intake_home: Vec<[f64; 3]>,
    hood_home: Vec<[f64; 3]>,
    n_intake: (usize, usize),
    pub arm_deg: f64,
    arm_set: f64,
    slam: Slam,
    slew_from: Option<f64>,
    pub hood_deg: f64,
    hood_goal: f64,
    fly: FlyState,
    launching: bool, launch_t: f64, tol_t: f64, kick_back_t: f64, was_held: bool,
    shots: u32, last_shot_t: f64, arm_hi: f64, arm_hi_t: f64,
    caught_t: f64, clear_until: f64, now: f64,
    want: Option<Want>,
}

impl Phys6328 {
    /// js `new Physical6328(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = Drives::default();
        let t = build6328(&mut d);
        let w = wheels6328(&t, &d);
        let surfaces = path_surfaces6328(&t);
        let mut path_wheels = w.floor;
        path_wheels.extend(w.kick);
        let n0 = path_wheels.len();
        let n_intake = (n0, n0 + w.intake.len());
        path_wheels.extend(w.intake);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.push(t.pivot);
        let path = Mechanism::new(Some(body), shafts, path_wheels, vec![S_BODY, S_INTAKE, S_RAMP, S_SLIDE, S_HOOD, S_WALLS, S_NET, S_BUMPER], 6, 1);
        let mut sw = vec![w.drum];
        sw.extend(w.hood);
        let shooter = Mechanism::new(Some(body), vec![t.drum, t.hood], sw, vec![S_HOOD], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), PRELOAD.to_vec(), Some((0.15, 8)));
        let mut me = Phys6328 {
            p, t, intake_home: w.intake_home, hood_home: w.hood_home, n_intake, arm_deg: 0.0, arm_set: 0.0, slam: Slam::Retract, slew_from: None,
            hood_deg: 0.0, hood_goal: 0.0, fly: FlyState::default(), launching: false, launch_t: 0.0, tol_t: 0.0, kick_back_t: 0.0, was_held: false,
            shots: 0, last_shot_t: 0.0, arm_hi: 0.0, arm_hi_t: 0.0, caught_t: 0.0, clear_until: -1.0, now: 0.0, want: None,
        };
        me.reset();
        me
    }

    /// js `reset()`: the slam up, the hood at its minimum
    pub fn reset(&mut self) {
        self.p.reset();
        self.arm_deg = intake::UP; self.arm_set = intake::UP; self.slam = Slam::Retract; self.slew_from = None;
        self.hood_deg = shooter::HOOD_MIN; self.hood_goal = shooter::HOOD_MIN;
        self.fly = FlyState::default();
        self.launching = false; self.launch_t = 0.0; self.tol_t = 0.0; self.kick_back_t = 0.0; self.was_held = false;
        self.shots = 0; self.last_shot_t = 0.0; self.arm_hi = 0.0; self.arm_hi_t = 0.0;
        self.caught_t = 0.0; self.clear_until = -1.0; self.now = 0.0;
        let t = &self.t;
        let d = &mut self.p.drives;
        for tr in [t.intake, t.floor, t.back_t, t.front_t, t.flywheel] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = Some(0.0); d.trains[tr].vel = None; }
        d.shafts[t.pivot].omega_ = 0.0; d.shafts[t.pivot].cmd = Some(0.0);
        self.want = None;
    }

    /// js `where(x, y, z)` (arm: its angle now)
    pub fn where_(arm: f64, [x, y, z]: [f64; 3]) -> Where {
        let in_frame = m::abs(x) < HALF_L + 0.01 && m::abs(z) < HALF && y > 0.12 && y < 0.8;
        let front = HALF_L + 0.01 + slide_ext(arm) + 0.06;
        let in_front = x >= HALF_L + 0.01 && x < m::max(front, 0.7) && m::abs(z) < HALF && y < 0.75;
        Where {
            own: in_frame || in_front,
            held: in_frame || (in_front && (y > 0.22 || x < 0.53) && x < HALF_L + slide_ext(arm) + 0.01 && !under_ramp(x, y, arm)),
            near: x > -HALF_L - 0.1 && x < 0.78 && m::abs(z) < HALF + 0.08 && y < 0.85,
        }
    }
    /// js `shooterBalls(near)`'s test
    pub fn shooter_ball([x, y, _z]: [f64; 3]) -> bool { y > 0.38 && x < 0.0 }

    /// js servoArm(T, deg, goal, dt)
    fn servo_arm(&mut self, goal: Option<f64>, dt: f64) {
        let a = (self.arm_deg + intake::COM_DEG) * DEG;
        let pv = self.t.pivot;
        let d = &mut self.p.drives;
        d.shafts[pv].cmd = Some(match goal { None => 0.0, Some(g) => clamp((SLAM_KP * (g - self.arm_deg)) / 360.0 / 12.0, -1.0, 1.0) });
        d.impulse(pv, -intake::MASS * 9.81 * intake::COM * m::cos(a) * dt);
    }
    /// js turnArm(T, deg, dt)
    fn turn_arm(&mut self, dt: f64) {
        let pv = self.t.pivot;
        let d = &mut self.p.drives;
        let mut deg = self.arm_deg + (d.omega(pv) * dt) / DEG;
        if deg < intake::DOWN { deg = intake::DOWN; if d.shafts[pv].omega_ < 0.0 { d.shafts[pv].omega_ = 0.0; } }
        if deg > intake::UP { deg = intake::UP; if d.shafts[pv].omega_ > 0.0 { d.shafts[pv].omega_ = 0.0; } }
        self.arm_deg = deg;
    }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, dt: f64, r: &PreIn) {
        let on = r.enabled;
        if self.slam == Slam::Deploy { self.arm_set = intake::DOWN; self.slew_from = Some(self.arm_deg); } else {
            if let Some(f) = self.slew_from.take() { self.arm_set = f; }
            let rate = (if self.slam == Slam::RetractSlow { SLAM_SLOW } else { SLAM_RETRACT }) / DEG;
            self.arm_set = m::min(intake::UP, self.arm_set + rate * dt);
        }
        self.servo_arm(if on { Some(self.arm_set) } else { None }, dt);
        self.hood_deg += clamp(self.hood_goal - self.hood_deg, -HOOD_RATE * dt, HOOD_RATE * dt);
        let ext = slide_ext(self.arm_deg);
        let k = slide_rate(self.arm_deg);
        self.p.surfaces[S_SLIDE].set_slide_k(k);
        self.p.drives.shafts[self.t.pivot].inertia = intake::J + SLIDE_MASS * k * k;
        let a = intake_rot(self.arm_deg);
        self.p.surfaces[S_INTAKE].set_pose([PIVOT[0], PIVOT[1], 0.0], 0.0, a);
        self.p.surfaces[S_RAMP].set_pose([PIVOT[0], PIVOT[1], 0.0], 0.0, a);
        let (i0, i1) = self.n_intake;
        place_intake(&mut self.p.path.wheels[i0..i1], &self.intake_home, self.arm_deg);
        self.p.surfaces[S_SLIDE].set_pose([ext, 0.0, 0.0], 0.0, 0.0);
        let contacts = std::mem::take(&mut self.p.path.contacts);
        {
            let net = self.p.surfaces[S_NET].bungee_mut().unwrap();
            net.place(ext);
            net.flex(&contacts, dt, NET_TREAD, S_NET);
        }
        self.p.path.contacts = contacts;
        self.p.surfaces[S_HOOD].set_pose([DRUM[0], DRUM[1], 0.0], 0.0, hood_rot(self.hood_deg));
        place_hood(&mut self.p.shooter.as_mut().unwrap().wheels[1..], &self.hood_home, self.hood_deg);
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
        clamp((intake::UP - self.arm_deg) / (intake::UP - intake::DOWN), 0.0, 1.0)
    }

    /// js `passTarget()`: their pass point, on the robot's side of the field
    pub fn pass_target(a: Alliance, pos: [f64; 3]) -> [f64; 2] {
        let pp = pass_pt();
        let (ax, az) = fw(pp[0], pp[1], a);
        let (bx, bz) = fw(pp[0], 8.069 - pp[1], a);
        if m::abs(az - pos[2]) < m::abs(bz - pos[2]) { [ax, az] } else { [bx, bz] }
    }

    /// js `postStep(dt, t, { on, feeding, outtake, setpoint, mode })`; returns the robot's stored
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut) -> Vec<usize> {
        let stored = self.p.account(c, t, i.pass, i.legal, stats);
        let (on, auto) = (i.on, i.auto);
        let intaking = on && i.intake;
        let held = on && i.feeding && i.has_shot;
        let track = on && !i.feeding && i.has_shot && i.prespin && auto;
        let pass = i.pass;
        let (mut want, mut dist, mut valid) = (None, 0.0, false);
        if held || track {
            let tgt = if pass { i.aim_pass } else { i.aim_hub };
            let aim = if !pass { i.shot_aim } else { tgt };
            let l = [i.pos[0] + m::cos(i.yaw) * LAUNCHER_X, i.pos[2] - m::sin(i.yaw) * LAUNCHER_X];
            dist = m::hypot2(aim[0] - l[0], aim[1] - l[1]);
            want = Some(shot_for(pass, dist, !held));
            let [lo, hi] = if pass { RANGE_PASS } else { RANGE_HUB };
            valid = dist >= lo && dist <= hi;
        }
        let trench = crate::mech::phys8793::under_trench(i.pos[0], i.pos[2], 0.4);
        self.hood_goal = match want { Some(w) if !trench => clamp(w.hood, shooter::HOOD_MIN, shooter::HOOD_MAX), _ => shooter::HOOD_MIN };
        let supply = if auto { shooter::SUPPLY_AUTO } else { shooter::SUPPLY_TELEOP };
        let tr = &self.t;
        let omega = self.p.drives.omega(tr.drum);
        if !on { self.fly.sp = omega; self.fly.init = false; self.p.drives.trains[tr.flywheel].cmd = None; } else {
            let volts = match want {
                Some(w) => flywheel_volts(&mut self.fly, w.fly, omega, dt, !pass && dist > shooter::BANG_MIN, false, supply),
                None => flywheel_volts(&mut self.fly, shooter::IDLE, omega, dt, false, true, supply),
            };
            self.p.drives.trains[tr.flywheel].cmd = Some(volts / 12.0);
        }
        let hood_ok = m::abs(self.hood_deg - self.hood_goal) <= HOOD_TOL;
        let fly_ok = match want { Some(w) => if pass { m::abs(omega - w.fly) < TOL_PASS_FLY } else { self.fly.at_goal }, None => false };
        let aimed = m::abs(i.aim_err.unwrap_or(f64::INFINITY)) <= tol_yaw();
        let q = i.quat;
        let tilt = m::acos(clamp(1.0 - 2.0 * (q[0] * q[0] + q[2] * q[2]), -1.0, 1.0));
        let in_tol = want.is_some() && hood_ok && fly_ok && aimed && tilt < tol_tilt();
        self.tol_t = if in_tol { 0.1 } else { m::max(0.0, self.tol_t - dt) };
        let go = held && valid && self.tol_t > 0.0 && i.pass_clear != Some(false);
        if go && !self.launching { self.launching = true; self.launch_t = 0.0; }
        if !go && self.launching { self.launching = false; self.kick_back_t = KICK_BACK; }
        if self.launching { self.launch_t += dt; }
        if stats.shots != self.shots { self.shots = stats.shots; self.last_shot_t = t; }
        if !self.launching { self.last_shot_t = m::max(self.last_shot_t, t - 0.5); }
        if self.slam != Slam::RetractSlow || self.arm_deg > self.arm_hi + 1.0 { self.arm_hi = self.arm_deg; self.arm_hi_t = t; }
        let stalled = t - self.arm_hi_t > 0.5;
        if !auto && !i.human && self.launching && (self.arm_deg > intake::UP - 3.0 || stalled) && t - self.last_shot_t > 0.75 && !self.p.held.is_empty() {
            self.launch_t = 0.0; self.slam = Slam::Deploy; self.last_shot_t = t;
        }
        if !held && self.was_held { self.slam = Slam::Deploy; if !self.launching { self.kick_back_t = m::max(self.kick_back_t, 0.0); } }
        self.was_held = held;
        self.now = t;
        let arm = self.arm_deg;
        let caught = intaking && !self.launching && arm > intake::DOWN + CAUGHT_DEG && {
            let owned = self.p.owned.clone();
            owned.iter().any(|&b| { let l = self.p.local(c.phys, c.body, c.fuel.balls[b].pos); under_ramp(l[0], l[1], arm) })
        };
        self.caught_t = if caught { self.caught_t + dt } else { 0.0 };
        if !auto && !i.human && self.caught_t > CAUGHT_T && t >= self.clear_until { self.clear_until = t + CAUGHT_CLEAR; self.caught_t = 0.0; }
        let mut roller = 0.0;
        if intaking { self.slam = Slam::Deploy; roller = 13.0; }
        if t < self.clear_until { self.slam = Slam::Retract; roller = 0.0; }
        if self.launching && self.launch_t >= COMPACT_DELAY && !intaking {
            self.slam = Slam::RetractSlow;
            if self.launch_t < COMPACT_DELAY + COMPACT_ROLL { roller = 13.0; }
        }
        if !intaking && !self.launching && i.stow { self.slam = Slam::Retract; }
        let (mut floor, mut kick) = (0.0, None);
        if self.launching { floor = 12.0; kick = want.map(|w| w.kicker); } else if self.kick_back_t > 0.0 { self.kick_back_t -= dt; kick = Some(-(want.map_or(6.0, |w| w.kicker))); }
        if on && i.outtake { floor = -12.0; kick = Some(-6.0); roller = -6.0; }
        if !on { floor = 0.0; kick = None; roller = 0.0; }
        let tr = &self.t;
        let d = &mut self.p.drives;
        d.trains[tr.intake].cmd = Some(clamp(roller / 12.0, -1.0, 1.0));
        d.trains[tr.floor].cmd = Some(floor / 12.0);
        kickers(d, tr, kick);
        self.want = want;
        *out = PostOut {
            hood_deg: hood_elevation(self.hood_deg),
            flywheel: self.exit_speed(),
            ready: in_tol && valid,
            feeding: if self.launching { 1.0 } else { 0.0 },
            intake_speed: if roller > 6.0 { 1.0 } else if roller < 0.0 { -1.0 } else { 0.0 },
        };
        stored
    }

    /// js `backOff()`: the AI's driver backing off FUEL caught under the ramp
    pub fn back_off(&self) -> bool { self.now < self.clear_until }
    /// js `launching`: it's launching its load (6328's AUTO waits on it)
    pub fn launching(&self) -> bool { self.launching }
    /// js `armTurn()`, `hoodTurn()`, `extNow()`
    pub fn arm_turn(&self) -> f64 { intake_rot(self.arm_deg) }
    pub fn hood_turn(&self) -> f64 { hood_rot(self.hood_deg) }
    pub fn ext_now(&self) -> f64 { slide_ext(self.arm_deg) }
    /// js `exitSpeed()`
    pub fn exit_speed(&self) -> f64 { exit_for(m::abs(self.p.drives.omega(self.t.drum)) * shooter::DRUM_R) }
    /// js `capacity()`
    /// js `loadTop()`: the top of its load (in its hopper, not up in the shooter)
    pub fn load_top(&mut self, phys: &PhysicsWorld, fuel: &crate::fuel::Fuel, body: RigidBodyHandle) -> f64 {
        let mut top: f64 = 0.0;
        for k in 0..self.p.held.len() {
            let [x, y, _] = self.p.local(phys, body, fuel.balls[self.p.held[k]].pos);
            if x > -0.05 { top = m::max(top, y + FUEL::radius); }
        }
        top
    }
    pub fn capacity(&self) -> usize { 70 }
}

/// js EXIT_CURVE, `exitFor(surf)`
const EXIT_CURVE: [[f64; 2]; 7] = [[0.0, 0.0], [7.6, 5.7], [12.7, 8.72], [15.2, 10.03], [17.8, 10.96], [20.3, 11.78], [25.4, 12.03]];
pub fn exit_for(surf: f64) -> f64 {
    let t = &EXIT_CURVE;
    for k in 1..t.len() { if surf <= t[k][0] { return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (surf - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); } }
    t[t.len() - 1][1]
}
