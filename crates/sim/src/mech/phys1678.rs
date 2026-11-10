//! js/mech/phys1678.js: 1678's Limestone in play, built of its physical mechanisms (r1678.rs). Its
//! controls are its code's (Superstructure, Targeting, the shot verifier and the subsystems'
//! setpoints); every roller coasts when it's set to idle, as theirs do. And what a driver of it
//! does: contractHopper when FUEL are left out over the intake while it fires, and the lid (their
//! climber) up once the hopper's full under it, down for a TRENCH.
use crate::constants::*;
use crate::mech::drives::{Drives, VelLoop};
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, FullWatch, Physical, Stats, Where};
use crate::mech::r1678::*;
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const R: f64 = FUEL::radius;
const FT: f64 = 0.3048;
/// its frame (27in square)
const HALF: f64 = 0.343;
/// js LID_N, LID_DOWN
const LID_N: usize = 40;
const LID_DOWN: f64 = 3.0;
/// js PIVOT_PID
const PIVOT_KP: f64 = 125.0 / 360.0;
const PIVOT_KS: f64 = 0.28;
const PIVOT_KG: f64 = 0.3;
/// js HOOD_MM
const HOOD_V: f64 = 300.0;
const HOOD_A: f64 = 2500.0;
/// js TOL
const TOL_HUB_AIM: f64 = 1.3 * DEG;
const TOL_FERRY_AIM: f64 = 4.0 * DEG;
const TOL_RPM: f64 = 80.0;
const TOL_FERRY_RPM: f64 = 150.0;
const TOL_HOOD: f64 = 0.5;
const TOL_SPEED: f64 = 1.0;
/// js LID_SPEED
const LID_UP: f64 = 0.3;
const LID_DOWN_SPEED: f64 = 0.4;
/// js EXIT_PER_DRUM_SURFACE
pub const EXIT_PER_DRUM_SURFACE: f64 = 0.72;

/// js `profile(p, v, goal, vmax, amax, dt)`
fn profile(p: f64, v: f64, goal: f64, vmax: f64, amax: f64, dt: f64) -> [f64; 2] {
    let d = goal - p;
    let dir = m::sign(d);
    if m::abs(d) < 1e-6 && m::abs(v) < 1e-6 { return [goal, 0.0]; }
    let want = dir * m::min(vmax, m::sqrt(2.0 * amax * m::abs(d)));
    let nv = v + m::max(-amax * dt, m::min(amax * dt, want - v));
    let np = p + nv * dt;
    if (goal - np) * dir < 0.0 { return [goal, 0.0]; }
    [np, nv]
}
fn clamp(x: f64, a: f64, b: f64) -> f64 { m::max(a, m::min(b, x)) }

/// js `servoArm(T, deg, goal, dt)`: their position loop toward goal (None: limp), the arm's weight
fn servo_arm(d: &mut Drives, piv: usize, deg: f64, goal: Option<f64>, dt: f64) {
    let a = (deg + intake::COM_DEG) * DEG;
    match goal {
        Some(g) => { let v = PIVOT_KP * (g - deg) + PIVOT_KG * m::cos(a) + PIVOT_KS * m::sign(g - deg); d.shafts[piv].cmd = Some(clamp(v / 12.0, -1.0, 1.0)); }
        None => d.shafts[piv].cmd = Some(0.0),
    }
    d.impulse(piv, -intake::MASS * 9.81 * intake::COM * m::cos(a) * dt);
}
/// js `turnArm(T, deg, dt)`
fn turn_arm(d: &mut Drives, piv: usize, mut deg: f64, dt: f64) -> f64 {
    deg += (d.omega(piv) * dt) / DEG;
    if deg < intake::DOWN { deg = intake::DOWN; if d.shafts[piv].omega_ < 0.0 { d.shafts[piv].omega_ = 0.0; } }
    if deg > intake::STOWED { deg = intake::STOWED; if d.shafts[piv].omega_ > 0.0 { d.shafts[piv].omega_ = 0.0; } }
    deg
}
/// A tunnel setpoint (js { rpm, max })
#[derive(Clone, Copy, Debug)]
struct TunnelSet { rpm: f64, max: Option<f64> }
/// js `tunnelLoop(T, set)`
fn tunnel_loop(d: &mut Drives, t: &Trains, set: Option<TunnelSet>) {
    let top = *t.back.last().unwrap();
    let per = d.shafts[top].k / (2.0 * m::PI);
    let tr = d.shafts[top].train.unwrap();
    d.trains[tr].vel = set.map(|s| VelLoop { target: s.rpm / 60.0 / per, kv: tunnel::kv(), kp: tunnel::KP, ks: tunnel::KS, lo: 0.0, hi: s.max.unwrap_or(12.0), per: Some(per), rate: 1000.0 });
    d.trains[tr].cmd = set.map(|_| 0.0);
}
/// js CONTRACT's steps: an angle, half a second's wait, or down
#[derive(Clone, Copy, Debug, PartialEq)]
enum Step { Deg(f64), Wait, Down }
const CONTRACT: [Step; 6] = [Step::Deg(80.0), Step::Deg(25.0), Step::Deg(80.0), Step::Wait, Step::Deg(50.0), Step::Down];
#[derive(Clone, Copy, Debug)]
struct Seq { i: usize, t: f64 }
#[derive(Clone, Copy, Debug, PartialEq)]
enum Prefill { None, Armed, Running, Done }
/// js this.set: their subsystems' setpoints (None: neutral, coasting)
#[derive(Clone, Copy, Debug, Default)]
struct Set { intake: Option<f64>, floor: Option<f64>, tunnel: Option<TunnelSet>, feeder: Option<f64>, shooter: Option<f64> }

/// js `_ferry()`'s answer: the kind of ferry, and where to
pub fn ferry(alliance: Alliance, pos: [f64; 3]) -> (Kind, [f64; 2]) {
    let from_wall = if alliance == Alliance::Blue { pos[0] + HALF_L } else { HALF_L - pos[0] };
    let (kind, fx, left, right) = if from_wall <= FIELD_L - 13.0 * FT { (Kind::Ferry, 0.5, 15.5 * FT, 8.0 * FT) }
        else if from_wall <= FIELD_L - 6.5 * FT { (Kind::Far, 0.5, 20.0 * FT, 5.0 * FT) } else { (Kind::Ferry, 5.5, 18.0 * FT, 9.0 * FT) };
    let l = fw(fx, left, alliance);
    let r = fw(fx, right, alliance);
    (kind, if m::sign(pos[2]) == m::sign(l.1) { [l.0, l.1] } else { [r.0, r.1] })
}

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool, pub bay_top: f64, pub lift_h: f64 }
/// What its postStep reads of the robot
pub struct PostIn {
    pub on: bool, pub feeding: bool, pub outtake: bool, pub setpoint: f64, pub pass: bool, pub legal: bool, pub intake: bool, pub has_shot: bool,
    /// where it aims: the HUB (aimHub) and its ferry target (aimPt), as its pose estimate has them
    pub aim_hub: [f64; 2], pub aim_ferry: [f64; 2],
    pub pos: [f64; 3], pub alliance: Alliance, pub est_vel: [f64; 2], pub aim_err: Option<f64>, pub pass_clear: Option<bool>,
    pub half_l: f64, pub linvel: [f64; 3], pub bay_top: f64, pub lift_h: f64,
}
pub use crate::mech::phys2910::PostOut;

pub struct Phys1678 {
    pub p: Physical,
    pub t: Trains,
    n_floor_tunnel: usize,
    intake_home: Vec<[f64; 3]>,
    hood_home: Vec<[f64; 3]>,
    pub arm_deg: f64,
    arm_goal: f64,
    arm_hold: bool,
    pub hood_deg: f64,
    hood_set: [f64; 2],
    hood_goal: f64,
    pub lift: f64,
    set: Set,
    was_intaking: bool, was_shooting: bool,
    prefill: Prefill,
    firing: bool, verified_t: f64, firing_t: f64,
    seq: Option<Seq>, contract_cool: f64, pub tucked: bool, near_trench: bool,
    hopper_t: f64, tunnel_t: f64,
    pub expanded: bool,
}

impl Phys1678 {
    /// js `new Physical1678(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = Drives::default();
        let t = build1678(&mut d);
        let w = wheels1678(&d, &t);
        let mut surfaces = path_surfaces1678(&t);
        surfaces[S_SLIDE] = surfaces[S_SLIDE].clone().on_slide(t.pivot, [1.0, 0.0, 0.0]);
        let n_floor_tunnel = w.floor.len() + w.tunnel.len();
        let mut path_wheels = w.floor;
        path_wheels.extend(w.tunnel);
        path_wheels.extend(w.intake);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.push(t.pivot);
        let mut path = Mechanism::new(Some(body), shafts, path_wheels, (0..surfaces.len()).collect(), 6, 1);
        path.curved = true;
        let mut sw = vec![w.drum];
        sw.extend(w.hood);
        let shooter = Mechanism::new(Some(body), vec![t.drum, t.hood], sw, vec![S_HOOD], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), PRELOAD.to_vec(), Some((0.1, 10)));
        let mut me = Phys1678 {
            p, t, n_floor_tunnel, intake_home: w.intake_home, hood_home: w.hood_home,
            arm_deg: 0.0, arm_goal: 0.0, arm_hold: true, hood_deg: 0.0, hood_set: [0.0; 2], hood_goal: 0.0, lift: 0.0, set: Set::default(),
            was_intaking: false, was_shooting: false, prefill: Prefill::None, firing: false, verified_t: 0.0, firing_t: 0.0,
            seq: None, contract_cool: 0.0, tucked: true, near_trench: false, hopper_t: 0.0, tunnel_t: 0.0, expanded: false,
        };
        me.reset();
        me
    }

    /// js `reset()`
    pub fn reset(&mut self) {
        self.p.reset();
        self.arm_deg = intake::STOWED; self.arm_goal = intake::STOWED; self.arm_hold = true;
        self.hood_deg = shooter::HOOD_MIN; self.hood_set = [shooter::HOOD_MIN, 0.0]; self.hood_goal = shooter::HOOD_MIN;
        self.lift = 0.0;
        self.set = Set::default();
        self.was_intaking = false; self.was_shooting = false;
        self.prefill = Prefill::None;
        self.firing = false; self.verified_t = 0.0; self.firing_t = 0.0;
        self.seq = None; self.contract_cool = 0.0; self.tucked = true; self.near_trench = false;
        self.hopper_t = 0.0; self.tunnel_t = 0.0;
        self.expanded = false;
        let d = &mut self.p.drives;
        for tr in [self.t.intake, self.t.floor, self.t.tunnel, self.t.feeder_t, self.t.shooter] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = None; }
        d.trains[self.t.tunnel].vel = None;
        for s in self.t.idlers { d.set_omega(s, 0.0); }
        d.shafts[self.t.pivot].omega_ = 0.0; d.shafts[self.t.pivot].cmd = Some(0.0);
    }

    /// js `where(x, y, z)`
    pub fn where_([x, y, z]: [f64; 3]) -> Where {
        let in_frame = m::abs(x) < HALF && m::abs(z) < HALF && y > 0.105 && y < 0.8;
        let in_front = x >= HALF && x < 0.72 && m::abs(z) < HALF && y < 0.8;
        Where {
            own: in_frame || in_front,
            held: in_frame || (in_front && y > 0.2 && x < 0.6),
            near: x > -HALF - 0.1 && x < 0.76 && m::abs(z) < HALF + 0.08 && y < 0.85,
        }
    }
    /// js `shooterBalls(near)`'s test
    pub fn shooter_ball([x, y, _]: [f64; 3]) -> bool { y > 0.38 && x < -0.03 }
    /// js `loadTop()`: the top of the FUEL it holds (robot frame), for the lid
    pub fn load_top(&mut self, phys: &PhysicsWorld, fuel: &crate::fuel::Fuel, body: RigidBodyHandle) -> f64 {
        let mut top = 0.0;
        for k in 0..self.p.held.len() {
            let b = self.p.held[k];
            let [x, y, _] = self.p.local(phys, body, fuel.balls[b].pos);
            if x > -0.06 { top = m::max(top, y + R); }
        }
        top
    }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, phys: &PhysicsWorld, fuel: &crate::fuel::Fuel, body: RigidBodyHandle, dt: f64, r: &PreIn) {
        let piv = self.t.pivot;
        servo_arm(&mut self.p.drives, piv, self.arm_deg, if r.enabled && self.arm_hold { Some(self.arm_goal) } else { None }, dt);
        self.hood_set = profile(self.hood_set[0], self.hood_set[1], self.hood_goal, HOOD_V, HOOD_A, dt);
        self.hood_deg = self.hood_set[0];
        if self.expanded && self.p.held.len() < LID_N - 6 && self.load_top(phys, fuel, body) < r.bay_top - LID_DOWN * R { self.expanded = false; }
        let ls = self.lift_scale(phys, fuel, body, r.bay_top, r.lift_h);
        self.lift += clamp(ls * r.lift_h - self.lift, -LID_DOWN_SPEED * dt, LID_UP * dt);
        let ext = slot_ext(self.arm_deg);
        let k = slot_rate(self.arm_deg);
        self.p.surfaces[S_SLIDE].set_slide_k(k);
        self.p.drives.shafts[piv].inertia = intake::J + SLOT_MASS * k * k;
        self.p.surfaces[S_INTAKE].set_pose([PIVOT[0], PIVOT[1], 0.0], 0.0, intake_rot(self.arm_deg));
        let nf = self.n_floor_tunnel;
        place_intake(&mut self.p.path.wheels[nf..], &self.intake_home, self.arm_deg);
        self.p.surfaces[S_HOOD].set_pose([DRUM[0], DRUM[1], 0.0], 0.0, hood_rot(self.hood_deg));
        place_hood(&mut self.p.shooter.as_mut().unwrap().wheels[1..], &self.hood_home, self.hood_deg);
        self.p.surfaces[S_SLIDE].set_pose([ext, 0.0, 0.0], 0.0, 0.0);
        self.p.surfaces[S_LID].set_pose([0.0, self.lift, 0.0], 0.0, 0.0);
        place_net(&mut self.p.surfaces[S_NET], ext, self.lift);
    }
    /// js `PhysicalRobot.stepMechanisms` (at its yield)
    pub fn mechanisms(&mut self, c: &mut Ctx, dt: f64, t: f64) {
        self.p.mech_prepare(c, t, &Self::where_, &Self::shooter_ball);
        self.p.mech_run(dt, &mut c.phys.bodies);
        self.p.mech_finish(c.fuel);
    }
    /// js `preSteps` after its yield; returns the robot's intakeDeploy
    pub fn post_mech(&mut self, dt: f64) -> f64 {
        self.arm_deg = turn_arm(&mut self.p.drives, self.t.pivot, self.arm_deg, dt);
        clamp((intake::STOWED - self.arm_deg) / (intake::STOWED - intake::DOWN), 0.0, 1.0)
    }

    /// js `_tunnelRange()`
    fn tunnel_range(&mut self, phys: &PhysicsWorld, fuel: &crate::fuel::Fuel, body: RigidBodyHandle) -> bool {
        let ([x0, y0], z, reach) = RANGE_TUNNEL;
        for k in 0..self.p.held.len() {
            let [x, y, bz] = self.p.local(phys, body, fuel.balls[self.p.held[k]].pos);
            if m::hypot2(x - x0, y - y0) < R * 0.8 && m::abs(bz - z) < reach + R { return true; }
        }
        false
    }
    /// js `_hopperRange()`
    fn hopper_range(&mut self, phys: &PhysicsWorld, fuel: &crate::fuel::Fuel, body: RigidBodyHandle) -> bool {
        let ([x0, y0], z, reach, half) = RANGE_HOPPER;
        let tn = m::tan(half * DEG);
        for k in 0..self.p.held.len() {
            let [x, y, bz] = self.p.local(phys, body, fuel.balls[self.p.held[k]].pos);
            let dx = x - x0;
            if dx > 0.0 && dx < reach + R && m::abs(y - y0) < R * 0.8 && m::abs(bz - z) < R + dx * tn { return true; }
        }
        false
    }

    /// js `postStep(dt, t, { on, feeding, outtake, setpoint, mode })`; returns the robot's stored
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut) -> Vec<usize> {
        let stored = self.p.account(c, t, i.pass, i.legal, stats);
        let body = c.body;
        self.hopper_t = if self.hopper_range(c.phys, c.fuel, body) { self.hopper_t + dt } else { 0.0 };
        self.tunnel_t = if self.tunnel_range(c.phys, c.fuel, body) { self.tunnel_t + dt } else { 0.0 };
        let (hopper_full, tunnel_full) = (self.hopper_t >= 0.75, self.tunnel_t >= 0.06);
        let on = i.on;
        let intaking = on && i.intake;
        let shooting = on && i.feeding && i.setpoint > 0.0 && i.has_shot;
        let s = &mut self.set;

        self.near_trench = on && nearing_trench(i.pos, i.linvel, i.half_l);
        if self.tucked && self.seq.is_none() && shooting && !intaking { self.tucked = false; self.arm_goal = intake::DOWN; self.arm_hold = true; }
        if intaking && self.seq.is_some() { self.seq = None; self.contract_cool = 1.0; self.arm_goal = intake::DOWN; s.intake = Some(1.0); }
        if intaking && !self.was_intaking {
            self.tucked = false; self.seq = None;
            self.arm_goal = intake::DOWN; self.arm_hold = true;
            s.intake = Some(1.0); s.feeder = None;
            self.prefill = Prefill::Armed;
        }
        if !intaking && self.was_intaking { s.intake = None; if self.prefill == Prefill::Running { prefill_stop(s); } self.prefill = Prefill::None; }
        if self.prefill == Prefill::Armed && hopper_full && !tunnel_full {
            self.prefill = Prefill::Running;
            s.tunnel = Some(TunnelSet { rpm: tunnel::PRELOAD, max: None }); s.floor = Some(3.0 / 12.0); s.feeder = Some(3.0 / 12.0);
        }
        if self.prefill == Prefill::Running && tunnel_full { prefill_stop(s); self.prefill = Prefill::Done; }

        let pass = i.pass;
        let fk = if pass { Some(ferry(i.alliance, i.pos).0) } else { None };
        let mut want = None;
        if shooting {
            let tgt = if pass { i.aim_ferry } else { i.aim_hub };
            let dist = m::hypot2(tgt[0] - i.pos[0], tgt[1] - i.pos[2]);
            let w = shot_for(fk.unwrap_or(Kind::Hub), dist);
            want = Some(w);
            s.shooter = Some(w.rpm);
            if !self.was_shooting { s.feeder = None; self.firing = false; }
            self.hood_goal = if crate::mech::phys8793::under_trench(i.pos[0], i.pos[2], 0.0) { shooter::HOOD_MIN } else { w.hood };
        }
        if !shooting && self.was_shooting {
            s.floor = None; s.tunnel = None; s.shooter = Some(shooter::SLOW);
            if !intaking { s.feeder = Some(-4.0 / 12.0); s.intake = None; self.arm_hold = false; } else { s.feeder = None; }
            self.firing = false;
        }
        if !shooting { self.hood_goal = shooter::HOOD_MIN; }
        if !on { *s = Set::default(); }
        let d = &self.p.drives;
        let rpm = (d.omega(self.t.drum) / (2.0 * m::PI)) * 60.0;
        let speed = m::hypot2(i.est_vel[0], i.est_vel[1]);
        let aim = m::abs(i.aim_err.unwrap_or(f64::INFINITY));
        let aimed = aim <= if pass { TOL_FERRY_AIM } else { TOL_HUB_AIM };
        self.verified_t = if aimed { 0.04 } else { m::max(0.0, self.verified_t - dt) };
        let verified = self.verified_t > 0.0;
        let spun_up = want.is_some_and(|w| m::abs(rpm - w.rpm) <= if pass { TOL_FERRY_RPM } else { TOL_RPM }) && m::abs(self.hood_deg - self.hood_goal) <= TOL_HOOD;
        let ready = spun_up && aimed && (pass || speed <= TOL_SPEED);
        if shooting && !self.firing && ready && i.pass_clear != Some(false) { self.firing = true; }
        let far = pass && fk == Some(Kind::Far);
        if shooting && self.firing {
            let w = want.unwrap();
            s.floor = if verified { Some(if far { 3.0 / 12.0 } else { 1.0 }) } else { None };
            s.feeder = if verified { Some(if far { 3.0 / 12.0 } else { 1.0 }) } else { None };
            s.tunnel = if verified { Some(TunnelSet { rpm: w.tunnel, max: Some(if far { 5.0 } else { 12.0 }) }) } else { None };
        }
        self.firing_t = if shooting && self.firing { self.firing_t + dt } else { 0.0 };
        self.contract_cool = m::max(0.0, self.contract_cool - dt);
        if self.seq.is_none() && !self.tucked && on && !intaking && self.firing_t > 1.0 && self.contract_cool == 0.0 {
            let mut out_front = false;
            for k in 0..self.p.held.len() { if self.p.local(c.phys, body, c.fuel.balls[self.p.held[k]].pos)[0] > HALF { out_front = true; break; } }
            if out_front { self.seq = Some(Seq { i: 0, t: 0.0 }); }
        }
        let s = &mut self.set;
        if let Some(q) = &mut self.seq {
            let step = CONTRACT[q.i];
            q.t += dt;
            s.intake = Some(2.0 / 12.0); self.arm_hold = true;
            self.arm_goal = match step { Step::Wait => self.arm_goal, Step::Deg(g) => g, Step::Down => intake::DOWN };
            let done = if step == Step::Wait { q.t >= 0.5 } else { m::abs(self.arm_deg - self.arm_goal) < 12.0 };
            if !on { self.seq = None; }
            else if done && { q.i += 1; q.i >= CONTRACT.len() } {
                self.seq = None;
                self.contract_cool = 1.0;
                s.intake = if intaking { Some(1.0) } else { None };
            } else if done { q.t = 0.0; }
        }
        if on && i.outtake { s.intake = Some(-1.0); s.floor = Some(-1.0); s.feeder = Some(-1.0); }

        let s = self.set;
        let d = &mut self.p.drives;
        d.trains[self.t.intake].cmd = s.intake;
        d.trains[self.t.floor].cmd = s.floor;
        d.trains[self.t.feeder_t].cmd = s.feeder;
        tunnel_loop(d, &self.t, s.tunnel);
        d.trains[self.t.shooter].cmd = match s.shooter {
            Some(sp) if sp != 0.0 => {
                let rps = d.omega(self.t.drum) / (2.0 * m::PI);
                let tgt = sp / 60.0;
                let volts = shooter::kv() * tgt + shooter::KP * (tgt - rps) + shooter::KS;
                Some(clamp(volts / 12.0, 0.0, 1.0))
            }
            _ => None,
        };
        self.was_intaking = intaking; self.was_shooting = shooting;
        let si = s.intake.unwrap_or(0.0);
        *out = PostOut {
            hood_deg: hood_elevation(self.hood_deg),
            flywheel: self.exit_speed(),
            ready: ready || (shooting && self.firing && verified),
            feeding: if shooting && self.firing && verified { 1.0 } else { 0.0 },
            intake_speed: if si > 0.5 { 1.0 } else if si < 0.0 { -1.0 } else { 0.0 },
        };
        stored
    }
    /// js `armTurn()`: the arm's turn from the CAD's pose (rad), for drawing it
    pub fn arm_turn(&self) -> f64 { intake_rot(self.arm_deg) }
    /// js `extNow()`
    pub fn ext_now(&self) -> f64 { slot_ext(self.arm_deg) }
    /// js `liftNow()` (lift_h: cfg.bay.lift.h, 0 if none)
    pub fn lift_now(&self, lift_h: f64) -> f64 { if lift_h != 0.0 { clamp(self.lift / lift_h, 0.0, 1.0) } else { 0.0 } }
    /// js `watchFull(dt, intaking)`
    pub fn watch_full(&mut self, phys: &PhysicsWorld, fuel: &crate::fuel::Fuel, body: RigidBodyHandle, dt: f64, intaking: bool, lift_h: f64) -> bool {
        let lid = lift_h != 0.0;
        if intaking && lid && !self.expanded && !self.tucked && self.p.held.len() >= LID_N { self.expanded = true; }
        let full = self.p.watch_full(phys, fuel, body, dt, intaking);
        if !full || self.expanded || self.tucked || !lid || self.lift_now(lift_h) > 0.5 { return full; }
        self.expanded = true;
        self.p.full_watch = FullWatch::default();
        self.p.full = false;
        false
    }
    /// js `liftScale()`
    fn lift_scale(&mut self, phys: &PhysicsWorld, fuel: &crate::fuel::Fuel, body: RigidBodyHandle, bay_top: f64, h: f64) -> f64 {
        if h == 0.0 { return 0.0; }
        let now = self.lift_now(h);
        let on_load = clamp((self.load_top(phys, fuel, body) - 0.01 - bay_top) / h, 0.0, 1.0);
        let fits = bay_top + h * on_load + 0.01 <= TRENCH::clearHeight - 0.005;
        let up = !self.tucked && self.expanded && !(self.near_trench && fits);
        if up { 1.0 } else if fits { m::min(now, on_load) } else { now }
    }
    /// js `exitSpeed()`
    pub fn exit_speed(&self) -> f64 {
        let s = m::abs(self.p.drives.omega(self.t.drum)) * shooter::DRUM_D / 2.0;
        if s <= 14.1 { EXIT_PER_DRUM_SURFACE * s } else if s <= 16.5 { 10.1 + (0.77 * (s - 14.1)) / 2.4 } else { m::min(11.1, 10.87 + (0.24 * (s - 16.5)) / 3.2) }
    }
    /// js `capacity()`
    pub fn capacity(&self) -> usize { 78 }
    /// js `floorSpin()`-like: the roller floor's speed, for drawing
    pub fn floor_spin(&self) -> f64 { self.p.drives.trains[self.t.floor].omega }
}

/// js `_prefillStop()`
fn prefill_stop(s: &mut Set) { s.tunnel = None; s.floor = None; s.feeder = None; }
/// js `_nearingTrench()`: under one, or within 1.5 m of one and driving toward it
fn nearing_trench(pos: [f64; 3], v: [f64; 3], half_l: f64) -> bool {
    [Alliance::Blue, Alliance::Red].iter().any(|&a| {
        let hx = crate::field::hub_center(a).0;
        let z_open = HALF_W - TRENCH::clearWidth;
        [1.0, -1.0].iter().any(|sz| {
            let (ox, oz, ohx, ohz) = (hx, (sz * (z_open + HALF_W)) / 2.0, TRENCH::depth / 2.0, (HALF_W - z_open) / 2.0);
            let (dx, dz) = (ox - pos[0], oz - pos[2]);
            if m::abs(dx) < ohx + half_l && m::abs(dz) < ohz + half_l { return true; }
            m::abs(dx) < ohx + half_l + 1.5 && m::abs(dz) < ohz + half_l + 0.3 && dx * v[0] > 0.0 && m::abs(v[0]) > 0.3
        })
    })
}
