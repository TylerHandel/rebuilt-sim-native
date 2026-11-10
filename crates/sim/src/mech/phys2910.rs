//! 2910's Re•Blitz in play (js/mech/phys2910.js), built of its physical mechanisms (r2910.rs; the
//! shared part: physical.rs). Its controls are its code's (SuperStructure and the intake, hopper and
//! shooter subsystems): the intake down by default, INTAKE at full, RETRACT while a shot's fed; the
//! hopper floor full while feeding, 7% when its CANranges say it's full with nothing waiting; the
//! drum on their velocity loop at their table's RPM, idling at 82% between shots, the hood to their
//! table's angle; a shot fed once the hood's there, the drum's up to speed and the chassis is aimed.
use crate::constants::*;
use crate::mech::drives::Drives;
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, FlatOut, Physical, Stats, Where};
use crate::mech::r2910::*;
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const R: f64 = FUEL::radius;
/// its frame (27in x 27.5in)
const HALF_L: f64 = 0.343;
const HALF_W: f64 = 0.349;

/// js PIVOT_MM: their intake pivot's Motion Magic (degrees of arm), Slot0 kP and kV
struct Mm { v: f64, a: f64, kp: f64, kv: f64 }
fn pivot_mm() -> Mm { Mm { v: (5500.0 / 60.0 / 40.0) * 360.0, a: 4000.0, kp: (5.0 * 40.0) / 360.0, kv: 12.0 / ((5800.0 / 60.0) / (40.0 / 360.0)) } }
/// js HOOD_MM
fn hood_mm() -> (f64, f64) { ((7000.0 / 60.0 / ((52.0 / 12.0) * (18.0 / 12.0) * (146.0 / 10.0))) * 360.0, 3000.0) }
/// js TOL
const TOL_HOOD: f64 = 3.0;
fn tol_aim() -> f64 { 4.0 * DEG }
fn tol_pass_aim() -> f64 { 15.0 * DEG }
const TOL_SPEED: f64 = 0.15;
const TOL_MIN_DIST: f64 = 1.5;
/// js IDLE: the drum's idle between shots, a share of the shot's speed
const IDLE: f64 = 0.82;
/// js EXIT_PER_DRUM_SURFACE
pub const EXIT_PER_DRUM_SURFACE: f64 = 0.8;

/// js `profile(p, v, goal, vmax, amax, dt)`: a trapezoidal profile's next setpoint
pub fn profile(p: f64, v: f64, goal: f64, vmax: f64, amax: f64, dt: f64) -> [f64; 2] {
    let d = goal - p;
    let dir = m::sign(d);
    if m::abs(d) < 1e-6 && m::abs(v) < 1e-6 { return [goal, 0.0]; }
    let v_stop = m::sqrt(2.0 * amax * m::abs(d));
    let want = dir * m::min(vmax, v_stop);
    let nv = v + m::max(-amax * dt, m::min(amax * dt, want - v));
    let np = p + nv * dt;
    if (goal - np) * dir < 0.0 { return [goal, 0.0]; }
    [np, nv]
}

/// What the robot hands its postStep (js postStep's { on, feeding, outtake, setpoint, mode }, and
/// what of the robot it reads)
pub struct PostIn {
    pub on: bool,
    pub feeding: bool,
    pub outtake: bool,
    pub setpoint: f64,
    pub pass: bool,
    /// r.cmd.intake
    pub intake: bool,
    /// r.shot.target, when there's a shot
    pub target: Option<[f64; 2]>,
    /// r.pos (x, z), r.estVel() (x, z), r.aimHub() (x, z)
    pub pos: [f64; 2],
    pub est_vel: [f64; 2],
    pub aim_hub: [f64; 2],
    /// r.aimErr (None: never aimed), r.passClear (None: never set)
    pub aim_err: Option<f64>,
    pub pass_clear: Option<bool>,
    /// r.lastInZone (a shot's legality)
    pub legal: bool,
}
/// What its postStep sets on the robot (js r.hoodDeg, flywheel, ready, feeding, intakeSpeed)
#[derive(Clone, Copy, Debug, Default)]
pub struct PostOut { pub hood_deg: f64, pub flywheel: f64, pub ready: bool, pub feeding: f64, pub intake_speed: f64 }

pub struct Phys2910 {
    pub p: Physical,
    pub t: Trains,
    intake_homes: Vec<[f64; 2]>,
    hood_homes: Vec<[f64; 2]>,
    pub arm_deg: f64,
    pub arm_set: [f64; 2],
    pub hood_deg: f64,
    pub hood_set: [f64; 2],
    pub hood_goal: Option<f64>,
    pub target_rpm: f64,
    pub full_t: f64,
    pub feed: bool,
    pub arm_goal: f64,
    flat: Option<FlatOut>,
}

/// its surfaces (r2910 path_surfaces2910's order): the intake's, the hood's
const S_INTAKE: usize = 1;
const S_HOOD: usize = 2;
/// the path's wheels: the floor's 10, the indexer's 9, then the intake's 3
const W_INTAKE: usize = 19;

impl Phys2910 {
    /// js `new Physical2910(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = Drives::default();
        let t = build2910(&mut d);
        let w = wheels2910(&d, &t);
        let surfaces = path_surfaces2910(&t);
        let intake_homes = w.intake.iter().map(|x| [x.center[0], x.center[1]]).collect();
        let hood_homes = w.hood.iter().map(|x| [x.center[0], x.center[1]]).collect();
        let mut path_wheels = w.floor;
        path_wheels.extend(w.indexer);
        path_wheels.extend(w.intake);
        // (its shafts: each wheel's, once, in order, then the pivot's)
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.push(t.pivot);
        let path = Mechanism::new(Some(body), shafts, path_wheels, vec![0, 1, 2, 3], 6, 1);
        let mut sw = vec![w.drum];
        sw.extend(w.hood);
        let shooter = Mechanism::new(Some(body), vec![t.drum, t.hood], sw, vec![S_HOOD], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), PRELOAD.to_vec(), Some((0.1, 10)));
        let mut me = Phys2910 { p, t, intake_homes, hood_homes, arm_deg: 0.0, arm_set: [0.0; 2], hood_deg: 0.0, hood_set: [0.0; 2], hood_goal: None,
            target_rpm: 0.0, full_t: 0.0, feed: false, arm_goal: DEPLOY, flat: None };
        me.reset();
        me
    }

    /// js `reset()`
    pub fn reset(&mut self) {
        self.p.reset();
        self.arm_deg = 0.0; self.arm_set = [0.0, 0.0];
        self.hood_deg = 0.0; self.hood_set = [0.0, 0.0];
        self.target_rpm = 0.0;
        self.full_t = 0.0;
        self.feed = false;
        self.arm_goal = DEPLOY;
        for tr in [self.t.intake, self.t.floor, self.t.shooter] { self.p.drives.trains[tr].omega = 0.0; self.p.drives.trains[tr].cmd = Some(0.0); }
        self.p.drives.shafts[self.t.pivot].omega_ = 0.0;
        self.p.drives.shafts[self.t.pivot].cmd = Some(0.0);
    }

    /// js `where(x, y, z)`: the hopper runs out over the intake
    pub fn where_(l: [f64; 3]) -> Where {
        let [x, y, z] = l;
        let in_frame = m::abs(x) < HALF_L && m::abs(z) < HALF_W && y > 0.105 && y < 0.62;
        let in_front = x >= HALF_L && x < 0.72 && m::abs(z) < HALF_W && y > BUMPER_Y0 && y < 0.6;
        Where {
            own: in_frame || in_front,
            held: in_frame || (in_front && y > 0.17 && x < 0.6),
            near: x > -HALF_L - 0.08 && x < 0.76 && m::abs(z) < HALF_W + 0.08 && y < 0.66,
        }
    }
    /// js `shooterBalls(near)`'s test
    pub fn shooter_ball(l: [f64; 3]) -> bool { l[1] > 0.33 && l[0] < -0.03 }

    /// js `armAngle()`
    fn arm_angle(&self) -> f64 { intake::CAD_DEG * DEG + intake_rot(self.arm_deg) }

    /// js `preSteps` up to its yield: the pivot's motor, the hood, the moving parts posed
    pub fn pre_mech(&mut self, dt: f64, on: bool, teleop: bool) {
        intake_limits(&mut self.p.drives, &self.t, teleop);
        let mm = pivot_mm();
        self.arm_set = profile(self.arm_set[0], self.arm_set[1], self.arm_goal, mm.v, mm.a, dt);
        let volts = if on { mm.kv * self.arm_set[1] + mm.kp * (self.arm_set[0] - self.arm_deg) } else { 0.0 };
        self.p.drives.shafts[self.t.pivot].cmd = Some(m::max(-1.0, m::min(1.0, -volts / 12.0)));
        let l = -intake::MASS * 9.81 * intake::COM * m::cos(self.arm_angle()) * dt;
        self.p.drives.impulse(self.t.pivot, l);
        let (hv, ha) = hood_mm();
        self.hood_set = profile(self.hood_set[0], self.hood_set[1], self.hood_goal.unwrap_or(0.0), hv, ha, dt);
        self.hood_deg = self.hood_set[0];
        self.p.surfaces[S_INTAKE].set_pose([PIVOT[0], PIVOT[1], 0.0], 0.0, intake_rot(self.arm_deg));
        place_intake(&mut self.p.path.wheels[W_INTAKE..W_INTAKE + 3], &self.intake_homes, self.arm_deg);
        self.p.surfaces[S_HOOD].set_pose([DRUM[0], DRUM[1], 0.0], 0.0, self.hood_deg * DEG);
        let sh = self.p.shooter.as_mut().unwrap();
        place_hood(&mut sh.wheels[1..3], &self.hood_homes, self.hood_deg);
    }
    /// js `PhysicalRobot.stepMechanisms` (at its yield)
    pub fn mechanisms(&mut self, c: &mut Ctx, dt: f64, t: f64) {
        self.p.mech_prepare(c, t, &Self::where_, &Self::shooter_ball);
        self.p.mech_run(dt, &mut c.phys.bodies);
        self.p.mech_finish(c.fuel);
    }
    /// js `preSteps` after its yield: the arm turns as its shaft did; returns the robot's intakeDeploy
    pub fn post_mech(&mut self, dt: f64) -> f64 {
        let om = self.p.drives.omega(self.t.pivot);
        self.arm_deg -= (om * dt) / DEG;
        let pv = &mut self.p.drives.shafts[self.t.pivot];
        if self.arm_deg > DEPLOY { self.arm_deg = DEPLOY; if pv.omega_ < 0.0 { pv.omega_ = 0.0; } }
        if self.arm_deg < -2.0 { self.arm_deg = -2.0; if pv.omega_ > 0.0 { pv.omega_ = 0.0; } }
        m::max(0.0, m::min(1.0, self.arm_deg / DEPLOY))
    }

    /// js `_range({ at, z })`: a CANrange's beam, tripped by a FUEL in it within reach
    fn range(&mut self, c: &mut Ctx, (at, z): ([f64; 2], f64)) -> bool {
        for k in 0..self.p.held.len() {
            let b = self.p.held[k];
            let [x, y, bz] = self.p.local(c.phys, c.body, c.fuel.balls[b].pos);
            if m::hypot2(x - at[0], y - at[1]) < R * 0.8 && m::abs(bz - z) < RANGE_REACH + R { return true; }
        }
        false
    }

    /// js `postStep(dt, t, { on, feeding, outtake, setpoint, mode })`; returns the robot's stored
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut) -> Vec<usize> {
        let hopper = self.range(c, RANGE_HOPPER);
        let feeder = self.range(c, RANGE_FEEDER);
        let stored = self.p.account(c, t, i.pass, i.legal, stats);
        let pass = i.pass;
        let mut want: Option<(f64, f64)> = None;
        let mut dist = 0.0;
        if i.setpoint > 0.0 { if let Some(tgt) = i.target {
            let lv = i.est_vel;
            dist = m::hypot2(tgt[0] - i.pos[0], tgt[1] - i.pos[1]);
            if pass { let tof = pass_tof(dist); dist = m::hypot2(tgt[0] - i.pos[0] - lv[0] * tof, tgt[1] - i.pos[1] - lv[1] * tof); }
            if !pass { let c = i.aim_hub; dist = m::hypot2(c[0] - i.pos[0], c[1] - i.pos[1]); }
            want = Some(if pass { pass_for(dist) } else { shot_for(dist) });
        } }
        match want {
            Some((rpm, hood)) => { self.target_rpm = rpm; self.hood_goal = Some(hood); }
            None => { let c = i.aim_hub; self.target_rpm = IDLE * shot_for(m::hypot2(c[0] - i.pos[0], c[1] - i.pos[1])).0; self.hood_goal = Some(0.0); }
        }
        if !i.on { self.target_rpm = 0.0; }
        let rps = self.p.drives.trains[self.t.shooter].omega / (2.0 * m::PI);
        let target = self.target_rpm / 60.0;
        let volts = if target > 0.0 { shooter::kv() * target + shooter::KP * (target - rps) } else { 0.0 };
        self.p.drives.trains[self.t.shooter].cmd = Some(if i.on { m::max(0.0, m::min(1.0, volts / 12.0)) } else { 0.0 });
        let v = i.est_vel;
        let speed = m::hypot2(v[0], v[1]);
        let hood_ok = m::abs(self.hood_deg - self.hood_goal.unwrap_or(0.0)) <= TOL_HOOD;
        let aim = m::abs(i.aim_err.unwrap_or(f64::INFINITY));
        let target_rpm = self.target_rpm;
        let flat = self.flat.get_or_insert_with(FlatOut::default);
        let flat_out = flat.step(dt, rps * 60.0, if pass && want.is_some() { target_rpm } else { 0.0 });
        let spun_up = m::abs(rps * 60.0 - target_rpm) <= 0.05 * target_rpm || flat_out;
        let ready = want.is_some() && hood_ok && spun_up && (if pass { aim <= tol_pass_aim() } else { aim <= tol_aim() && speed <= TOL_SPEED && dist >= TOL_MIN_DIST });
        self.feed = i.on && i.feeding && ready && i.pass_clear != Some(false);
        self.full_t = if hopper { self.full_t + dt } else { 0.0 };
        self.p.drives.trains[self.t.floor].cmd = Some(if !i.on { 0.0 } else if self.feed { 1.0 } else if self.full_t > 0.25 && !feeder { 0.07 } else { 0.0 });
        let intaking = i.on && i.intake;
        let it = self.t.intake;
        if intaking { self.arm_goal = DEPLOY; self.p.drives.trains[it].cmd = Some(1.0); }
        else if self.feed { self.arm_goal = RETRACT; self.p.drives.trains[it].cmd = Some(0.15); }
        else { self.arm_goal = if i.on { DEPLOY } else { self.arm_goal }; self.p.drives.trains[it].cmd = None; }
        if i.on && i.outtake { self.p.drives.trains[it].cmd = Some(-1.0); }
        let ic = self.p.drives.trains[it].cmd.unwrap_or(0.0);
        *out = PostOut {
            hood_deg: hood_elevation(self.hood_deg),
            flywheel: self.exit_speed(),
            ready,
            feeding: if self.feed { 1.0 } else { 0.0 },
            intake_speed: if ic > 0.5 { 1.0 } else if ic < 0.0 { -1.0 } else { 0.0 },
        };
        stored
    }

    /// js `passTarget()`: their FieldConstants' points to pass to, the nearer
    pub fn pass_target(alliance: Alliance, pos: [f64; 2]) -> [f64; 2] {
        let a = fw(4.0, 2.5, alliance);
        let b = fw(4.0, FIELD_W - 2.5, alliance);
        if m::hypot2(a.0 - pos[0], a.1 - pos[1]) <= m::hypot2(b.0 - pos[0], b.1 - pos[1]) { [a.0, a.1] } else { [b.0, b.1] }
    }
    /// js `exitSpeed()`
    pub fn exit_speed(&self) -> f64 { EXIT_PER_DRUM_SURFACE * m::abs(self.p.drives.omega(self.t.drum)) * shooter::DRUM_D / 2.0 }
    /// js `capacity()`
    pub fn capacity(&self) -> usize { 60 }
    /// js `floorSpin()`
    pub fn floor_spin(&self) -> f64 { self.p.drives.trains[self.t.floor].omega }
}
