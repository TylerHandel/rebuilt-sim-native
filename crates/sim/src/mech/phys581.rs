//! js/mech/phys581.js: 581's robot in play, built of its physical mechanisms (r581.rs). Its
//! controls are its code's: the deploy homing out, then their position loop (Motion Magic for the
//! compactions); their robot, hopper and shooter state machines (warm-up, prepare, score or feed,
//! the compaction after a delay, beast mode, unjamming); their power budgets per state; the
//! shooter's torque-current loop with the feeder's feedforward; their hopper and tower range
//! sensors.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor};
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, Physical, Stats, Where};
use crate::mech::r581::*;
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const TAU: f64 = 2.0 * m::PI;
const R: f64 = FUEL::radius;
/// js HALF_L, HALF_W: its frame (27 x 28.5in)
const HALF_L_581: f64 = 0.343;
const HALF_W_581: f64 = 0.362;
/// js TOL
fn tol_score_goal() -> f64 { 5.0 * IN }
fn tol_feed_goal() -> f64 { 20.0 * IN }
fn tol_min_aim() -> f64 { 2.0 * DEG }
fn tol_loose() -> f64 { 45.0 * DEG }
const TOL_SCORE_SPEED: f64 = 1.5;
const TOL_FEED_SPEED: f64 = 4.0;
const TOL_LOOKAHEAD: f64 = 0.25;
const TOL_HOOD: f64 = 3.0;
const TOL_FEED_HOOD: f64 = 5.0;
const COMPACTION_DELAY: f64 = 0.7;
/// js HOOD
const HOOD_KP: f64 = 300.0 / 360.0;
const HOOD_MAX_V: f64 = 10.0;
fn hood_free() -> f64 { (7530.0 / 60.0 / ((62.0 / 8.0) * (154.0 / 10.0))) * 360.0 }
/// js POWER: [shooter, intake, deploy, (unused), feeder, floor] supply limits
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Power { Idle, Scoring, ScoringFar, BeastMode, Feeding, FeedingFar, PrioritizeIntake }
fn power(p: Power) -> [f64; 6] {
    match p {
        Power::Idle => [50.0, 25.0, 25.0, 40.0, 10.0, 10.0],
        Power::Scoring => [50.0, 20.0, 15.0, 40.0, 20.0, 20.0],
        Power::ScoringFar => [50.0, 20.0, 15.0, 40.0, 10.0, 10.0],
        Power::BeastMode => [50.0, 10.0, 15.0, 40.0, 30.0, 30.0],
        Power::Feeding => [35.0, 20.0, 15.0, 40.0, 30.0, 30.0],
        Power::FeedingFar => [35.0, 20.0, 15.0, 40.0, 15.0, 15.0],
        Power::PrioritizeIntake => [50.0, 80.0, 15.0, 40.0, 10.0, 10.0],
    }
}
fn clamp(x: f64, a: f64, b: f64) -> f64 { m::max(a, m::min(b, x)) }
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
/// js `dutyFor(motor, amps, w)`
fn duty_for(mo: &Motor, amps: f64, w: f64) -> f64 {
    let eb = (w * mo.ratio) / mo.kv;
    clamp((amps * mo.rm + eb) / 12.0, -1.0, 1.0)
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum DeployState { Unhomed, HomeOutward, Ready }
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum RobotState { Idle, WarmupScore, PrepareScore, PrepareFeed, Score, Feed }
impl RobotState {
    fn has_feed(self) -> bool { matches!(self, RobotState::PrepareFeed | RobotState::Feed) }
    fn has_score(self) -> bool { matches!(self, RobotState::WarmupScore | RobotState::PrepareScore | RobotState::Score) }
}
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum HopperState { IdleDeployed, Intaking, Ejecting, Unjamming, Score, Feed, ScoreAndIntake, FeedAndIntake }
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ShooterState { Off, Idle, Prepare, Shoot }
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Capacity { Low, Medium, High }

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool }
/// What its postStep reads of the robot
pub struct PostIn {
    pub on: bool, pub feeding: bool, pub outtake: bool, pub setpoint: f64, pub pass: bool, pub legal: bool, pub intake: bool,
    pub auto: bool, pub teleop: bool, pub phase_time: f64,
    pub has_shot: bool, pub shot_aim: [f64; 2], pub aim_feed: [f64; 2], pub aim_hub: [f64; 2],
    pub pos: [f64; 3], pub alliance: Alliance, pub est_vel: [f64; 2], pub aim_err: Option<f64>, pub pass_clear: Option<bool>, pub in_zone: bool,
}
pub use crate::mech::phys2910::PostOut;

pub struct Phys581 {
    pub p: Physical,
    pub t: Trains,
    intake_home: Vec<[f64; 3]>,
    floor_home: Vec<[f64; 3]>,
    hood_home: Vec<[f64; 3]>,
    n: (usize, usize),
    pub ext: f64,
    d_state: DeployState, d_goal: f64, d_mm: Option<[f64; 2]>,
    pub hood_deg: f64,
    hood_goal: f64,
    robot_state: RobotState, state_t: f64,
    hopper_state: HopperState, hopper_t: f64,
    shooter_state: ShooterState, shooter_t: f64,
    at_goal_t: f64, range_avg: f64, tower_t: f64, tower: bool,
    unjam_t: f64, last_shot_t: f64, shots: u32,
    power: Power,
    s_intake: Option<f64>, s_floor: Option<f64>, s_feeder: f64,
}

impl Phys581 {
    /// js `new Physical581(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = Drives::default();
        let t = build581(&mut d);
        let w = wheels581(&t, &d);
        let surfaces = path_surfaces581(&t);
        let mut path_wheels = w.floor;
        let n_floor = path_wheels.len();
        path_wheels.extend(w.tower);
        let n_intake = path_wheels.len();
        path_wheels.extend(w.intake);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.push(t.deploy);
        let path = Mechanism::new(Some(body), shafts, path_wheels, vec![S_BODY, S_SLIDE, S_FLOOR, S_HOOD, S_NET], 6, 1);
        let mut sw = vec![w.drum];
        sw.extend(w.hood);
        let shooter = Mechanism::new(Some(body), vec![t.drum, t.hood], sw, vec![S_HOOD], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), PRELOAD.to_vec(), None);
        let mut me = Phys581 {
            p, t, intake_home: w.intake_home, floor_home: w.floor_home, hood_home: w.hood_home, n: (n_floor, n_intake),
            ext: 0.0, d_state: DeployState::Unhomed, d_goal: 0.0, d_mm: None, hood_deg: 0.0, hood_goal: 0.0,
            robot_state: RobotState::Idle, state_t: 0.0, hopper_state: HopperState::IdleDeployed, hopper_t: 0.0, shooter_state: ShooterState::Idle, shooter_t: 0.0,
            at_goal_t: 0.0, range_avg: 0.0, tower_t: 0.0, tower: false, unjam_t: 0.0, last_shot_t: 0.0, shots: 0, power: Power::Idle,
            s_intake: None, s_floor: None, s_feeder: 0.0,
        };
        me.reset();
        me
    }

    /// js `reset()`
    pub fn reset(&mut self) {
        self.p.reset();
        self.ext = 0.0;
        self.d_state = DeployState::Unhomed; self.d_goal = 0.0; self.d_mm = None;
        self.hood_deg = shooter::HOOD_CAD; self.hood_goal = shooter::HOOD_CAD;
        self.robot_state = RobotState::Idle; self.state_t = 0.0;
        self.hopper_state = HopperState::IdleDeployed; self.hopper_t = 0.0;
        self.shooter_state = ShooterState::Idle; self.shooter_t = 0.0;
        self.at_goal_t = 0.0; self.range_avg = 20.0 * IN; self.tower_t = 0.0; self.tower = false;
        self.unjam_t = 0.0; self.last_shot_t = 0.0; self.shots = 0;
        self.power = Power::Idle;
        self.s_intake = None; self.s_floor = None; self.s_feeder = 0.0;
        let t = &self.t;
        let d = &mut self.p.drives;
        for tr in [t.intake, t.floor, t.feeder_t, t.shooter] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = None; }
        d.shafts[t.deploy].omega_ = 0.0; d.shafts[t.deploy].cmd = Some(0.0);
    }

    /// js `where(x, y, z)` (ext: the intake's)
    pub fn where_(ext: f64, [x, y, z]: [f64; 3]) -> Where {
        let in_frame = m::abs(x) < HALF_L_581 && m::abs(z) < HALF_W_581 && y > 0.09 && y < 0.75;
        let front = HALF_L_581 + ext + 0.05;
        let in_front = x >= HALF_L_581 && x < front && m::abs(z) < 0.345 && y < 0.56;
        Where { own: in_frame || in_front, held: in_frame || (in_front && y > 0.2), near: x > -HALF_L_581 - 0.1 && x < front + 0.08 && m::abs(z) < HALF_W_581 + 0.08 && y < 0.8 }
    }
    /// js `shooterBalls(near)`'s test
    pub fn shooter_ball([x, y, _z]: [f64; 3]) -> bool { y > 0.38 && x < -0.03 }
    /// js `parked(b)`: one left on the shelf at the hopper's front, out of the floor's reach
    fn parked(&mut self, c: &Ctx, b: usize) -> bool {
        let l = self.p.local(c.phys, c.body, c.fuel.balls[b].pos);
        let [dx, dy] = slide_offset(self.ext);
        l[0] - dx > 0.5 && l[0] - dx < 0.62 && l[1] - dy > 0.31 && l[1] - dy < 0.4
    }

    /// js `_deployStep(dt, on)`
    fn deploy_step(&mut self, dt: f64, on: bool) {
        let dp = self.t.deploy;
        let inches = self.ext / IN;
        let d = &mut self.p.drives;
        let w = d.shafts[dp].omega_;
        if !on || self.d_state == DeployState::Unhomed { d.shafts[dp].cmd = Some(0.0); }
        else if self.d_state == DeployState::HomeOutward { d.shafts[dp].cmd = Some(deploy::HOME_V / 12.0); }
        else {
            let mut goal = self.d_goal;
            if let Some(mm) = &mut self.d_mm { *mm = profile(mm[0], mm[1], goal, deploy::MM_V, deploy::MM_A, dt); goal = mm[0]; }
            let amps = clamp(deploy::KP * (goal - inches), -35.0, 100.0);
            d.shafts[dp].cmd = Some(duty_for(d.shafts[dp].motor.as_ref().unwrap(), amps, w));
        }
        let floor_torque = floor::MASS * 9.81 * floor::COM * m::cos(floor_rot(self.ext) + 0.21);
        d.impulse(dp, (slide::MASS * 9.81 * slide::DIR[1] * slide::pinion() + floor_torque * floor_rate(self.ext) * slide::pinion()) * dt);
    }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, dt: f64, r: &PreIn) {
        let on = r.enabled;
        self.deploy_step(dt, on);
        let v = clamp(HOOD_KP * (self.hood_goal - self.hood_deg), -HOOD_MAX_V, HOOD_MAX_V);
        if on { self.hood_deg += (v / 12.0) * hood_free() * dt; }
        let [dx, dy] = slide_offset(self.ext);
        self.p.surfaces[S_SLIDE].set_pose([dx, dy, 0.0], 0.0, 0.0);
        let (nf, ni) = self.n;
        place_intake(&mut self.p.path.wheels[ni..], &self.intake_home, self.ext);
        place_floor_surface(&mut self.p.surfaces, &mut self.p.drives, self.t.cam, self.ext);
        place_floor(&mut self.p.path.wheels[..nf], &self.floor_home, self.ext);
        self.p.surfaces[S_HOOD].set_pose([DRUM[0], DRUM[1], 0.0], 0.0, hood_rot(self.hood_deg));
        place_hood(&mut self.p.shooter.as_mut().unwrap().wheels[1..], &self.hood_home, self.hood_deg);
        place_net(&mut self.p.surfaces, self.ext);
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
        let dp = self.t.deploy;
        let d = &mut self.p.drives;
        self.ext += d.omega(dp) * slide::pinion() * dt;
        if self.ext < 0.0 { self.ext = 0.0; if d.shafts[dp].omega_ < 0.0 { d.shafts[dp].omega_ = 0.0; } }
        if self.ext > slide::OUT { self.ext = slide::OUT; if d.shafts[dp].omega_ > 0.0 { d.shafts[dp].omega_ = 0.0; } }
        clamp(self.ext / slide::OUT, 0.0, 1.0)
    }

    /// js `_hopperRange()`: their hopper's range sensor, looking along the load's top
    fn hopper_range(&mut self, c: &Ctx) -> f64 {
        let ([x0, y0], z, reach, half) = (ranges::HOPPER_AT, ranges::HOPPER_Z, ranges::HOPPER_REACH, ranges::HOPPER_HALF);
        let tn = m::tan(half * DEG);
        let mut d = reach;
        for b in self.p.held.clone() {
            let [x, y, bz] = self.p.local(c.phys, c.body, c.fuel.balls[b].pos);
            let dx = x - x0;
            let cone = R + dx * tn;
            if dx > 0.0 && m::abs(y - y0) < cone && m::abs(bz - z) < cone { d = m::min(d, dx - R); }
        }
        m::max(0.0, d)
    }
    /// js `_towerBeam()`
    fn tower_beam(&mut self, c: &Ctx) -> bool {
        let [x0, y0] = ranges::TOWER_AT;
        let held = self.p.held.clone();
        held.iter().any(|&b| { let l = self.p.local(c.phys, c.body, c.fuel.balls[b].pos); m::hypot2(l[0] - x0, l[1] - y0) < R * 0.8 })
    }
    /// js `_feedPoint()` (and `passTarget()`): its feed point on its side, or the backup one if the HUB's in the way
    pub fn feed_point(a: Alliance, pos: [f64; 3]) -> [f64; 2] {
        let f = |x: f64, y: f64| { let (px, pz) = fw(x, y, a); [px, pz] };
        let pts = [[f(0.79, 3.0), f(1.39, 1.0)], [f(0.79, FIELD_W - 3.0), f(1.39, FIELD_W - 1.0)]];
        let [main, backup] = if m::abs(pts[0][0][1] - pos[2]) <= m::abs(pts[1][0][1] - pos[2]) { pts[0] } else { pts[1] };
        let (cx, cz) = crate::field::hub_center(a);
        let (dx, dz) = (main[0] - pos[0], main[1] - pos[2]);
        let l2 = { let v = dx * dx + dz * dz; if v == 0.0 { 1.0 } else { v } };
        let u = clamp(((cx - pos[0]) * dx + (cz - pos[2]) * dz) / l2, 0.0, 1.0);
        let blocked = m::hypot2(pos[0] + u * dx - cx, pos[2] + u * dz - cz) < HUB::size / 2.0 + 0.15;
        if blocked { backup } else { main }
    }
    /// js `_nearTrench()`
    fn near_trench(pos: [f64; 3], v: [f64; 3]) -> bool {
        let mut any = false;
        for a in [Alliance::Blue, Alliance::Red] {
            let hx = crate::field::hub_center(a).0;
            let z_open = HALF_W - TRENCH::clearWidth;
            for sz in [1.0, -1.0] {
                let (ox, oz, ohx, ohz) = (hx, (sz * (z_open + HALF_W)) / 2.0, TRENCH::depth / 2.0, (HALF_W - z_open) / 2.0);
                let (dx, dz) = (ox - pos[0], oz - pos[2]);
                if m::abs(dx) < ohx + HALF_L_581 && m::abs(dz) < ohz + HALF_L_581 { any = true; }
                else if m::abs(dx) < ohx + HALF_L_581 + 1.0 && m::abs(dz) < ohz + HALF_L_581 + 0.3 && dx * v[0] > 0.0 && m::abs(v[0]) > 0.3 { any = true; }
                if any { return true; }
            }
        }
        false
    }

    /// js `postStep(dt, t, { on, feeding, outtake, setpoint, mode })`; returns the robot's stored and
    /// whether it lets go of the robot's heading (under a TRENCH)
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut) -> (Vec<usize>, bool) {
        let mut stored = self.p.account(c, t, i.pass, i.legal, stats);
        stored.retain(|&b| !self.parked(c, b));
        let on = i.on;
        let rng = self.hopper_range(c);
        self.range_avg += (rng - self.range_avg) * m::min(1.0, dt / 0.5);
        let capacity = if self.range_avg <= ranges::high() { Capacity::High } else if self.range_avg <= ranges::medium() { Capacity::Medium } else { Capacity::Low };
        self.tower_t = if self.tower_beam(c) { 0.25 } else { m::max(0.0, self.tower_t - dt) };
        self.tower = self.tower_t > 0.0;
        if on && self.d_state == DeployState::Unhomed { self.d_state = DeployState::HomeOutward; }
        if self.d_state == DeployState::HomeOutward && self.ext >= slide::OUT - 0.002 { self.d_state = DeployState::Ready; self.ext = slide::OUT; }
        let homed = self.d_state == DeployState::Ready;
        let dp = self.t.deploy;
        let deploy_omega = self.p.drives.omega(dp);
        let ext = self.ext;
        let deploy_to = |me: &mut Self, inches: f64, mm: bool| {
            if !homed { return; }
            let g = clamp(inches, deploy::MIN, deploy::MAX);
            if mm && me.d_mm.is_none() { me.d_mm = Some([ext / IN, (deploy_omega * slide::pinion()) / IN]); }
            if !mm { me.d_mm = None; }
            me.d_goal = g;
        };
        let at_intake = homed && m::abs(self.ext / IN - deploy::INTAKE) <= deploy::TOL;
        let intaking = on && i.intake;
        let eject = on && i.outtake;
        let pass = i.pass;
        let held = on && i.feeding && i.setpoint > 0.0 && i.has_shot;
        let speed = m::hypot2(i.est_vel[0], i.est_vel[1]);
        let beast = i.teleop && i.phase_time > 135.0;
        let (mut want, mut dist) = (None, 0.0);
        if held {
            let tgt = if pass { i.aim_feed } else { i.aim_hub };
            let aim = if speed > 0.2 && !pass { i.shot_aim } else { tgt };
            dist = m::hypot2(aim[0] - i.pos[0], aim[1] - i.pos[2]);
            want = Some(shot_for(pass, dist));
        }
        let prev = self.robot_state;
        let (dr, sh) = (self.t.drum, self.t.shooter);
        let rpm = (self.p.drives.omega(dr) / TAU) * 60.0;
        let hub_d = m::hypot2(i.aim_hub[0] - i.pos[0], i.aim_hub[1] - i.pos[2]);
        let warm = on && !held && i.teleop && capacity == Capacity::High && i.in_zone && !intaking && m::abs(i.aim_err.unwrap_or(m::PI)) < m::PI / 2.0;
        if !held { self.robot_state = if warm { RobotState::WarmupScore } else { RobotState::Idle }; }
        else if !(if pass { self.robot_state.has_feed() } else { self.robot_state.has_score() }) || self.robot_state == RobotState::WarmupScore {
            self.robot_state = if pass { RobotState::PrepareFeed } else { RobotState::PrepareScore };
        }
        if self.robot_state != prev { self.state_t = 0.0; }
        self.state_t += dt;
        let tol = if self.robot_state == RobotState::Score { shooter::RPM_TOL_SHOOTING } else if pass { shooter::RPM_TOL_FEEDING } else { shooter::RPM_TOL };
        self.at_goal_t = if want.is_some_and(|w| m::abs(rpm - w.0) <= tol) { 1.0 / 15.0 } else { m::max(0.0, self.at_goal_t - dt) };
        let shooter_ok = self.at_goal_t > 0.0;
        let lv = c.phys.bodies[c.body].linvel();
        let near_trench = Self::near_trench(i.pos, [lv.x as f64, lv.y as f64, lv.z as f64]);
        self.hood_goal = if !homed { shooter::HOOD_CAD } else { match want { Some(w) if !(self.robot_state == RobotState::PrepareScore && near_trench) => w.1, _ => shooter::HOOD_IDLE } };
        let hood_ok = m::abs(self.hood_goal - self.hood_deg) <= if pass { TOL_FEED_HOOD } else { TOL_HOOD };
        let err = m::abs(i.aim_err.unwrap_or(f64::INFINITY));
        let wy = c.phys.bodies[c.body].angvel().y as f64;
        let turn = m::abs(wy);
        let aim_tol = m::max(tol_min_aim(), m::atan2(if pass { tol_feed_goal() } else { tol_score_goal() }, if dist == 0.0 { 1.0 } else { dist }));
        let loose = if pass && self.state_t > 1.0 && self.robot_state == RobotState::Feed { tol_loose() } else { 0.0 };
        let aimed = err <= m::max(aim_tol, loose) || (err < m::PI && turn * TOL_LOOKAHEAD >= err && i.aim_err.is_some_and(|e| m::sign(e) == m::sign(wy)));
        if held {
            let ok = if pass { aimed && speed <= TOL_FEED_SPEED && shooter_ok && hood_ok && i.pass_clear != Some(false) }
                else { aimed && speed <= TOL_SCORE_SPEED && shooter_ok && hood_ok && !near_trench };
            let fire = if pass { RobotState::Feed } else { RobotState::Score };
            if ok && self.robot_state != fire { self.robot_state = fire; self.state_t = 0.0; }
            if !ok && self.robot_state == fire { self.robot_state = if pass { RobotState::PrepareFeed } else { RobotState::PrepareScore }; self.state_t = 0.0; }
        }
        let firing = matches!(self.robot_state, RobotState::Score | RobotState::Feed);
        if stats.shots != self.shots { self.shots = stats.shots; self.last_shot_t = t; }
        if !firing { self.last_shot_t = m::max(self.last_shot_t, t - 0.5); }
        if firing && self.unjam_t <= 0.0 && self.state_t > 1.5 && t - self.last_shot_t > 1.0 && !stored.is_empty() { self.unjam_t = 0.6; }
        if self.unjam_t > 0.0 { self.unjam_t -= dt; if self.unjam_t <= 0.0 { self.last_shot_t = t; } }
        let h_prev = self.hopper_state;
        self.hopper_state = if eject { HopperState::Ejecting } else if self.unjam_t > 0.0 { HopperState::Unjamming }
            else if firing { match (pass, intaking) { (true, true) => HopperState::FeedAndIntake, (true, false) => HopperState::Feed, (false, true) => HopperState::ScoreAndIntake, (false, false) => HopperState::Score } }
            else if intaking { HopperState::Intaking } else { HopperState::IdleDeployed };
        if self.hopper_state != h_prev { self.hopper_t = 0.0; }
        self.hopper_t += dt;
        let fill = !self.tower && at_intake && matches!(capacity, Capacity::Medium | Capacity::High);
        match self.hopper_state {
            HopperState::IdleDeployed => { deploy_to(self, deploy::INTAKE, false); self.s_intake = None; self.s_floor = if fill { Some(1.0) } else { None }; self.s_feeder = if fill { 1.0 } else { 0.0 }; }
            HopperState::Intaking => {
                deploy_to(self, deploy::INTAKE, false); self.s_intake = Some(12.0);
                self.s_floor = if fill { Some(1.0) } else if self.tower { None } else { Some(2.0) };
                self.s_feeder = if fill { 1.0 } else if self.tower { 0.0 } else { -1.0 };
            }
            HopperState::Ejecting => { deploy_to(self, deploy::INTAKE, false); self.s_intake = Some(-12.0); self.s_floor = Some(-12.0); self.s_feeder = 0.0; }
            HopperState::Unjamming => { deploy_to(self, deploy::INTAKE, false); self.s_intake = Some(-12.0); self.s_floor = Some(10.0); self.s_feeder = 10.0; }
            HopperState::Score => {
                self.s_feeder = 10.0;
                if beast { deploy_to(self, deploy::BEAST_MODE_COMPACTION, false); self.s_intake = Some(5.0); self.s_floor = Some(10.0); }
                else if self.hopper_t >= COMPACTION_DELAY { deploy_to(self, deploy::SCORE_COMPACTION, true); self.s_intake = None; self.s_floor = Some(10.0); }
                else { deploy_to(self, deploy::SCORE_COMPACTION_WAITING, true); self.s_intake = Some(5.0); self.s_floor = Some(10.0); }
            }
            HopperState::Feed => {
                self.s_feeder = 10.0; self.s_floor = Some(10.0);
                if self.hopper_t >= COMPACTION_DELAY { deploy_to(self, deploy::FEED_COMPACTION, false); self.s_intake = None; }
                else { if self.d_goal != deploy::FEED_COMPACTION { deploy_to(self, deploy::INTAKE, false); } self.s_intake = Some(5.0); }
            }
            HopperState::ScoreAndIntake | HopperState::FeedAndIntake => { deploy_to(self, deploy::INTAKE, false); self.s_intake = Some(12.0); self.s_floor = Some(10.0); self.s_feeder = 10.0; }
        }
        if !on { self.s_intake = None; self.s_floor = None; self.s_feeder = 0.0; }
        let far = if pass { let fp = Self::feed_point(i.alliance, i.pos); m::hypot2(fp[0] - i.pos[0], fp[1] - i.pos[2]) > 7.0 } else { hub_d > 4.0 };
        let starts_prepare = matches!(self.robot_state, RobotState::PrepareScore | RobotState::PrepareFeed);
        self.power = if firing || starts_prepare {
            if pass { if far { Power::FeedingFar } else { Power::Feeding } } else if beast { Power::BeastMode } else if far { Power::ScoringFar } else { Power::Scoring }
        } else if i.auto { Power::PrioritizeIntake } else { Power::Idle };
        let [p_s, p_i, p_d, _, p_f, p_c] = power(self.power);
        let tr = &self.t;
        let d = &mut self.p.drives;
        d.trains[tr.shooter].motor.supply_limit = p_s;
        d.trains[tr.intake].motor.supply_limit = p_i;
        d.shafts[tr.deploy].motor.as_mut().unwrap().supply_limit = p_d;
        d.trains[tr.feeder_t].motor.supply_limit = p_f;
        d.trains[tr.floor].motor.supply_limit = p_c;
        d.trains[tr.intake].cmd = self.s_intake.map(|v| v / 12.0);
        d.trains[tr.floor].cmd = self.s_floor.map(|v| v / 12.0);
        d.trains[tr.feeder_t].cmd = Some(self.s_feeder / 12.0);
        let s_prev = self.shooter_state;
        self.shooter_state = if !on { ShooterState::Off } else if firing { ShooterState::Shoot } else if want.is_some() || warm { ShooterState::Prepare } else { ShooterState::Idle };
        if self.shooter_state != s_prev { self.shooter_t = 0.0; }
        self.shooter_t += dt;
        let target = if self.shooter_state == ShooterState::Off { 0.0 } else if let Some(w) = want { w.0 } else if warm { shot_for(false, hub_d).0 } else { shooter::IDLE };
        if target > 0.0 {
            let rps = d.omega(dr) / TAU;
            let tgt = target / 60.0;
            let fm = &d.trains[tr.feeder_t].motor;
            let feeder_amps = m::abs(fm.current) / fm.count;
            let mut ff = feeder_ff(feeder_amps);
            if self.shooter_state == ShooterState::Shoot {
                if self.shooter_t < 0.7 && capacity == Capacity::High { ff = m::max(ff, 20.0); }
                else if self.shooter_t < 0.1 { ff = m::max(ff, 15.0); }
            }
            let amps = clamp(shooter::KS + shooter::KV * tgt + shooter::KP * (tgt - rps) + ff, -200.0, 200.0);
            let tw = d.trains[sh].omega;
            d.trains[sh].cmd = Some(duty_for(&d.trains[sh].motor, amps, tw));
        } else { d.trains[sh].cmd = None; }
        let si = self.s_intake.unwrap_or(0.0);
        *out = PostOut {
            hood_deg: hood_elevation(self.hood_deg),
            flywheel: self.exit_speed(),
            ready: want.is_some() && shooter_ok && hood_ok && aimed,
            feeding: if firing { 1.0 } else { 0.0 },
            intake_speed: if si > 6.0 { 1.0 } else if si < 0.0 { -1.0 } else { 0.0 },
        };
        let let_go = crate::mech::phys8793::under_trench(i.pos[0], i.pos[2], 0.2);
        (stored, let_go)
    }

    /// js `extNow()`, `floorTurn()`, `hoodTurn()`
    pub fn ext_now(&self) -> f64 { self.ext }
    pub fn floor_turn(&self) -> f64 { floor_rot(self.ext) }
    pub fn hood_turn(&self) -> f64 { hood_rot(self.hood_deg) }
    /// js `exitSpeed()`
    pub fn exit_speed(&self) -> f64 { 0.87 * m::abs(self.p.drives.omega(self.t.drum)) * shooter::DRUM_R }
    /// js `capacity()`
    pub fn capacity(&self) -> usize { 50 }
}
