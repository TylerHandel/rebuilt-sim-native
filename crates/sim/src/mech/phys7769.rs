//! js/mech/phys7769.js: 7769's CHUNK in play, built of its physical mechanisms (r7769.rs). Their
//! code isn't public: its controls are what the team said on Chief Delphi and estimates where they
//! said nothing. The slide runs out on its position loop at the team's deploy current limit (down
//! to 5 A once out); the four X60s on their velocity loop at the shot table's RPM, the hood to its
//! angle; once the wheels, hood and heading are there (and, at the HUB, stopped), the floor and the
//! feeder run. While it shoots the slide runs in and out (the team's "Truffle Shuffle").
use crate::constants::*;
use crate::mech::drives::VelLoop;
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, Physical, Stats, Where};
use crate::mech::r7769::*;
use crate::mech::r7769::{HALF_L, slide};
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const TAU: f64 = 2.0 * m::PI;
fn clamp(x: f64, a: f64, b: f64) -> f64 { m::max(a, m::min(b, x)) }
/// js HALF: the hopper's half width (its walls)
const HALF: f64 = 0.35;
/// js TOL: its gates
const TOL_RPM: f64 = 0.04;
const TOL_SPEED: f64 = 0.25;
fn tol_aim() -> f64 { 4.0 * DEG }
fn tol_pass_aim() -> f64 { 10.0 * DEG }
/// js IDLE
const IDLE_SHARE: f64 = 0.7;
const IDLE_RPM: f64 = 1500.0;
/// js SHOTS, PASSES: distance (m) -> RPM, hood (deg)
pub const SHOTS: [[f64; 3]; 10] = [[1.3, 2430.0, 0.0], [2.0, 2685.0, 3.0], [2.5, 2720.0, 6.0], [3.0, 2720.0, 10.0], [3.5, 2815.0, 13.0], [4.0, 2875.0, 16.0], [4.5, 2980.0, 19.0], [5.0, 3095.0, 22.0], [5.5, 3220.0, 24.0], [6.0, 3350.0, 26.0]];
pub const PASSES: [[f64; 3]; 11] = [[2.5, 1580.0, 35.0], [3.0, 1770.0, 35.0], [4.0, 2145.0, 35.0], [5.0, 2490.0, 35.0], [6.0, 2835.0, 35.0], [7.0, 3325.0, 35.0], [8.0, 3680.0, 35.0], [9.0, 3970.0, 35.0], [10.0, 4500.0, 35.0], [11.0, 4885.0, 35.0], [12.0, 5440.0, 35.0]];
fn interp(t: &[[f64; 3]], x: f64, j: usize) -> f64 {
    if x <= t[0][0] { return t[0][j]; }
    for k in 1..t.len() {
        if x <= t[k][0] { return t[k - 1][j] + ((t[k][j] - t[k - 1][j]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); }
    }
    t[t.len() - 1][j]
}
/// js `shotFor(kind, d)`: (rpm, hood)
pub fn shot_for(pass: bool, d: f64) -> (f64, f64) { let t: &[[f64; 3]] = if pass { &PASSES } else { &SHOTS }; (interp(t, d, 1), interp(t, d, 2)) }
/// js `slideVolts(goal, ext)`
pub fn slide_volts(goal: f64, ext: f64) -> f64 { clamp((12.0 * (goal - ext)) / 0.025, -12.0, 12.0) }
/// js SHUFFLE
const SHUFFLE_DEPTH: f64 = 0.25;
const SHUFFLE_PERIOD: f64 = 1.0;
const SHUFFLE_DELAY: f64 = 0.4;
/// js `shuffleGoal(t)`
pub fn shuffle_goal(t: f64) -> f64 {
    if t < SHUFFLE_DELAY { slide::OUT } else if (t - SHUFFLE_DELAY) % SHUFFLE_PERIOD < SHUFFLE_PERIOD / 2.0 { slide::OUT - SHUFFLE_DEPTH } else { slide::OUT }
}
/// js EXIT_PER_WHEEL_SURFACE
const EXIT_PER_WHEEL_SURFACE: f64 = 0.45;

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool }
/// What its postStep reads of the robot
pub struct PostIn {
    pub on: bool, pub feeding: bool, pub outtake: bool, pub setpoint: f64, pub pass: bool, pub legal: bool, pub intake: bool,
    /// whether it has a shot (js r.shot)
    pub has_shot: bool,
    /// where it aims: its pass point's and the HUB's aim points
    pub aim_pass: [f64; 2], pub aim_hub: [f64; 2],
    pub pos: [f64; 3], pub yaw: f64, pub est_vel: [f64; 2], pub aim_err: Option<f64>, pub pass_clear: Option<bool>, pub in_zone: bool,
}
pub use crate::mech::phys2910::PostOut;

pub struct Phys7769 {
    pub p: Physical,
    pub t: Trains,
    intake_home: Vec<[f64; 3]>,
    pub ext: f64,
    slide_goal: Option<f64>,
    pub hood_deg: f64,
    hood_goal: f64,
    feed: bool,
    shoot_t: f64,
    want: Option<(f64, f64)>,
    fire_t: f64,
    last_shot_t: f64,
    shots: u32,
    parked: Vec<usize>,
}

impl Phys7769 {
    /// js `new Physical7769(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = crate::mech::drives::Drives::default();
        let t = build7769(&mut d);
        let w = wheels7769(&t);
        let surfaces = path_surfaces7769(&t);
        let mut path_wheels = w.intake;
        path_wheels.extend(w.feed);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.extend([t.belts, t.slide]);
        let path = Mechanism::new(Some(body), shafts, path_wheels, vec![S_BODY, S_FINE, S_SLIDE, S_KICK, S_BELTS, S_NET, S_BUMPER], 6, 1);
        let shooter = Mechanism::new(Some(body), vec![t.wheels], w.shooter, vec![S_FIXED_HOOD, S_HOOD], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), preload(), Some((0.25, 6)));
        let mut me = Phys7769 {
            p, t, intake_home: w.intake_home, ext: 0.0, slide_goal: None, hood_deg: 0.0, hood_goal: 0.0, feed: false, shoot_t: 0.0, want: None,
            fire_t: 0.0, last_shot_t: 0.0, shots: 0, parked: Vec::new(),
        };
        me.reset();
        me
    }

    /// js `reset()`: the intake in, the hood down
    pub fn reset(&mut self) {
        self.p.reset();
        self.ext = 0.0; self.slide_goal = None;
        self.hood_deg = 0.0; self.hood_goal = 0.0;
        self.feed = false; self.shoot_t = 0.0; self.want = None;
        self.fire_t = 0.0; self.last_shot_t = 0.0; self.shots = 0; self.parked.clear();
        let t = &self.t;
        let d = &mut self.p.drives;
        for tr in [t.intake, t.floor, t.feed_t, t.shooter] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = None; d.trains[tr].vel = None; }
        d.shafts[t.slide].omega_ = 0.0; d.shafts[t.slide].cmd = None;
    }

    /// js `frontX()`
    fn front_x(ext: f64) -> f64 { FRONT_X + slide_offset(ext)[0] }
    /// js `where(x, y, z)` (ext: the intake's, as it is now)
    pub fn where_(ext: f64, [x, y, z]: [f64; 3]) -> Where {
        let in_frame = m::abs(x) < HALF_L && m::abs(z) < HALF && y > 0.105 && y < 0.62;
        let front = Self::front_x(ext) + 0.03;
        let in_front = x >= HALF_L && x < m::max(front, HALF_L + 0.2) && m::abs(z) < HALF && y < 0.6;
        Where {
            own: in_frame || in_front,
            held: in_frame || (in_front && y > 0.2 && x < front),
            near: x > -HALF_L - 0.1 && x < m::max(front, HALF_L + 0.2) + 0.1 && m::abs(z) < HALF + 0.1 && y < 0.75,
        }
    }
    /// js `shooterBalls(near)`'s test: up the ramp and round the wheels
    pub fn shooter_ball([x, y, _z]: [f64; 3]) -> bool { y > 0.33 && x < 0.03 }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, dt: f64, r: &PreIn) {
        let on = r.enabled;
        let s = self.t.slide;
        let d = &mut self.p.drives;
        slide_stops(d, s, self.ext);
        let out = self.ext >= slide::OUT - 0.005;
        d.shafts[s].motor.as_mut().unwrap().stator_limit = if self.slide_goal.is_some_and(|g| g >= slide::OUT - 0.005) && out { slide::HELD } else { slide::DEPLOY };
        d.shafts[s].cmd = match self.slide_goal { Some(g) if on => Some(slide_volts(g, self.ext) / 12.0), _ => None };
        d.impulse(s, slide::MASS * 9.81 * slide::dir()[1] * slide::pinion() * dt);
        if on { self.hood_deg += clamp(self.hood_goal - self.hood_deg, -hood::RATE * dt, hood::RATE * dt); }
        place_slide(&mut self.p.surfaces, self.ext);
        let n = self.intake_home.len();
        place_intake(&mut self.p.path.wheels[..n], &self.intake_home, self.ext);
        self.p.surfaces[S_HOOD].set_pose([AXLE[0], AXLE[1], 0.0], 0.0, hood_rot(self.hood_deg));
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
        let s = self.t.slide;
        let d = &mut self.p.drives;
        self.ext += d.omega(s) * slide::pinion() * dt;
        if self.ext < 0.0 { self.ext = 0.0; if d.shafts[s].omega_ < 0.0 { d.shafts[s].omega_ = 0.0; } }
        if self.ext > slide::OUT { self.ext = slide::OUT; if d.shafts[s].omega_ > 0.0 { d.shafts[s].omega_ = 0.0; } }
        clamp(self.ext / slide::OUT, 0.0, 1.0)
    }

    /// js `passTarget()`: 3 m out from its ALLIANCE WALL, 2 m in from the nearer side wall
    pub fn pass_target(a: Alliance, pos: [f64; 3]) -> [f64; 2] {
        let (ax, az) = fw(3.0, 2.0, a);
        let (bx, bz) = fw(3.0, FIELD_W - 2.0, a);
        if m::hypot2(ax - pos[0], az - pos[2]) <= m::hypot2(bx - pos[0], bz - pos[2]) { [ax, az] } else { [bx, bz] }
    }

    /// js `postStep(dt, t, { on, feeding, outtake, setpoint, mode })`; returns the robot's stored
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut) -> Vec<usize> {
        let mut stored = self.p.account(c, t, i.pass, i.legal, stats);
        let (on, pass) = (i.on, i.pass);
        let intaking = on && i.intake;
        let has = !stored.is_empty();
        let mut want = None;
        if i.setpoint > 0.0 && i.has_shot {
            let tgt = if pass { i.aim_pass } else { i.aim_hub };
            let dist = m::hypot2(tgt[0] - i.pos[0], tgt[1] - i.pos[2]);
            want = Some(shot_for(pass, dist));
        }
        let trench = crate::mech::phys8793::under_trench(i.pos[0], i.pos[2], 0.45);
        self.hood_goal = match want { Some((_, h)) if !trench => clamp(h, shooter::HOOD_MIN, shooter::HOOD_MAX), _ => 0.0 };
        let mut rpm = 0.0;
        if on {
            if let Some((r, _)) = want { rpm = r; }
            else if has && i.in_zone { let c = i.aim_hub; rpm = IDLE_SHARE * shot_for(false, m::hypot2(c[0] - i.pos[0], c[1] - i.pos[2])).0; }
            else { rpm = IDLE_RPM; }
        }
        let (tt, wh) = (self.t.shooter, self.t.wheels);
        let d = &mut self.p.drives;
        if rpm > 0.0 { d.trains[tt].vel = Some(VelLoop { target: (rpm / 60.0) * TAU, kv: shooter::kv(), kp: shooter::KP, ks: 0.0, lo: 0.0, hi: 12.0, per: Some(1.0 / TAU), rate: 1000.0 }); }
        else { d.trains[tt].vel = None; d.trains[tt].cmd = None; }
        let now = (d.omega(wh) * 60.0) / TAU;
        let speed = m::hypot2(i.est_vel[0], i.est_vel[1]);
        let aim = m::abs(i.aim_err.unwrap_or(f64::INFINITY));
        let spun = want.is_some_and(|(r, _)| m::abs(now - r) <= TOL_RPM * r);
        let hood_ok = m::abs(self.hood_deg - self.hood_goal) <= hood::TOL;
        let ready = want.is_some() && spun && hood_ok && (if pass { aim <= tol_pass_aim() } else { aim <= tol_aim() && speed <= TOL_SPEED });
        self.feed = on && i.feeding && ready && i.pass_clear != Some(false);
        let shooting = on && i.feeding && want.is_some();
        self.shoot_t = if shooting { self.shoot_t + dt } else { 0.0 };
        let (mut floor, mut feed, mut roller) = (if self.feed { 1.0 } else { 0.0 }, if self.feed { 1.0 } else { 0.0 }, 0.0);
        let skewed = crate::mech::phys8793::under_trench(i.pos[0], i.pos[2], 0.9) && m::abs(m::sin(i.yaw)) > 0.25 && self.ext < 0.05;
        if shooting && !intaking && self.ext > 0.05 && self.shoot_t > SHUFFLE_DELAY { self.slide_goal = Some(shuffle_goal(self.shoot_t)); roller = 12.0; }
        else if intaking { if !skewed { self.slide_goal = Some(slide::OUT); } roller = if self.ext > 0.2 { 12.0 } else { 0.0 }; }
        else if let Some(g) = self.slide_goal { if g < slide::OUT && g > 0.0 { self.slide_goal = Some(slide::OUT); } }
        // (what its driver does: firing with nothing shot for a second, the last few left in the
        // corners aren't counted as the load until it next intakes)
        if stats.shots != self.shots { self.shots = stats.shots; self.last_shot_t = t; }
        self.fire_t = if self.feed { self.fire_t + dt } else if shooting { self.fire_t } else { 0.0 };
        if self.feed && self.fire_t > 1.5 && t - self.last_shot_t > 1.0 { for &b in &self.p.held { if !self.parked.contains(&b) { self.parked.push(b); } } }
        if intaking && !shooting { self.parked.clear(); }
        if !self.parked.is_empty() { stored.retain(|b| !self.parked.contains(b)); }
        if on && i.outtake { roller = -12.0; floor = -0.5; feed = -0.5; }
        if !on { roller = 0.0; floor = 0.0; feed = 0.0; }
        let d = &mut self.p.drives;
        d.trains[self.t.intake].cmd = if roller != 0.0 { Some(roller / 12.0) } else { None };
        d.trains[self.t.floor].cmd = if floor != 0.0 { Some(floor) } else { None };
        d.trains[self.t.feed_t].cmd = if feed != 0.0 { Some(feed) } else { None };
        self.want = want;
        *out = PostOut {
            hood_deg: hood_elevation(self.hood_deg),
            flywheel: self.exit_speed(),
            ready,
            feeding: if self.feed { 1.0 } else { 0.0 },
            intake_speed: if roller > 6.0 { 1.0 } else if roller < 0.0 { -1.0 } else { 0.0 },
        };
        stored
    }

    /// js `exitSpeed()`: about how fast a FUEL leaves at the wheels' speed now (m/s)
    pub fn exit_speed(&self) -> f64 { EXIT_PER_WHEEL_SURFACE * m::abs(self.p.drives.omega(self.t.wheels)) * shooter::wheel()[0] }
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
    pub fn capacity(&self) -> usize { 60 }
    /// js `extNow()`: how far out the intake is (m)
    pub fn ext_now(&self) -> f64 { self.ext }
    /// js `shotStatus()`
    pub fn shot_status(&self) -> Option<&'static str> {
        let (rpm, _) = self.want?;
        if self.feed { return Some("SHOOTING"); }
        if m::abs(self.hood_deg - self.hood_goal) > hood::TOL { return Some("Hood"); }
        if m::abs((self.p.drives.omega(self.t.wheels) * 60.0) / TAU - rpm) > TOL_RPM * rpm { return Some("Spinning up"); }
        None
    }
    /// js `mechPose()`: its moving parts, for drawing them, as its drawing's rig names them (the
    /// shooter's wheels drawn no faster than 80 rad/s, as its anim draws them), and the hood's turn
    /// (js hoodTurn(): its anim turns the hood by that)
    pub fn mech_pose(&self) -> [(&'static str, f64); 8] {
        let (d, t) = (&self.p.drives, &self.t);
        [("mech.ext", self.ext), ("mech.hood", hood_rot(self.hood_deg)), ("mech.kick", kick_rot(self.ext)),
         ("mech.roller", d.omega(t.roller)), ("mech.belts", d.omega(t.belts)), ("mech.feed", d.omega(t.feed)), ("mech.wheels", m::min(80.0, d.omega(t.wheels))),
         ("hoodTurn", hood_rot(self.hood_deg))]
    }
    /// js `floorSpin()`
    pub fn floor_spin(&self) -> f64 { self.p.drives.omega(self.t.belts) }
}
