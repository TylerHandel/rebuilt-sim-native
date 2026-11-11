//! js/mech/phys1114.js: 1114's Simbot Tim in play, built of its physical mechanisms (r1114.rs). Its
//! controls are its code's (Simbotics/2026-Simbot-Tim): aim (their ShootMoveLUT's RPM less 50 and
//! hood angle, the flywheel idling at 1000 RPM otherwise), shoot (the elevator, floor and side
//! rollers once the flywheel's within 500 RPM and the heading within 7 deg; AgitateIntake pulling
//! the box in once within 400 and 5 deg, 0.65 s on), intake (LowerIntake then
//! RunIntakeBallevator: the box out on its position loop, the roller at 12 V, 20 A out when pushed
//! in; after 3 s the floor at 3 V while their hood CANrange sees nothing), eject.
use crate::constants::*;
use crate::mech::drives::{Drives, Motor, VelLoop};
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, Physical, Stats, Where};
use crate::mech::r1114::*;
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const TAU: f64 = 2.0 * m::PI;
/// js HALF_L, HALF_W: its frame (24.5 x 30in)
const HALF_L: f64 = 0.311;
const HALF_W: f64 = 0.381;
/// js GATE
const GATE_RPM: f64 = 500.0;
fn gate_aim() -> f64 { 7.0 * DEG }
const GATE_AGITATE_RPM: f64 = 400.0;
fn gate_agitate_aim() -> f64 { 5.0 * DEG }
const GATE_AGITATE_WAIT: f64 = 0.65;
/// js HOOD: their hood's position loop
fn hood_kp() -> f64 { shooter::HOOD_KP / shooter::hood_deg_per_rot() }
fn hood_free() -> f64 { (7530.0 / 60.0) * shooter::hood_deg_per_rot() }
fn clamp(x: f64, a: f64, b: f64) -> f64 { m::max(a, m::min(b, x)) }
/// js LINEUP
const LINEUP_NEAR: f64 = 1.6;
const LINEUP_FAR: f64 = 4.5;
/// js EXIT_PER_FLY_SURFACE
const EXIT_PER_FLY_SURFACE: f64 = 0.86;
/// js `dutyFor(motor, amps, w)`: a motor's duty for torque current amps at its output speed w
fn duty_for(mo: &Motor, amps: f64, w: f64) -> f64 {
    let eb = (w * mo.ratio) / mo.kv;
    clamp((amps * mo.rm + eb) / 12.0, -1.0, 1.0)
}

/// js slideMode: what their slide's asked for
#[derive(Clone, Copy, Debug, PartialEq)]
enum SlideMode { Volts(f64), Pos(f64), Amps(f64) }

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool }
/// What its postStep reads of the robot
pub struct PostIn {
    pub on: bool, pub feeding: bool, pub outtake: bool, pub setpoint: f64, pub pass: bool, pub legal: bool, pub intake: bool,
    /// js r.match.isAuto; r.cmd.spin, r.cmd.lineup (the AUTO runner's and the AI's)
    pub auto: bool, pub spin: Option<bool>, pub lineup: bool,
    pub has_shot: bool, pub aim_pass: [f64; 2], pub aim_hub: [f64; 2], pub pos: [f64; 3], pub aim_err: Option<f64>,
}
pub use crate::mech::phys2910::PostOut;

pub struct Phys1114 {
    pub p: Physical,
    pub t: Trains,
    intake_home: Vec<[f64; 3]>,
    hood_home: Vec<[f64; 3]>,
    n_intake: usize,
    pub ext: f64,
    pub hood_deg: f64,
    hood_goal: Option<f64>,
    aim: bool,
    aim_pass: bool,
    intake_t: f64, lowered: bool, pushed_t: f64, deploying: bool, ballev_t: f64,
    shoot_t: f64,
    slide_mode: SlideMode,
    want: Option<(f64, f64)>,
    fire_t: f64, last_shot_t: f64, shots: u32, parked: Vec<usize>,
    compacting: bool,
}

impl Phys1114 {
    /// js `new Physical1114(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = Drives::default();
        let t = build1114(&mut d);
        let w = wheels1114(&t, &d);
        let surfaces = path_surfaces1114(&t);
        let mut path_wheels = w.floor;
        path_wheels.extend(w.squish);
        path_wheels.extend(w.side);
        path_wheels.extend(w.passive);
        path_wheels.extend(w.elev);
        path_wheels.extend(w.wall);
        let n_intake = path_wheels.len();
        path_wheels.extend(w.intake);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.push(t.slide);
        let path = Mechanism::new(Some(body), shafts, path_wheels, vec![S_BODY, S_SLIDE, S_PANEL, S_KICKER, S_HOOD, S_BLANKET, S_BUMPER], 6, 1);
        let mut sw = vec![w.fly];
        sw.extend(w.hood);
        let shooter = Mechanism::new(Some(body), vec![t.fly, t.hood], sw, vec![S_HOOD], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), PRELOAD.to_vec(), Some((0.3, 6)));
        let mut me = Phys1114 {
            p, t, intake_home: w.intake_home, hood_home: w.hood_home, n_intake, ext: 0.0, hood_deg: 0.0, hood_goal: None, aim: false, aim_pass: false,
            intake_t: -1.0, lowered: false, pushed_t: -1.0, deploying: false, ballev_t: f64::NAN, shoot_t: -1.0, slide_mode: SlideMode::Volts(0.0),
            want: None, fire_t: 0.0, last_shot_t: 0.0, shots: 0, parked: Vec::new(), compacting: false,
        };
        me.reset();
        me
    }

    /// js `reset()`: the box all the way in, the hood on its stop
    pub fn reset(&mut self) {
        self.p.reset();
        self.ext = 0.0;
        self.hood_deg = shooter::HOOD_HOME; self.hood_goal = None;
        self.aim = false; self.aim_pass = false;
        self.intake_t = -1.0; self.lowered = false; self.pushed_t = -1.0; self.deploying = false;
        self.shoot_t = -1.0;
        let t = &self.t;
        let d = &mut self.p.drives;
        for tr in [t.intake, t.floor, t.side_t, t.elev, t.shooter] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = None; d.trains[tr].vel = None; }
        d.shafts[t.slide].omega_ = 0.0; d.shafts[t.slide].cmd = None;
        self.slide_mode = SlideMode::Volts(0.0);
        self.want = None;
        self.fire_t = 0.0; self.last_shot_t = 0.0; self.shots = 0; self.parked.clear();
    }

    /// js `where(x, y, z)` (ext: the box's, as it is now)
    pub fn where_(ext: f64, [x, y, z]: [f64; 3]) -> Where {
        let in_frame = m::abs(x) < HALF_L && m::abs(z) < HALF_W && y > 0.105 && y < 0.8;
        let front = HALF_L + ext + 0.02;
        let in_front = x >= HALF_L && x < front && m::abs(z) < 0.375 && y < 0.75;
        Where { own: in_frame || in_front, held: in_frame || (in_front && y > 0.2), near: x > -HALF_L - 0.1 && x < front + 0.1 && m::abs(z) < HALF_W + 0.1 && y < 0.9 }
    }
    /// js `shooterBalls(near)`'s test
    pub fn shooter_ball([x, y, _z]: [f64; 3]) -> bool { y > 0.47 && x < 0.08 }

    /// js `_slideStep(dt, on)`
    fn slide_step(&mut self, dt: f64, on: bool) {
        let s = self.t.slide;
        let d = &mut self.p.drives;
        slide_stops(d, s, self.ext);
        let cmd = if !on { None } else {
            match self.slide_mode {
                SlideMode::Volts(v) => if v == 0.0 { None } else { Some(v / 12.0) },
                SlideMode::Pos(turns) => Some(clamp((4.0 * (turns - slide_turns(self.ext))) / 12.0, -1.0, 1.0)),
                SlideMode::Amps(a) => { let w = d.shafts[s].omega_; Some(duty_for(d.shafts[s].motor.as_ref().unwrap(), a, w)) }
            }
        };
        d.shafts[s].cmd = cmd;
        d.impulse(s, slide::MASS * 9.81 * slide::DIR[1] * slide::PINION * dt);
    }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, dt: f64, r: &PreIn) {
        let on = r.enabled;
        self.slide_step(dt, on);
        if let (true, Some(g)) = (on, self.hood_goal) {
            let v = clamp(hood_kp() * (g - self.hood_deg), -12.0, 12.0);
            self.hood_deg = clamp(self.hood_deg + (v / 12.0) * hood_free() * dt, shooter::HOOD_MIN - 2.0, shooter::HOOD_HOME);
        }
        place_slide(&mut self.p.surfaces, self.ext);
        let n = self.n_intake;
        place_intake(&mut self.p.path.wheels[n..n + 1], &self.intake_home, self.ext);
        self.p.surfaces[S_HOOD].set_pose([FLY[0], FLY[1], 0.0], 0.0, hood_rot(self.hood_deg));
        place_hood(&mut self.p.shooter.as_mut().unwrap().wheels[1..], &self.hood_home, self.hood_deg);
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
        self.ext += d.omega(s) * slide::PINION * dt;
        if self.ext < 0.0 { self.ext = 0.0; if d.shafts[s].omega_ < 0.0 { d.shafts[s].omega_ = 0.0; } }
        if self.ext > slide::OUT { self.ext = slide::OUT; if d.shafts[s].omega_ > 0.0 { d.shafts[s].omega_ = 0.0; } }
        clamp(self.ext / slide::OUT, 0.0, 1.0)
    }

    /// js `_range()`: their hood CANrange sees a FUEL near the top of the elevator
    fn range(&mut self, c: &Ctx) -> bool {
        let [x0, y0] = range::AT;
        let held = self.p.held.clone();
        held.iter().any(|&b| { let l = self.p.local(c.phys, c.body, c.fuel.balls[b].pos); m::abs(l[0] - x0) < range::HALF && l[1] > y0 })
    }
    /// js `passTarget()`: 2 m out from the ALLIANCE WALL, 1.5 m in from the side wall on its side
    pub fn pass_target(a: Alliance, pos: [f64; 3]) -> [f64; 2] {
        let (ax, az) = fw(2.0, 1.5, a);
        let (bx, bz) = fw(2.0, FIELD_W - 1.5, a);
        if m::abs(az - pos[2]) <= m::abs(bz - pos[2]) { [ax, az] } else { [bx, bz] }
    }

    /// js `postStep(dt, t, { on, feeding, outtake, setpoint, mode })`; returns the robot's stored
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut) -> Vec<usize> {
        let mut stored = self.p.account(c, t, i.pass, i.legal, stats);
        let (on, feeding, pass) = (i.on, i.feeding, i.pass);
        let turns = slide_turns(self.ext);
        let intaking = on && i.intake;
        let eject = on && i.outtake;
        let has = !stored.is_empty();
        let lineup = !i.auto && on && has && i.lineup && !feeding;
        if !i.auto {
            self.aim = on && (lineup || (i.setpoint > 0.0 && i.has_shot && (feeding || !intaking)));
            self.aim_pass = pass && !lineup;
        } else {
            if on && i.setpoint > 0.0 && i.has_shot { self.aim = true; self.aim_pass = pass && i.spin != Some(true); }
            if (intaking && i.spin != Some(true) && !feeding) || !on || (i.spin == Some(false) && !feeding) { self.aim = false; }
        }
        let mut want = None;
        if self.aim {
            let tgt = if self.aim_pass { i.aim_pass } else { i.aim_hub };
            let d = m::hypot2(tgt[0] - i.pos[0], tgt[1] - i.pos[2]);
            let w = shot_for(self.aim_pass, if lineup { clamp(d, LINEUP_NEAR, LINEUP_FAR) } else { d });
            self.hood_goal = Some(w.1);
            want = Some(w);
        } else { self.hood_goal = None; }
        let tr = &self.t;
        let d = &mut self.p.drives;
        let lp = match want { Some((r, _)) => r, None => if on { shooter::IDLE } else { 0.0 } };
        if lp != 0.0 { d.trains[tr.shooter].vel = Some(VelLoop { target: (lp / 60.0) * TAU, kv: shooter::KV, kp: shooter::KP, ks: 0.0, lo: -12.0, hi: 12.0, per: Some(1.0 / TAU), rate: 1000.0 }); }
        else { d.trains[tr.shooter].vel = None; d.trains[tr.shooter].cmd = None; }
        let rpm = (d.trains[tr.shooter].omega * 60.0) / TAU;
        let sp = want.map_or(0.0, |w| w.0);
        let err = m::abs(i.aim_err.unwrap_or(f64::INFINITY));
        let shooting = on && feeding && want.is_some();
        if shooting && self.shoot_t < 0.0 { self.shoot_t = t; }
        if !shooting { self.shoot_t = -1.0; }
        let fire = shooting && sp >= 1.0 && rpm >= sp - GATE_RPM && err <= gate_aim();
        let agitate = shooting && sp >= 1.0 && rpm >= sp - GATE_AGITATE_RPM && err <= gate_agitate_aim();
        if intaking && self.intake_t < 0.0 { self.intake_t = t; self.lowered = false; self.pushed_t = -1.0; self.deploying = false; self.ballev_t = -1.0; }
        if !intaking { self.intake_t = -1.0; }
        let (mut roller, mut floor_v) = (0.0, 0.0);
        self.slide_mode = SlideMode::Volts(0.0);
        if intaking {
            if !self.lowered {
                self.slide_mode = SlideMode::Pos(slide_pos::DROP);
                self.lowered = turns > slide_pos::OUT_CHECK && m::abs(turns - slide_pos::DROP) <= slide_pos::tol();
                if self.lowered { self.ballev_t = t; }
            }
            if self.lowered {
                roller = 12.0;
                if turns < slide_pos::REDEPLOY || self.deploying {
                    if self.pushed_t < 0.0 { self.pushed_t = t; }
                    else if t - self.pushed_t > 0.5 {
                        self.slide_mode = SlideMode::Amps(20.0);
                        self.deploying = true;
                        if turns > slide_pos::OUT { self.deploying = false; }
                    }
                } else { self.pushed_t = -1.0; }
                if t - self.ballev_t > 3.0 && !self.range(c) { floor_v = 3.0; }
            }
        } else if shooting {
            if agitate && turns >= 1.0 && t - self.shoot_t > GATE_AGITATE_WAIT && turns > slide_pos::AGITATE_MIN { self.slide_mode = SlideMode::Volts(slide::PULL); roller = 6.0; }
            if agitate && t - self.shoot_t > GATE_AGITATE_WAIT { roller = 6.0; self.compacting = true; }
        }
        if eject { roller = -12.0; }
        if stats.shots != self.shots { self.shots = stats.shots; self.last_shot_t = t; }
        self.fire_t = if fire { self.fire_t + dt } else if shooting { self.fire_t } else { 0.0 };
        if fire && self.fire_t > 1.5 && t - self.last_shot_t > 1.0 { for &b in &self.p.held { if !self.parked.contains(&b) { self.parked.push(b); } } }
        if intaking && !shooting { self.parked.clear(); }
        if !self.parked.is_empty() { stored.retain(|b| !self.parked.contains(b)); }
        if roller > 0.1 && turns < slide_pos::ROLLERS_OFF && !self.compacting { roller = 0.0; }
        self.compacting = false;
        if !on { roller = 0.0; }
        let tr = &self.t;
        let d = &mut self.p.drives;
        d.trains[tr.intake].cmd = if roller == 0.0 { None } else { Some(roller / 12.0) };
        if fire {
            d.trains[tr.floor].cmd = Some(1.0); d.trains[tr.side_t].cmd = Some(10.0 / 12.0);
            d.trains[tr.elev].vel = Some(VelLoop { target: (ballevator::SHOOT / 60.0) * TAU, kv: ballevator::KV, kp: ballevator::KP, ks: 0.0, lo: -12.0, hi: 12.0, per: Some(1.0 / TAU), rate: 1000.0 });
        } else {
            d.trains[tr.floor].cmd = if floor_v != 0.0 { Some(floor_v / 12.0) } else { None }; d.trains[tr.side_t].cmd = None;
            d.trains[tr.elev].vel = None; d.trains[tr.elev].cmd = None;
        }
        if !on { d.trains[tr.floor].cmd = None; d.trains[tr.side_t].cmd = None; d.trains[tr.elev].vel = None; d.trains[tr.elev].cmd = None; d.trains[tr.shooter].vel = None; d.trains[tr.shooter].cmd = None; }
        self.want = want;
        *out = PostOut {
            hood_deg: hood_elevation(self.hood_deg),
            flywheel: self.exit_speed(),
            ready: want.is_some() && rpm >= sp - GATE_RPM && err <= gate_aim(),
            feeding: if fire { 1.0 } else { 0.0 },
            intake_speed: if roller > 6.0 { 1.0 } else if roller < 0.0 { -1.0 } else { 0.0 },
        };
        stored
    }

    /// js `extNow()`: how far out the box is (m along its racks)
    pub fn ext_now(&self) -> f64 { self.ext }
    /// js `hoodTurn()`, `kickerTurn()`, `floorSpin()`
    pub fn hood_turn(&self) -> f64 { hood_rot(self.hood_deg) }
    pub fn kicker_turn(&self) -> f64 { kicker_rot(self.ext) }
    pub fn floor_spin(&self) -> f64 { self.p.drives.trains[self.t.floor].omega * 0.5 }
    /// js `exitSpeed()`
    pub fn exit_speed(&self) -> f64 { EXIT_PER_FLY_SURFACE * m::abs(self.p.drives.omega(self.t.fly)) * shooter::FLY_R }
    /// js `capacity()`
    pub fn capacity(&self) -> usize { 82 }
}
