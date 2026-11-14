//! js/mech/phys1706.js: 1706's Mirage in play, built of its physical mechanisms (r1706.rs). Their
//! 2026 code isn't public: its controls are what their CAD and thread say the robot does, run the
//! way their 2025 code runs its mechanisms (Motion Magic on the arm, velocity loops on the rollers).
//! The arm out while it intakes, folding in to feed the bowls while it shoots; each turret aims
//! itself (the robot's twin-turret model turns it to the heading this gives) and its spindexer and
//! kicker feed once it's on the shot. (The pump and the rock are off by default in the browser
//! game, and so not here.)
use crate::ballistics::{solve_moving_shot, ShotTable};
use crate::constants::*;
use crate::mech::drives::{Drives, VelLoop};
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, Physical, Stats, Where};
use crate::mech::r1706::*;
use crate::mech::r1706::{HALF_L, HALF_W};
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const TAU: f64 = 2.0 * m::PI;
/// js SPIN_RPS, KICK_RPM: the spindexers' and kickers' setpoints while a turret is ready
const SPIN_RPS: f64 = 12.0;
const KICK_RPM: f64 = 3000.0;
/// js EXIT, EL_MAP, EXIT_MAP
const EXIT_X: f64 = 0.06;
const EXIT_Y: f64 = 0.62;
const EL_MAP: [[f64; 2]; 13] = [[0.0, 84.5], [4.3, 76.3], [7.7, 72.2], [10.6, 69.0], [12.1, 67.0], [14.6, 65.5], [16.6, 63.1], [17.5, 61.7], [18.9, 61.0], [23.8, 55.7], [30.0, 48.4], [40.0, 40.5], [45.0, 37.5]];
const EXIT_MAP: [[f64; 2]; 13] = [[0.0, 0.0], [4.7, 6.15], [5.6, 7.3], [6.07, 8.0], [6.43, 8.56], [6.84, 9.16], [7.19, 9.66], [7.53, 9.98], [7.95, 10.46], [8.52, 12.11], [8.8, 13.3], [9.1, 15.5], [9.5, 21.9]];
fn interp(t: &[[f64; 2]], x: f64, a: usize, b: usize) -> f64 {
    if x <= t[0][a] { return t[0][b]; }
    for k in 1..t.len() {
        if x <= t[k][a] { return t[k - 1][b] + ((t[k][b] - t[k - 1][b]) * (x - t[k - 1][a])) / (t[k][a] - t[k - 1][a]); }
    }
    t[t.len() - 1][b]
}
/// js util.js `clamp(v, lo, hi)`
fn clamp(v: f64, lo: f64, hi: f64) -> f64 { if v < lo { lo } else if v > hi { hi } else { v } }
/// js `hoodFor(el)`: the hood swing (deg) for an elevation (EL_MAP reversed: in order of elevation)
pub fn hood_for(el: f64) -> f64 {
    let mut r = EL_MAP;
    r.reverse();
    clamp(interp(&r, el, 1, 0), 0.0, shooter::HOOD_MAX)
}
/// js `elevationAt(h)`
pub fn elevation_at(h: f64) -> f64 { interp(&EL_MAP, h, 0, 1) }
/// js `accelFor(v)`: the accelerators' speed (rad/s) for an exit speed; `exitAt(w)`, back
pub fn accel_for(v: f64) -> f64 { interp(&EXIT_MAP, v, 0, 1) / shooter::ACCEL_R }
pub fn exit_at(w: f64) -> f64 { interp(&EXIT_MAP, m::abs(w) * shooter::ACCEL_R, 1, 0) }

/// js `profile(p, v, goal, vmax, amax, dt)`
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
/// js `speedLoop(a, target, kP)` (kV: the motor's own, 12 V over its free speed)
fn speed_loop(d: &mut Drives, a: Drv, target: Option<f64>, kp: f64) {
    let free = match a { Drv::S(s) => d.shafts[s].motor.as_ref().unwrap().m.free, Drv::T(t) => d.trains[t].motor.m.free };
    let kv = (12.0 * 2.0 * m::PI) / free;
    let vel = target.map(|target| VelLoop { target, kv, kp, ks: 0.0, lo: -12.0, hi: 12.0, per: None, rate: 1000.0 });
    let cmd = target.map(|_| 0.0);
    match a { Drv::S(s) => { d.shafts[s].vel = vel; d.shafts[s].cmd = cmd; } Drv::T(t) => { d.trains[t].vel = vel; d.trains[t].cmd = cmd; } }
}

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool, pub intake_deploy: f64, pub turret_yaws: [f64; 2] }
/// What its postStep reads of the robot
pub struct PostIn<'a> {
    pub on: bool, pub feeding: bool, pub outtake: bool, pub pass: bool, pub legal: bool, pub intake: bool,
    /// its shot's target (None: no shot), and where it aims at the HUB
    pub target: Option<[f64; 2]>, pub aim_hub: [f64; 2],
    pub table: &'a ShotTable, pub est_vel: [f64; 2], pub est_omega: f64, pub yaw: f64, pub turret_yaws: [f64; 2],
}
/// What its postStep says back: the robot's (as phys2910's), and each turret's own (js r.tws[i]:
/// fly, hood, ok)
pub use crate::mech::phys2910::PostOut;
pub type Tws = [(f64, f64, bool); 2];

pub struct Phys1706 {
    pub p: Physical,
    pub t: Trains,
    intake_home: Vec<[f64; 3]>,
    pub arm_deg: f64,
    arm_set: [f64; 2],
    arm_goal: f64,
    pub hood: [f64; 2],
    hood_set: [[f64; 2]; 2],
    hood_goal: [f64; 2],
    pub phi: [f64; 2],
    /// js aimPsis: where each turret wants to point (None: no shot)
    pub aim_psis: [Option<f64>; 2],
    ready_t: [bool; 2],
    pub ext: f64,
    last_deploy: Option<f64>,
}

impl Phys1706 {
    /// js `new Physical1706(robot)`
    pub fn new(body: RigidBodyHandle) -> Self {
        let mut d = Drives::default();
        let t = build1706(&mut d);
        let w = wheels1706(&t);
        let surfaces = path_surfaces1706(&t);
        let sw = shooter_wheels(&t);
        let mut path_wheels = w.intake;
        path_wheels.extend(w.kicker);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &path_wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.extend([t.pivot, t.spin[0], t.spin[1]]);
        let path_surfs = vec![S_BODY, S_BOWL, S_INTAKE, S_HOPPER, S_SPIN[0], S_SPIN[1], S_TURRET[0], S_TURRET[1], S_HOOD[0], S_HOOD[1], S_BUMPER, S_NET];
        let path = Mechanism::new(Some(body), shafts, path_wheels, path_surfs, 6, 1);
        let sw: Vec<_> = sw.into_iter().flatten().collect();
        let sh_shafts = sw.iter().map(|x| x.shaft).collect();
        let shooter = Mechanism::new(Some(body), sh_shafts, sw, vec![S_TURRET[0], S_TURRET[1], S_HOOD[0], S_HOOD[1]], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), preload(), Some((0.12, 10)));
        let mut me = Phys1706 {
            p, t, intake_home: w.intake_home, arm_deg: 0.0, arm_set: [0.0; 2], arm_goal: 0.0, hood: [0.0; 2], hood_set: [[0.0; 2]; 2], hood_goal: [0.0; 2],
            phi: [0.0; 2], aim_psis: [None; 2], ready_t: [false; 2], ext: 0.0, last_deploy: None,
        };
        me.reset();
        me
    }

    /// js `reset()` (and the robot's turretYaws: [90, -90] deg, robot.rs)
    pub fn reset(&mut self) {
        self.p.reset();
        self.arm_deg = 0.0; self.arm_set = [0.0, 0.0]; self.arm_goal = 0.0;
        self.hood = [0.0; 2]; self.hood_set = [[0.0; 2]; 2]; self.hood_goal = [0.0; 2];
        self.phi = [0.0; 2];
        self.aim_psis = [None; 2]; self.ready_t = [false; 2];
        let t = &self.t;
        let d = &mut self.p.drives;
        for tr in [t.intake, t.kicker[0].train, t.kicker[1].train, t.shooter[0].train, t.shooter[1].train] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = None; d.trains[tr].vel = None; }
        for sh in [t.pivot, t.spin[0], t.spin[1]] { d.shafts[sh].omega_ = 0.0; d.shafts[sh].cmd = Some(0.0); d.shafts[sh].vel = None; }
        self.ext = 0.0;
    }

    /// js `where(x, y, z)` (ext, arm: the box's and the arm's, as they are now)
    pub fn where_(ext: f64, arm: f64, [x, y, z]: [f64; 3]) -> Where {
        let in_frame = m::abs(x) < HALF_L && m::abs(z) < HALF_W && y > 0.09 && y < 0.8;
        let in_box = x >= HALF_L && x < HALF_L + ext + 0.01 && m::abs(z) < BOX_SIDE && y > 0.16 && y < 0.78;
        let on_arm = arm > 60.0 && x >= HALF_L && x < 0.66 && m::abs(z) < 0.37 && y > BUMPER_Y0 && y < 0.45;
        Where {
            own: in_frame || in_box || on_arm,
            held: in_frame || in_box || (on_arm && y > 0.24 && x < 0.56),
            near: x > -HALF_L - 0.1 && x < 0.75 && m::abs(z) < HALF_W + 0.1 && y < 0.9,
        }
    }
    /// js `shooterBalls(near)`'s test: up in a turret
    pub fn shooter_ball([x, y, z]: [f64; 3]) -> bool { y > 0.36 && TURRET.iter().any(|[tx, tz]| m::hypot2(x - tx, z - tz) < 0.17) }

    /// js `servoArm(T, deg, set, goal, dt)`: the arm's Motion Magic toward its goal, and its weight
    fn servo_arm(&mut self, dt: f64) {
        let pv = self.t.pivot;
        let d = &mut self.p.drives;
        let ratio = d.shafts[pv].motor.as_ref().unwrap().ratio;
        let rot = |rad: f64| (rad * ratio) / TAU;
        self.arm_set = profile(self.arm_set[0], self.arm_set[1], self.arm_goal * DEG, intake::MM_V, intake::MM_A, dt);
        d.shafts[pv].cmd = Some(clamp((intake::KP * (rot(self.arm_set[0]) - rot(self.arm_deg * DEG)) + intake::KV * rot(self.arm_set[1])) / 12.0, -1.0, 1.0));
        let a = arm_rot(self.arm_deg);
        let cx = intake::COM[0] * m::cos(a) - intake::COM[1] * m::sin(a);
        d.impulse(pv, intake::MASS * 9.81 * cx * dt);
    }
    /// js `turnArm(T, deg, dt)`
    fn turn_arm(&mut self, dt: f64) {
        let pv = self.t.pivot;
        let d = &mut self.p.drives;
        self.arm_deg += (d.omega(pv) * dt) / DEG;
        if self.arm_deg < 0.0 { self.arm_deg = 0.0; if d.shafts[pv].omega_ < 0.0 { d.shafts[pv].omega_ = 0.0; } }
        if self.arm_deg > intake::MAX { self.arm_deg = intake::MAX; if d.shafts[pv].omega_ > 0.0 { d.shafts[pv].omega_ = 0.0; } }
    }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, dt: f64, r: &PreIn) {
        if let Some(ld) = self.last_deploy {
            if m::abs(r.intake_deploy - ld) > 1e-6 {
                self.arm_deg = intake::OUT * r.intake_deploy; self.arm_set = [self.arm_deg * DEG, 0.0]; self.p.drives.shafts[self.t.pivot].omega_ = 0.0;
            }
        }
        if r.enabled { self.servo_arm(dt); }
        else { let pv = self.t.pivot; self.p.drives.shafts[pv].cmd = Some(0.0); self.p.drives.shafts[pv].omega_ = 0.0; self.arm_set = [self.arm_deg * DEG, 0.0]; }
        for i in 0..2 {
            self.hood_set[i] = profile(self.hood_set[i][0], self.hood_set[i][1], self.hood_goal[i], shooter::HOOD_V, shooter::HOOD_A, dt);
            self.hood[i] = self.hood_set[i][0];
        }
        self.place(r.turret_yaws);
    }
    /// js `_place(yaws)`
    fn place(&mut self, yaws: [f64; 2]) {
        self.ext = box_ext(&path().ext, self.arm_deg);
        let sf = &mut self.p.surfaces;
        sf[S_INTAKE].set_pose([PIVOT[0], PIVOT[1], 0.0], 0.0, arm_rot(self.arm_deg));
        let n = self.intake_home.len();
        place_intake(&mut self.p.path.wheels[..n], &self.intake_home, self.arm_deg);
        place_box(sf, &mut self.p.drives, self.t.cam, self.arm_deg);
        sf[S_NET].slack_mut().unwrap().ext = self.ext;
        let sw = &mut self.p.shooter.as_mut().unwrap().wheels;
        for i in 0..2 {
            place_spin(sf, i, self.phi[i]);
            place_turret(sf, i, yaws[i], self.hood[i]);
            place_shooter(i, &mut sw[3 * i..3 * i + 3], yaws[i], self.hood[i]);
        }
    }
    /// js `PhysicalRobot.stepMechanisms` (at its yield)
    pub fn mechanisms(&mut self, c: &mut Ctx, dt: f64, t: f64) {
        let (ext, arm) = (self.ext, self.arm_deg);
        self.p.mech_prepare(c, t, &|l| Self::where_(ext, arm, l), &Self::shooter_ball);
        self.p.mech_run(dt, &mut c.phys.bodies);
        self.p.mech_finish(c.fuel);
    }
    /// js `preSteps` after its yield; returns the robot's intakeDeploy
    pub fn post_mech(&mut self, dt: f64) -> f64 {
        self.turn_arm(dt);
        for i in 0..2 { self.phi[i] = (self.phi[i] + self.p.drives.omega(self.t.spin[i]) * dt) % TAU; }
        let d = clamp(self.arm_deg / intake::OUT, 0.0, 1.0);
        self.last_deploy = Some(d);
        d
    }

    /// js `_aim(i, tgt, table)`: turret i's shot, from where its FUEL will leave 0.05 s ahead:
    /// (psi, el, v, d)
    fn aim(&self, c: &Ctx, ti: usize, tgt: [f64; 2], i: &PostIn) -> Option<(f64, f64, f64, f64)> {
        let b = &c.phys.bodies[c.body];
        let (v, w, l) = (i.est_vel, i.est_omega, 0.05);
        let ct = b.translation();
        let yaw = i.yaw + w * l;
        let (cy, sy) = (m::cos(yaw), m::sin(yaw));
        let [tx, tz] = TURRET[ti];
        let (ox, oz) = (tx * cy + tz * sy, -tx * sy + tz * cy);
        let (px, pz) = (ct.x as f64 + v[0] * l + ox, ct.z as f64 + v[1] * l + oz);
        let (vx, vz) = (v[0] + w * oz, v[1] - w * ox);
        let (dx, dz) = (tgt[0] - px, tgt[1] - pz);
        let dd = { let h = m::hypot2(dx, dz); if h == 0.0 { 1.0 } else { h } };
        let exit = [px + (dx / dd) * EXIT_X, EXIT_Y, pz + (dz / dd) * EXIT_X];
        solve_moving_shot(i.table, exit, [vx, vz], tgt).map(|s| (s.psi, s.theta / DEG, s.v, s.dist))
    }

    /// js `postStep(dt, t, { on, feeding, outtake, mode })`; returns the robot's stored, and each
    /// turret's own (tws)
    pub fn post_step(&mut self, c: &mut Ctx, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut) -> (Vec<usize>, Tws) {
        let stored = self.p.account(c, t, i.pass, i.legal, stats);
        let on = i.on;
        // ---- the intake
        let intaking = on && i.intake;
        let unjam = on && i.outtake;
        let shooting = on && i.feeding && i.target.is_some() && !unjam;
        self.arm_goal = if intaking || unjam { intake::OUT } else { 0.0 };
        let arm_ready = m::abs(self.arm_deg - intake::OUT) < intake::TOL;
        let sweep = shooting && !intaking && self.arm_deg > 30.0;
        let tt = self.t.intake;
        self.p.drives.trains[tt].cmd = if !on { Some(0.0) } else if unjam { Some(-0.5) } else if (intaking && arm_ready) || sweep { Some(1.0) } else { None };
        // ---- the turrets: aiming and spinning up while the robot has a shot
        let pass = i.pass && i.target.is_some();
        let point = i.target.map(|tg| if pass { tg } else { i.aim_hub });
        let mut any_ready = false;
        let mut tws: Tws = [(0.0, 0.0, false); 2];
        for k in 0..2 {
            let sol = if on && !unjam { point.and_then(|p| self.aim(c, k, p, i)) } else { None };
            self.aim_psis[k] = sol.map(|s| s.0);
            let sh = self.t.shooter[k];
            let d = &mut self.p.drives;
            if let Some(s) = sol {
                self.hood_goal[k] = hood_for(s.1);
                speed_loop(d, Drv::T(sh.train), Some(accel_for(s.2) / shooter::K_ACCEL), shooter::KP);
            } else {
                self.hood_goal[k] = 0.0;
                speed_loop(d, Drv::T(sh.train), None, 0.2);
            }
            let yaw_err = match sol { Some(s) => m::abs(crate::robot::wrap_angle(s.0 - (i.yaw + i.turret_yaws[k]))), None => f64::INFINITY };
            let ready = match sol {
                Some(s) => yaw_err < shooter::TOL_YAW * DEG && m::abs(self.hood[k] - self.hood_goal[k]) < shooter::TOL_HOOD && m::abs(exit_at(d.omega(sh.accel)) - s.2) < shooter::TOL_V,
                None => false,
            };
            self.ready_t[k] = ready;
            // ---- that side's spindexer and kicker: feeding while it's ready and the shot's held
            let feed = on && i.feeding && ready && !unjam;
            let (sp, kt) = (Drv::S(self.t.spin[k]), Drv::T(self.t.kicker[k].train));
            if unjam { speed_loop(d, sp, Some(-SPIN_RPS * TAU), 0.2); speed_loop(d, kt, Some(-(KICK_RPM / 60.0) * TAU), 0.2); }
            else if feed { speed_loop(d, sp, Some(SPIN_RPS * TAU), 0.2); speed_loop(d, kt, Some((KICK_RPM / 60.0) * TAU), 0.2); }
            else { speed_loop(d, sp, None, 0.2); speed_loop(d, kt, None, 0.2); }
            if feed { any_ready = true; }
            tws[k] = (exit_at(d.omega(sh.accel)), elevation_at(self.hood[k]), ready);
        }
        let d = &self.p.drives;
        let ic = d.trains[tt].cmd;
        let k = if self.ready_t[1] && !self.ready_t[0] { 1 } else { 0 };
        *out = PostOut {
            hood_deg: elevation_at(self.hood[k]),
            flywheel: exit_at(d.omega(self.t.shooter[k].accel)),
            ready: self.ready_t.iter().any(|x| *x),
            feeding: if any_ready { 1.0 } else { 0.0 },
            intake_speed: match ic { Some(x) if x > 0.0 => 1.0, Some(x) if x < 0.0 => -1.0, _ => 0.0 },
        };
        (stored, tws)
    }

    /// js `extNow()`: how far the hopper box is out (m)
    pub fn ext_now(&self) -> f64 { self.ext }
    /// js `armTurn()`: the arm's turn from its CAD pose (rad), for drawing it
    pub fn arm_turn(&self) -> f64 { arm_rot(self.arm_deg) }
    /// js `capacity()`
    /// js `loadTop()`: the top of its load
    pub fn load_top(&mut self, phys: &PhysicsWorld, fuel: &crate::fuel::Fuel, body: RigidBodyHandle) -> f64 {
        let mut top: f64 = 0.0;
        for k in 0..self.p.held.len() {
            let [x, y, _] = self.p.local(phys, body, fuel.balls[self.p.held[k]].pos);
            if y + FUEL::radius > top { top = y + FUEL::radius; } let _ = x;
        }
        top
    }
    pub fn capacity(&self) -> usize { 64 }
    /// js `mechPose()`: the spindexers' turns, the hoods' swings (rad), the accelerators' speeds
    pub fn mech_pose(&self) -> [(&'static str, f64); 6] {
        let d = &self.p.drives;
        [("mech.spin.0", self.phi[0] * SPIN_DIR[0]), ("mech.spin.1", self.phi[1] * SPIN_DIR[1]),
         ("mech.hood.0", self.hood[0] * DEG), ("mech.hood.1", self.hood[1] * DEG),
         ("mech.accel.0", d.omega(self.t.shooter[0].accel)), ("mech.accel.1", d.omega(self.t.shooter[1].accel))]
    }
}
