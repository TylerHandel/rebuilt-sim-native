//! js/mech/phys8793.js: 8793 in play, built of its physical mechanisms (r8793.rs). Its controls
//! are its code's (ShooterMode): the rollers and vertical rollers always running, with its
//! automatic jam recovery; the turret indexer while the shot is on; the flywheel on its velocity
//! loop toward the speed that gives the exit speed the aim asks for (SHOT_TABLE); and the intake arm
//! on its pivot's motors under their position loop, down when the game wants it (intakeWant),
//! where the FUEL squeezed under its rollers can lift it. A TRENCH arm in its way pushes it round
//! its pivot, and the robot feels that push (trenchArm, trenchPush).
use crate::constants::*;
use crate::mech::drives::{Drives, VelLoop};
use crate::mech::mechanism::Mechanism;
use crate::mech::physical::{Ctx, Physical, Stats, Where};
use crate::mech::r8793::*;
use crate::mech::turret_drive::{drive8793, TurretDrive};
use crate::robot::apply_quat;
use jsmath as m;
use rapier3d::prelude::*;

const DEG: f64 = m::PI / 180.0;
const TAU: f64 = 2.0 * m::PI;
/// the frame's half length and width (27.5in)
const HALF: f64 = 0.349;
const TURRET_X: f64 = -0.127;
/// js JAM: their jam recovery
const JAM_AMPS: f64 = 55.0;
const JAM_RPS: f64 = 10.0;
const JAM_DEBOUNCE: f64 = 0.125;
const JAM_REVERSE: f64 = 0.125;
/// js HOLD_FULL, EXIT_K (SHOT_SCALE 1, HOLD_SPEED 0, FEED_PUSH 0,0, STOP_UNJAM 'all': as played)
const HOLD_FULL: f64 = 0.5;
const EXIT_K: f64 = 1.035;

/// One of the TRENCH arms' parts as a box (world: center, half sizes), js nav.js TRENCH_ARMS
#[derive(Clone, Copy, Debug)]
pub struct Beam { pub x: f64, pub y: f64, pub z: f64, pub hx: f64, pub hy: f64, pub hz: f64 }
/// js `TRENCH_ARMS`
pub fn trench_arms() -> &'static [Beam] {
    static A: std::sync::OnceLock<Vec<Beam>> = std::sync::OnceLock::new();
    A.get_or_init(|| {
        let mut out = Vec::new();
        let arm_h = TRENCH::armTop - TRENCH::clearHeight;
        for a in [Alliance::Blue, Alliance::Red] {
            let hx = crate::field::hub_center(a).0;
            for sw in [1.0, -1.0] {
                for p in trench_arm_parts(sw == TRENCH::fixedZ) {
                    let g = (p.g0 + p.g1) / 2.0;
                    let s = if a == Alliance::Blue { 1.0 } else { -1.0 };
                    out.push(Beam { x: hx + (s * (p.u0 + p.u1)) / 2.0, y: TRENCH::clearHeight + arm_h / 2.0, z: sw * (HALF_W - g), hx: (p.u1 - p.u0) / 2.0, hy: arm_h / 2.0, hz: (p.g1 - p.g0) / 2.0 });
                }
            }
        }
        out
    })
}
/// js nav.js `underTrench(x, z, margin)`: under a TRENCH arm's opening
pub fn under_trench(x: f64, z: f64, margin: f64) -> bool {
    [Alliance::Blue, Alliance::Red].iter().any(|&a| {
        let hx = crate::field::hub_center(a).0;
        let z_open = HALF_W - TRENCH::clearWidth;
        [1.0, -1.0].iter().any(|sz| {
            let (oz, ohx, ohz) = ((sz * (z_open + HALF_W)) / 2.0, TRENCH::depth / 2.0, (HALF_W - z_open) / 2.0);
            m::abs(x - hx) < ohx + margin && m::abs(z - oz) < ohz + margin
        })
    })
}

/// js `armSamples(hull, width)`: its side outline every 2 cm and rings of it two thirds and a third
/// of the way in to its middle; at five places across its width
fn arm_samples(hull: &[[f64; 2]], width: f64) -> (Vec<[f64; 2]>, [f64; 5]) {
    let mut ring = Vec::new();
    for (i, a) in hull.iter().enumerate() {
        let b = hull[(i + 1) % hull.len()];
        let n = m::max(1.0, m::ceil(m::hypot2(b[0] - a[0], b[1] - a[1]) / 0.02));
        let mut k = 0.0;
        while k < n { ring.push([a[0] + ((b[0] - a[0]) * k) / n, a[1] + ((b[1] - a[1]) * k) / n]); k += 1.0; }
    }
    let cx = hull.iter().fold(0.0, |s, p| s + p[0]) / hull.len() as f64;
    let cy = hull.iter().fold(0.0, |s, p| s + p[1]) / hull.len() as f64;
    let mut pts = ring.clone();
    for f in [2.0 / 3.0, 1.0 / 3.0] { for &[x, y] in &ring { pts.push([cx + (x - cx) * f, cy + (y - cy) * f]); } }
    (pts, [-0.5 * width, -0.25 * width, 0.0 * width, 0.25 * width, 0.5 * width])
}

/// A TRENCH arm's push on the intake arm (js trenchArm's hit)
#[derive(Clone, Copy, Debug)]
pub struct Hit { depth: f64, point: [f64; 3], n: [f64; 3], lever: f64, torque: f64 }

/// What its preStep reads of the robot
pub struct PreIn { pub enabled: bool, pub intake_deploy: f64, pub intake_want: bool, pub turret_yaw: f64, pub hood_deg: f64, pub pos: [f64; 3], pub quat: [f64; 4] }
/// What its postStep reads of the robot
pub struct PostIn { pub on: bool, pub feeding: bool, pub outtake: bool, pub setpoint: f64, pub pass: bool, pub legal: bool, pub aim_err: Option<f64>, pub pass_clear: Option<bool> }
/// What it tells the robot after
#[derive(Clone, Copy, Debug, Default)]
pub struct PostOut8793 { pub flywheel: f64, pub intake_speed: f64, pub feeding: f64 }

pub struct Phys8793 {
    pub p: Physical,
    pub t: Trains,
    /// where the arm's wheels start in the path's list (they're last)
    n_arm: usize,
    jam_t: f64, unjam_t: f64, full_t: f64, ix_back_t: f64,
    pub target_rps: f64,
    hold_t: Option<f64>,
    pub feed: bool,
    pub arm: ArmState,
    pub deploy: f64,
    /// js counts of its own: holds, stopUnjams
    pub holds: u32,
    pub stop_unjams: u32,
    // (the TRENCH arms against the intake arm)
    hull: Vec<[f64; 2]>,
    fold_pivot: [f64; 2],
    width: f64,
    arm_pts: Option<(Vec<[f64; 2]>, [f64; 5])>,
    trench_hard: Option<bool>,
    pub trench_f: f64,
    prev_pose: Option<([f64; 3], [f64; 4], f64)>,
    last_dir: Option<f64>,
    contact: Option<Hit>,
    w0: f64,
    pub arm_collider: ColliderHandle,
    pub arm_groups: u32,
    pub arm_groups_rigid: u32,
    turret: Option<TurretDrive>,
}

impl Phys8793 {
    /// js `new Physical8793(robot)`: fold, the robot's intake.fold (its hull and pivot); width, intake.width
    pub fn new(body: RigidBodyHandle, hull: Vec<[f64; 2]>, fold_pivot: [f64; 2], width: f64, stow_deg: f64) -> Self {
        let mut d = Drives::default();
        let t = build8793(&mut d);
        let surfaces = path_surfaces8793(&t);
        let arm = arm_wheels(&t);
        let mut wheels = conveyor_wheels(&t);
        wheels.extend(indexer_wheels(&t));
        let n_arm = wheels.len();
        wheels.extend(arm);
        let sw = shooter_wheels(&t, 60.0);
        let mut shafts: Vec<usize> = Vec::new();
        for x in &wheels { if !shafts.contains(&x.shaft) { shafts.push(x.shaft); } }
        shafts.push(t.pivot);
        let n_surf = surfaces.len();
        let mut path = Mechanism::new(Some(body), shafts, wheels, (0..n_surf).collect(), 6, 1);
        path.fast = Some(PATH_FAST);
        let shooter = Mechanism::new(Some(body), vec![t.large, t.small], sw, vec![], 6, 8);
        let p = Physical::new(d, surfaces, path, Some(shooter), PRELOAD.to_vec(), None);
        let mut me = Phys8793 {
            p, t, n_arm, jam_t: 0.0, unjam_t: 0.0, full_t: 0.0, ix_back_t: 0.0, target_rps: 0.0, hold_t: None, feed: false,
            arm: arm_state(stow_deg * DEG), deploy: 0.0, holds: 0, stop_unjams: 0,
            hull, fold_pivot, width, arm_pts: None, trench_hard: None, trench_f: 0.0, prev_pose: None, last_dir: None, contact: None, w0: 0.0,
            arm_collider: ColliderHandle::invalid(), arm_groups: 0, arm_groups_rigid: 0, turret: None,
        };
        me.reset();
        me
    }

    /// js `reset()`
    pub fn reset(&mut self) {
        self.p.reset();
        self.jam_t = 0.0; self.unjam_t = 0.0; self.full_t = 0.0; self.ix_back_t = 0.0;
        self.target_rps = 0.0;
        self.feed = false;
        let d = &mut self.p.drives;
        for tr in [self.t.horizontal, self.t.indexer, self.t.shooter] { d.trains[tr].omega = 0.0; d.trains[tr].cmd = Some(0.0); d.trains[tr].vel = None; }
        self.arm = arm_state(self.arm.stow);
        d.shafts[self.t.pivot].omega_ = 0.0; d.shafts[self.t.pivot].cmd = Some(0.0);
        self.deploy = 0.0;
    }

    /// js `where(x, y, z)` (deployed: the robot's intakeDeploy over a half)
    pub fn where_(deployed: bool, [x, y, z]: [f64; 3]) -> Where {
        let inside = m::abs(x) < HALF && m::abs(z) < HALF && y > 0.105 && y < 0.7;
        let on_arm = deployed && x >= HALF && x < HALF + 0.33 && m::abs(z) < ARM_HALF_WIDTH && y < 0.45;
        Where {
            own: inside || on_arm,
            held: inside || (on_arm && y > 0.17),
            near: x > -HALF - 0.05 && x < HALF + 0.42 && m::abs(z) < HALF + 0.08 && y < 0.75,
        }
    }
    /// js `shooterBalls(near)`'s test
    pub fn shooter_ball(l: [f64; 3]) -> bool { l[1] > 0.3 }

    /// js `preSteps` up to its yield
    pub fn pre_mech(&mut self, phys: &mut PhysicsWorld, dt: f64, r: &PreIn) {
        let piv = self.t.pivot;
        if m::abs(r.intake_deploy - self.deploy) > 1e-6 {
            self.arm.roll = (1.0 - r.intake_deploy) * self.arm.stow; self.arm.set = None; self.p.drives.shafts[piv].omega_ = 0.0;
        }
        servo_arm(&mut self.p.drives, piv, &mut self.arm, r.intake_want, dt, r.enabled);
        self.contact = self.trench_arm(phys, dt, r);
        self.w0 = self.p.drives.omega(piv);
        let roll = self.arm.roll;
        self.p.surfaces[S_ARM].set_pose([ARM_PIVOT[0], ARM_PIVOT[1], 0.0], 0.0, roll);
        let na = self.n_arm;
        place_arm(&mut self.p.path.wheels[na..], roll);
        self.p.surfaces[S_TURRET].set_pose([TURRET_X, 0.0, 0.0], r.turret_yaw, 0.0);
        place_shooter(&mut self.p.shooter.as_mut().unwrap().wheels, r.hood_deg, r.turret_yaw, TURRET_X);
    }
    /// js `PhysicalRobot.stepMechanisms` (at its yield)
    pub fn mechanisms(&mut self, c: &mut Ctx, dt: f64, t: f64, intake_deploy: f64) {
        let deployed = intake_deploy > 0.5;
        self.p.mech_prepare(c, t, &|l| Self::where_(deployed, l), &Self::shooter_ball);
        self.p.mech_run(dt, &mut c.phys.bodies);
        self.p.mech_finish(c.fuel);
    }
    /// js `preSteps` after its yield; returns the robot's intakeDeploy
    pub fn post_mech(&mut self, phys: &mut PhysicsWorld, body: RigidBodyHandle, dt: f64) -> f64 {
        if let Some(hit) = self.contact { self.trench_push(phys, body, hit, dt); }
        turn_arm(&mut self.p.drives, self.t.pivot, &mut self.arm, dt);
        let tol = pivot::TOL * 2.0 * m::PI;
        let st = &self.arm;
        self.deploy = if st.roll <= tol { 1.0 } else if st.roll >= st.stow - tol { 0.0 } else { 1.0 - st.roll / st.stow };
        self.deploy
    }

    /// js `trenchArm(dt)`: the TRENCH arm against the intake arm; the contact, or None
    fn trench_arm(&mut self, phys: &mut PhysicsWorld, dt: f64, r: &PreIn) -> Option<Hit> {
        let piv = self.t.pivot;
        let roll = self.arm.roll;
        let beams: Vec<Beam> = trench_arms().iter().copied().filter(|a| m::abs(a.x - r.pos[0]) < a.hx + 1.0 && m::abs(a.z - r.pos[2]) < a.hz + 1.0).collect();
        let was = self.trench_hard.unwrap_or(false);
        self.trench_hard = Some(false);
        self.trench_f = 0.0;
        let ac = self.arm_collider;
        let set_groups = |phys: &mut PhysicsWorld, g: u32| phys.colliders[ac].set_collision_groups(crate::groups(g));
        if beams.is_empty() { if was { set_groups(phys, self.arm_groups); } self.prev_pose = None; return None; }
        if self.arm_pts.is_none() { self.arm_pts = Some(arm_samples(&self.hull, self.width)); }
        let [px, py] = self.fold_pivot;
        let top = py + self.hull.iter().map(|[x, y]| x * m::sin(roll) + y * m::cos(roll)).fold(f64::NEG_INFINITY, m::max);
        if !was && r.pos[1] + top < TRENCH::clearHeight - 0.08 { self.prev_pose = None; return None; }
        let (pts, zs) = self.arm_pts.clone().unwrap();
        let (pts, zs) = (&pts, &zs);
        let (pos, q) = (r.pos, r.quat);
        let world = |l: [f64; 3], q: [f64; 4], pos: [f64; 3]| { let v = apply_quat(l, q); [v[0] + pos[0], v[1] + pos[1], v[2] + pos[2]] };
        // (whether the arm turned up by roll is in a TRENCH arm grown by grow)
        let inside = |roll: f64, grow: f64| -> bool {
            let (c, s) = (m::cos(roll), m::sin(roll));
            for &[x, y] in pts {
                let (lx, ly) = (px + x * c - y * s, py + x * s + y * c);
                for &z in zs {
                    let v = world([lx, ly, z], q, pos);
                    for a in &beams {
                        let bottom = a.y - a.hy - grow;
                        let (dx, dy) = (a.hx + grow - m::abs(v[0] - a.x), v[1] - bottom);
                        if dx <= 0.0 || dy <= 0.0 || v[1] > TRENCH::height || m::abs(v[2] - a.z) > a.hz { continue; }
                        return true;
                    }
                }
            }
            false
        };
        // (the deepest: where it pushes, the face each point in it came in through, and the torque)
        let prev = self.prev_pose;
        let deepest = |roll: f64| -> Option<Hit> {
            let (c, s) = (m::cos(roll), m::sin(roll));
            let (mut best, mut best_side): (Option<Hit>, Option<Hit>) = (None, None);
            let (mut torque, mut side) = (0.0, 0.0);
            let qi = [-q[0], -q[1], -q[2], q[3]];
            for &[x, y] in pts {
                let (lx, ly) = (px + x * c - y * s, py + x * s + y * c);
                for &z in zs {
                    let v = world([lx, ly, z], q, pos);
                    for a in &beams {
                        let bottom = a.y - a.hy - 0.0;
                        let (dx, dy) = (a.hx + 0.0 - m::abs(v[0] - a.x), v[1] - bottom);
                        if dx <= 0.0 || dy <= 0.0 || v[1] > TRENCH::height || m::abs(v[2] - a.z) > a.hz { continue; }
                        let mut n: Option<[f64; 3]> = None;
                        if let Some((ppos, pq, proll)) = prev {
                            let (pc, ps) = (m::cos(proll), m::sin(proll));
                            let w = world([px + x * pc - y * ps, py + x * ps + y * pc, z], pq, ppos);
                            if w[1] <= bottom + 1e-4 { n = Some([0.0, -1.0, 0.0]); }
                            else if m::abs(w[0] - a.x) >= a.hx { n = Some([m::sign(w[0] - a.x), 0.0, 0.0]); }
                        }
                        let n = n.unwrap_or(if dx < dy { [{ let sg = m::sign(v[0] - a.x); if sg == 0.0 || sg.is_nan() { 1.0 } else { sg } }, 0.0, 0.0] } else { [0.0, -1.0, 0.0] });
                        let d = if n[1] != 0.0 { dy } else { dx };
                        let nl = apply_quat(n, qi);
                        let lever = (lx - px) * nl[1] - (ly - py) * nl[0];
                        if n[1] != 0.0 { torque += d * lever; } else { side += d * lever; }
                        if best.is_none_or(|b| d > b.depth) { best = Some(Hit { depth: d, point: v, n, lever, torque: 0.0 }); }
                        if n[1] == 0.0 && best_side.is_none_or(|b| d > b.depth) { best_side = Some(Hit { depth: d, point: v, n, lever, torque: 0.0 }); }
                    }
                }
            }
            if let Some(mut b) = best_side { b.torque = side; return Some(b); }
            best.map(|mut b| { b.torque = torque; b })
        };
        let hit = deepest(roll);
        self.prev_pose = Some((pos, q, roll));
        if hit.is_none() && !(was && inside(roll, 0.03)) { if was { set_groups(phys, self.arm_groups); } return None; }
        let stow = self.arm.stow;
        let seek = |dir: f64| -> Option<f64> {
            let end = if dir < 0.0 { 0.0 } else { stow };
            let (mut lo, mut clear) = (roll, None);
            let mut k = 1.0;
            while k <= 150.0 && clear.is_none() {
                let a = roll + dir * k * DEG;
                if (a - end) * dir >= 0.0 { if !inside(end, 0.0) { clear = Some(end); } break; }
                if !inside(a, 0.0) { clear = Some(a); } else { lo = a; }
                k += 1.0;
            }
            let mut clear = clear?;
            for _ in 0..6 { let mm = (lo + clear) / 2.0; if inside(mm, 0.0) { lo = mm; } else { clear = mm; } }
            Some(clear)
        };
        let mut dir = match hit { Some(h) => { let s = m::sign(h.torque); if s == 0.0 || s.is_nan() { -1.0 } else { s } } None => 1.0 };
        let mut clear = if hit.is_some() { seek(dir) } else { None };
        if hit.is_some_and(|h| h.n[1] != 0.0) {
            let other = seek(-dir);
            let last = self.last_dir.unwrap_or(-1.0);
            let dd = |c: Option<f64>| c.map_or(f64::INFINITY, |c| m::abs(c - roll));
            let pick = if m::abs(dd(clear) - dd(other)) < 2.0 * DEG { last } else if dd(clear) <= dd(other) { dir } else { -dir };
            if pick != dir { dir = pick; clear = other; }
        }
        let Some(clear) = clear else {
            if !was { set_groups(phys, self.arm_groups_rigid); }
            self.trench_hard = Some(true);
            return None;
        };
        if was { set_groups(phys, self.arm_groups); }
        self.last_dir = Some(dir);
        let rate = (clear - roll) / dt;
        let (lo, hi) = (self.p.drives.shafts[piv].lo, self.p.drives.shafts[piv].hi);
        if dir < 0.0 { self.p.drives.stop(piv, m::min(lo, rate), m::min(hi, rate)); } else { self.p.drives.stop(piv, m::max(lo, rate), m::max(hi, rate)); }
        hit
    }
    /// js `trenchPush(hit, w0, dt)`: what the TRENCH arm's push on the intake arm took, back on the robot
    fn trench_push(&mut self, phys: &mut PhysicsWorld, body: RigidBodyHandle, hit: Hit, dt: f64) {
        let piv = self.t.pivot;
        let d = &mut self.p.drives;
        let w1 = d.omega(piv);
        let j = d.j(piv);
        let sh = &mut d.shafts[piv];
        let mo = sh.motor.as_mut().unwrap();
        let (cur, lim) = (mo.current, mo.limited);
        let motor = match sh.cmd { None => 0.0, Some(cmd) => mo.torque(w1, cmd, 12.0).0 };
        mo.current = cur; mo.limited = lim;
        let tau = (j * (w1 - self.w0)) / dt - motor - sh.load + sh.drag * w1;
        let s = m::sign(hit.torque);
        let f = m::max(0.0, (s * tau) / m::max(0.05, s * hit.lever));
        self.trench_f = f;
        if f <= 0.0 { return; }
        let k = f * dt;
        let jv = [hit.n[0] * k, hit.n[1] * k, hit.n[2] * k];
        phys.bodies[body].apply_impulse_at_point(Vector::new(jv[0] as f32, jv[1] as f32, jv[2] as f32), Vector::new(hit.point[0] as f32, hit.point[1] as f32, hit.point[2] as f32), true);
    }

    /// js `armTurn()`: the arm's turn up from down on its stop (rad), for drawing it
    pub fn arm_turn(&self) -> f64 { self.arm.roll }

    /// js `postStep(dt, t, { on, feeding, outtake, setpoint, mode })`; returns the robot's stored
    pub fn post_step(&mut self, c: &mut Ctx, dt: f64, t: f64, i: &PostIn, stats: &mut Stats, out: &mut PostOut8793) -> Vec<usize> {
        let stored = self.p.account(c, t, i.pass, i.legal, stats);
        let (on, outtake) = (i.on, i.outtake);
        let d = &mut self.p.drives;
        let h = self.t.horizontal;
        let motor_rps = m::abs(d.trains[h].omega) / (2.0 * m::PI);
        let stalled = m::abs(d.trains[h].motor.current) > JAM_AMPS && motor_rps < JAM_RPS;
        if self.unjam_t > 0.0 {
            self.unjam_t -= dt;
            if self.unjam_t <= 0.0 { self.unjam_t = 0.0; self.jam_t = 0.0; }
        } else if self.full_t > 0.0 && !self.feed {
            self.full_t -= dt;
        } else {
            self.jam_t = if on && !outtake && stalled { self.jam_t + dt } else { 0.0 };
            if self.jam_t > JAM_DEBOUNCE && !self.feed { self.full_t = HOLD_FULL; self.jam_t = 0.0; self.holds += 1; }
            if self.jam_t > JAM_DEBOUNCE { self.unjam_t = JAM_REVERSE; self.jam_t = 0.0; stats.unjams += 1; }
        }
        let unjam = on && self.unjam_t > 0.0;
        let holding = self.full_t > 0.0 && !self.feed && !outtake;
        intake8793(d, &self.t, on && !holding, if outtake || unjam { -1.0 } else { 1.0 });
        let was_feeding = self.feed;
        self.feed = on && i.feeding && i.setpoint > 0.0 && m::abs(i.aim_err.unwrap_or(0.0)) < 10.0 * DEG && i.pass_clear != Some(false);
        if was_feeding && !self.feed && on && !outtake && !(self.unjam_t > 0.0) { self.unjam_t = JAM_REVERSE; self.jam_t = 0.0; self.stop_unjams += 1; }
        if self.ix_back_t > 0.0 { self.ix_back_t -= dt; }
        d.trains[self.t.indexer].cmd = Some(if self.feed { if unjam { -DUTY_INDEXER } else { DUTY_INDEXER } }
            else if on && (outtake || self.ix_back_t > 0.0 || self.unjam_t > 0.0) { -DUTY_INDEXER } else { 0.0 });
        if i.setpoint > 0.0 { self.target_rps = m::min(shooter::MAX_RPS, rps_for(i.setpoint)); self.hold_t = Some(t); }
        else if !(on && self.target_rps != 0.0 && t - self.hold_t.unwrap_or(-9.0) < 1.5) { self.target_rps = 0.0; }
        flywheel_loop(d, self.t.shooter, if on { self.target_rps } else { 0.0 });
        let hc = d.trains[h].cmd.unwrap_or(0.0);
        *out = PostOut8793 {
            flywheel: EXIT_K * exit_speed(m::abs(d.trains[self.t.shooter].omega) / (2.0 * m::PI)),
            intake_speed: if hc < 0.0 { 1.0 } else if hc > 0.0 { -1.0 } else { 0.0 },
            feeding: if self.feed { 1.0 } else { 0.0 },
        };
        stored
    }

    /// js `capacity()`
    pub fn capacity(&self) -> usize { 9 }
    /// js `turretStep(cur, goal, dt, t)`: the turret on its motor
    pub fn turret_step(&mut self, cur: f64, goal: f64, dt: f64, t: f64) -> f64 {
        self.turret.get_or_insert_with(drive8793).step(cur, goal, dt, Some(t))
    }
    /// the flywheel's speed (motor rotations/s), the rollers' and the turret indexer's
    pub fn flywheel_rps(&self) -> f64 { m::abs(self.p.drives.trains[self.t.shooter].omega) / TAU }
}

/// js `flywheelLoop(t, targetRps)`: their velocity loop on the Talon (0: 0 V)
fn flywheel_loop(d: &mut Drives, shooter: usize, target_rps: f64) {
    if !(target_rps > 0.0) { d.trains[shooter].vel = None; d.trains[shooter].cmd = Some(0.0); return; }
    d.trains[shooter].vel = Some(VelLoop { target: target_rps * 2.0 * m::PI, kv: shooter::KV, kp: shooter::KP, ks: 0.0, lo: -12.0, hi: 12.0, per: Some(1.0 / (2.0 * m::PI)), rate: 1000.0 });
}
