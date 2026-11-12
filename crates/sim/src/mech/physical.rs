//! js/mech/physical.js: what every robot built of physical mechanisms shares. The FUEL it takes in
//! stay bodies on the FIELD the whole way, moved only by its rollers, wheels and its CAD's surfaces
//! (Mechanisms on the robot's own body), until its shooter flings them. Here: its frame, which FUEL
//! it has hold of, what got in, what got shot, and when it's full. (What's drawn only, the FUEL's
//! squash and wear, is the app's.)
//!
//! FUEL are fuel.balls indices. JS Sets and Maps iterate in insertion order, and some of that order
//! reaches the physics (the order a Mechanism meets FUEL in), so they're Vecs here, kept in the
//! same order.
use std::collections::HashSet;

use crate::constants::*;
use crate::fuel::Fuel;
use crate::game_match::{LaunchBy, LaunchInfo};
use crate::mech::drives::Drives;
use crate::mech::mechanism::{Mechanism, Skip, Surface, Toucher, Touch};
use jsmath as m;
use rapier3d::prelude::*;

const R: f64 = FUEL::radius;

/// js PACKED_T, STUCK_T, FULL_DROP
pub const PACKED_T: f64 = 0.4;
pub const STUCK_T: f64 = 2.0;
pub const FULL_DROP: usize = 3;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FullBy { Packed, Stuck }

/// js `FullWatch`: full when its intake can't put more in
#[derive(Clone, Debug, Default)]
pub struct FullWatch { pub full: bool, pub by: Option<FullBy>, base: usize, wait_t: f64, pack_t: f64, full_at: usize }

impl FullWatch {
    /// js `step(dt, n, waiting, packed)`
    pub fn step(&mut self, dt: f64, n: usize, waiting: bool, packed: bool) -> bool {
        let (ni, base) = (n as i64, self.base as i64);
        if ni >= base + 2 || ni < base - FULL_DROP as i64 { self.base = n; self.wait_t = 0.0; }
        let fa = self.full_at as i64;
        if self.full && (ni <= fa - FULL_DROP as i64 || (self.by == Some(FullBy::Stuck) && ni >= fa + FULL_DROP as i64)) { self.full = false; }
        self.pack_t = if packed { self.pack_t + if waiting { dt } else { 0.0 } } else { 0.0 };
        if waiting && !self.full { self.wait_t += dt; }
        let by = if self.pack_t >= PACKED_T { Some(FullBy::Packed) } else if self.wait_t >= STUCK_T { Some(FullBy::Stuck) } else { None };
        if !self.full && by.is_some() {
            self.full = true; self.by = by; self.full_at = n; self.base = n; self.wait_t = 0.0; self.pack_t = 0.0;
        }
        self.full
    }
}

/// js `FlatOut`: passing, it goes once it's about as fast as it'll go
#[derive(Clone, Debug, Default)]
pub struct FlatOut { t: f64, rate: f64, last: Option<f64> }

impl FlatOut {
    pub fn step(&mut self, dt: f64, speed: f64, target: f64) -> bool {
        let rate = match self.last { None => 0.0, Some(l) => (speed - l) / dt };
        self.last = Some(speed);
        self.rate += (rate - self.rate) * (1.0 - m::exp(-dt / 0.1));
        self.t = if target > 0.0 && speed < target && speed > 0.5 * target && self.rate < 0.05 * target { self.t + dt } else { 0.0 };
        self.t >= 0.25
    }
}

/// js `mouthPressed(mouth, load)`
pub fn mouth_pressed(mouth: &[[f64; 3]], load: &[[f64; 3]]) -> bool {
    let d = 2.0 * R + 0.01;
    mouth.iter().any(|[x, y, z]| load.iter().any(|[u, v, w]| (u - x) * (u - x) + (v - y) * (v - y) + (w - z) * (w - z) < d * d))
}

/// js `topCount(contacts, up, held, x, x0)`
pub fn top_count(contacts: &[Touch], up: [f64; 3], held: &dyn Fn(RigidBodyHandle) -> bool, x: &dyn Fn(RigidBodyHandle) -> f64, x0: f64) -> usize {
    let mut pressed: Vec<RigidBodyHandle> = Vec::new();
    for c in contacts {
        if matches!(c.wheel, Toucher::Wheel(_)) { continue; }
        if c.n[0] * up[0] + c.n[1] * up[1] + c.n[2] * up[2] > -0.5 || !held(c.ball) { continue; }
        if x(c.ball) > x0 && !pressed.contains(&c.ball) { pressed.push(c.ball); }
    }
    pressed.len()
}

/// js `where()`'s answer: { own, held, near }
#[derive(Clone, Copy, Debug, Default)]
pub struct Where { pub own: bool, pub held: bool, pub near: bool }

/// What a robot tells its mechanisms about itself (the parts of js Robot they read)
pub struct Ctx<'a> {
    pub body: RigidBodyHandle,
    pub alliance: Alliance,
    /// cfg.frame.length, width
    pub frame: (f64, f64),
    pub fuel: &'a mut Fuel,
    pub phys: &'a mut PhysicsWorld,
}

/// js robot.stats (what a physical robot counts)
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats { pub intaked: u32, pub shots: u32, pub passes: u32, pub unjams: u32, pub dropped: u32 }

struct Job { near: Vec<RigidBodyHandle>, owned: Vec<RigidBodyHandle>, skip: Option<Skip>, sb: Vec<RigidBodyHandle> }

/// js `PhysicalRobot`
pub struct Physical {
    pub drives: Drives,
    /// every surface of its mechanisms (they refer to them by index)
    pub surfaces: Vec<Surface>,
    pub path: Mechanism,
    pub shooter: Option<Mechanism>,
    pub preload: Vec<[f64; 3]>,
    /// js naturalFull: { topX, topN }
    pub natural_full: Option<(f64, usize)>,
    pub held: Vec<usize>,
    pub near: Vec<usize>,
    pub in_shooter: Vec<usize>,
    pub wheeled: Vec<usize>,
    pub owned: Vec<usize>,
    pub shot: Vec<(usize, f64)>,
    pub counted: Vec<usize>,
    pub full: bool,
    pub full_watch: FullWatch,
    /// js this.pose: the robot's pose as last read (t, q), or not read since account
    pose: Option<([f64; 3], [f64; 4])>,
    bumper_y0: Option<f64>,
    shared: Option<HashSet<usize>>,
    job: Option<Job>,
}

fn v3(v: Vector) -> [f64; 3] { [v.x as f64, v.y as f64, v.z as f64] }

impl Physical {
    pub fn new(drives: Drives, surfaces: Vec<Surface>, path: Mechanism, shooter: Option<Mechanism>, preload: Vec<[f64; 3]>, natural_full: Option<(f64, usize)>) -> Self {
        Physical {
            drives, surfaces, path, shooter, preload, natural_full,
            held: Vec::new(), near: Vec::new(), in_shooter: Vec::new(), wheeled: Vec::new(), owned: Vec::new(), shot: Vec::new(), counted: Vec::new(),
            full: false, full_watch: FullWatch::default(), pose: None, bumper_y0: None, shared: None, job: None,
        }
    }

    /// js `reset()`
    pub fn reset(&mut self) {
        self.held.clear();
        self.near.clear();
        self.in_shooter.clear();
        self.wheeled.clear();
        self.owned.clear();
        self.shot.clear();
        self.counted.clear();
        self.full = false;
        self.full_watch = FullWatch::default();
    }

    /// js `watchFull(dt, intaking)`
    pub fn watch_full(&mut self, phys: &PhysicsWorld, fuel: &Fuel, body: RigidBodyHandle, dt: f64, intaking: bool) -> bool {
        let waiting = intaking && self.mouth_pressed_now(phys, fuel, body);
        let packed = waiting && self.packed(phys, fuel, body);
        self.full = self.full_watch.step(dt, self.held.len(), waiting, packed);
        self.full
    }
    /// js `_packed()`
    fn packed(&mut self, phys: &PhysicsWorld, fuel: &Fuel, body: RigidBodyHandle) -> bool {
        let (top_x, top_n) = self.natural_full.expect("naturalFull");
        let q = phys.bodies[body].rotation();
        let (qx, qy, qz, qw) = (q.x as f64, q.y as f64, q.z as f64, q.w as f64);
        let up = [2.0 * (qx * qy - qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz + qw * qx)];
        let by_body: Vec<(RigidBodyHandle, usize)> = self.held.iter().map(|&b| (fuel.balls[b].body, b)).collect();
        let xs: Vec<(RigidBodyHandle, f64)> = by_body.iter().map(|&(h, b)| (h, self.local(phys, body, fuel.balls[b].pos)[0])).collect();
        top_count(&self.path.contacts, up, &|h| by_body.iter().any(|(k, _)| *k == h), &|h| xs.iter().find(|(k, _)| *k == h).map_or(0.0, |x| x.1), top_x) >= top_n
    }
    /// js `_mouthPressed()`
    fn mouth_pressed_now(&mut self, phys: &PhysicsWorld, fuel: &Fuel, body: RigidBodyHandle) -> bool {
        let mouth: Vec<usize> = self.owned.iter().copied().filter(|b| !self.held.contains(b)).collect();
        let mouth: Vec<[f64; 3]> = mouth.iter().map(|&b| self.local(phys, body, fuel.balls[b].pos)).collect();
        let held = self.held.clone();
        let load: Vec<[f64; 3]> = held.iter().map(|&b| self.local(phys, body, fuel.balls[b].pos)).collect();
        mouth_pressed(&mouth, &load)
    }

    /// js `_pose()`
    fn read_pose(&mut self, phys: &PhysicsWorld, body: RigidBodyHandle) -> ([f64; 3], [f64; 4]) {
        let b = &phys.bodies[body];
        let (t, q) = (b.translation(), b.rotation());
        let p = ([t.x as f64, t.y as f64, t.z as f64], [q.x as f64, q.y as f64, q.z as f64, q.w as f64]);
        self.pose = Some(p);
        p
    }
    /// js `_local(p)`: a world point in the robot's frame
    pub fn local(&mut self, phys: &PhysicsWorld, body: RigidBodyHandle, p: [f64; 3]) -> [f64; 3] {
        let (t, q) = match self.pose { Some(p) => p, None => self.read_pose(phys, body) };
        let (dx, dy, dz) = (p[0] - t[0], p[1] - t[1], p[2] - t[2]);
        let (x, y, z, w) = (-q[0], -q[1], -q[2], q[3]);
        let ix = w * dx + y * dz - z * dy;
        let iy = w * dy + z * dx - x * dz;
        let iz = w * dz + x * dy - y * dx;
        let iw = -x * dx - y * dy - z * dz;
        [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x]
    }
    /// js `_world(l)`: a point in the robot's frame in the world (reading the pose afresh)
    pub fn world(&mut self, phys: &PhysicsWorld, body: RigidBodyHandle, l: [f64; 3]) -> [f64; 3] {
        let (t, q) = self.read_pose(phys, body);
        let [x, y, z] = l;
        let (qx, qy, qz, qw) = (q[0], q[1], q[2], q[3]);
        let ix = qw * x + qy * z - qz * y;
        let iy = qw * y + qz * x - qx * z;
        let iz = qw * z + qx * y - qy * x;
        let iw = -qx * x - qy * y - qz * z;
        [t[0] + ix * qw + iw * -qx + iy * -qz - iz * -qy, t[1] + iy * qw + iw * -qy + iz * -qx - ix * -qz, t[2] + iz * qw + iw * -qz + ix * -qy - iy * -qx]
    }

    /// js `load(balls)`: the preload into its slots (what doesn't fit goes to its HUMAN PLAYER);
    /// returns the robot's stored
    pub fn load(&mut self, c: &mut Ctx, balls: &[usize]) -> Vec<usize> {
        for (i, &b) in balls.iter().enumerate() {
            if i >= self.preload.len() { c.fuel.add_to_chute(c.phys, c.alliance, b); continue; }
            let p = self.world(c.phys, c.body, self.preload[i]);
            c.fuel.launch(c.phys, b, p, [0.0; 3], Default::default());
            c.fuel.balls[b].in_flight = false;
            c.fuel.own(c.phys, b, 0.0);
            self.held.push(b);
            if !self.owned.contains(&b) { self.owned.push(b); }
            if !self.counted.contains(&b) { self.counted.push(b); }
        }
        self.held.clone()
    }

    /// js `_bumperY0()`
    fn bumper_y0(&mut self) -> f64 {
        if self.bumper_y0.is_none() {
            let ys: Vec<f64> = self.path.surfaces.iter().filter(|&&s| self.surfaces[s].is_block() && self.surfaces[s].name.starts_with("bumper")).filter_map(|&s| self.surfaces[s].block_bottom()).collect();
            let mut v = BUMPER_Y0;
            if !ys.is_empty() { v = m::max(v, ys.iter().copied().fold(f64::INFINITY, m::min)); }
            self.bumper_y0 = Some(v);
        }
        self.bumper_y0.unwrap()
    }

    /// js `mechPrepare(dt, t)`: which FUEL are its. where_(local): its ball path's own; shooter_ball(local):
    /// one its shooter's wheels might touch
    pub fn mech_prepare(&mut self, c: &mut Ctx, t: f64, where_: &dyn Fn([f64; 3]) -> Where, shooter_ball: &dyn Fn([f64; 3]) -> bool) {
        self.read_pose(c.phys, c.body);
        let pos = v3(c.phys.bodies[c.body].translation());
        let (mut near, mut held, mut owned) = (Vec::new(), Vec::new(), Vec::new());
        let (fl, fw) = c.frame;
        let by0 = self.bumper_y0();
        for i in 0..c.fuel.balls.len() {
            if c.fuel.balls[i].state != crate::fuel::State::Field { continue; }
            let p = c.fuel.balls[i].pos;
            if m::abs(p[0] - pos[0]) > 1.0 || m::abs(p[2] - pos[2]) > 1.0 || p[1] > 1.0 { continue; }
            if let Some(k) = self.shot.iter().position(|s| s.0 == i) {
                if t - self.shot[k].1 < 0.35 { continue; }
                self.shot.remove(k);
            }
            let l = self.local(c.phys, c.body, p);
            let under = l[1] < by0 && m::abs(l[0]) < fl / 2.0 + BUMPER_T && m::abs(l[2]) < fw / 2.0 + BUMPER_T;
            let mut w = where_(l);
            if under { w.own = false; w.held = false; }
            if w.own {
                c.fuel.own(c.phys, i, t);
                owned.push(i);
                if w.held { held.push(i); }
            }
            if w.own || w.near { near.push(i); }
        }
        for k in 0..self.owned.len() {
            let b = self.owned[k];
            if owned.contains(&b) || c.fuel.balls[b].state != crate::fuel::State::Field { continue; }
            let [x, y, z] = self.local(c.phys, c.body, c.fuel.balls[b].pos);
            c.fuel.unown(c.phys, b, y < BUMPER_Y1 && (m::abs(x) >= fl / 2.0 || m::abs(z) >= fw / 2.0));
        }
        self.owned = owned;
        self.near = near;
        self.held = held;
        let sb: Vec<usize> = match &self.shooter {
            Some(_) => self.near.clone().into_iter().filter(|&b| { let l = self.local(c.phys, c.body, c.fuel.balls[b].pos); shooter_ball(l) }).collect(),
            None => Vec::new(),
        };
        let sb: Vec<RigidBodyHandle> = sb.iter().map(|&b| c.fuel.balls[b].body).collect();
        if self.shared.is_none() {
            let shared = match &self.shooter { Some(s) => s.surfaces.iter().copied().filter(|sf| self.path.surfaces.contains(sf)).collect(), None => HashSet::new() };
            self.shared = Some(shared);
        }
        let shared = self.shared.as_ref().unwrap();
        let skip = if !shared.is_empty() && !sb.is_empty() { Some(Skip { surfaces: shared.clone(), balls: sb.iter().copied().collect() }) } else { None };
        self.job = Some(Job {
            near: self.near.iter().map(|&b| c.fuel.balls[b].body).collect(),
            owned: self.owned.iter().map(|&b| c.fuel.balls[b].body).collect(),
            skip, sb,
        });
    }
    /// js `mechRun(dt)`: its mechanisms' step
    pub fn mech_run(&mut self, dt: f64, bodies: &mut RigidBodySet) {
        let j = self.job.as_ref().expect("mechPrepare first");
        self.path.step(dt, bodies, &mut self.drives, &mut self.surfaces, &j.near, Some(&j.owned), j.skip.as_ref());
        if let Some(s) = &mut self.shooter { s.step(dt, bodies, &mut self.drives, &mut self.surfaces, &j.sb, None, None); }
    }
    /// js `mechFinish()` (the squash and wear are drawn only: the FUEL touched are marked)
    pub fn mech_finish(&mut self, fuel: &mut Fuel) {
        for &b in &self.near { fuel.balls[b].mech_touched = true; }
        self.job = None;
    }

    /// js `account(t, mode)`: what got in, and what got shot; returns the robot's stored
    pub fn account(&mut self, c: &mut Ctx, t: f64, pass: bool, legal: bool, stats: &mut Stats) -> Vec<usize> {
        self.pose = None;
        let stored = self.held.clone();
        let around = self.near.clone();
        self.counted.retain(|b| around.contains(b));
        for &b in &stored { if !self.counted.contains(&b) { self.counted.push(b); stats.intaked += 1; } }
        self.shot.retain(|&(_, ts)| !(t - ts > 0.35));
        let Some(shooter) = &self.shooter else { return stored };
        let by_body = |h: RigidBodyHandle| self.near.iter().copied().find(|&b| c.fuel.balls[b].body == h);
        let mut touching: Vec<usize> = Vec::new();
        for ct in &shooter.contacts { if let Some(b) = by_body(ct.ball) { if !touching.contains(&b) { touching.push(b); } } }
        for &b in &touching { if !self.in_shooter.contains(&b) { self.in_shooter.push(b); } }
        for ct in &shooter.contacts {
            if matches!(ct.wheel, Toucher::Wheel(_)) { if let Some(b) = by_body(ct.ball) { if !self.wheeled.contains(&b) { self.wheeled.push(b); } } }
        }
        let rb = &c.phys.bodies[c.body];
        let (rv, w, com) = (v3(rb.linvel()), v3(rb.angvel()), v3(rb.center_of_mass()));
        for b in self.in_shooter.clone() {
            if touching.contains(&b) { continue; }
            self.in_shooter.retain(|&x| x != b);
            let Some(k) = self.wheeled.iter().position(|&x| x == b) else { continue };
            self.wheeled.remove(k);
            let bd = &c.phys.bodies[c.fuel.balls[b].body];
            let (u, p) = (v3(bd.linvel()), v3(bd.translation()));
            let d = [p[0] - com[0], p[1] - com[1], p[2] - com[2]];
            let v = [u[0] - rv[0] - (w[1] * d[2] - w[2] * d[1]), u[1] - rv[1] - (w[2] * d[0] - w[0] * d[2]), u[2] - rv[2] - (w[0] * d[1] - w[1] * d[0])];
            if m::hypot3(v[0], v[1], v[2]) < 3.0 || v[1] < 0.0 { continue; }
            self.counted.retain(|&x| x != b);
            self.owned.retain(|&x| x != b);
            self.shot.retain(|s| s.0 != b);
            self.shot.push((b, t));
            c.fuel.release(c.phys, b, LaunchInfo { by: LaunchBy::Robot, alliance: Some(c.alliance), legal, t: Some(t) }, Some(0.35));
            stats.shots += 1;
            if pass { stats.passes += 1; }
        }
        stored
    }
}
