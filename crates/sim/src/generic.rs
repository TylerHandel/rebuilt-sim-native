//! js robot.js for a generic robot (one not built of physical mechanisms: 971, 4946, 3928, 4930):
//! its intake grabs FUEL its spinning roller touches and pulls them up over the bumper into the
//! hopper (hopper.rs holds them), where they're carried to the feed; the feed takes them one at a
//! time up their path into the shooter, which launches them with its own scatter.

use jsmath as m;
use rapier3d::prelude::*;
use serde_json::Value;

use crate::constants::*;
use crate::fuel::{clamp, Fuel, LaunchOpts, State};
use crate::game_match::{LaunchBy, LaunchInfo};
use crate::hopper::{self, ArmBox, Done, End, Env, Hopper, Spec, V};
use crate::rng::Mulberry32;
use crate::robot::{apply_quat, approach, ClimbState, Robot};

const R: f64 = FUEL::radius;
const INTAKE_SPEED: f64 = 3.5;
const BLOCK_D: f64 = 2.0 * R * 0.85;
const HOLD_GAP: f64 = 2.0 * R * 0.9;
const ROLLER_R: f64 = 0.032;
const ROLLER_Y: f64 = 0.1;
const STALL_T: f64 = 0.35;
const FULL_T: f64 = 0.75;
const FEED_SPEED: f64 = 7.0;
const DEG: f64 = m::PI / 180.0;

fn num(v: &Value, k: &str) -> f64 { v[k].as_f64().unwrap_or_else(|| panic!("config: no number {k}")) }
fn opt(v: &Value, k: &str) -> Option<f64> { v.get(k).and_then(|x| x.as_f64()) }
fn vf(x: f64, y: f64, z: f64) -> Vector { Vector::new(x as f32, y as f32, z as f32) }
fn grp(g: u32) -> InteractionGroups { crate::groups(g) }

/// js a grabbed FUEL on its way in (robot.captured)
#[derive(Clone, Copy, Debug, Default)]
pub struct Cap { pub b: usize, pub t0: f64, pub z: f64, pub over: bool, pub coast: f64, pub y_prev: f64, pub wait: bool, pub wait_t: f64, pub held: bool, pub vs: Option<[f64; 3]> }

/// js robot.geoCap: what its hopper holds retracted, extended and (a compacting intake) compacted
#[derive(Clone, Copy, Debug)]
pub struct GeoCap { pub retracted: f64, pub extended: f64, pub compacted: Option<f64> }

/// js _intakePath(z)
#[derive(Clone, Copy, Debug)]
struct Path { lip_x: f64, entry_x: f64, entry_y: f64, lift_y: f64 }

/// A generic robot's own state (js Robot's fields the physical robots don't use)
pub struct Generic {
    pub hopper: Hopper,
    pub captured: Vec<Cap>,
    pub geo: GeoCap,
    pub cap_limited: bool,
    pub intake_push: f64,
    pub load_balls: Vec<ColliderHandle>,
    pub intake_tokens: f64,
    pub stalled: bool,
    pub stall_t: f64,
    pub roller_speed: f64,
    pub wall_push: f64,
    pub outtake_timer: f64,
    pub feed_timer: f64,
    pub next_period: f64,
    pub lane: usize,
    pub lane_last: Vec<Option<u64>>,
    pub feeding: bool,
    pub intake_speed: f64,
    pub prev_vel: Option<[f64; 2]>,
    pub prev_omega: f64,
    pub pivot_load: f64,
    pub pivot_moving: bool,
    pub force_deploy: bool,
    pub spin_hold: f64,
    pub spin_hold_t: f64,
    pub top_y: f64,
    pub top_load: f64,
    pub top_stowed: f64,
    pub grow_top: f64,
    pub load_top: f64,
    pub load_high_t: f64,
}

impl Generic {
    /// js the generic parts of Robot's constructor (its hopper as robots.json's geoCap measured it)
    pub fn new(cfg: &Value, height: f64) -> Generic {
        let g = &cfg["geoCap"];
        assert!(g.is_object(), "robot without geoCap: re-run tools/sync-assets.mjs");
        Generic {
            hopper: Hopper::new(Spec::from(&cfg["bay"])),
            captured: Vec::new(),
            geo: GeoCap { retracted: num(g, "retracted"), extended: num(g, "extended"), compacted: opt(g, "compacted") },
            cap_limited: g["capLimited"].as_bool().unwrap_or(false),
            intake_push: num(g, "intakePush"),
            load_balls: Vec::new(),
            intake_tokens: 0.0, stalled: false, stall_t: 0.0, roller_speed: 1.0, wall_push: 0.0, outtake_timer: 0.0,
            feed_timer: 0.0, next_period: 0.0, lane: 0, lane_last: Vec::new(), feeding: false, intake_speed: 0.0,
            prev_vel: None, prev_omega: 0.0, pivot_load: 0.0, pivot_moving: false, force_deploy: false, spin_hold: 0.0, spin_hold_t: 0.0,
            top_y: height, top_load: height, top_stowed: height, grow_top: 0.0, load_top: 0.0, load_high_t: 0.0,
        }
    }
    /// js reset()'s generic part
    pub fn reset(&mut self) {
        self.hopper.clear();
        self.captured.clear();
        self.prev_vel = None;
        self.prev_omega = 0.0;
        self.pivot_load = 0.0;
        self.pivot_moving = false;
        self.intake_speed = 0.0;
        self.feed_timer = 0.0;
        self.lane = 0;
        self.intake_tokens = 0.0;
        self.stalled = false;
        self.roller_speed = 1.0;
        self.wall_push = 0.0;
        self.outtake_timer = 0.0;
        self.feeding = false;
    }
}

/// js `foldTop(ic, deploy)`: a folding intake's top (robot frame), 0 without one
pub fn fold_top(ic: &Value, deploy: f64) -> f64 {
    let Some(f) = ic.get("fold").filter(|x| x.is_object()) else { return 0.0 };
    let a = (1.0 - deploy) * num(f, "stowDeg") * m::PI / 180.0;
    let (c, s) = (m::cos(a), m::sin(a));
    let mut top = f64::NEG_INFINITY;
    for p in f["hull"].as_array().unwrap() { let (x, y) = (p[0].as_f64().unwrap(), p[1].as_f64().unwrap()); top = m::max(top, x * s + y * c); }
    f["pivot"][1].as_f64().unwrap() + top
}

impl Robot {
    fn g(&self) -> &Generic { self.generic.as_ref().unwrap() }
    fn g_mut(&mut self) -> &mut Generic { self.generic.as_mut().unwrap() }

    /// js `worldToLocal(x, z)`
    pub fn world_to_local(&self, x: f64, z: f64) -> [f64; 2] {
        let (dx, dz) = (x - self.pos[0], z - self.pos[2]);
        let (c, s) = (m::cos(self.yaw), m::sin(self.yaw));
        [dx * c - dz * s, dx * s + dz * c]
    }
    fn world_to_local3(&self, p: [f64; 3]) -> V { let l = self.world_to_local(p[0], p[2]); [l[0], p[1] - self.pos[1], l[1]] }
    fn world_to_local_vec(&self, x: f64, z: f64) -> [f64; 2] { let (c, s) = (m::cos(self.yaw), m::sin(self.yaw)); [x * c - z * s, x * s + z * c] }
    fn forward(&self) -> [f64; 2] { [m::cos(self.yaw), -m::sin(self.yaw)] }

    /// js `bayFront()`: the hopper's front now (an extending hopper moves it forward)
    pub fn bay_front(&self) -> f64 { num(&self.cfg["bay"], "x1") + opt(&self.cfg["storage"], "extLen").unwrap_or(0.0) * self.hopper_deploy }
    /// js `capacity()` for a generic robot
    pub fn capacity_generic(&self) -> usize {
        let g = self.g().geo;
        m::floor(g.retracted + (g.extended - g.retracted) * self.hopper_deploy + 1e-6) as usize
    }
    fn max_capacity_generic(&self) -> f64 { self.g().geo.extended }

    /// js `_compactorX(dep)`
    fn compactor_x(&self, dep: f64) -> f64 {
        let ic = &self.cfg["intake"];
        let a = &ic["arm"];
        let deg = num(a, "stowDeg") + (num(a, "deployDeg") - num(a, "stowDeg")) * dep;
        let x = num(a, "x") + num(a, "len") * m::cos(deg * DEG);
        let y = num(a, "y") + num(a, "len") * m::sin(deg * DEG);
        let h = &self.g().hopper;
        if y < h.floor_at(m::min(x, h.front - 0.01), 0.0) { return f64::INFINITY; }
        m::max(x, opt(ic, "compactMin").unwrap_or(f64::NEG_INFINITY))
    }
    /// js `_capBehind(x)`
    fn cap_behind(&self, x: f64) -> f64 {
        let b = &self.cfg["bay"];
        let g = self.g().geo;
        let (front, full) = (self.bay_front(), self.capacity_generic() as f64);
        let c = m::min(full, g.compacted.unwrap_or(g.retracted));
        if x >= front { return full; }
        let (x0, x1) = (num(b, "x0"), num(b, "x1"));
        if x >= x1 { return c + ((full - c) * (x - x1)) / m::max(0.01, front - x1); }
        (c * m::max(0.0, x - x0)) / m::max(0.01, x1 - x0)
    }

    /// js `loadFuel(balls)` for a generic robot: dropped loosely into the hopper
    pub fn load_generic(&mut self, phys: &mut PhysicsWorld, fuel: &mut Fuel, balls: &[usize], rng: &mut Mulberry32) {
        let front = self.bay_front();
        self.g_mut().hopper.front = front;
        for &b in balls { fuel.to_robot(phys, b); self.stored.push(b); }
        let mut r = || rng.random();
        self.g_mut().hopper.fill(balls, &mut r);
    }

    /// js `_hopper(dt, on)`'s want: the extension a generic robot's load needs out
    pub(crate) fn hopper_need(&self) -> f64 {
        let Some(g) = &self.generic else { return 0.0 };
        let in_fixed = g.geo.compacted.unwrap_or(g.geo.retracted);
        m::min(1.0, m::max(0.0, (self.stored.len() as f64 - in_fixed) / m::max(1.0, g.geo.extended - in_fixed)))
    }

    /// js `_intakePath(z)`
    fn intake_path(&self, z: f64) -> Path {
        let front = self.bay_front();
        let ic = &self.cfg["intake"];
        let lip = ic.get("lip").filter(|x| x.is_object());
        let entry_x = front - R - 0.02;
        let entry_y = self.g().hopper.floor_at(entry_x, z) + R + 0.005;
        let (lip_x, lip_y) = match lip { Some(l) => (num(l, "x"), num(l, "y")), None => (self.half_l + 0.04, BUMPER_Y1 + R + 0.025) };
        Path { lip_x, entry_x, entry_y, lift_y: m::max(entry_y, lip_y) }
    }
    /// js `_frontRoller()`: (x, y, r, grip)
    fn front_roller(&self) -> (f64, f64, f64, f64) {
        let ic = &self.cfg["intake"];
        let r = opt(ic, "rollerR").unwrap_or(ROLLER_R);
        (self.half_l + num(ic, "reach") - r, opt(ic, "rollerY").unwrap_or(ROLLER_Y), r, opt(ic, "squeeze").unwrap_or(0.75) * 0.0254)
    }
    /// js `_offRollers(l, path)`
    fn off_rollers(&self, l: V, path: &Path) -> bool {
        let (ax, ay, rr, grip) = self.front_roller();
        let (bx, by) = (path.lip_x, path.lift_y);
        let (ex, ey) = (bx - ax, by - ay);
        let len2 = { let q = ex * ex + ey * ey; if q == 0.0 { 1.0 } else { q } };
        let u = clamp(((l[0] - ax) * ex + (l[1] - ay) * ey) / len2, 0.0, 1.0);
        let d = m::hypot2(l[0] - ax - u * ex, l[1] - ay - u * ey);
        d > R + rr + grip + 0.04
    }
    /// js `_holdPoint(path)`: (x, y, ix, iy)
    fn hold_point(path: &Path) -> (f64, f64, f64, f64) {
        let (mut dx, mut dy) = (path.lip_x - path.entry_x, path.lift_y - path.entry_y);
        let len = m::hypot2(dx, dy);
        if len > 0.01 { dx /= len; dy /= len; } else { dx = 1.0; dy = 0.0; }
        let k = m::max(len, HOLD_GAP);
        (path.entry_x + dx * k, path.entry_y + dy * k, -dx, -dy)
    }

    /// js `_guideCaptured(dt)`: the rollers drag grabbed FUEL up over the bumper and into the hopper
    pub(crate) fn guide_captured(&mut self, phys: &mut PhysicsWorld, fuel: &Fuel, dt: f64) {
        if self.g().captured.is_empty() { return; }
        let b = &phys.bodies[self.body];
        let (tr, lv, w) = (b.translation(), b.linvel(), b.angvel().y as f64);
        let (tr, lv) = ([tr.x as f64, tr.y as f64, tr.z as f64], [lv.x as f64, lv.y as f64, lv.z as f64]);
        let (c, sn) = (m::cos(self.yaw), m::sin(self.yaw));
        let pull = opt(&self.cfg["intake"], "pull").unwrap_or(INTAKE_SPEED);
        for k in 0..self.g().captured.len() {
            let mut cap = self.g().captured[k];
            let p = phys.bodies[fuel.balls[cap.b].body].translation();
            let (dx, dz) = (p.x as f64 - tr[0], p.z as f64 - tr[2]);
            let (lx, lz, ly) = (dx * c - dz * sn, dx * sn + dz * c, p.y as f64 - tr[1]);
            let path = self.intake_path(cap.z);
            if !cap.over && ly >= path.lift_y - 0.03 { cap.over = true; cap.coast = pull; cap.y_prev = ly; }
            let up = !cap.over;
            let (mut tx, mut ty) = if up { (path.lip_x, path.lift_y) } else { (path.entry_x, path.entry_y) };
            let mut speed = pull;
            if cap.over && !cap.wait {
                cap.coast = m::sqrt(m::max(0.25, cap.coast * cap.coast + 2.0 * 9.81 * (cap.y_prev - ly))) * (1.0 - 1.5 * dt);
                cap.y_prev = ly;
                speed = cap.coast;
            }
            if cap.wait { let h = Self::hold_point(&path); tx = h.0; ty = h.1; }
            let (mut vx, mut vy, mut vz) = (tx - lx, ty - ly, cap.z - lz);
            let dist = { let h = m::hypot3(vx, vy, vz); if h == 0.0 || h.is_nan() { 1.0 } else { h } };
            let sp = m::min(speed, dist / (2.0 * dt));
            vx *= sp / dist; vy *= sp / dist; vz *= sp / dist;
            let vs = [lv[0] + w * dz + vx * c + vz * sn, vy + 9.81 * dt, lv[2] - w * dx - vx * sn + vz * c];
            cap.vs = Some(vs);
            phys.bodies[fuel.balls[cap.b].body].set_linvel(vf(vs[0], vs[1], vs[2]), true);
            self.g_mut().captured[k] = cap;
        }
    }

    /// js `_intake(dt, t, on)`
    pub(crate) fn intake_generic(&mut self, phys: &mut PhysicsWorld, fuel: &mut Fuel, dt: f64, t: f64, on: bool) {
        let ic = self.cfg["intake"].clone();
        let compacts = ic["compacts"].as_bool() == Some(true);
        let mut want = on && self.cmd.intake;
        if ic["latched"].as_bool() == Some(true) && self.intake_deploy >= 1.0 { want = true; }
        if self.g().force_deploy { want = true; }
        if compacts && on && (self.cmd.shoot || self.cmd.pass) && !self.cmd.intake { want = false; }
        if on && self.cmd.outtake { want = true; }
        let fold = ic.get("fold").is_some_and(|x| x.is_object());
        if fold && !compacts && self.cfg["storage"]["extend"].as_str() == Some("intake") && self.stored.len() as f64 > self.g().geo.retracted { want = true; }
        let deploy_time = num(&ic, "deployTime");
        let next = approach(self.intake_deploy, if want { 1.0 } else { 0.0 }, dt / (if want { deploy_time } else { opt(&ic, "retractTime").unwrap_or(deploy_time) }));
        let squeezing = compacts && next < self.intake_deploy;
        let limit = opt(&ic, "compactPush").unwrap_or(self.g().intake_push);
        let full = squeezing && self.stored.len() as f64 > self.cap_behind(self.compactor_x(next));
        let hard = squeezing && self.g().hopper.wall_pressure > limit;
        let wp = self.g().hopper.wall_pressure;
        let moving = m::abs(next - self.intake_deploy) > 1e-6 && !(full || hard);
        {
            let g = self.g_mut();
            g.pivot_load = if squeezing { if full || hard { 1.0 } else { m::min(1.0, wp / limit) } } else { 0.0 };
            g.pivot_moving = moving;
        }
        if !(full || hard) { self.intake_deploy = next; }
        let wall = if compacts { self.compactor_x(self.intake_deploy) } else { f64::INFINITY };
        self.g_mut().hopper.wall = wall;
        self.hopper_step(phys, dt, on);
        let deployed = self.intake_deploy > 0.85;
        let gnow = if deployed { groups(GROUP::INTAKE, GROUP::BALL | GROUP::ROBOT_BARRIER) } else { 0 };
        phys.colliders[self.intake_roller].set_collision_groups(grp(gnow));
        for c in self.side_plates { phys.colliders[c].set_collision_groups(grp(if deployed { groups(GROUP::INTAKE, GROUP::BALL) } else { 0 })); }

        let running = on && self.cmd.intake && deployed;
        let intake_speed = if on && self.cmd.outtake { -1.0 } else if running { 1.0 } else { 0.0 };
        self.g_mut().intake_speed = intake_speed;
        self.full = self.g().stall_t > FULL_T;
        let width = num(&ic, "width");
        let row = m::max(2.0, m::floor(width / (2.0 * R))) as usize;
        let rate = num(&ic, "rate");
        let max_cap = self.max_capacity_generic();
        if running && !self.g().stalled && self.g().captured.len() < row && (self.stored.len() as f64) < max_cap * 2.0 + 10.0 {
            {
                let g = self.g_mut();
                g.intake_tokens = m::min(m::max(4.0, 2.0 * rate * dt), g.intake_tokens + rate * dt);
            }
            let (rx, ry, rr, grip) = self.front_roller();
            let touch = R + rr + grip;
            let hw = width / 2.0 + 0.3 * R;
            let n = fuel.balls.len();
            let mut i = 0;
            while i < n && self.g().intake_tokens >= 1.0 && self.g().captured.len() < row {
                let bi = i;
                i += 1;
                let b = &fuel.balls[bi];
                if b.state != State::Field || b.captor.is_some() { continue; }
                let p = b.pos;
                if m::abs(p[0] - self.pos[0]) > 1.3 || m::abs(p[2] - self.pos[2]) > 1.3 { continue; }
                if p[1] - self.pos[1] > 0.33 { continue; }
                let l = self.world_to_local3(p);
                let (dx, dy) = (l[0] - rx, l[1] - ry);
                if dx * dx + dy * dy > touch * touch || dx < -0.2 * R || m::abs(l[2]) > hw { continue; }
                if b.hub_fresh { self.fouls.push(("G408", "Caught FUEL released by the HUB before it touched the carpet")); }
                let bm = &mut fuel.balls[bi];
                bm.captor = Some(self.key.clone());
                bm.hub_fresh = false;
                bm.in_flight = false;
                bm.launch = None;
                bm.ignoring = false;
                phys.colliders[bm.col].set_collision_groups(grp(groups(GROUP::BALL, GROUP::STATIC)));
                phys.bodies[bm.body].wake_up(true);
                let hz = num(&self.cfg["bay"], "hw") - R - 0.01;
                let lim = m::min(hz, width / 2.0 - R);
                let z = clamp(l[2], -lim, lim);
                let g = self.g_mut();
                g.captured.push(Cap { b: bi, t0: t, z, ..Cap::default() });
                g.intake_tokens -= 1.0;
            }
        } else {
            self.g_mut().intake_tokens = 0.0;
        }
        self.settle_captured(phys, fuel, t, dt, running);
        if on && self.cmd.outtake && !self.stored.is_empty() {
            self.g_mut().outtake_timer += dt;
            while self.g().outtake_timer >= 0.08 {
                self.g_mut().outtake_timer -= 0.08;
                if !self.start_outtake() { break; }
            }
        } else { self.g_mut().outtake_timer = 0.0; }
    }

    /// js `_settleCaptured(t, dt, running)`
    fn settle_captured(&mut self, phys: &mut PhysicsWorld, fuel: &mut Fuel, t: f64, dt: f64, running: bool) {
        let deployed = self.intake_deploy > 0.85;
        let (mut waited, mut outside): (f64, f64) = (0.0, 0.0);
        let mut pushed: Vec<u64> = Vec::new();
        let mut i = self.g().captured.len();
        while i > 0 {
            i -= 1;
            let mut cap = self.g().captured[i];
            let bi = cap.b;
            let l = self.world_to_local3(fuel.balls[bi].pos);
            let path = self.intake_path(cap.z);
            let block: Vec<u64> = self.g().hopper.list.iter().filter(|e| e.tr.is_none() && {
                let (dx, dy, dz) = (e.p[0] - path.entry_x, e.p[1] - path.entry_y, e.p[2] - cap.z);
                dx * dx + dy * dy + dz * dz < BLOCK_D * BLOCK_D
            }).map(|e| e.id).collect();
            let shut = (self.cfg["bay"]["pack"].as_bool() == Some(false) || self.g().cap_limited) && self.stored.len() >= self.capacity_generic();
            let off = !cap.over && cap.wait_t == 0.0 && self.off_rollers(l, &path);
            let state = fuel.balls[bi].state;
            let lost = state != State::Field || (cap.wait_t == 0.0 && t - cap.t0 > 0.8) || (cap.wait_t != 0.0 && !deployed) || off;
            if !lost && block.is_empty() && !shut && l[0] <= path.entry_x + 0.02 && l[1] >= path.entry_y - 0.04 {
                self.g_mut().captured.remove(i);
                let v = phys.bodies[fuel.balls[bi].body].linvel();
                let v = [v.x as f64, v.y as f64, v.z as f64];
                let pv = self.velocity_at(fuel.balls[bi].pos);
                let lv = self.world_to_local_vec(v[0] - pv[0], v[2] - pv[1]);
                fuel.to_robot(phys, bi);
                fuel.balls[bi].captor = None;
                self.stored.push(bi);
                self.stats.intaked += 1;
                let push = self.g().intake_push;
                let g = self.g_mut();
                let k = g.hopper.add(bi, l, [lv[0], v[1], lv[1]]);
                g.hopper.list[k].push = push;
                g.hopper.list[k].push_t = 0.4;
            } else if lost || (!running && !cap.over && cap.wait_t == 0.0) {
                self.g_mut().captured.remove(i);
                fuel.balls[bi].captor = None;
                self.stats.dropped += 1;
                if fuel.balls[bi].state == State::Field {
                    fuel.balls[bi].ignoring = true;
                    fuel.balls[bi].ignore_until = t + 0.25;
                    if l[0] < self.half_l + R && m::abs(l[2]) < self.half_w + R { self.drop_in_front(phys, fuel, bi, l[2]); }
                }
            } else if !block.is_empty() || shut {
                cap.wait = true;
                cap.t0 = t;
                cap.wait_t += dt;
                let h = Self::hold_point(&path);
                let (hx, hy) = (l[0] - h.0, l[1] - h.1);
                if m::hypot3(hx, hy, l[2] - cap.z) < 0.02 { cap.held = true; }
                let (c, sn) = (m::cos(self.yaw), m::sin(self.yaw));
                let v = phys.bodies[fuel.balls[bi].body].linvel();
                let v = [v.x as f64, v.y as f64, v.z as f64];
                let vs = cap.vs.unwrap_or(v);
                let blocked = (vs[0] - v[0]) * -h.2 * c + (vs[2] - v[2]) * h.2 * sn + (vs[1] - v[1]) * -h.3 > 0.15;
                let into = hx * h.2 + hy * h.3;
                let sq = if cap.held && blocked { m::max(0.0, into - 0.005) } else { 0.0 };
                let f = hopper::fuel_force(sq);
                if f > 0.0 {
                    outside += f;
                    let j = f * dt;
                    phys.bodies[self.body].apply_impulse(vf((h.2 * c) * j, 0.0, (-h.2 * sn) * j), true);
                }
                let ff = (if running { self.g().intake_push } else { 0.0 }) + f;
                let nb = block.len() as f64;
                for id in &block {
                    if pushed.contains(id) { continue; }
                    pushed.push(*id);
                    let e = self.g_mut().hopper.list.iter_mut().find(|e| e.id == *id).unwrap();
                    let (dx, dy, dz) = (e.p[0] - path.entry_x - R, e.p[1] - path.entry_y + 0.5 * R, e.p[2] - cap.z);
                    let d = { let q = m::hypot3(dx, dy, dz); if q == 0.0 || q.is_nan() { 1.0 } else { q } };
                    let k = ff / nb / d;
                    let mut x = e.ext.unwrap_or([0.0; 3]);
                    x[0] += dx * k; x[1] += dy * k; x[2] += dz * k;
                    e.ext = Some(x);
                }
                waited = m::max(waited, cap.wait_t);
                self.g_mut().captured[i] = cap;
            } else {
                cap.wait = false;
                if cap.wait_t != 0.0 { cap.wait_t += dt; waited = m::max(waited, cap.wait_t); }
                self.g_mut().captured[i] = cap;
            }
        }
        let g = self.g_mut();
        g.wall_push = outside;
        g.stalled = waited > STALL_T;
        g.stall_t = if g.stalled { g.stall_t + dt } else { 0.0 };
        g.roller_speed = if g.stalled { 0.0 } else if waited > 0.0 { 0.35 } else { 1.0 };
    }

    /// js `_dropInFront(b, lz)`
    fn drop_in_front(&mut self, phys: &mut PhysicsWorld, fuel: &mut Fuel, bi: usize, lz: f64) {
        let w = self.half_w - R;
        let p = self.local_to_world(self.half_l + R + 0.01, R + 0.01, clamp(lz, -w, w));
        let v = self.velocity_at(p);
        let f = self.forward();
        let b = &mut phys.bodies[fuel.balls[bi].body];
        b.set_translation(vf(p[0], p[1], p[2]), true);
        b.set_linvel(vf(v[0] + 0.3 * f[0], 0.0, v[1] + 0.3 * f[1]), true);
        b.set_angvel(vf(0.0, 0.0, 0.0), true);
        fuel.balls[bi].pos = p;
    }

    /// js `_startOuttake()`
    fn start_outtake(&mut self) -> bool {
        let front = self.bay_front();
        let mut best: Option<usize> = None;
        let mut bd = f64::INFINITY;
        for (k, e) in self.g().hopper.list.iter().enumerate() {
            if e.tr.is_some() { continue; }
            let d = (front - e.p[0]) + 0.5 * e.p[1];
            if d < bd { bd = d; best = Some(k); }
        }
        let Some(k) = best else { return false };
        let iw = num(&self.cfg["intake"], "width");
        let z = clamp(self.g().hopper.list[k].p[2], -iw / 2.0 + 0.08, iw / 2.0 - 0.08);
        let path = self.intake_path(z);
        let out = [m::max(self.half_l, front) + 0.2, 0.18, z];
        let lift_y = path.lift_y;
        self.g_mut().hopper.start_transit(k, &[[path.entry_x, path.entry_y, z], [path.lip_x, lift_y, z]], End::At(out), out, 5.0, Done::Outtake, false, None, 0.2);
        true
    }

    /// js `_feedAt(x, y, z)`: a point on the feed path now (on a rotor's arm, 4946's: turned with it)
    fn feed_at(&self, x: f64, y: f64, z: f64) -> V {
        let b = &self.cfg["bay"];
        let rotates = b["feed"]["rotates"].as_bool() == Some(true);
        let Some(r) = b.get("rotor").filter(|x| x.is_object() && rotates) else { return [x, y, z] };
        let (rx, rz) = (num(r, "x"), num(r, "z"));
        let a = self.g().hopper.fin_angle * opt(r, "dir").unwrap_or(1.0);
        let (c, s, dx, dz) = (m::cos(a), m::sin(a), x - rx, z - rz);
        [rx + dx * c + dz * s, y, rz - dx * s + dz * c]
    }

    /// where a transit's end is now (js tr.end())
    fn end_now(&self, e: &End) -> V {
        match *e {
            End::At(v) => v,
            End::Turret(ti) => self.turret_exit_pub(ti),
            End::Fixed(lane_z) => { let x = self.fixed_exit_pub(self.hood_deg); [x[0], x[1], lane_z] }
        }
    }

    /// js `_startFeed(turret, lane)`: the FUEL nearest the feed starts up the feed path
    fn start_feed(&mut self, turret: Option<usize>, rng: &mut Mulberry32) -> bool {
        let sh = self.cfg["shooter"].clone();
        let f = self.cfg["bay"]["feed"].clone();
        let fixed = sh["type"].as_str() == Some("fixed");
        let mut lane_z = opt(&f, "z").unwrap_or(0.0);
        let mut ti = 0;
        if self.turrets.len() > 1 {
            let Some(tt) = turret else { return false };
            if !self.tws[tt].ok { return false; }
            ti = tt;
            self.g_mut().lane = ti;
        }
        assert!(sh.get("feedPaced").is_none_or(|x| !x.as_bool().unwrap_or(false)), "a paced feed: not ported");
        if fixed {
            let lanes: Vec<f64> = sh["lanes"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
            let n = lanes.len();
            let prev = self.g().lane % n;
            let lane = if n > 1 { (prev + 1 + m::floor(rng.random() * (n - 1) as f64) as usize) % n } else { 0 };
            self.g_mut().lane = lane;
            lane_z = lanes[lane];
        } else if let Some(zs) = f.get("zs").and_then(|x| x.as_array()) { lane_z = zs[ti].as_f64().unwrap(); }
        let mut fp = self.feed_at(num(&f, "x"), 0.0, lane_z);
        fp[1] = self.g().hopper.floor_at(fp[0], fp[2]) + R;
        let mut best: Option<usize> = None;
        let mut bd = f64::INFINITY;
        for (k, e) in self.g().hopper.list.iter().enumerate() {
            if e.tr.is_some() { continue; }
            let (dx, dy, dz) = (e.p[0] - fp[0], e.p[1] - fp[1], e.p[2] - fp[2]);
            let d = dx * dx + dy * dy + dz * dz;
            if d < bd { bd = d; best = Some(k); }
        }
        let Some(k) = best else { return false };
        if let Some(reach) = opt(&f, "reach") { if reach != 0.0 && bd > reach * reach { return false; } }
        if !fixed { self.g_mut().lane += 1; }
        let via_v = match f.get("vias").and_then(|x| x.as_array()) { Some(v) => v[ti].clone(), None => f["via"].clone() };
        let via: Vec<V> = via_v.as_array().unwrap().iter().map(|p| {
            let (x, y) = (p[0].as_f64().unwrap(), p[1].as_f64().unwrap());
            let z = p.get(2).and_then(|v| v.as_f64()).unwrap_or(lane_z);
            self.feed_at(x, y, z)
        }).collect();
        let end = if fixed { End::Fixed(lane_z) } else { End::Turret(ti) };
        let end_v = self.end_now(&end);
        let speed = opt(&f, "speed");
        let g = self.g_mut();
        g.hopper.start_transit(k, &via, end, end_v, speed.unwrap_or(FEED_SPEED), Done::Fire, true, Some((ti, lane_z.to_bits())), if speed.is_some_and(|s| s != 0.0) { f64::INFINITY } else { 0.2 });
        let tr = g.hopper.list[k].tr.as_mut().unwrap();
        tr.feed = true;
        tr.turret = ti;
        true
    }

    /// js `_fire(e, mode)`: launched from where it got to, with the shooter's scatter
    fn fire(&mut self, phys: &mut PhysicsWorld, fuel: &mut Fuel, e: &hopper::Entry, pass: bool, rng: &mut Mulberry32) {
        let sh = self.cfg["shooter"].clone();
        let b = e.b;
        self.stored.retain(|&x| x != b);
        let exit = self.local_to_world(e.p[0], e.p[1], e.p[2]);
        let ti = e.tr.as_ref().map_or(0, |t| t.turret);
        let twin = !self.tws.is_empty();
        let fixed = sh["type"].as_str() == Some("fixed");
        let mut psi = if fixed { self.yaw + self.aim_offset } else { self.yaw + self.turret_yaws[ti] };
        psi += crate::human_player::gauss(rng) * num(&sh, "yawSigma") * DEG;
        let th = (if twin { self.tws[ti].hood } else { self.hood_deg }) * DEG + crate::human_player::gauss(rng) * num(&sh, "angleSigma") * DEG;
        let v = (if twin { self.tws[ti].fly } else { self.flywheel }) * (1.0 + crate::human_player::gauss(rng) * num(&sh, "speedSigma"));
        let lv = self.velocity_at(exit);
        let vel = [m::cos(psi) * m::cos(th) * v + lv[0], m::sin(th) * v, -m::sin(psi) * m::cos(th) * v + lv[1]];
        let info = LaunchInfo { by: LaunchBy::Robot, alliance: Some(self.alliance), legal: self.last_in_zone, t: Some(self.last_t) };
        fuel.launch(phys, b, exit, vel, LaunchOpts { info: Some(info), ignore_robot: Some(0.35), t: Some(self.last_t), ..Default::default() });
        let drop = num(&sh, "shotDrop");
        if twin { self.tws[ti].fly *= 1.0 - drop; } else { self.flywheel *= 1.0 - drop; }
        self.stats.shots += 1;
        if pass { self.stats.passes += 1; }
    }

    /// js the generic feed at the end of _shooter: twin turrets each at their share of the rate,
    /// a fixed shooter's lanes at random gaps, a single stream on its beat
    pub(crate) fn feed_generic(&mut self, want_shoot: bool, ready: bool, has: bool, dt: f64, rng: &mut Mulberry32) {
        let sh = self.cfg["shooter"].clone();
        let bps = num(&sh, "bps");
        let fixed = sh["type"].as_str() == Some("fixed");
        self.g_mut().feeding = false;
        if self.turrets.len() > 1 {
            let per = self.tws.len() as f64 / bps;
            for i in 0..self.tws.len() {
                self.tws[i].feed_t = m::min(self.tws[i].feed_t + dt, per * 2.0);
                if want_shoot && self.tws[i].ok && has && self.shot.is_some() {
                    while self.tws[i].feed_t >= per && self.start_feed(Some(i), rng) { self.tws[i].feed_t -= per; }
                    self.g_mut().feeding = true;
                }
            }
        } else {
            let period = 1.0 / bps;
            if self.g().next_period == 0.0 { self.g_mut().next_period = period; }
            { let g = self.g_mut(); g.feed_timer = m::min(g.feed_timer + dt, period * 2.0); }
            if want_shoot && ready && has && self.shot.is_some() {
                while self.g().feed_timer >= self.g().next_period && self.start_feed(None, rng) {
                    let np = self.g().next_period;
                    self.g_mut().feed_timer -= np;
                    let next = if fixed { period * (0.5 + rng.random()) } else { period };
                    self.g_mut().next_period = next;
                }
                self.g_mut().feeding = true;
            }
        }
        if self.g().hopper.list.iter().any(|e| e.tr.as_ref().is_some_and(|t| t.feed && t.t < t.tt && !t.held)) { self.g_mut().feeding = true; }
    }

    /// js `_spin(w, dt, t, on, setpoint)` on (fly, holdV, holdT)
    pub(crate) fn spin(&self, fly: &mut f64, hold_v: &mut f64, hold_t: &mut f64, dt: f64, t: f64, on: bool, mut setpoint: f64) {
        let sh = &self.cfg["shooter"];
        let (smax, tau) = (num(sh, "speedMax"), num(sh, "spinTau"));
        if setpoint > 0.0 { *hold_v = setpoint; *hold_t = t; }
        else if on && *hold_v != 0.0 && t - *hold_t < 1.5 { setpoint = *hold_v; }
        let target = m::min(setpoint, smax);
        let err = target - *fly;
        if err > 0.0 { *fly += m::min(err * (1.0 - m::exp(-dt / 0.06)), (smax / (2.0 * tau)) * dt); }
        else { *fly += m::max(err * (1.0 - m::exp(-dt / 0.06)), -(smax / 3.0) * dt); }
    }
    /// js `_flywheelStep(dt, t, on, setpoint)`
    pub(crate) fn flywheel_step(&mut self, dt: f64, t: f64, on: bool, setpoint: f64) {
        let (mut fly, mut hv, mut ht) = (self.flywheel, self.g().spin_hold, self.g().spin_hold_t);
        self.spin(&mut fly, &mut hv, &mut ht, dt, t, on, setpoint);
        self.flywheel = fly;
        let g = self.g_mut();
        g.spin_hold = hv; g.spin_hold_t = ht;
    }

    /// js `_armsNear()`: the TRENCH arms near it, as oriented boxes in its frame
    fn arms_near(&self) -> Vec<ArmBox> {
        let mut out = Vec::new();
        let inv = [-self.quat[0], -self.quat[1], -self.quat[2], self.quat[3]];
        for a in crate::mech::phys8793::trench_arms() {
            if m::abs(a.x - self.pos[0]) > a.hx + 1.0 || m::abs(a.z - self.pos[2]) > a.hz + 1.0 { continue; }
            out.push(ArmBox {
                c: apply_quat([a.x - self.pos[0], a.y - self.pos[1], a.z - self.pos[2]], inv),
                ax: [apply_quat([1.0, 0.0, 0.0], inv), apply_quat([0.0, 1.0, 0.0], inv), apply_quat([0.0, 0.0, 1.0], inv)],
                h: [a.hx, a.hy, a.hz],
                v: apply_quat([-self.vel[0], 0.0, -self.vel[2]], inv),
            });
        }
        out
    }

    /// js `_stepTopColliders()`'s load: a sphere for each held FUEL above the frame's top
    fn step_load_balls(&mut self, phys: &mut PhysicsWorld) {
        let height = self.height;
        let body = self.body;
        let lbg = groups(GROUP::ROBOT, GROUP::ROBOT | GROUP::INTAKE);
        let g = self.generic.as_mut().unwrap();
        let mut n = 0;
        for e in &g.hopper.list {
            if e.tr.is_some() || e.p[1] + R < height - 0.01 { continue; }
            if n >= g.load_balls.len() {
                let c = phys.insert_collider(ColliderBuilder::ball((R + 0.004) as f32).density(0.0).friction(0.05).restitution(0.1).collision_groups(grp(0)), Some(body));
                g.load_balls.push(c);
            }
            let c = g.load_balls[n];
            phys.colliders[c].set_translation_wrt_parent(vf(e.p[0], e.p[1], e.p[2]));
            phys.colliders[c].set_collision_groups(grp(lbg));
            n += 1;
        }
        for k in n..g.load_balls.len() { phys.colliders[g.load_balls[k]].set_collision_groups(grp(0)); }
    }

    /// js `_stepHopper(dt)`: the load's top, the top colliders, the load stepped in the robot's frame
    /// (feel its acceleration and turning), the FUEL that got where it was going fired or let out,
    /// and the TRENCH arms' push on it back on the robot
    pub(crate) fn step_hopper_generic(&mut self, phys: &mut PhysicsWorld, fuel: &mut Fuel, dt: f64, rng: &mut Mulberry32) {
        let bay = self.cfg["bay"].clone();
        let mut load_top: f64 = 0.0;
        for e in &self.g().hopper.list { if e.tr.is_none() { load_top = m::max(load_top, e.p[1] + R); } }
        let dome = bay.get("dome").is_some_and(|x| x.is_object());
        assert!(bay.get("lift").is_none_or(|x| x.is_null()), "a generic robot's lid on its climber: not ported");
        let fold = fold_top(&self.cfg["intake"], self.intake_deploy);
        let height = self.height;
        {
            let g = self.g_mut();
            g.grow_top = if dome { load_top + 0.005 } else { 0.0 };
            g.top_load = m::max(height, g.grow_top);
            g.load_top = load_top;
            g.load_high_t = if dome && load_top > TRENCH::clearHeight - 0.035 { g.load_high_t + dt } else { 0.0 };
            g.top_stowed = m::max(height, g.grow_top);
            g.top_y = m::max(g.top_load, fold);
        }
        self.step_load_balls(phys);
        self.step_top_colliders(phys);
        let (mut ax, mut az, mut alpha) = (0.0, 0.0, 0.0);
        if let Some(pv) = self.g().prev_vel {
            ax = clamp((self.vel[0] - pv[0]) / dt, -25.0, 25.0);
            az = clamp((self.vel[2] - pv[1]) / dt, -25.0, 25.0);
            alpha = clamp((self.omega - self.g().prev_omega) / dt, -60.0, 60.0);
        }
        let (vel, omega) = (self.vel, self.omega);
        { let g = self.g_mut(); g.prev_vel = Some([vel[0], vel[2]]); g.prev_omega = omega; }
        let qi = [-self.quat[0], -self.quat[1], -self.quat[2], self.quat[3]];
        let acc = apply_quat([ax, 0.0, az], qi);
        let gv = apply_quat([0.0, -9.81, 0.0], qi);
        let f = &bay["feed"];
        let feed_point = self.feed_at(num(f, "x"), 0.0, opt(f, "z").unwrap_or(0.0));
        let front = self.bay_front();
        let boxes = self.arms_near();
        let env = Env {
            acc, g: Some(gv), w: omega, alpha, feeding: self.g().feeding, intaking: self.g().intake_speed > 0.0,
            feed_point: Some(feed_point), feed_converge: self.cfg["shooter"]["type"].as_str() != Some("fixed"), boxes: &boxes,
        };
        // (the transits' gate: the shot on, its turret on target, the trigger held; js _startFeed)
        let gate_ok = self.ready && self.shot.is_some() && self.enabled && (self.cmd.shoot || self.cmd.pass);
        let tws_ok: Vec<bool> = self.tws.iter().map(|w| w.ok).collect();
        let twin = self.turrets.len() > 1;
        let ends: Vec<V> = (0..self.turrets.len()).map(|ti| self.turret_exit_pub(ti)).collect();
        let fx = if self.cfg["shooter"]["type"].as_str() == Some("fixed") { self.fixed_exit_pub(self.hood_deg) } else { [0.0; 2] };
        let mut h = std::mem::replace(&mut self.g_mut().hopper, Hopper::new(Spec::from(&bay)));
        h.front = front;
        let done = h.step(dt, &env,
            &|e: &End| match *e { End::At(v) => v, End::Turret(ti) => ends[ti], End::Fixed(z) => [fx[0], fx[1], z] },
            &|tr| gate_ok && (!twin || tws_ok[tr.turret]));
        self.g_mut().hopper = h;
        let pass = match self.shot { Some(s) => s.pass, None => self.last_pass };
        for e in done {
            match e.tr.as_ref().unwrap().done {
                Done::Fire => self.fire(phys, fuel, &e, pass, rng),
                Done::Outtake => {
                    self.stored.retain(|&x| x != e.b);
                    let p = self.local_to_world(e.p[0], e.p[1], e.p[2]);
                    let fw = self.forward();
                    let v = self.velocity_at(p);
                    let info = LaunchInfo { by: LaunchBy::Robot, alliance: Some(self.alliance), legal: self.in_alliance_zone(self.alliance), t: Some(self.last_t) };
                    fuel.launch(phys, e.b, p, [v[0] + fw[0] * 2.8, 0.6, v[1] + fw[1] * 2.8], LaunchOpts { info: Some(info), ignore_robot: Some(0.3), t: Some(self.last_t), ..Default::default() });
                }
            }
        }
        // (the push a TRENCH arm gave the FUEL comes back on the robot)
        let bf = self.g().hopper.box_force;
        if bf[0] * bf[0] + bf[1] * bf[1] + bf[2] * bf[2] != 0.0 {
            let w = apply_quat(bf, self.quat);
            let w = [w[0] * dt, w[1] * dt, w[2] * dt];
            let at = apply_quat(self.g().hopper.box_at, self.quat);
            let at = [at[0] + self.pos[0], at[1] + self.pos[1], at[2] + self.pos[2]];
            phys.bodies[self.body].apply_impulse_at_point(vf(w[0], w[1], w[2]), vf(at[0], at[1], at[2]), true);
        }
    }

    /// whether it's a climbing state in which nothing guides its captured FUEL
    pub(crate) fn guides(&self) -> bool { !matches!(self.climb_state, ClimbState::Climbing | ClimbState::Hanging | ClimbState::Descending) }
}
