//! js/fuel.js: all 504 FUEL. Each is a rigid body while it's on the FIELD; FUEL in a HUB's
//! funnel stays one too. FUEL a robot holds (emulated), in an OUTPOST CHUTE, or waiting to be
//! returned by field staff has its body disabled. (What's drawn only, the wear and squash, is the
//! app's, later.)
use crate::constants::*;
use crate::field;
use crate::game_match::{ai, LaunchInfo, Match, ALLIANCES};
use crate::rng::Mulberry32;
use crate::world::Outposts;
use jsmath as m;
use rapier3d::prelude::*;

const R: f64 = FUEL::radius;
fn normal_groups() -> u32 { groups(GROUP::BALL, GROUP::STATIC | GROUP::TERRAIN | GROUP::BALL | GROUP::HELD | GROUP::ROBOT | GROUP::INTAKE | GROUP::WHEEL) }
fn no_robot_groups() -> u32 { groups(GROUP::BALL, GROUP::STATIC | GROUP::TERRAIN | GROUP::BALL | GROUP::HELD) }
fn held_groups() -> u32 { groups(GROUP::HELD, GROUP::STATIC | GROUP::TERRAIN | GROUP::BALL) }
fn in_robot_query() -> u32 { groups(GROUP::BALL, GROUP::ROBOT) }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State { Off, Field, Chute, Robot, Out }
impl State {
    /// its name in the JS game (b.state), as the parity digest has it
    pub fn name(self) -> &'static str {
        match self { State::Off => "off", State::Field => "field", State::Chute => "chute", State::Robot => "robot", State::Out => "out" }
    }
}

/// One FUEL (js: an entry of fuel.balls)
#[derive(Debug, Clone)]
pub struct Ball {
    pub id: usize,
    pub body: RigidBodyHandle,
    pub col: ColliderHandle,
    pub state: State,
    pub pos: [f64; 3],
    pub prev_vel: [f64; 3],
    pub in_flight: bool,
    pub launch: Option<LaunchInfo>,
    pub hub_fresh: bool,
    pub ignore_until: f64,
    pub ignoring: bool,
    pub held: bool,
    pub in_corral: Option<Alliance>,
    pub in_hub: Option<Alliance>,
    pub hub_t: f64,
    pub mech_touched: bool,
    /// thrown in from the OUTPOST AREA at this time: not "out" until it has entered
    pub from_outside: Option<f64>,
    pub chute_alliance: Option<Alliance>,
}

/// A FUEL waiting for field staff to bring it back (fuel.outQueue)
#[derive(Debug, Clone, Copy)]
pub struct OutEntry { pub b: usize, pub at: f64, pub x: f64, pub z: f64 }

/// How a launch goes (fuel.launch's info)
#[derive(Debug, Clone, Copy, Default)]
pub struct LaunchOpts { pub info: Option<LaunchInfo>, pub hub_fresh: bool, pub from_outside: bool, pub ignore_robot: Option<f64>, pub t: Option<f64> }

pub struct Fuel {
    pub balls: Vec<Ball>,
    pub out_queue: Vec<OutEntry>,
    pub chute: [Vec<usize>; 2],
    pub outposts: Outposts,
}

fn v3(v: Vector) -> [f64; 3] { [v.x as f64, v.y as f64, v.z as f64] }
fn vf(x: f64, y: f64, z: f64) -> Vector { Vector::new(x as f32, y as f32, z as f32) }

impl Fuel {
    /// js `new FuelManager(...)`: every FUEL's body, made after the field's, disabled
    pub fn new(phys: &mut PhysicsWorld, outposts: Outposts) -> Self {
        let mut balls = Vec::with_capacity(FUEL::total as usize);
        for i in 0..FUEL::total as usize {
            let body = phys.insert_body(RigidBodyBuilder::dynamic()
                .translation(vf(0.0, -10.0 - i as f64 * 0.2, 0.0))
                .ccd_enabled(true)
                .angular_damping(FUEL::angularDamping as f32)
                .can_sleep(true));
            let col = phys.insert_collider(ColliderBuilder::ball(R as f32)
                .mass(FUEL::mass as f32)
                .restitution(FUEL::restitution as f32)
                .friction(FUEL::friction as f32)
                .collision_groups(crate::groups(normal_groups())), Some(body));
            phys.bodies[body].set_enabled(false);
            balls.push(Ball { id: i, body, col, state: State::Off, pos: [0.0, -10.0, 0.0], prev_vel: [0.0; 3], in_flight: false, launch: None,
                hub_fresh: false, ignore_until: 0.0, ignoring: false, held: false, in_corral: None, in_hub: None, hub_t: 0.0,
                mech_touched: false, from_outside: None, chute_alliance: None });
        }
        Fuel { balls, out_queue: Vec::new(), chute: [Vec::new(), Vec::new()], outposts }
    }

    /// js `stage(preload)`: the FUEL where a match starts them; returns the ones preloaded
    pub fn stage(&mut self, phys: &mut PhysicsWorld, preload: usize, rng: &mut Mulberry32) -> Vec<usize> {
        self.out_queue.clear();
        for i in 0..self.balls.len() { self.disable(phys, i, State::Off); }
        let mut i = 0;
        let neutral = m::min(FUEL::neutralMax, FUEL::total - 2.0 * FUEL::perDepot - 2.0 * FUEL::perChute - preload as f64) as usize;
        for s in field::neutral_slots(neutral) { self.place(phys, i, s.x, s.y, s.z); i += 1; }
        for a in ALLIANCES { for s in field::depot_slots(a) { self.place(phys, i, s.x, s.y, s.z); i += 1; } }
        self.chute = [Vec::new(), Vec::new()];
        for a in ALLIANCES {
            for _ in 0..FUEL::perChute as usize {
                self.balls[i].state = State::Chute;
                self.balls[i].chute_alliance = Some(a);
                self.chute[ai(a)].push(i);
                i += 1;
            }
        }
        let mut pre = Vec::new();
        for _ in 0..preload { self.balls[i].state = State::Robot; pre.push(i); i += 1; }
        // any leftovers (only if counts change) go to the NEUTRAL ZONE edges
        while i < self.balls.len() {
            let x = rand(rng, -0.5, 0.5);
            let z = rand(rng, -3.4, 3.4);
            self.place(phys, i, x, R + 0.01, z);
            i += 1;
        }
        pre
    }

    fn place(&mut self, phys: &mut PhysicsWorld, i: usize, x: f64, y: f64, z: f64) {
        let b = &mut self.balls[i];
        b.state = State::Field;
        b.in_flight = false;
        b.launch = None;
        b.hub_fresh = false;
        b.in_corral = None;
        if b.in_hub.is_some() { b.in_hub = None; phys.colliders[b.col].set_friction(FUEL::friction as f32); }
        let body = &mut phys.bodies[b.body];
        body.set_enabled(true);
        body.set_translation(vf(x, y, z), false);
        body.set_linvel(vf(0.0, 0.0, 0.0), false);
        body.set_angvel(vf(0.0, 0.0, 0.0), false);
        body.set_rotation(Rotation::IDENTITY, false);
        phys.colliders[b.col].set_collision_groups(crate::groups(normal_groups()));
        b.held = false;
        b.ignoring = false;
        phys.bodies[b.body].sleep();
        b.pos = [x, y, z];
        b.prev_vel = [0.0; 3];
    }

    fn disable(&mut self, phys: &mut PhysicsWorld, i: usize, state: State) {
        let b = &mut self.balls[i];
        b.state = state;
        phys.bodies[b.body].set_linvel(vf(0.0, 0.0, 0.0), false);
        phys.bodies[b.body].set_enabled(false);
        b.in_flight = false;
        b.in_corral = None;
        if b.in_hub.is_some() { b.in_hub = None; phys.colliders[b.col].set_friction(FUEL::friction as f32); }
    }

    /// js `toRobot(b)`
    pub fn to_robot(&mut self, phys: &mut PhysicsWorld, i: usize) {
        self.disable(phys, i, State::Robot);
        self.balls[i].hub_fresh = false;
        self.balls[i].launch = None;
    }

    /// js `launch(b, pos, vel, info)`: a FUEL put into play at pos moving at vel
    pub fn launch(&mut self, phys: &mut PhysicsWorld, i: usize, pos: [f64; 3], vel: [f64; 3], o: LaunchOpts) {
        let b = &mut self.balls[i];
        b.state = State::Field;
        let body = &mut phys.bodies[b.body];
        body.set_enabled(true);
        body.set_translation(vf(pos[0], pos[1], pos[2]), true);
        body.set_linvel(vf(vel[0], vel[1], vel[2]), true);
        body.set_angvel(vf(0.0, 0.0, 0.0), true);
        body.wake_up(true);
        b.pos = pos;
        b.prev_vel = vel;
        b.in_flight = true;
        b.hub_fresh = o.hub_fresh;
        b.in_corral = None;
        if b.in_hub.is_some() { b.in_hub = None; phys.colliders[b.col].set_friction(FUEL::friction as f32); }
        b.launch = o.info;
        b.from_outside = if o.from_outside { Some(o.t.unwrap_or(0.0)) } else { None };
        if let Some(ig) = o.ignore_robot.filter(|x| *x != 0.0) {
            phys.colliders[b.col].set_collision_groups(crate::groups(no_robot_groups()));
            b.held = false;
            b.ignoring = true;
            b.ignore_until = o.t.unwrap_or(0.0) + ig;
        } else {
            phys.colliders[b.col].set_collision_groups(crate::groups(normal_groups()));
            b.held = false;
            b.ignoring = false;
        }
    }

    /// js `own(b, t)`: a robot's physical ball path has hold of it
    pub fn own(&mut self, phys: &mut PhysicsWorld, i: usize, t: f64) {
        let b = &mut self.balls[i];
        if !b.held { phys.colliders[b.col].set_collision_groups(crate::groups(held_groups())); }
        b.held = true;
        b.ignoring = true;
        b.ignore_until = t + 0.05;
        b.in_flight = false;
        b.launch = None;
        b.hub_fresh = false;
    }
    /// js `unown(b, solid)`
    pub fn unown(&mut self, phys: &mut PhysicsWorld, i: usize, solid: bool) {
        let b = &mut self.balls[i];
        if !b.held { return; }
        b.held = false;
        if solid {
            phys.colliders[b.col].set_collision_groups(crate::groups(normal_groups()));
            b.ignoring = false;
        } else {
            phys.colliders[b.col].set_collision_groups(crate::groups(no_robot_groups()));
        }
    }
    /// js `release(b, info)`: one a robot has just shot out (info.t; ignore_robot defaults to 0.35)
    pub fn release(&mut self, phys: &mut PhysicsWorld, i: usize, info: LaunchInfo, ignore_robot: Option<f64>) {
        self.unown(phys, i, false);
        let b = &mut self.balls[i];
        b.prev_vel = v3(phys.bodies[b.body].linvel());
        b.in_flight = true;
        b.launch = Some(info);
        b.ignore_until = info.t.unwrap_or(0.0) + ignore_robot.unwrap_or(0.35);
    }

    /// js `preStep(dt)`: air drag and the Magnus lift, the HUB's push, the carpet's roll
    pub fn pre_step(&mut self, phys: &mut PhysicsWorld, dt: f64) {
        let k = FUEL::dragK;
        for b in &self.balls {
            if b.state != State::Field { continue; }
            let body = &mut phys.bodies[b.body];
            if body.is_sleeping() { continue; }
            let v = v3(body.linvel());
            let (mut vx, mut vy, mut vz) = (v[0], v[1], v[2]);
            let sp = m::hypot3(vx, vy, vz);
            if sp > 0.3 {
                if !b.held {
                    let a = crate::ballistics::magnus(v, v3(body.angvel()));
                    vx += a[0] * dt; vy += a[1] * dt; vz += a[2] * dt;
                }
                let f = m::max(0.0, 1.0 - k * sp * dt);
                vx *= f; vy *= f; vz *= f;
            }
            if b.in_hub.is_some() { vy -= HUB::extraGravity * dt; }
            let grounded = b.pos[1] < R + 0.012 && m::abs(vy) < 0.35;
            if grounded {
                let hs = m::hypot2(vx, vz);
                let dec = FUEL::rollDecel * dt;
                let f = if hs <= dec + 0.01 { 0.0 } else { (hs - dec) / hs };
                vx *= f; vz *= f;
                let w = v3(body.angvel());
                body.set_angvel(vf(w[0] * f, w[1] * f, w[2] * f), false);
            }
            body.set_linvel(vf(vx, vy, vz), false);
        }
    }

    /// js `postStep(dt, t)`
    pub fn post_step(&mut self, phys: &mut PhysicsWorld, dt: f64, t: f64) {
        for i in 0..self.balls.len() {
            if self.balls[i].state != State::Field { continue; }
            let bh = self.balls[i].body;
            if phys.bodies[bh].is_sleeping() { continue; }
            let p = v3(phys.bodies[bh].translation());
            let v = v3(phys.bodies[bh].linvel());
            let b = &mut self.balls[i];
            b.pos = p;
            let (ex, ey, ez) = (b.prev_vel[0], b.prev_vel[1] - 9.81 * dt, b.prev_vel[2]);
            // (a hard knock wears its skin: drawn only, the app's)
            if b.mech_touched { b.mech_touched = false; }
            if b.in_flight || b.hub_fresh {
                let dev = m::hypot3(v[0] - ex, v[1] - ey, v[2] - ez);
                let sp = m::hypot3(v[0], v[1], v[2]);
                if dev > 0.35 + 0.03 * sp * sp * dt * 60.0 || p[1] < R + 0.015 {
                    b.in_flight = false;
                    b.hub_fresh = false;
                }
            }
            b.prev_vel = v;
            if b.ignoring && (t >= b.ignore_until || (!b.held && v[1] < -0.5)) {
                if in_robot(phys, p) { self.balls[i].ignore_until = t + 0.1; }
                else {
                    let b = &mut self.balls[i];
                    phys.colliders[b.col].set_collision_groups(crate::groups(normal_groups()));
                    b.held = false;
                    b.ignoring = false;
                }
            }
        }
    }

    /// js `update(dt, t)`: per frame, the HUBS, the CORRALS, FUEL leaving the FIELD and coming back
    pub fn update(&mut self, phys: &mut PhysicsWorld, _dt: f64, t: f64, m_: &mut Match, rng: &mut Mulberry32) {
        for i in 0..self.balls.len() {
            if self.balls[i].state != State::Field { continue; }
            let p = self.balls[i].pos;
            if let Some(hub) = self.balls[i].in_hub {
                let (cx, _) = field::hub_center(hub);
                let s = if hub == Alliance::Blue { 1.0 } else { -1.0 };
                let u = s * (p[0] - cx);
                if u > HUB::size / 2.0 + 0.1 + R * 0.6 || p[1] < HUB::exitHeight - 0.25 {
                    let b = &mut self.balls[i];
                    b.in_hub = None;
                    b.hub_fresh = true;
                    phys.colliders[b.col].set_friction(FUEL::friction as f32);
                    b.prev_vel = v3(phys.bodies[b.body].linvel());
                } else {
                    phys.bodies[self.balls[i].body].wake_up(true); // never parked inside: the load keeps flowing out
                    continue;
                }
            } else if p[1] < HUB::funnelBottomY && p[1] > HUB::exitHeight - 0.05 {
                for a in ALLIANCES {
                    let (cx, cz) = field::hub_center(a);
                    if m::abs(p[0] - cx) < HUB::size / 2.0 - 0.05 && m::abs(p[2] - cz) < HUB::size / 2.0 - 0.05 {
                        self.enter_hub(phys, i, a, t, m_);
                        break;
                    }
                }
                if self.balls[i].in_hub.is_some() { continue; }
            }
            // Corral (behind the OUTPOST base opening): still on the carpet but off the FIELD
            self.balls[i].in_corral = None;
            if m::abs(p[0]) > HALF_L + 0.02 {
                for a in ALLIANCES {
                    let c = if a == Alliance::Blue { self.outposts.blue.corral } else { self.outposts.red.corral };
                    if p[0] > c.x0 - 0.05 && p[0] < c.x1 + 0.05 && p[2] > c.z0 - 0.05 && p[2] < c.z1 + 0.05 && p[1] < 0.5 { self.balls[i].in_corral = Some(a); }
                }
            }
            if self.balls[i].in_corral.is_some() { continue; }
            if let Some(fo) = self.balls[i].from_outside {
                let inside = m::abs(p[0]) < HALF_L && m::abs(p[2]) < HALF_W;
                if inside { self.balls[i].from_outside = None; } else if t - fo < 3.0 { continue; }
            }
            // left the FIELD
            let out = m::abs(p[0]) > HALF_L + 0.08 || m::abs(p[2]) > HALF_W + 0.08 || p[1] < -0.5;
            if out {
                let rx = clamp(p[0], -HALF_L + 0.35, HALF_L - 0.35);
                let rz = clamp(p[2], -HALF_W + 0.35, HALF_W - 0.35);
                self.disable(phys, i, State::Out);
                let at = t + rand(rng, 3.0, 6.0);
                self.out_queue.push(OutEntry { b: i, at, x: rx, z: rz });
            }
        }
        // field staff returning FUEL that left the FIELD
        let mut i = self.out_queue.len();
        while i > 0 {
            i -= 1;
            let q = self.out_queue[i];
            if t < q.at { continue; }
            let at = [0.0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5, 2.0, -2.0].iter()
                .map(|d| (q.x, clamp(q.z + d, -HALF_W + 0.35, HALF_W - 0.35)))
                .find(|&(x, z)| clear_of_robots(phys, x, z));
            let Some((x, z)) = at else { self.out_queue[i].at = t + 0.5; continue; };
            self.out_queue.remove(i);
            self.launch(phys, q.b, [x, R + 0.02, z], [0.0; 3], LaunchOpts::default());
            self.balls[q.b].in_flight = false;
        }
    }

    fn enter_hub(&mut self, phys: &mut PhysicsWorld, i: usize, a: Alliance, t: f64, m_: &mut Match) {
        m_.fuel_entered_hub(a, self.balls[i].launch);
        let b = &mut self.balls[i];
        b.in_hub = Some(a);
        b.hub_t = t;
        phys.colliders[b.col].set_friction(HUB::fuelFriction as f32);
        b.launch = None;
        b.in_flight = false;
    }

    pub fn chute_count(&self, a: Alliance) -> usize { self.chute[ai(a)].len() }
    /// js `takeFromChute(a)`
    pub fn take_from_chute(&mut self, a: Alliance) -> Option<usize> {
        let c = &mut self.chute[ai(a)];
        if c.is_empty() { None } else { Some(c.remove(0)) }
    }
    /// js `addToChute(a, b)`
    pub fn add_to_chute(&mut self, phys: &mut PhysicsWorld, a: Alliance, i: usize) {
        self.disable(phys, i, State::Chute);
        self.balls[i].chute_alliance = Some(a);
        self.chute[ai(a)].push(i);
    }
    /// js `corralBalls(a)`
    pub fn corral_balls(&self, a: Alliance) -> Vec<usize> {
        self.balls.iter().filter(|b| b.state == State::Field && b.in_corral == Some(a)).map(|b| b.id).collect()
    }
}

/// js `rand(lo, hi)` (util.js)
pub fn rand(rng: &mut Mulberry32, lo: f64, hi: f64) -> f64 { lo + rng.random() * (hi - lo) }
/// js `clamp(v, lo, hi)` (util.js)
pub fn clamp(v: f64, lo: f64, hi: f64) -> f64 { if v < lo { lo } else if v > hi { hi } else { v } }

/// js `_inRobot(p)`: whether a FUEL at p is well inside a robot's bumpers or frame (more than ~3 cm in)
fn in_robot(phys: &PhysicsWorld, p: [f64; 3]) -> bool {
    let shape = rapier3d::geometry::Ball::new((0.6 * R) as f32);
    let pose = Pose::from_translation(vf(p[0], p[1], p[2]));
    phys.intersect_shape(pose, &shape, QueryFilter::default().groups(crate::groups(in_robot_query()))).next().is_some()
}
/// js `_clearOfRobots({x, z})`: whether a FUEL put down at (x, z) would be clear of every robot
fn clear_of_robots(phys: &PhysicsWorld, x: f64, z: f64) -> bool {
    let shape = rapier3d::geometry::Ball::new((R + 0.45) as f32);
    let pose = Pose::from_translation(vf(x, R + 0.02, z));
    phys.intersect_shape(pose, &shape, QueryFilter::default().groups(crate::groups(in_robot_query()))).next().is_none()
}
