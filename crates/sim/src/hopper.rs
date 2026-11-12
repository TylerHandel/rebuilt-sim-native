//! js hopper.js: FUEL inside a generic robot (one not built of physical mechanisms). Held FUEL is
//! simulated in the robot's own frame with a light position-based solver instead of rigid bodies:
//! gravity, the robot's acceleration and rotation (the load sloshes), soft ball-to-ball contact
//! (the foam's spring, fuel_force), the hopper's walls, floor, nets and parts, and what the
//! mechanisms do to it (a powered floor, a rotor, a belt). FUEL on its way to the shooter (or back
//! out through the intake) follows its path.
//!
//! Frame: x forward, y up, z to the side, origin at the robot center on the carpet. Bit for bit as
//! the JS: three.js's Vector3 arithmetic on f64s (divideScalar is a multiply by the reciprocal).

use jsmath as m;
use serde_json::Value;

use crate::constants::FUEL;

pub type V = [f64; 3];

const R: f64 = FUEL::radius;
const D_BALL: f64 = 2.0 * R;
const X_MAX: f64 = 3.0 * 0.0254;
const R_WALL: f64 = R * 0.96;
const G: f64 = 9.81;
const ITER: usize = 3;
const MU_FUEL: f64 = 0.5;
const FRICTION_FN: f64 = 20.0;
const MAX_SPEED: f64 = 5.0;
const MAX_TOUCH: usize = 4;
const ARM_FRICTION: f64 = 0.35;

/// js `fuelForce(x)`: the force (N) a FUEL squeezed by x (m) pushes back with
pub fn fuel_force(x: f64) -> f64 {
    if x <= 0.0 { 0.0 } else { FUEL::springRate0 * FUEL::stiffen * (m::exp(m::min(x, X_MAX) / FUEL::stiffen) - 1.0) }
}
fn fuel_stiffness(x: f64) -> f64 { FUEL::springRate0 * m::exp(m::min(m::max(0.0, x), X_MAX) / FUEL::stiffen) }

fn clamp(v: f64, lo: f64, hi: f64) -> f64 { crate::fuel::clamp(v, lo, hi) }
fn dist(a: V, b: V) -> f64 { let (x, y, z) = (a[0] - b[0], a[1] - b[1], a[2] - b[2]); m::sqrt(x * x + y * y + z * z) }
fn len(a: V) -> f64 { m::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) }
fn dot(a: V, b: V) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
fn n(v: &Value, k: &str) -> f64 { v[k].as_f64().unwrap_or_else(|| panic!("bay: no number {k}")) }
fn o(v: &Value, k: &str) -> Option<f64> { v.get(k).and_then(|x| x.as_f64()) }
fn obj<'a>(v: &'a Value, k: &str) -> Option<&'a Value> { v.get(k).filter(|x| x.is_object()) }
fn pts2(v: &Value) -> Vec<[f64; 2]> { v.as_array().unwrap().iter().map(|p| [p[0].as_f64().unwrap(), p[1].as_f64().unwrap()]).collect() }

/// js `pwl(pts, x)`: piecewise linear through [x, y] points, flat past the ends
fn pwl(pts: &[[f64; 2]], x: f64) -> f64 {
    if x <= pts[0][0] { return pts[0][1]; }
    for i in 1..pts.len() {
        let [x1, y1] = pts[i];
        if x <= x1 { let [x0, y0] = pts[i - 1]; return y0 + ((y1 - y0) * (x - x0)) / (x1 - x0); }
    }
    pts[pts.len() - 1][1]
}

/// js `touch(e, nx, ny, nz, w)`: a contact patch, the deepest MAX_TOUCH kept
fn touch(t: &mut [f32; 16], nt: &mut usize, nx: f64, ny: f64, nz: f64, w: f64) {
    let mut i = *nt;
    if i >= MAX_TOUCH {
        let mut k = 0;
        for j in 1..MAX_TOUCH { if t[4 * j + 3] > t[4 * k + 3] { k = j; } }
        if (t[4 * k + 3] as f64) <= w { return; }
        i = k;
    } else { *nt += 1; }
    t[4 * i] = nx as f32; t[4 * i + 1] = ny as f32; t[4 * i + 2] = nz as f32; t[4 * i + 3] = w as f32;
}

/// A rotor under the load (js spec.rotor): a Dye Rotor or a spindexer
#[derive(Clone, Debug)]
pub struct Rotor { pub x: f64, pub z: f64, pub y: f64, pub r: f64, pub spin: f64, pub idle: f64, pub dir: f64, pub fins: f64, pub fin_r0: f64, pub sweep: f64, pub grip: f64, pub drag: f64, pub pull: f64 }

/// A post or a box in the load's way (js spec.obstacles)
#[derive(Clone, Debug)]
pub enum Obstacle { Post { x: f64, z: f64, r: f64, y0: f64, y1: f64 }, Box([f64; 6]) }

/// js the hopper's spec (cfg.bay), as far as the generic robots use it
#[derive(Clone, Debug)]
pub struct Spec {
    pub x0: f64, pub x1: f64, pub hw: f64, pub top: f64,
    floor_pts: Option<Vec<[f64; 2]>>, floor_ab: [f64; 4],
    funnel: Option<(f64, f64)>,
    taper: Option<(Vec<[f64; 2]>, Option<f64>, f64)>,
    above: Option<f64>,
    pub lift_h: Option<f64>,
    slope: Option<(f64, f64)>,
    pub dome: Option<[f64; 5]>,
    bevel: Option<(f64, f64)>,
    round: Option<(f64, f64)>,
    chamfer: Option<f64>,
    obstacles: Vec<Obstacle>,
    pub rotors: Vec<Rotor>,
    pub drive: String,
    drive_speed: f64,
    grip: Option<f64>,
    feed_pull: Option<f64>,
    feed_reach_pull: f64,
}

impl Spec {
    pub fn from(b: &Value) -> Spec {
        let f = &b["floor"];
        let rotor = |r: &Value| Rotor {
            x: n(r, "x"), z: n(r, "z"), y: o(r, "y").unwrap_or(0.0), r: n(r, "r"), spin: o(r, "spin").unwrap_or(0.0), idle: o(r, "idle").unwrap_or(f64::NAN),
            dir: o(r, "dir").unwrap_or(1.0), fins: o(r, "fins").unwrap_or(1.0), fin_r0: o(r, "finR0").unwrap_or(0.0), sweep: o(r, "sweep").unwrap_or(0.0),
            grip: o(r, "grip").unwrap_or(40.0), drag: o(r, "drag").unwrap_or(0.0), pull: o(r, "pull").unwrap_or(0.0),
        };
        let rotors = match (b.get("rotors").and_then(|x| x.as_array()), obj(b, "rotor")) {
            (Some(rs), _) => rs.iter().map(rotor).collect(),
            (None, Some(r)) => vec![rotor(r)],
            _ => vec![],
        };
        assert!(b.get("hook").is_none_or(|x| x.is_null()), "a rotor's hook: not ported");
        assert!(b.get("ramp").is_none_or(|x| x.is_null()), "an intake box's ramp: not ported");
        let obstacles = b.get("obstacles").and_then(|x| x.as_array()).map(|a| a.iter().map(|q| match q.get("box").and_then(|x| x.as_array()) {
            Some(bx) => { let v: Vec<f64> = bx.iter().map(|x| x.as_f64().unwrap()).collect(); Obstacle::Box([v[0], v[1], v[2], v[3], v[4], v[5]]) }
            None => Obstacle::Post { x: n(q, "x"), z: n(q, "z"), r: n(q, "r"), y0: n(q, "y0"), y1: n(q, "y1") },
        }).collect()).unwrap_or_default();
        let feed = &b["feed"];
        Spec {
            x0: n(b, "x0"), x1: n(b, "x1"), hw: n(b, "hw"), top: o(b, "top").unwrap_or(0.0),
            floor_pts: f.get("pts").filter(|x| x.is_array()).map(pts2),
            floor_ab: [o(f, "a").unwrap_or(0.0), o(f, "b").unwrap_or(0.0), o(f, "lo").unwrap_or(0.0), o(f, "hi").unwrap_or(0.0)],
            funnel: obj(b, "funnel").map(|x| (n(x, "slope"), n(x, "cap"))),
            taper: obj(b, "taper").map(|t| (pts2(&t["pts"]), o(t, "spin"), o(t, "center").unwrap_or(0.0))),
            above: o(b, "above"),
            lift_h: obj(b, "lift").map(|l| n(l, "h")),
            slope: obj(b, "slope").map(|s| (n(s, "x"), n(s, "y"))),
            dome: obj(b, "dome").map(|d| [n(d, "h"), n(d, "x0"), n(d, "cx"), n(d, "cz"), n(d, "rHole")]),
            bevel: obj(b, "bevel").map(|s| (n(s, "dx"), n(s, "y"))),
            round: obj(b, "round").map(|s| (n(s, "x"), n(s, "r"))),
            chamfer: o(b, "chamfer"),
            obstacles, rotors,
            drive: b["drive"].as_str().unwrap_or("").to_string(),
            drive_speed: o(b, "driveSpeed").unwrap_or(0.0),
            grip: o(b, "grip"),
            feed_pull: o(feed, "pull"),
            feed_reach_pull: o(feed, "reach").unwrap_or(0.14),
        }
    }
}

/// Where a transit's path ends (js tr.end(), which may move): fixed, at a turret's exit, at a fixed
/// shooter's exit in a lane
#[derive(Clone, Copy, Debug)]
pub enum End { At(V), Turret(usize), Fixed(f64) }
/// What happens when a transit gets there (js tr.onDone): it's fired, or let out over the intake
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Done { Fire, Outtake }

/// js e.tr: a FUEL on its way along a path
#[derive(Clone, Debug)]
pub struct Transit {
    pub pts: Vec<V>, pub end: End, pub t: f64, pub tt: f64, pub gated: bool, pub queue: Option<(usize, u64)>, pub held: bool,
    pub feed: bool, pub turret: usize, pub done: Done,
}

/// js a held FUEL (Hopper list entry)
#[derive(Clone, Debug)]
pub struct Entry {
    pub id: u64, pub b: usize, pub p: V, pub v: V, pub prev: V, pub tr: Option<Transit>,
    pub touch: [f32; 16], pub n_touch: usize, pub push: f64, pub push_t: f64, pub ext: Option<V>,
}

/// js `env` of Hopper.step
pub struct Env<'a> {
    pub acc: V, pub g: Option<V>, pub w: f64, pub alpha: f64, pub feeding: bool, pub intaking: bool,
    pub feed_point: Option<V>, pub feed_converge: bool, pub boxes: &'a [ArmBox],
}

/// js an oriented box of _armsNear (robot frame): center, axes, half sizes, velocity
#[derive(Clone, Copy, Debug)]
pub struct ArmBox { pub c: V, pub ax: [V; 3], pub h: [f64; 3], pub v: V }

/// js `pushOutOfBoxes(p, boxes, r, step)`: how far p moved out of them, if it did
fn push_out_of_boxes(p: &mut V, boxes: &[ArmBox], r: f64, step: f64) -> Option<V> {
    let mut moved: Option<V> = None;
    for b in boxes {
        let d = [p[0] - b.c[0], p[1] - b.c[1], p[2] - b.c[2]];
        let l = [dot(d, b.ax[0]), dot(d, b.ax[1]), dot(d, b.ax[2])];
        if m::abs(l[0]) > b.h[0] + r || m::abs(l[1]) > b.h[1] + r || m::abs(l[2]) > b.h[2] + r { continue; }
        let q = [clamp(l[0], -b.h[0], b.h[0]), clamp(l[1], -b.h[1], b.h[1]), clamp(l[2], -b.h[2], b.h[2])];
        let (dx, dy, dz) = (l[0] - q[0], l[1] - q[1], l[2] - q[2]);
        let dd = m::hypot3(dx, dy, dz);
        let mv;
        if dd > 1e-6 {
            if dd >= r { continue; }
            let k = (r - dd) / dd;
            mv = [dx * k, dy * k, dz * k];
        } else {
            let vin = -dot(b.v, b.ax[0]);
            let side = if vin != 0.0 { -m::sign(vin) } else { let s = m::sign(l[0]); if s == 0.0 || s.is_nan() { 1.0 } else { s } };
            let back = b.h[0] + side * l[0] + r;
            let down = l[1] + b.h[1] + r;
            mv = if back < down { [side * back, 0.0, 0.0] } else { [0.0, -down, 0.0] };
        }
        // (as three's addScaledVector, in turn: 0 + ax0 * mv0, then + ax1 * mv1, then + ax2 * mv2)
        let mut q2 = [0.0f64; 3];
        for a in 0..3 { q2[a] = 0.0 + b.ax[0][a] * mv[0]; }
        for a in 0..3 { q2[a] += b.ax[1][a] * mv[1]; }
        for a in 0..3 { q2[a] += b.ax[2][a] * mv[2]; }
        let mut qv = q2;
        if step != 0.0 {
            let ql = len(qv);
            let s = if ql != 0.0 { 1.0 / ql } else { 1.0 };
            let nn = [qv[0] * s, qv[1] * s, qv[2] * s];
            let push = ql;
            let mut t = [b.v[0] * step, b.v[1] * step, b.v[2] * step];
            let tn = dot(t, nn);
            for a in 0..3 { t[a] += nn[a] * -tn; }
            let tl = len(t);
            let k = if tl > 1e-9 { m::min(tl, ARM_FRICTION * push) / tl } else { 0.0 };
            for a in 0..3 { qv[a] += t[a] * k; }
        }
        for a in 0..3 { p[a] += qv[a]; }
        let mut mm = moved.unwrap_or([0.0; 3]);
        for a in 0..3 { mm[a] += qv[a]; }
        moved = Some(mm);
    }
    moved
}

/// js `Hopper`
#[derive(Clone, Debug)]
pub struct Hopper {
    pub spec: Spec,
    pub list: Vec<Entry>,
    pub front: f64,
    pub wall: f64,
    pub pressure: f64,
    pub wall_pressure: f64,
    pub quiet: f64,
    pub fin_angle: f64,
    pub dome_scale: f64,
    pub lift_scale: f64,
    pub box_force: V,
    pub box_at: V,
    next_id: u64,
}

impl Hopper {
    pub fn new(spec: Spec) -> Hopper {
        let front = spec.x1;
        Hopper { spec, list: Vec::new(), front, wall: f64::INFINITY, pressure: 0.0, wall_pressure: 0.0, quiet: 0.0, fin_angle: 0.0, dome_scale: 1.0, lift_scale: 1.0, box_force: [0.0; 3], box_at: [0.0; 3], next_id: 0 }
    }

    pub fn clear(&mut self) { self.list.clear(); }
    pub fn count(&self) -> usize { self.list.len() }

    pub fn floor_at(&self, x: f64, z: f64) -> f64 {
        let s = &self.spec;
        let mut y = match &s.floor_pts { Some(p) => pwl(p, x), None => clamp(s.floor_ab[0] + s.floor_ab[1] * x, s.floor_ab[2], s.floor_ab[3]) };
        if let Some((slope, cap)) = s.funnel {
            let mut d = f64::INFINITY;
            for r in &s.rotors { d = m::min(d, m::hypot2(x - r.x, z - r.z) - r.r); }
            if d > 0.0 { y += m::min(cap, d * slope); }
        }
        y
    }

    pub fn hw_at(&self, x: f64) -> f64 { match &self.spec.taper { Some((p, _, _)) => pwl(p, x), None => self.spec.hw } }

    fn abreast(&self, i: usize) -> bool {
        let p = self.list[i].p;
        if self.hw_at(p[0]) >= 2.0 * R + 0.01 { return false; }
        self.list.iter().enumerate().any(|(j, o)| j != i && o.tr.is_none() && m::abs(o.p[0] - p[0]) < R && o.p[2] * p[2] < 0.0 && m::abs(o.p[2] - p[2]) < 2.2 * R)
    }

    pub fn drop_height(&self, x: f64, z: f64) -> f64 {
        let mut y = self.floor_at(x, z) + R_WALL;
        for e in &self.list {
            if e.tr.is_some() { continue; }
            let (dx, dz) = (e.p[0] - x, e.p[2] - z);
            let h2 = dx * dx + dz * dz;
            if h2 < D_BALL * D_BALL { y = m::max(y, e.p[1] + m::sqrt(D_BALL * D_BALL - h2)); }
        }
        y
    }

    pub fn top_at(&self, x: f64, z: f64) -> f64 {
        let s = &self.spec;
        if let Some(a) = s.above { return self.floor_at(x, 0.0) + a; }
        let mut top = match s.lift_h { Some(h) => s.top + h * self.lift_scale, None => s.top };
        if let Some((sx, sy)) = s.slope { if x > sx { top += (sy - top) * clamp((x - sx) / m::max(0.05, self.front - sx), 0.0, 1.0); } }
        if let Some(d) = s.dome { return top + d[0] * self.dome_scale * self.dome_shape(x, z); }
        if let Some((dx, by)) = s.bevel { if x > self.front - dx { top += ((by - top) * (x - self.front + dx)) / dx; } }
        top
    }

    pub fn dome_shape(&self, x: f64, z: f64) -> f64 {
        let d = self.spec.dome.unwrap();
        let hw = self.spec.hw;
        let u = (x - d[1]) / m::max(0.05, self.front - d[1]);
        let v = (z + hw) / (2.0 * hw);
        if u <= 0.0 || u >= 1.0 || v <= 0.0 || v >= 1.0 { return 0.0; }
        let r = m::hypot2(x - d[2], z - d[3]);
        let hole = clamp((r - d[4]) / 0.14, 0.0, 1.0);
        m::sin(m::PI * u) * m::sin(m::PI * v) * hole * hole * (3.0 - 2.0 * hole)
    }

    pub fn add(&mut self, b: usize, p: V, v: V) -> usize {
        self.next_id += 1;
        self.list.push(Entry { id: self.next_id, b, p, v, prev: [0.0; 3], tr: None, touch: [0.0; 16], n_touch: 0, push: 0.0, push_t: 0.0, ext: None });
        self.quiet = 0.0;
        self.list.len() - 1
    }

    pub fn remove(&mut self, id: u64) -> Option<Entry> {
        let i = self.list.iter().position(|e| e.id == id)?;
        self.quiet = 0.0;
        Some(self.list.remove(i))
    }

    /// js `fill(balls, rng)`: preload, dropped in loosely and let settle
    pub fn fill(&mut self, balls: &[usize], rng: &mut dyn FnMut() -> f64) {
        let s = self.spec.clone();
        let step = 2.0 * R * 0.97;
        let mut slots: Vec<V> = Vec::new();
        for layer in 0..8 {
            let mut x = s.x0 + R;
            while x <= self.front - R + 1e-6 {
                let hw = self.hw_at(x);
                let mut z = -hw + R;
                while z <= hw - R + 1e-6 {
                    let y = self.floor_at(x, z) + R + layer as f64 * step;
                    if y <= self.top_at(x, z) - R + 0.02 { slots.push([x, y, z]); }
                    z += step;
                }
                x += step;
            }
        }
        for (i, &b) in balls.iter().enumerate() {
            let mut p = slots.get(i).or(slots.last()).copied().unwrap_or([0.0, 0.3, 0.0]);
            p[0] += (rng() - 0.5) * 0.02;
            p[2] += (rng() - 0.5) * 0.02;
            self.add(b, p, [0.0; 3]);
        }
        let env = Env { acc: [0.0; 3], g: None, w: 0.0, alpha: 0.0, feeding: false, intaking: false, feed_point: None, feed_converge: false, boxes: &[] };
        for _ in 0..30 { self.step(1.0 / 120.0, &env, &|_| [0.0; 3], &|_| true); }
    }

    /// js `startTransit(e, via, end, speed, onDone, opts)`
    #[allow(clippy::too_many_arguments)]
    pub fn start_transit(&mut self, i: usize, via: &[V], end: End, end_now: V, speed: f64, done: Done, gated: bool, queue: Option<(usize, u64)>, max_t: f64) {
        let e = &self.list[i];
        let mut pts = vec![e.p];
        pts.extend_from_slice(via);
        let mut l = 0.0;
        let mut prev = pts[0];
        for q in pts[1..].iter().copied().chain([end_now]) { l += dist(prev, q); prev = q; }
        let tt = clamp(l / speed, 0.04, max_t);
        self.list[i].tr = Some(Transit { pts, end, t: 0.0, tt, gated, queue, held: false, feed: false, turret: 0, done });
        self.quiet = 0.0;
    }

    /// js `_transitLen(tr)`: the path's length now, and from its last via point to its end
    fn transit_len(tr: &Transit, end: V) -> (f64, f64) {
        let mut pts = tr.pts.clone();
        pts.push(end);
        let mut l = 0.0;
        for i in 1..pts.len() { l += dist(pts[i - 1], pts[i]); }
        (l, dist(pts[pts.len() - 2], pts[pts.len() - 1]))
    }
    /// js `traveled(tr)`
    pub fn traveled(tr: &Transit, end: V) -> f64 { Self::transit_len(tr, end).0 * ease_s(m::min(1.0, tr.t / tr.tt)) }

    fn transit_pos(tr: &Transit, end: V) -> V {
        let mut pts = tr.pts.clone();
        pts.push(end);
        let mut segs = Vec::new();
        let mut l = 0.0;
        for i in 1..pts.len() { let d = dist(pts[i - 1], pts[i]); segs.push(d); l += d; }
        let u = m::min(1.0, tr.t / tr.tt);
        let mut s = l * ease_s(u);
        for i in 0..segs.len() {
            if s <= segs[i] || i == segs.len() - 1 {
                let a = if segs[i] > 0.0 { m::min(1.0, s / segs[i]) } else { 1.0 };
                let (p, q) = (pts[i], pts[i + 1]);
                return [p[0] + (q[0] - p[0]) * a, p[1] + (q[1] - p[1]) * a, p[2] + (q[2] - p[2]) * a];
            }
            s -= segs[i];
        }
        pts[pts.len() - 1]
    }

    /// js `step(dt, env)`. end: where a transit's End is now; gate: whether a gated transit may go on.
    /// Returns the FUEL whose transits got there this step (removed from the load, in order: js onDone)
    pub fn step(&mut self, dt: f64, env: &Env, end: &dyn Fn(&End) -> V, gate: &dyn Fn(&Transit) -> bool) -> Vec<Entry> {
        // transits first, the one nearest its end first: a gated one that may not go on waits at its
        // last via point, and the ones behind it on the same queue a ball apart
        let mut moving: Vec<(usize, Option<(f64, f64)>, f64)> = Vec::new();
        for (i, e) in self.list.iter().enumerate() {
            let Some(tr) = &e.tr else { continue };
            let l = if tr.gated { Some(Self::transit_len(tr, end(&tr.end))) } else { None };
            let left = match l { Some((ln, _)) => ln * (1.0 - ease_s(m::min(1.0, tr.t / tr.tt))), None => 0.0 };
            moving.push((i, l, left));
        }
        moving.sort_by(|a, b| (a.2 - b.2).partial_cmp(&0.0).unwrap_or(std::cmp::Ordering::Equal));
        let mut queue_end: Vec<((usize, u64), f64)> = Vec::new();
        let mut done_ids: Vec<u64> = Vec::new();
        for (i, l, left) in moving {
            let ok = { let tr = self.list[i].tr.as_ref().unwrap(); !l.is_some() || gate(tr) };
            let e = &mut self.list[i];
            let tr = e.tr.as_mut().unwrap();
            let mut t = m::min(tr.tt, tr.t + dt);
            tr.held = false;
            if let Some((ln, last)) = l {
                if !ok {
                    let ahead = queue_end.iter().find(|(q, _)| Some(*q) == tr.queue).map(|(_, a)| *a);
                    let min_left = m::max(last, match ahead { Some(a) => a + D_BALL, None => 0.0 });
                    if left >= last - 1e-6 {
                        let t_stop = if ln > 0.0 { tr.tt * ease_u(m::max(0.0, 1.0 - min_left / ln)) } else { 0.0 };
                        if t > t_stop { t = m::max(tr.t, t_stop); tr.held = true; }
                    }
                }
            }
            tr.t = t;
            if let Some((ln, _)) = l {
                let v = ln * (1.0 - ease_s(m::min(1.0, t / tr.tt)));
                let q = tr.queue;
                match queue_end.iter_mut().find(|(k, _)| Some(*k) == q) { Some(x) => x.1 = v, None => if let Some(q) = q { queue_end.push((q, v)); } }
            }
            e.prev = e.p;
            let tr = e.tr.as_ref().unwrap();
            e.p = Self::transit_pos(tr, end(&tr.end));
            let k = 1.0 / dt;
            e.v = [(e.p[0] - e.prev[0]) * k, (e.p[1] - e.prev[1]) * k, (e.p[2] - e.prev[2]) * k];
            if tr.t >= tr.tt { done_ids.push(e.id); }
        }
        let mut done = Vec::new();
        for id in done_ids { if let Some(e) = self.remove(id) { done.push(e); } }

        let s = &self.spec;
        let mut spin = 0.0;
        if s.drive == "rotor" {
            spin = if env.feeding { s.rotors[0].spin } else { s.rotors[0].idle };
            self.fin_angle = (self.fin_angle + spin * dt) % (2.0 * m::PI);
        }
        let (w, al) = (env.w, env.alpha);
        let (ax, ay, az) = (env.acc[0], env.acc[1], env.acc[2]);
        let (gx, gy, gz) = match env.g { Some(g) => (g[0], g[1], g[2]), None => (0.0, -G, 0.0) };
        let running = env.feeding || env.intaking;
        if env.intaking && self.list.iter().any(|e| e.push != 0.0) { self.quiet = 0.0; }
        let still = m::abs(w) < 0.05 && m::hypot2(ax, az) < 0.3 && !running && (spin == 0.0 || spin.is_nan()) && !self.list.iter().any(|e| e.tr.as_ref().is_some_and(|t| !t.held));
        if self.list.iter().any(|e| e.ext.is_some()) { self.quiet = 0.0; }
        if still && self.quiet > 0.4 { return done; }

        let mut max_v: f64 = 0.0;
        let nl = self.list.len();
        for i in 0..nl {
            if self.list[i].tr.is_some() { continue; }
            let (p, v) = (self.list[i].p, self.list[i].v);
            let mut fx = gx - ax + w * w * p[0] - 2.0 * w * v[2] - al * p[2];
            let mut fy = gy - ay;
            let mut fz = gz - az + w * w * p[2] + 2.0 * w * v[0] + al * p[0];
            let s = &self.spec;
            let on_floor = p[1] - R_WALL - self.floor_at(p[0], p[2]) < 0.02;
            if s.drive == "floor" && on_floor {
                if env.feeding {
                    let (mut tx, mut tz) = (-s.drive_speed, 0.0);
                    if let (true, Some(fp)) = (env.feed_converge, env.feed_point) {
                        let (dx, dz) = (fp[0] - p[0], fp[2] - p[2]);
                        let d = m::hypot2(dx, dz);
                        if d > 0.03 { tx = (s.drive_speed * dx) / d; tz = (s.drive_speed * dz) / d; }
                    }
                    fx += 12.0 * (tx - v[0]); fz += 12.0 * (tz - v[2]);
                } else if env.intaking { fx += 12.0 * (-0.5 - v[0]); }
            } else if s.drive == "rotor" && on_floor {
                let mut r: Option<&Rotor> = None;
                let (mut dx, mut dz, mut d) = (0.0, 0.0, f64::INFINITY);
                for q in &s.rotors {
                    let (qx, qz) = (p[0] - q.x, p[2] - q.z);
                    let qd = m::hypot2(qx, qz);
                    if qd < q.r && qd < d { r = Some(q); dx = qx; dz = qz; d = qd; }
                }
                let sp = match r { Some(r) => spin * r.dir, None => 0.0 };
                let fa = match r { Some(r) => self.fin_angle * r.dir, None => 0.0 };
                if let Some(r) = r {
                    if d > 0.05 && sp != 0.0 && !sp.is_nan() {
                        let th = m::atan2(-dz, dx);
                        let per = (2.0 * m::PI) / r.fins;
                        let mut gap = (if sp > 0.0 { th - fa } else { fa - th }) % per;
                        if gap < 0.0 { gap += per; }
                        let r0 = r.fin_r0;
                        let sw = if r.sweep != 0.0 { m::sign(r.spin * r.dir) * r.sweep / (r.r - r0) } else { 0.0 };
                        if sw != 0.0 {
                            gap = (if sp > 0.0 { th - fa - sw * (d - r0) } else { fa + sw * (d - r0) - th }) % per;
                            if gap < 0.0 { gap += per; }
                        }
                        let fin = d > r0 - R * 0.5 && gap < R / d + 0.08;
                        let k = if fin { r.grip } else { r.drag };
                        let ux = -m::sin(th) * sp * d;
                        let uz = -m::cos(th) * sp * d;
                        if fin && sw != 0.0 {
                            let ph = th - gap * m::sign(sp);
                            let (c, s2) = (m::cos(ph), m::sin(ph));
                            let (mut tx, mut tz) = (c - d * sw * s2, -s2 - d * sw * c);
                            let tl = m::hypot2(tx, tz); tx /= tl; tz /= tl;
                            let (mut nx, mut nz) = (-m::sign(sp) * s2, -m::sign(sp) * c);
                            let a = nx * tx + nz * tz; nx -= a * tx; nz -= a * tz;
                            let nlen = m::hypot2(nx, nz); nx /= nlen; nz /= nlen;
                            let un = ux * nx + uz * nz;
                            let vn = v[0] * nx + v[2] * nz;
                            let vt = v[0] * tx + v[2] * tz;
                            fx += k * (un - vn) * nx + k * 0.5 * (-r.pull - vt) * tx;
                            fz += k * (un - vn) * nz + k * 0.5 * (-r.pull - vt) * tz;
                        } else {
                            fx += k * (ux - v[0]);
                            fz += k * (uz - v[2]);
                        }
                    }
                }
            } else if s.drive == "belt" && p[1] - R_WALL - self.floor_at(p[0], p[2]) < 0.5 * R {
                let b = (self.floor_at(p[0] + 0.01, p[2]) - self.floor_at(p[0] - 0.01, p[2])) / 0.02;
                let nn = m::hypot2(1.0, b);
                let (tx, ty) = (-1.0 / nn, -b / nn);
                let g_along = fy * ty;
                fx -= g_along * tx; fy -= g_along * ty;
                let mut sp = if running { s.drive_speed } else { 0.0 };
                let mut k = s.grip.unwrap_or(15.0);
                if let Some((_, Some(tspin), _)) = &s.taper { if sp != 0.0 && p[2] * tspin < -0.02 && self.abreast(i) { sp = -0.5 * sp; k *= 0.5; } }
                fx += k * (sp * tx - v[0]);
                fy += 15.0 * (sp * ty - v[1]);
                fz += -4.0 * v[2];
                if let Some((_, _, center)) = &s.taper { if sp != 0.0 && self.hw_at(p[0]) < s.hw { fz -= center * p[2]; } }
            }
            let fp_pull = s.feed_pull;
            let reach = s.feed_reach_pull;
            let front = self.front;
            let e = &mut self.list[i];
            if let Some(x) = e.ext.take() {
                fx += x[0] / FUEL::mass; fy += x[1] / FUEL::mass; fz += x[2] / FUEL::mass;
            }
            if e.push != 0.0 {
                if env.intaking && e.push_t > 0.0 && front - p[0] < D_BALL * 1.2 { fx -= e.push / FUEL::mass; }
                e.push_t -= dt;
                if e.push_t <= 0.0 { e.push = 0.0; }
            }
            if let (true, Some(fpt), Some(pull)) = (env.feeding, env.feed_point, fp_pull) {
                let (dx, dz) = (fpt[0] - p[0], fpt[2] - p[2]);
                let d = m::hypot2(dx, dz);
                if d > 0.05 && d < reach + R { fx += (pull * dx) / d; fz += (pull * dz) / d; }
            }
            let mut v = e.v;
            v[0] += fx * dt; v[1] += fy * dt; v[2] += fz * dt;
            let k = 1.0 - 0.6 * dt;
            v = [v[0] * k, v[1] * k, v[2] * k];
            e.v = v;
            e.prev = e.p;
            e.p = [e.p[0] + v[0] * dt, e.p[1] + v[1] * dt, e.p[2] + v[2] * dt];
        }

        // rigid things outside over the load (a TRENCH arm) push FUEL down out of the way; what that
        // takes pushes back on the robot
        let boxes = if env.boxes.is_empty() { None } else { Some(env.boxes) };
        self.box_force = [0.0; 3];
        self.box_at = [0.0; 3];
        let mut box_w = 0.0;
        // constraints: ball-ball contact, then the hopper around them
        self.list.sort_by(|a, b| (a.p[0] - b.p[0]).partial_cmp(&0.0).unwrap_or(std::cmp::Ordering::Equal));
        let n = self.list.len();
        let (mut squeeze, mut wall_squeeze): (f64, f64) = (0.0, 0.0);
        let mut lam: std::collections::HashMap<usize, f64> = std::collections::HashMap::new();
        let wgt = 2.0 / FUEL::mass;
        let mut pairs: Vec<(usize, usize, usize)> = Vec::new();
        for e in self.list.iter_mut() { e.n_touch = 0; }
        for it in 0..ITER {
            for i in 0..n {
                for j in i + 1..n {
                    let (a, b) = { let (l, r) = self.list.split_at_mut(j); (&mut l[i], &mut r[0]) };
                    let dx = b.p[0] - a.p[0];
                    if dx > D_BALL { break; }
                    let (dy, dz) = (b.p[1] - a.p[1], b.p[2] - a.p[2]);
                    let d2 = dx * dx + dy * dy + dz * dz;
                    if d2 >= D_BALL * D_BALL { continue; }
                    let (at, bt) = (a.tr.is_some(), b.tr.is_some());
                    if at && bt { continue; }
                    let d = { let q = m::sqrt(d2); if q == 0.0 || q.is_nan() { 1e-4 } else { q } };
                    if it == ITER - 1 {
                        squeeze = m::max(squeeze, fuel_force(D_BALL - d));
                        if self.wall < self.front && m::max(a.p[0], b.p[0]) > self.wall - D_BALL - R { wall_squeeze = m::max(wall_squeeze, fuel_force(D_BALL - d)); }
                        if !at && !bt {
                            touch(&mut a.touch, &mut a.n_touch, dx / d, dy / d, dz / d, d / 2.0);
                            touch(&mut b.touch, &mut b.n_touch, -dx / d, -dy / d, -dz / d, d / 2.0);
                        }
                    }
                    if it == 0 && !at && !bt { pairs.push((i, j, i * 4096 + j)); }
                    let mut corr = (D_BALL - d) / d;
                    if !at && !bt {
                        let key = i * 4096 + j;
                        let l0 = lam.get(&key).copied().unwrap_or(0.0);
                        let x = D_BALL - d;
                        let k = fuel_stiffness(x);
                        let soft = 1.0 / (k * dt * dt);
                        let dl = m::max(-l0, (fuel_force(x) / k - soft * l0) / (wgt + soft));
                        lam.insert(key, l0 + dl);
                        corr = (dl * wgt) / d;
                    }
                    let ka = if at { 0.0 } else if bt { 1.0 } else { 0.5 };
                    let kb = if bt { 0.0 } else if at { 1.0 } else { 0.5 };
                    let (cx, cy, cz) = (dx * corr, (if d2 > 1e-8 { dy } else { 1e-4 }) * corr, dz * corr);
                    a.p[0] -= cx * ka; a.p[1] -= cy * ka; a.p[2] -= cz * ka;
                    b.p[0] += cx * kb; b.p[1] += cy * kb; b.p[2] += cz * kb;
                }
            }
            if it == ITER - 2 {
                for &(i, j, key) in &pairs {
                    let ln = lam.get(&key).copied().unwrap_or(0.0);
                    if ln <= 0.0 { continue; }
                    let (a, b) = { let (l, r) = self.list.split_at_mut(j); (&mut l[i], &mut r[0]) };
                    let (mut nx, mut ny, mut nz) = (b.p[0] - a.p[0], b.p[1] - a.p[1], b.p[2] - a.p[2]);
                    let nd = { let h = m::hypot3(nx, ny, nz); if h == 0.0 || h.is_nan() { 1.0 } else { h } };
                    nx /= nd; ny /= nd; nz /= nd;
                    let (rx, ry, rz) = ((b.p[0] - b.prev[0]) - (a.p[0] - a.prev[0]), (b.p[1] - b.prev[1]) - (a.p[1] - a.prev[1]), (b.p[2] - b.prev[2]) - (a.p[2] - a.prev[2]));
                    let rn = rx * nx + ry * ny + rz * nz;
                    let (tx, ty, tz) = (rx - rn * nx, ry - rn * ny, rz - rn * nz);
                    let tl = m::hypot3(tx, ty, tz);
                    if tl < 1e-7 { continue; }
                    let f = 0.5 * m::min(1.0, (MU_FUEL * m::min(ln, FRICTION_FN * dt * dt) * wgt) / tl);
                    a.p[0] += tx * f; a.p[1] += ty * f; a.p[2] += tz * f;
                    b.p[0] -= tx * f; b.p[1] -= ty * f; b.p[2] -= tz * f;
                }
            }
            for i in 0..n { if self.list[i].tr.is_none() { let mut p = self.list[i].p; self.bounds(&mut p); self.list[i].p = p; } }
            if let Some(bx) = boxes {
                for i in 0..n {
                    if self.list[i].tr.is_some() { continue; }
                    let mut p = self.list[i].p;
                    let d = push_out_of_boxes(&mut p, bx, R_WALL, dt / ITER as f64);
                    self.list[i].p = p;
                    let Some(d) = d else { continue };
                    for a in 0..3 { self.box_force[a] += d[a]; }
                    let w = len(d);
                    for a in 0..3 { self.box_at[a] += p[a] * w; }
                    box_w += w;
                }
            }
        }
        if box_w > 0.0 {
            let k = 1.0 / box_w;
            self.box_at = [self.box_at[0] * k, self.box_at[1] * k, self.box_at[2] * k];
            let f = FUEL::mass / (dt * dt);
            self.box_force = [self.box_force[0] * f, self.box_force[1] * f, self.box_force[2] * f];
            self.quiet = 0.0;
        }
        self.pressure = squeeze;
        self.wall_pressure = wall_squeeze;

        for i in 0..n {
            if self.list[i].tr.is_some() { continue; }
            let (p, prev) = (self.list[i].p, self.list[i].prev);
            let k = 1.0 / dt;
            let mut v = [(p[0] - prev[0]) * k, (p[1] - prev[1]) * k, (p[2] - prev[2]) * k];
            if p[1] - R_WALL - self.floor_at(p[0], p[2]) < 0.004 { let f = 1.0 - m::min(1.0, 2.0 * dt); v[0] *= f; v[2] *= f; }
            let sp = len(v);
            if sp > MAX_SPEED { let f = MAX_SPEED / sp; v = [v[0] * f, v[1] * f, v[2] * f]; }
            max_v = m::max(max_v, sp);
            self.list[i].v = v;
        }
        self.quiet = if still && max_v < 0.03 { self.quiet + dt } else { 0.0 };
        done
    }

    fn bounds(&self, p: &mut V) {
        let s = &self.spec;
        p[0] = clamp(p[0], s.x0 + R_WALL, m::min(self.front, self.wall) - R_WALL);
        if s.taper.is_some() {
            let lim = self.hw_at(p[0]) - R_WALL;
            if m::abs(p[2]) > lim {
                let sz = m::sign(p[2]);
                let k = (self.hw_at(p[0] + 0.005) - self.hw_at(p[0] - 0.005)) / 0.01;
                let nn = m::hypot2(1.0, k);
                let d = (m::abs(p[2]) - lim) / nn;
                p[2] -= (sz * d) / nn; p[0] += (k * d) / nn;
            }
        }
        p[2] = clamp(p[2], -s.hw + R_WALL, s.hw - R_WALL);
        let fl = self.floor_at(p[0], p[2]) + R_WALL;
        let top = self.top_at(p[0], p[2]) - R_WALL;
        if p[1] < fl {
            let e = 0.01;
            let gx = (self.floor_at(p[0] + e, p[2]) - self.floor_at(p[0] - e, p[2])) / (2.0 * e);
            let gz = (self.floor_at(p[0], p[2] + e) - self.floor_at(p[0], p[2] - e)) / (2.0 * e);
            let k = (fl - p[1]) / (1.0 + gx * gx + gz * gz);
            p[0] -= gx * k; p[1] += k; p[2] -= gz * k;
        } else if p[1] > top && top > fl {
            let e = 0.01;
            let gx = (self.top_at(p[0] + e, p[2]) - self.top_at(p[0] - e, p[2])) / (2.0 * e);
            let gz = (self.top_at(p[0], p[2] + e) - self.top_at(p[0], p[2] - e)) / (2.0 * e);
            let k = (p[1] - top) / (1.0 + gx * gx + gz * gz);
            p[0] += gx * k; p[1] -= k; p[2] += gz * k;
        }
        if let Some((rx, rr)) = s.round {
            if p[0] < rx {
                let dx = p[0] - rx;
                let d = m::hypot2(dx, p[2]);
                let lim = rr - R_WALL;
                if d > lim { p[0] = rx + (dx * lim) / d; p[2] *= lim / d; }
            }
        }
        if let Some(ch) = s.chamfer {
            let sz = { let q = m::sign(p[2]); if q == 0.0 || q.is_nan() { 1.0 } else { q } };
            let g = (p[0] - s.x0) - m::abs(p[2]) + s.hw - ch - R_WALL * std::f64::consts::SQRT_2;
            if g < 0.0 { p[0] -= g / 2.0; p[2] += (sz * g) / 2.0; }
        }
        for ob in &s.obstacles {
            match *ob {
                Obstacle::Box([x0, x1, y0, y1, z0, z1]) => {
                    let (qx, qy, qz) = (clamp(p[0], x0, x1), clamp(p[1], y0, y1), clamp(p[2], z0, z1));
                    let (dx, dy, dz) = (p[0] - qx, p[1] - qy, p[2] - qz);
                    let d = m::hypot3(dx, dy, dz);
                    if d >= R_WALL { continue; }
                    if d > 1e-6 { let k = (R_WALL - d) / d; p[0] += dx * k; p[1] += dy * k; p[2] += dz * k; }
                    else { p[0] = x1 + R_WALL; }
                }
                Obstacle::Post { x, z, r, y0, y1 } => {
                    if p[1] + R_WALL < y0 || p[1] - R_WALL > y1 { continue; }
                    let (dx, dz) = (p[0] - x, p[2] - z);
                    let d = m::hypot2(dx, dz);
                    let mm = r + R_WALL;
                    if d >= mm { continue; }
                    if d > 1e-6 { p[0] = x + (dx / d) * mm; p[2] = z + (dz / d) * mm; } else { p[0] = x + mm; }
                }
            }
        }
    }
}

/// js `Hopper._easeS(u)`: the distance along the path after time u (ease in, then full speed)
pub fn ease_s(u: f64) -> f64 { (if u < 0.3 { (u * u) / 0.6 } else { u - 0.15 }) / 0.85 }
/// js `Hopper._easeU(q)`: back
pub fn ease_u(q: f64) -> f64 { if 0.85 * q < 0.15 { m::sqrt(0.6 * 0.85 * q) } else { 0.85 * q + 0.15 } }
