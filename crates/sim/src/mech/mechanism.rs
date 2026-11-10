//! js/mech/mechanism.js (all but Motor, Shaft and Train: drives.rs): wheels and surfaces that move
//! FUEL only by touching it. Each step, every wheel or surface touching a FUEL pushes on it through
//! the foam's squeeze and friction where they slide, and the FUEL pushes back on the wheel's shaft
//! and on the frame. Ported operation for operation (every sum in the JS's order, its Math through
//! jsmath) so the solve comes out to the bit; what's only for drawing (wear, the tally) isn't here.
use std::collections::{HashMap, HashSet};

use crate::constants::FUEL;
use crate::mech::drives::Drives;
use jsmath as m;
use rapier3d::prelude::*;

pub type V3 = [f64; 3];

const R: f64 = FUEL::radius;
/// js BUMPER_MU: what a bumper's fabric grips FUEL with
pub const BUMPER_MU: f64 = 0.35;
const MB: f64 = FUEL::mass;
const IB: f64 = 0.4 * MB * R * R;
const TWIST_K: f64 = (3.0 * m::PI) / 16.0;
const ZETA: f64 = 0.25;
const MU_FREE: f64 = 0.04;
pub const MU_FUEL: f64 = 0.3;
const MU_SIDE: f64 = 0.3;
const NEAR_M: f64 = 0.008;

// ------------------------------------------------------------------ foam
struct Dense { sc: f64, sd: f64, smax: f64 }
const DENSE: Dense = Dense { sc: 3.5 * FUEL::stiffen, sd: 0.9 * 2.0 * FUEL::radius, smax: 5.0 * 0.0254 };

/// js `foam(d)`: a contact's dent d (m) -> its force (N) and slope (N/m)
pub fn foam(d: f64) -> (f64, f64) {
    let (k0, xc, s) = (FUEL::springRate0, FUEL::stiffen, m::max(0.0, 2.0 * d));
    if s <= DENSE.sc { let e = m::exp(s / xc); return (k0 * xc * (e - 1.0), 2.0 * k0 * e); }
    let u = m::min(s, DENSE.smax);
    let e = m::exp(u / xc);
    let g = (DENSE.sd - DENSE.sc) / (DENSE.sd - u);
    let fe = k0 * xc * (e - 1.0);
    let df = k0 * e * g + fe * g / (DENSE.sd - u);
    (fe * g + df * (s - u), 2.0 * df)
}

/// js `curvature(rc)`
pub fn curvature(rc: f64) -> f64 { m::sqrt(rc / (R + rc)) }

/// js `squeeze(d, kt, fs)`: the foam (scaled fs) in series with a tread kt
fn squeeze(d: f64, kt: f64, fs: f64) -> (f64, f64) {
    if !(kt < f64::INFINITY) { let (f, k) = foam(d); return (f * fs, k * fs); }
    let mut xf = d * 0.5;
    for _ in 0..6 {
        let (f, k) = foam(xf);
        let g = fs * f - kt * (d - xf);
        let dg = fs * k + kt;
        xf = m::min(d, m::max(0.0, xf - g / dg));
    }
    let (f, k) = foam(xf);
    (fs * f, 1.0 / (1.0 / (fs * k) + 1.0 / kt))
}

// ------------------------------------------------------------------ vectors (the JS helpers)
fn sub(a: V3, b: V3) -> V3 { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
fn add(a: V3, b: V3) -> V3 { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
fn mul(a: V3, s: f64) -> V3 { [a[0] * s, a[1] * s, a[2] * s] }
fn dot(a: V3, b: V3) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
fn cross(a: V3, b: V3) -> V3 { [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]] }
fn len(a: V3) -> f64 { m::hypot3(a[0], a[1], a[2]) }
fn norm(a: V3) -> V3 { let l = len(a); let l = if l == 0.0 { 1.0 } else { l }; [a[0] / l, a[1] / l, a[2] / l] }
/// a quaternion (x, y, z, w)
pub type Q = [f64; 4];
fn qrot(q: Q, v: V3) -> V3 {
    let (u, s) = ([q[0], q[1], q[2]], q[3]);
    let t = mul(cross(u, v), 2.0);
    add(add(v, mul(t, s)), cross(u, t))
}
fn qrot_inv(q: Q, v: V3) -> V3 { qrot([-q[0], -q[1], -q[2], q[3]], v) }

/// A mechanism's frame: pose and motion in the world
#[derive(Clone, Copy, Debug)]
pub struct Frame { pub p: V3, pub q: Q, pub v: V3, pub w: V3, pub com: V3 }

/// js `frameAt(fr, tau)`
fn frame_at(fr: &Frame, tau: f64) -> Frame {
    let a = mul(fr.w, tau);
    let ang = len(a);
    let s = if ang > 1e-12 { m::sin(ang / 2.0) / ang } else { 0.5 };
    let dq = [a[0] * s, a[1] * s, a[2] * s, m::cos(ang / 2.0)];
    let q0 = fr.q;
    let (dx, dy, dz, dw) = (dq[0], dq[1], dq[2], dq[3]);
    let q = [
        dw * q0[0] + dx * q0[3] + dy * q0[2] - dz * q0[1],
        dw * q0[1] - dx * q0[2] + dy * q0[3] + dz * q0[0],
        dw * q0[2] + dx * q0[1] - dy * q0[0] + dz * q0[3],
        dw * q0[3] - dx * q0[0] - dy * q0[1] - dz * q0[2],
    ];
    let com = add(fr.com, mul(fr.v, tau));
    Frame { p: add(com, qrot(dq, sub(fr.p, fr.com))), q, v: fr.v, w: fr.w, com }
}

// ------------------------------------------------------------------ wheels
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Kind { Plain, Omni, Mecanum }

/// js `Wheel`: on drives' shaft `shaft`, center and axis in the frame
#[derive(Clone, Debug)]
pub struct Wheel {
    pub name: String,
    pub shaft: usize,
    pub center: V3,
    pub axis: V3,
    pub radius: f64,
    pub width: f64,
    pub kind: Kind,
    pub mecanum: f64,
    pub mu: f64,
    pub tread: f64,
    pub covered: [bool; 2],
    /// on a swinging arm: its pivot's shaft, a point on its axis and its direction (frame)
    pub pivot: Option<(usize, V3, V3)>,
}

impl Wheel {
    pub fn new(name: &str, shaft: usize, center: V3, axis: V3, radius: f64, width: f64) -> Self {
        let n = m::hypot3(axis[0], axis[1], axis[2]);
        Wheel { name: name.into(), shaft, center, axis: [axis[0] / n, axis[1] / n, axis[2] / n], radius, width, kind: Kind::Plain, mecanum: 45.0, mu: 1.0, tread: f64::INFINITY, covered: [false, false], pivot: None }
    }
    pub fn mu(mut self, mu: f64) -> Self { self.mu = mu; self }
    pub fn tread(mut self, t: f64) -> Self { self.tread = t; self }
    pub fn kind(mut self, k: Kind) -> Self { self.kind = k; self }
    pub fn mecanum(mut self, deg: f64) -> Self { self.kind = Kind::Mecanum; self.mecanum = deg; self }
    /// js `onPivot(shaft, pivot, axis)`
    pub fn on_pivot(mut self, shaft: usize, pivot: V3, axis: V3) -> Self { self.pivot = Some((shaft, pivot, axis)); self }
}

/// js `abutting(wheels)`: a wheel's edge another's tread carries on from
pub fn abutting(wheels: &mut [Wheel]) {
    for i in 0..wheels.len() {
        for j in 0..wheels.len() {
            if i == j { continue }
            let (a, b) = (&wheels[i], &wheels[j]);
            if m::abs(a.radius - b.radius) > 0.004 || m::abs(dot(a.axis, b.axis)) < 0.999 { continue }
            if a.pivot.map(|p| p.0) != b.pivot.map(|p| p.0) { continue }
            let d = sub(b.center, a.center);
            let s = dot(d, a.axis);
            if len(sub(d, mul(a.axis, s))) > 0.002 || m::abs(m::abs(s) - (a.width + b.width) / 2.0) > 0.001 { continue }
            wheels[i].covered[if s > 0.0 { 1 } else { 0 }] = true;
        }
    }
}

// ------------------------------------------------------------------ surfaces
/// a surface's shaft: none (fixed), or drives' shaft it swings on or slides with
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mount { Fixed, Pivot { shaft: usize, pivot: V3, axis: V3 }, Slide { shaft: usize, dir: V3, k: f64 } }

#[derive(Clone, Debug)]
enum Shape { Mesh, Block { c: V3, h: V3, r: f64 } }

#[derive(Clone, Debug, Default)]
struct Near { x: f64, y: f64, z: f64, t: Vec<i32> }

/// js `Surface` (a triangle mesh) and `Block` (a solid foam block: a bumper)
#[derive(Clone, Debug)]
pub struct Surface {
    pub name: String,
    pub mu: f64,
    pub tread: f64,
    pub mount: Mount,
    cell: f64,
    shape: Shape,
    pub pos: V3,
    pub axes: [V3; 3],
    tri: Vec<f64>,
    plane: Vec<f64>,
    bx: Vec<f64>,
    pub lo: V3,
    pub hi: V3,
    g0: [f64; 3],
    gn: [f64; 3],
    cell0: Vec<i32>,
    node_box: Vec<f64>,
    node_a: Vec<i32>,
    node_b: Vec<i32>,
    leaf_tri: Vec<i32>,
    near: HashMap<RigidBodyHandle, Near>,
}

/// a face a FUEL touches: its depth and normal (out of the surface)
#[derive(Clone, Copy, Debug)]
pub struct Hit { pub depth: f64, pub n: V3 }

impl Surface {
    /// js `new Surface({ name, vertices, indices, mu, pos, yaw, roll, cell, tread })`
    pub fn mesh(name: &str, vertices: &[f64], indices: &[u32], mu: f64, pos: V3, yaw: f64, roll: f64, cell: f64, tread: f64) -> Self {
        let v = vertices;
        // (no zero-area triangles)
        let mut idx = Vec::with_capacity(indices.len());
        for t in 0..indices.len() / 3 {
            let (a, b, c) = (indices[t * 3] as usize * 3, indices[t * 3 + 1] as usize * 3, indices[t * 3 + 2] as usize * 3);
            let (ux, uy, uz) = (v[b] - v[a], v[b + 1] - v[a + 1], v[b + 2] - v[a + 2]);
            let (vx, vy, vz) = (v[c] - v[a], v[c + 1] - v[a + 1], v[c + 2] - v[a + 2]);
            if m::hypot3(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx) > 1e-12 { idx.extend_from_slice(&indices[t * 3..t * 3 + 3]); }
        }
        let n = idx.len() / 3;
        let mut tri = vec![0.0; n * 9];
        for t in 0..n { for k in 0..3 { for j in 0..3 { tri[t * 9 + k * 3 + j] = v[idx[t * 3 + k] as usize * 3 + j]; } } }
        let mut plane = vec![0.0; n * 4];
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for t in 0..n {
            let o = t * 9;
            let (ux, uy, uz) = (tri[o + 3] - tri[o], tri[o + 4] - tri[o + 1], tri[o + 5] - tri[o + 2]);
            let (vx, vy, vz) = (tri[o + 6] - tri[o], tri[o + 7] - tri[o + 1], tri[o + 8] - tri[o + 2]);
            let (mut nx, mut ny, mut nz) = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
            let l = m::hypot3(nx, ny, nz);
            let l = if l == 0.0 { 1.0 } else { l };
            nx /= l; ny /= l; nz /= l;
            plane[t * 4] = nx; plane[t * 4 + 1] = ny; plane[t * 4 + 2] = nz;
            plane[t * 4 + 3] = nx * tri[o] + ny * tri[o + 1] + nz * tri[o + 2];
            for k in 0..3 { for j in 0..3 { lo[j] = m::min(lo[j], tri[o + k * 3 + j]); hi[j] = m::max(hi[j], tri[o + k * 3 + j]); } }
        }
        let mut bx = vec![0.0; n * 6];
        for t in 0..n { for j in 0..3 {
            bx[t * 6 + j] = m::min(m::min(tri[t * 9 + j], tri[t * 9 + 3 + j]), tri[t * 9 + 6 + j]);
            bx[t * 6 + 3 + j] = m::max(m::max(tri[t * 9 + j], tri[t * 9 + 3 + j]), tri[t * 9 + 6 + j]);
        } }
        let g0 = [m::floor((lo[0] - R) / cell), m::floor((lo[1] - R) / cell), m::floor((lo[2] - R) / cell)];
        let gn = [0, 1, 2].map(|j| m::floor((hi[j] + R) / cell) - g0[j] + 1.0);
        let mut cell0 = vec![0i32; n * 3];
        for t in 0..n { for j in 0..3 {
            cell0[t * 3 + j] = (m::floor(m::min(m::min(tri[t * 9 + j], tri[t * 9 + 3 + j]), tri[t * 9 + 6 + j]) / cell) - g0[j]) as i32;
        } }
        let mut s = Surface {
            name: name.into(), mu, tread, mount: Mount::Fixed, cell, shape: Shape::Mesh, pos: [0.0; 3], axes: [[0.0; 3]; 3],
            tri, plane, bx, lo: lo.map(|x| x - R), hi: hi.map(|x| x + R), g0, gn, cell0,
            node_box: Vec::new(), node_a: Vec::new(), node_b: Vec::new(), leaf_tri: Vec::new(), near: HashMap::new(),
        };
        s.set_pose(pos, yaw, roll);
        s.build_tree(n);
        s
    }

    /// js `new Block({ name, x0, x1, y0, y1, hw, r, mu, tread })`
    pub fn block(name: &str, x0: f64, x1: f64, y0: f64, y1: f64, hw: f64, r: f64, mu: f64, tread: f64) -> Self {
        let mut v = Vec::new();
        for z in [-hw, hw] { for (x, y) in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)] { v.extend_from_slice(&[x, y, z]); } }
        let mut ix = Vec::new();
        for [a, b, c, d] in [[1u32, 2, 6, 5], [2, 3, 7, 6], [0, 1, 5, 4], [0, 4, 7, 3], [0, 3, 2, 1], [4, 5, 6, 7]] { ix.extend_from_slice(&[a, b, c, a, c, d]); }
        let mut s = Surface::mesh(name, &v, &ix, mu, [0.0; 3], 0.0, 0.0, 0.08, tread);
        s.shape = Shape::Block { c: [(x0 + x1) / 2.0, (y0 + y1) / 2.0, 0.0], h: [(x1 - x0) / 2.0 - r, (y1 - y0) / 2.0 - r, hw - r], r };
        s
    }

    /// A surface as the browser game's own code built it (tools/export-mech.mjs): its triangles
    /// (`tri`, the zero-area ones already gone: built from them again, the same tree and order), its
    /// pose, and a Block's section
    pub fn from_tri(name: &str, tri: &[f64], mu: f64, tread: f64, cell: f64, pos: V3, axes: [V3; 3], block: Option<(V3, V3, f64)>) -> Self {
        let idx: Vec<u32> = (0..(tri.len() / 3) as u32).collect();
        let mut s = Surface::mesh(name, tri, &idx, mu, pos, 0.0, 0.0, cell, tread);
        s.axes = axes;
        if let Some((c, h, r)) = block { s.shape = Shape::Block { c, h, r }; }
        s
    }

    pub fn is_block(&self) -> bool { matches!(self.shape, Shape::Block { .. }) }
    /// a Block's bottom edge (its center's y less its half height and rounding)
    pub fn block_bottom(&self) -> Option<f64> { if let Shape::Block { c, h, r } = self.shape { Some(c[1] - h[1] - r) } else { None } }

    /// js `onShaft(shaft, pivot, axis)`
    pub fn on_shaft(mut self, shaft: usize, pivot: V3, axis: V3) -> Self { self.mount = Mount::Pivot { shaft, pivot, axis }; self }
    /// js `sf.slideK = k`: m it slides per radian of its shaft
    pub fn set_slide_k(&mut self, k: f64) { if let Mount::Slide { k: kk, .. } = &mut self.mount { *kk = k; } }
    /// js `onSlide(shaft, dir)` (slideK set later)
    pub fn on_slide(mut self, shaft: usize, dir: V3) -> Self { self.mount = Mount::Slide { shaft, dir, k: 0.0 }; self }

    fn build_tree(&mut self, n: usize) {
        let b = &self.bx;
        let mut order: Vec<i32> = (0..n as i32).collect();
        let mut cen = vec![0.0; n * 3];
        for t in 0..n { for j in 0..3 { cen[t * 3 + j] = (b[t * 6 + j] + b[t * 6 + 3 + j]) / 2.0; } }
        let (mut bxs, mut na, mut nb) = (Vec::new(), Vec::new(), Vec::new());
        const LEAF: usize = 4;
        fn build(s: usize, e: usize, order: &mut Vec<i32>, cen: &[f64], b: &[f64], bxs: &mut Vec<f64>, na: &mut Vec<i32>, nb: &mut Vec<i32>) -> i32 {
            let id = na.len();
            na.push(0); nb.push(0);
            let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for &t in &order[s..e] {
                let o = t as usize * 6;
                for j in 0..3 { if b[o + j] < lo[j] { lo[j] = b[o + j]; } if b[o + 3 + j] > hi[j] { hi[j] = b[o + 3 + j]; } }
            }
            bxs.extend_from_slice(&[lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]]);
            if e - s <= LEAF { na[id] = s as i32; nb[id] = -1 - (e - s) as i32; return id as i32; }
            let ax = if hi[0] - lo[0] >= hi[1] - lo[1] && hi[0] - lo[0] >= hi[2] - lo[2] { 0 } else if hi[1] - lo[1] >= hi[2] - lo[2] { 1 } else { 2 };
            order[s..e].sort_by(|&a, &c| {
                let d = cen[a as usize * 3 + ax] - cen[c as usize * 3 + ax];
                if d != 0.0 { d.partial_cmp(&0.0).unwrap() } else { a.cmp(&c) }
            });
            let mid = (s + e) >> 1;
            let l = build(s, mid, order, cen, b, bxs, na, nb);
            na[id] = l;
            let r = build(mid, e, order, cen, b, bxs, na, nb);
            nb[id] = r;
            id as i32
        }
        if n > 0 { build(0, n, &mut order, &cen, b, &mut bxs, &mut na, &mut nb); }
        self.node_box = bxs;
        self.node_a = na;
        self.node_b = nb;
        self.leaf_tri = order;
    }

    /// js `setPose(pos, yaw, roll)`
    pub fn set_pose(&mut self, pos: V3, yaw: f64, roll: f64) {
        self.pos = pos;
        let (cy, sy, cr, sr) = (m::cos(yaw), m::sin(yaw), m::cos(roll), m::sin(roll));
        self.axes = [[cy * cr, sr, -sy * cr], [-cy * sr, cr, sy * sr], [sy, 0.0, cy]];
    }
    pub fn to_local(&self, p: V3) -> V3 { let d = sub(p, self.pos); [dot(d, self.axes[0]), dot(d, self.axes[1]), dot(d, self.axes[2])] }
    pub fn dir_to_frame(&self, v: V3) -> V3 { add(add(mul(self.axes[0], v[0]), mul(self.axes[1], v[1])), mul(self.axes[2], v[2])) }

    /// js `_near(px, py, pz, who)`
    fn near_tris(&mut self, px: f64, py: f64, pz: f64, who: RigidBodyHandle) -> &Near {
        let fresh = match self.near.get(&who) {
            Some(c) => !(m::abs(px - c.x) <= NEAR_M && m::abs(py - c.y) <= NEAR_M && m::abs(pz - c.z) <= NEAR_M),
            None => true,
        };
        if fresh {
            let r = R + NEAR_M + 1e-9;
            let (nbx, na, nbn, leaf, b) = (&self.node_box, &self.node_a, &self.node_b, &self.leaf_tri, &self.bx);
            let mut t = Vec::new();
            let mut stack = vec![0i32];
            while let Some(id) = stack.pop() {
                let o = id as usize * 6;
                if px + r < nbx[o] || px - r > nbx[o + 3] || py + r < nbx[o + 1] || py - r > nbx[o + 4] || pz + r < nbx[o + 2] || pz - r > nbx[o + 5] { continue }
                let nb = nbn[id as usize];
                if nb >= 0 { stack.push(nb); stack.push(na[id as usize]); continue }
                let s0 = na[id as usize];
                for mm in s0..(s0 - 1 - nb) {
                    let tt = leaf[mm as usize];
                    let o = tt as usize * 6;
                    if px + r < b[o] || px - r > b[o + 3] || py + r < b[o + 1] || py - r > b[o + 4] || pz + r < b[o + 2] || pz - r > b[o + 5] { continue }
                    t.push(tt);
                }
            }
            self.near.insert(who, Near { x: px, y: py, z: pz, t });
        }
        &self.near[&who]
    }

    /// js `touches(p, who)` (a Block's, or a mesh's)
    pub fn touches(&mut self, p: V3, who: RigidBodyHandle) -> Vec<Hit> {
        if let Shape::Block { c, h, r } = self.shape {
            let d = [p[0] - c[0], p[1] - c[1], p[2] - c[2]];
            let q = [m::abs(d[0]) - h[0], m::abs(d[1]) - h[1], m::abs(d[2]) - h[2]];
            let out = q.map(|v| m::max(v, 0.0));
            let ol = m::hypot3(out[0], out[1], out[2]);
            let (dist, n);
            if ol > 0.0 {
                dist = ol - r;
                n = [0, 1, 2].map(|k| (out[k] / ol) * m::sign(d[k]));
            } else {
                let mx = m::max_of(&q);
                let k = q.iter().position(|&v| v == mx).unwrap();
                dist = q[k] - r;
                let mut nn = [0.0; 3];
                let sg = m::sign(d[k]);
                nn[k] = if sg == 0.0 || sg.is_nan() { 1.0 } else { sg };
                n = nn;
            }
            let depth = R - dist;
            return if depth > 0.0 { vec![Hit { depth, n }] } else { Vec::new() };
        }
        let (px, py, pz, r) = (p[0], p[1], p[2], R);
        if px < self.lo[0] || px > self.hi[0] || py < self.lo[1] || py > self.hi[1] || pz < self.lo[2] || pz > self.hi[2] || self.node_a.is_empty() { return Vec::new(); }
        let cell = self.cell;
        let i0 = m::max(0.0, m::floor((px - r) / cell) - self.g0[0]);
        let j0 = m::max(0.0, m::floor((py - r) / cell) - self.g0[1]);
        let k0 = m::max(0.0, m::floor((pz - r) / cell) - self.g0[2]);
        let near = self.near_tris(px, py, pz, who).t.clone();
        let (tr, pl, b, gn, cell0) = (&self.tri, &self.plane, &self.bx, self.gn, &self.cell0);
        let ntri = (tr.len() / 9) as f64;
        // (the faces found, in the grid's order: key, depth, normal)
        let mut hits: Vec<(f64, f64, V3)> = Vec::new();
        for &t in &near {
            let t = t as usize;
            let o = t * 6;
            if px + r < b[o] || px - r > b[o + 3] || py + r < b[o + 1] || py - r > b[o + 4] || pz + r < b[o + 2] || pz - r > b[o + 5] { continue }
            let sd = pl[t * 4] * px + pl[t * 4 + 1] * py + pl[t * 4 + 2] * pz - pl[t * 4 + 3];
            if sd >= r || sd <= -r { continue }
            let cr = m::sqrt(r * r - sd * sd);
            let (cx, cy, cz) = (px - pl[t * 4] * sd, py - pl[t * 4 + 1] * sd, pz - pl[t * 4 + 2] * sd);
            if cx + cr < b[o] || cx - cr > b[o + 3] || cy + cr < b[o + 1] || cy - cr > b[o + 4] || cz + cr < b[o + 2] || cz - cr > b[o + 5] { continue }
            let q = closest_on_tri(px, py, pz, tr, t * 9);
            let (dx, dy, dz) = (px - q[0], py - q[1], pz - q[2]);
            let dl = m::sqrt(dx * dx + dy * dy + dz * dz);
            if dl >= r || dl < 1e-9 { continue }
            let ci = m::max(i0, cell0[t * 3] as f64);
            let cj = m::max(j0, cell0[t * 3 + 1] as f64);
            let ck = m::max(k0, cell0[t * 3 + 2] as f64);
            let key = ((ci * gn[1] + cj) * gn[2] + ck) * ntri + t as f64;
            let mut at = hits.len();
            hits.push((0.0, 0.0, [0.0; 3]));
            while at > 0 && hits[at - 1].0 > key { hits[at] = hits[at - 1]; at -= 1; }
            hits[at] = (key, r - dl, [dx / dl, dy / dl, dz / dl]);
        }
        let mut out: Vec<Hit> = Vec::new();
        for (_, depth, n) in hits {
            let mut merged = false;
            for f in out.iter_mut() {
                if f.n[0] * n[0] + f.n[1] * n[1] + f.n[2] * n[2] > 0.9 {
                    if depth > f.depth { f.depth = depth; f.n = n; }
                    merged = true;
                    break;
                }
            }
            if !merged { out.push(Hit { depth, n }); }
        }
        out
    }
}

/// js `closestOnTri` (Ericson 5.1.5)
fn closest_on_tri(px: f64, py: f64, pz: f64, t: &[f64], o: usize) -> V3 {
    let (ax, ay, az) = (t[o], t[o + 1], t[o + 2]);
    let (abx, aby, abz) = (t[o + 3] - ax, t[o + 4] - ay, t[o + 5] - az);
    let (acx, acy, acz) = (t[o + 6] - ax, t[o + 7] - ay, t[o + 8] - az);
    let (apx, apy, apz) = (px - ax, py - ay, pz - az);
    let d1 = abx * apx + aby * apy + abz * apz;
    let d2 = acx * apx + acy * apy + acz * apz;
    if d1 <= 0.0 && d2 <= 0.0 { return [ax, ay, az]; }
    let (bpx, bpy, bpz) = (px - t[o + 3], py - t[o + 4], pz - t[o + 5]);
    let d3 = abx * bpx + aby * bpy + abz * bpz;
    let d4 = acx * bpx + acy * bpy + acz * bpz;
    if d3 >= 0.0 && d4 <= d3 { return [t[o + 3], t[o + 4], t[o + 5]]; }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 { let v = d1 / (d1 - d3); return [ax + abx * v, ay + aby * v, az + abz * v]; }
    let (cpx, cpy, cpz) = (px - t[o + 6], py - t[o + 7], pz - t[o + 8]);
    let d5 = abx * cpx + aby * cpy + abz * cpz;
    let d6 = acx * cpx + acy * cpy + acz * cpz;
    if d6 >= 0.0 && d5 <= d6 { return [t[o + 6], t[o + 7], t[o + 8]]; }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 { let w = d2 / (d2 - d6); return [ax + acx * w, ay + acy * w, az + acz * w]; }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return [t[o + 3] + (t[o + 6] - t[o + 3]) * w, t[o + 4] + (t[o + 7] - t[o + 4]) * w, t[o + 5] + (t[o + 8] - t[o + 5]) * w];
    }
    let den = 1.0 / (va + vb + vc);
    let (v, w) = (vb * den, vc * den);
    [ax + abx * v + acx * w, ay + aby * v + acy * w, az + abz * v + acz * w]
}

/// js `surfaceBounds`: each surface's bounds as a box in the frame (lo > hi: none)
fn surface_bounds(surfaces: &[&Surface]) -> Vec<f64> {
    let mut sb = vec![0.0; surfaces.len() * 6];
    for (i, sf) in surfaces.iter().enumerate() {
        let o = i * 6;
        let (lo, hi) = (sf.lo, sf.hi);
        if !(lo[0].is_finite() && hi[0].is_finite()) { sb[o] = 1.0; sb[o + 3] = -1.0; continue }
        let (c0, c1, c2) = ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0);
        let (h0, h1, h2) = ((hi[0] - lo[0]) / 2.0, (hi[1] - lo[1]) / 2.0, (hi[2] - lo[2]) / 2.0);
        let [ex, ey, ez] = sf.axes;
        let p = sf.pos;
        for k in 0..3 {
            let c = p[k] + ex[k] * c0 + ey[k] * c1 + ez[k] * c2;
            let h = m::abs(ex[k]) * h0 + m::abs(ey[k]) * h1 + m::abs(ez[k]) * h2 + 1e-6;
            sb[o + k] = c - h;
            sb[o + 3 + k] = c + h;
        }
    }
    sb
}

// ------------------------------------------------------------------ the mechanism
/// what a contact is with: one of the mechanism's wheels, a surface (an index into the robot's
/// surfaces), or another FUEL
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Toucher { Wheel(usize), Surface(usize), Fuel }

/// js mech.contacts' entries: the firmest of each wheel or surface and FUEL over the step
#[derive(Clone, Copy, Debug)]
pub struct Touch { pub wheel: Toucher, pub ball: RigidBodyHandle, pub depth: f64, pub force: f64, pub x: V3, pub n: V3, pub torque: f64 }
/// js mech.dents' entries (its last sub-step's)
#[derive(Clone, Copy, Debug)]
pub struct Dent { pub wheel: Toucher, pub ball: RigidBodyHandle, pub depth: f64, pub n: V3 }

#[derive(Clone, Copy, Debug)]
struct St { b: RigidBodyHandle, c: V3, v: V3, w: V3 }

#[derive(Clone, Copy, Debug, Default)]
struct Dir { u: V3, cb: V3, cb2: V3, lk: f64, pk: f64 }

#[derive(Clone, Copy, Debug)]
struct Fric { d: Dir, mu: f64, m: f64, lam: f64 }

#[derive(Clone, Copy, Debug)]
struct Piv { shaft: usize, a: V3, r: V3 }

#[derive(Clone, Debug)]
struct Contact {
    st: usize,
    st2: Option<usize>,
    who: Toucher,
    /// the shaft its impulse turns (None: fixed or another FUEL)
    shaft: Option<usize>,
    a: V3,
    n: V3,
    depth: f64,
    x: V3,
    rb: V3,
    rb2: V3,
    rw: V3,
    f: f64,
    k: f64,
    c: f64,
    lam_n: f64,
    fric: Vec<Fric>,
    plain: bool,
    piv: Option<Piv>,
    im: Option<f64>,
    is_surface: bool,
    // (prepare)
    hk: f64,
    g: f64,
    dn: Dir,
    fv: V3,
    ca: V3,
    cp: V3,
    rx: f32,
    ry: f32,
    rz: f32,
    lam_t: f64,
}

/// A wheel placed in the world for a step
#[derive(Clone, Copy)]
struct Placed { i: usize, c: V3, a: V3, p: Option<V3>, pa: V3 }

/// js `Mechanism`. Its surfaces are indices into the robot's (a surface can be two mechanisms':
/// a shooter's hood is its ball path's too)
pub struct Mechanism {
    pub body: Option<RigidBodyHandle>,
    pub shafts: Vec<usize>,
    pub wheels: Vec<Wheel>,
    pub surfaces: Vec<usize>,
    pub iterations: usize,
    pub gravity: V3,
    pub sub: usize,
    pub carried: bool,
    pub fast: Option<(f64, usize)>,
    pub curved: bool,
    g_share: f64,
    pub v: f64,
    pub n_sub: usize,
    pub contacts: Vec<Touch>,
    pub dents: Vec<Dent>,
}

/// What a step leaves to another mechanism: these surfaces, for these FUEL (js step's skip)
pub struct Skip { pub surfaces: HashSet<usize>, pub balls: HashSet<RigidBodyHandle> }

fn v3f(v: Vector) -> V3 { [v.x as f64, v.y as f64, v.z as f64] }

impl Mechanism {
    /// js `new Mechanism({ body, shafts, wheels, surfaces, iterations, gravity, substeps, sub, fast, curved })`
    pub fn new(body: Option<RigidBodyHandle>, shafts: Vec<usize>, mut wheels: Vec<Wheel>, surfaces: Vec<usize>, substeps: usize, sub: usize) -> Self {
        abutting(&mut wheels);
        Mechanism {
            body, shafts, wheels, surfaces, iterations: 6, gravity: [0.0, -9.81, 0.0], sub, carried: true, fast: None, curved: false,
            g_share: (substeps as f64 + 1.0) / (2.0 * substeps as f64), v: 12.0, n_sub: 1, contacts: Vec::new(), dents: Vec::new(),
        }
    }

    fn frame(&self, bodies: &RigidBodySet) -> Frame {
        match self.body.and_then(|h| bodies.get(h)) {
            None => Frame { p: [0.0; 3], q: [0.0, 0.0, 0.0, 1.0], v: [0.0; 3], w: [0.0; 3], com: [0.0; 3] },
            Some(b) => {
                let r = b.rotation();
                Frame { p: v3f(b.translation()), q: [r.x as f64, r.y as f64, r.z as f64, r.w as f64], v: v3f(b.linvel()), w: v3f(b.angvel()), com: v3f(b.center_of_mass()) }
            }
        }
    }

    /// js `step(dt, balls, together, skip)`
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, dt: f64, bodies: &mut RigidBodySet, drives: &mut Drives, all_surfaces: &mut [Surface], balls: &[RigidBodyHandle], together: Option<&[RigidBodyHandle]>, skip: Option<&Skip>) {
        let together: Option<HashSet<RigidBodyHandle>> = together.map(|t| t.iter().copied().collect());
        // (the trains its shafts are on, in order, each once)
        let mut trains: Vec<usize> = Vec::new();
        for &s in &self.shafts { if let Some(t) = drives.shafts[s].train { if !trains.contains(&t) { trains.push(t); } } }
        let v = self.v;
        let shafts = self.shafts.clone();
        let drive = |drives: &mut Drives, h: f64, slice: Option<(f64, f64)>| {
            for &s in &shafts { drives.integrate_shaft(s, h, v, slice); }
            for &t in &trains { drives.integrate_train(t, h, v, slice); }
        };
        let fr0 = self.frame(bodies);
        let mut n_sub = self.sub;
        if let Some((speed, fsub)) = self.fast {
            if n_sub < fsub {
                let s2 = speed * speed;
                for &b in balls {
                    let bd = &bodies[b];
                    let (vv, t) = (v3f(bd.linvel()), v3f(bd.translation()));
                    let r = sub(t, fr0.com);
                    let w = cross(fr0.w, r);
                    let (a, bb, c) = (vv[0] - fr0.v[0] - w[0], vv[1] - fr0.v[1] - w[1], vv[2] - fr0.v[2] - w[2]);
                    if a * a + bb * bb + c * c > s2 { n_sub = fsub; break; }
                }
            }
        }
        self.n_sub = n_sub;
        let sub_motor = n_sub > 1;
        if !sub_motor { drive(drives, dt, None); }
        let mut fr = fr0;
        let place = |fr: &Frame, wheels: &[Wheel]| -> Vec<Placed> {
            wheels.iter().enumerate().map(|(i, wh)| {
                let mut w = Placed { i, c: add(fr.p, qrot(fr.q, wh.center)), a: qrot(fr.q, wh.axis), p: None, pa: [0.0; 3] };
                if let Some((_, pv, pa)) = wh.pivot { w.p = Some(add(fr.p, qrot(fr.q, pv))); w.pa = qrot(fr.q, pa); }
                w
            }).collect()
        };
        let mut wp = place(&fr, &self.wheels);
        let (g, n) = (self.gravity, n_sub);
        let h = dt / n as f64;
        // ball states (insertion order)
        let mut sts: Vec<St> = Vec::new();
        let mut st_of: HashMap<RigidBodyHandle, usize> = HashMap::new();
        let g_share = self.g_share;
        let state = |b: RigidBodyHandle, bodies: &RigidBodySet, sts: &mut Vec<St>, st_of: &mut HashMap<RigidBodyHandle, usize>| -> usize {
            if let Some(&i) = st_of.get(&b) { return i; }
            let bd = &bodies[b];
            let (t, vv, w) = (v3f(bd.translation()), v3f(bd.linvel()), v3f(bd.angvel()));
            let f = if n == 1 { g_share * dt } else { 0.0 };
            sts.push(St { b, c: t, v: [vv[0] + g[0] * f, vv[1] + g[1] * f, vv[2] + g[2] * f], w });
            st_of.insert(b, sts.len() - 1);
            sts.len() - 1
        };
        if n > 1 {
            for &b in balls {
                let bd = &bodies[b];
                let (bc, vv) = (v3f(bd.translation()), v3f(bd.linvel()));
                let reach = m::hypot3(vv[0], vv[1], vv[2]) * dt + 0.01;
                let by_wheel = wp.iter().any(|pw| {
                    let wh = &self.wheels[pw.i];
                    let d = sub(bc, pw.c);
                    let s = dot(d, pw.a);
                    m::abs(s) < wh.width / 2.0 + R + reach && len(sub(d, mul(pw.a, s))) < wh.radius + R + reach
                });
                let by_surface = !by_wheel && !self.surfaces.is_empty() && self.surfaces.iter().any(|&si| {
                    let sf = &mut all_surfaces[si];
                    let l = sf.to_local(qrot_inv(fr.q, sub(bc, fr.p)));
                    !sf.touches(l, b).is_empty()
                });
                if by_wheel || by_surface { state(b, bodies, &mut sts, &mut st_of); }
            }
            if let Some(tg) = &together { for &b in balls { if tg.contains(&b) { state(b, bodies, &mut sts, &mut st_of); } } }
        }
        let mut reaction = match self.body.and_then(|hb| bodies.get(hb)) { Some(b) if b.is_dynamic() => Some(Reaction::new(fr0)), _ => None };
        // (the step's contacts: wheel -> ball -> the firmest, in order)
        let mut seen: Vec<(Toucher, Vec<Touch>)> = Vec::new();
        self.dents.clear();
        for k in 0..n {
            if self.carried && n > 1 && k > 0 && (fr0.v.iter().any(|&x| x != 0.0) || fr0.w.iter().any(|&x| x != 0.0)) {
                fr = frame_at(&fr0, k as f64 * h);
                wp = place(&fr, &self.wheels);
            }
            if sub_motor { drive(drives, h, Some((k as f64 * h, dt))); }
            if n > 1 { for st in sts.iter_mut() { st.v = add(st.v, mul(g, h)); } }
            let at: Vec<(RigidBodyHandle, V3)> = if n > 1 { sts.iter().map(|s| (s.b, s.c)).collect() } else { balls.iter().map(|&b| (b, v3f(bodies[b].translation()))).collect() };
            let mut cs = self.contacts_at(&wp, &fr, &at, bodies, drives, all_surfaces, together.as_ref(), skip, &mut sts, &mut st_of, &state);
            self.solve(&mut cs, h, &fr, reaction.as_mut(), bodies, drives, &mut sts, fr.w);
            if n > 1 { for st in sts.iter_mut() { st.c = add(st.c, mul(st.v, h)); } }
            for ct in &cs {
                if ct.st2.is_some() { continue }
                let key = ct.who;
                let ball = sts[ct.st].b;
                let torque = -(ct.lam_n * ct.dn.lk + ct.fric.iter().fold(0.0, |s, f| s + f.lam * f.d.lk)) / h;
                let e = Touch { wheel: key, ball, depth: ct.depth, force: ct.lam_n / h, x: ct.x, n: ct.n, torque };
                let list = match seen.iter_mut().position(|(w, _)| *w == key) { Some(i) => &mut seen[i].1, None => { seen.push((key, Vec::new())); &mut seen.last_mut().unwrap().1 } };
                match list.iter_mut().find(|t| t.ball == ball) {
                    Some(was) => if ct.lam_n / h > was.force { *was = e; },
                    None => list.push(e),
                }
                if k == n - 1 { self.dents.push(Dent { wheel: key, ball, depth: ct.depth, n: ct.n }); }
            }
        }
        if n == 1 {
            let f = self.g_share * dt;
            for st in &sts {
                let b = &mut bodies[st.b];
                b.set_linvel(Vector::new((st.v[0] - g[0] * f) as f32, (st.v[1] - g[1] * f) as f32, (st.v[2] - g[2] * f) as f32), true);
                b.set_angvel(Vector::new(st.w[0] as f32, st.w[1] as f32, st.w[2] as f32), true);
            }
        } else {
            for st in &sts {
                let vv = sub(st.v, mul(g, dt));
                let p = sub(st.c, mul(add(vv, mul(g, self.g_share * dt)), dt));
                let b = &mut bodies[st.b];
                b.set_translation(Vector::new(p[0] as f32, p[1] as f32, p[2] as f32), true);
                b.set_linvel(Vector::new(vv[0] as f32, vv[1] as f32, vv[2] as f32), true);
                b.set_angvel(Vector::new(st.w[0] as f32, st.w[1] as f32, st.w[2] as f32), true);
            }
        }
        if let (Some(rc), Some(hb)) = (reaction, self.body) { rc.apply(&mut bodies[hb]); }
        self.contacts = seen.into_iter().flat_map(|(_, l)| l).collect();
    }

    /// js `_contacts(W, fr, at, state)`
    #[allow(clippy::too_many_arguments)]
    fn contacts_at(
        &self, wp: &[Placed], fr: &Frame, at: &[(RigidBodyHandle, V3)], bodies: &RigidBodySet, drives: &Drives, all_surfaces: &mut [Surface],
        together: Option<&HashSet<RigidBodyHandle>>, skip: Option<&Skip>, sts: &mut Vec<St>, st_of: &mut HashMap<RigidBodyHandle, usize>,
        state: &dyn Fn(RigidBodyHandle, &RigidBodySet, &mut Vec<St>, &mut HashMap<RigidBodyHandle, usize>) -> usize,
    ) -> Vec<Contact> {
        let mut cs = Vec::new();
        let sb = { let refs: Vec<&Surface> = self.surfaces.iter().map(|&i| &all_surfaces[i]).collect(); surface_bounds(&refs) };
        for &(b, bc) in at {
            let in_frame = if self.surfaces.is_empty() { [0.0; 3] } else { qrot_inv(fr.q, sub(bc, fr.p)) };
            for (si, &sidx) in self.surfaces.iter().enumerate() {
                if let Some(sk) = skip { if sk.surfaces.contains(&sidx) && sk.balls.contains(&b) { continue } }
                let o = si * 6;
                if sb[o] <= sb[o + 3] && (in_frame[0] < sb[o] || in_frame[0] > sb[o + 3] || in_frame[1] < sb[o + 1] || in_frame[1] > sb[o + 4] || in_frame[2] < sb[o + 2] || in_frame[2] > sb[o + 5]) { continue }
                let sf = &mut all_surfaces[sidx];
                let hits = sf.touches(sf.to_local(in_frame), b);
                for hit in hits {
                    let st = state(b, bodies, sts, st_of);
                    let n = qrot(fr.q, sf.dir_to_frame(hit.n));
                    let x = sub(bc, mul(n, R - hit.depth / 2.0));
                    let rb = sub(x, bc);
                    let (mut a, mut rw, mut shaft) = ([0.0, 0.0, 1.0], [0.0; 3], None);
                    match sf.mount {
                        Mount::Slide { shaft: s, dir, k } => {
                            let l = mul(qrot(fr.q, dir), k);
                            let az = if m::abs(dir[2]) < 0.9 { [0.0, 0.0, 1.0] } else { [1.0, 0.0, 0.0] };
                            a = qrot(fr.q, az);
                            rw = cross(l, a);
                            shaft = Some(s);
                        }
                        Mount::Pivot { shaft: s, pivot, axis } => {
                            a = qrot(fr.q, axis);
                            let d = sub(x, add(fr.p, qrot(fr.q, pivot)));
                            rw = sub(d, mul(a, dot(d, a)));
                            shaft = Some(s);
                        }
                        Mount::Fixed => {}
                    }
                    let t1 = norm(cross(n, if m::abs(n[1]) < 0.9 { [0.0, 1.0, 0.0] } else { [1.0, 0.0, 0.0] }));
                    let t2 = cross(n, t1);
                    let (f, k) = squeeze(hit.depth, sf.tread, 1.0);
                    let fric = [t1, t2].map(|u| { let ru = cross(rb, u); Fric { d: Dir { u, ..Default::default() }, mu: sf.mu, m: 1.0 / (1.0 / MB + dot(ru, ru) / IB), lam: 0.0 } }).to_vec();
                    cs.push(new_contact(st, None, Toucher::Surface(sidx), shaft, a, n, hit.depth, x, rb, [0.0; 3], rw, f, k, 2.0 * ZETA * m::sqrt(k * MB), fric, true, None, None, true));
                }
            }
            for pw in wp {
                let wh = &self.wheels[pw.i];
                let (c, a) = (pw.c, pw.a);
                let hw = wh.width / 2.0;
                let (dx, dy, dz) = (bc[0] - c[0], bc[1] - c[1], bc[2] - c[2]);
                let s = dx * a[0] + dy * a[1] + dz * a[2];
                if m::abs(s) > hw + R { continue }
                let reach = wh.radius + R;
                if dx * dx + dy * dy + dz * dz - s * s >= reach * reach { continue }
                let d = [dx, dy, dz];
                let rad = sub(d, mul(a, s));
                let rl = len(rad);
                let (n, depth, rim, side);
                if m::abs(s) <= hw {
                    if rl < 1e-6 { continue }
                    n = mul(rad, 1.0 / rl);
                    depth = R + wh.radius - rl;
                    rim = add(c, add(mul(a, s), mul(n, wh.radius)));
                    side = false;
                } else if wh.covered[if s > 0.0 { 1 } else { 0 }] {
                    continue;
                } else if rl <= wh.radius {
                    n = mul(a, m::sign(s));
                    depth = R - (m::abs(s) - hw);
                    rim = add(c, add(mul(a, m::sign(s) * hw), rad));
                    side = true;
                } else {
                    let e = add(c, add(mul(a, m::sign(s) * hw), mul(if rl > 1e-6 { mul(rad, 1.0 / rl) } else { [0.0; 3] }, wh.radius)));
                    let q = sub(bc, e);
                    let ql = len(q);
                    if ql < 1e-6 { continue }
                    n = mul(q, 1.0 / ql);
                    depth = R - ql;
                    rim = e;
                    side = false;
                }
                if depth <= 0.0 { continue }
                let st = state(b, bodies, sts, st_of);
                let x = sub(bc, mul(n, R - depth / 2.0));
                let rb = sub(x, bc);
                let rw = sub(rim, add(c, mul(a, dot(sub(rim, c), a))));
                let t1 = cross(a, n);
                let t1 = if len(t1) < 1e-6 { norm(cross(n, [1.0, 0.0, 0.0])) } else { norm(t1) };
                let t2 = cross(n, t1);
                let dirs: [(V3, f64); 2] = if side { [(t1, MU_SIDE), (t2, MU_SIDE)] }
                    else if wh.kind == Kind::Omni { [(t1, wh.mu), (t2, MU_FREE)] }
                    else if wh.kind == Kind::Mecanum {
                        let gg = (wh.mecanum * m::PI) / 180.0;
                        let (cg, sg) = (m::cos(gg), m::sin(gg));
                        [(add(mul(t1, cg), mul(t2, sg)), wh.mu), (add(mul(t1, -sg), mul(t2, cg)), MU_FREE)]
                    } else { [(t1, wh.mu), (t2, wh.mu)] };
                let (f, k) = squeeze(depth, if side { f64::INFINITY } else { wh.tread }, if side || !self.curved { 1.0 } else { curvature(wh.radius) });
                let cdamp = 2.0 * ZETA * m::sqrt(k * MB);
                let piv = match (pw.p, wh.pivot) {
                    (Some(p), Some((ps, _, _))) => { let dp = sub(x, p); Some(Piv { shaft: ps, a: pw.pa, r: sub(dp, mul(pw.pa, dot(dp, pw.pa))) }) }
                    _ => None,
                };
                let pw_of = |u: V3| piv.map_or(0.0, |pv| { let q = dot(cross(pv.r, u), pv.a); q * q / drives.j(pv.shaft) });
                let jj = drives.j(wh.shaft);
                let fric = dirs.iter().map(|&(u, mu)| {
                    let ru = cross(rb, u);
                    let wu = dot(cross(rw, u), a);
                    let inv = 1.0 / MB + dot(ru, ru) / IB + (wu * wu) / jj + pw_of(u);
                    Fric { d: Dir { u, ..Default::default() }, mu, m: 1.0 / inv, lam: 0.0 }
                }).collect();
                let im = piv.map(|_| 1.0 / MB + pw_of(n));
                cs.push(new_contact(st, None, Toucher::Wheel(pw.i), Some(wh.shaft), a, n, depth, x, rb, [0.0; 3], rw, f, k, cdamp, fric, side || wh.kind == Kind::Plain, piv, im, false));
            }
        }
        if let Some(tg) = together {
            let t: Vec<(RigidBodyHandle, V3)> = at.iter().filter(|(b, _)| tg.contains(b)).copied().collect();
            for i in 0..t.len() {
                for j in i + 1..t.len() {
                    let ((b1, c1), (b2, c2)) = (t[i], t[j]);
                    let (ex, ey, ez) = (c1[0] - c2[0], c1[1] - c2[1], c1[2] - c2[2]);
                    if ex * ex + ey * ey + ez * ez >= 4.0 * R * R { continue }
                    let d = [ex, ey, ez];
                    let dl = len(d);
                    let depth = 2.0 * R - dl;
                    if depth <= 0.0 || dl < 1e-6 { continue }
                    let st = state(b1, bodies, sts, st_of);
                    let st2 = state(b2, bodies, sts, st_of);
                    let n = mul(d, 1.0 / dl);
                    let x = sub(c1, mul(n, dl / 2.0));
                    let (rb, rb2) = (sub(x, c1), sub(x, c2));
                    let t1 = norm(cross(n, if m::abs(n[1]) < 0.9 { [0.0, 1.0, 0.0] } else { [1.0, 0.0, 0.0] }));
                    let t2 = cross(n, t1);
                    let (ff, fk) = foam(depth / 2.0);
                    let k = fk / 2.0;
                    let fric = [t1, t2].map(|u| { let (r1, r2) = (cross(rb, u), cross(rb2, u)); Fric { d: Dir { u, ..Default::default() }, mu: MU_FUEL, m: 1.0 / (2.0 / MB + (dot(r1, r1) + dot(r2, r2)) / IB), lam: 0.0 } }).to_vec();
                    cs.push(new_contact(st, Some(st2), Toucher::Fuel, None, [0.0, 0.0, 1.0], n, depth, x, rb, rb2, [0.0; 3], ff, k, 2.0 * ZETA * m::sqrt(k * MB / 2.0), fric, true, None, Some(2.0 / MB), false));
                }
            }
        }
        cs
    }

    /// js `_solve(C, h, fr, reaction, frameW)` (projected Gauss-Seidel)
    #[allow(clippy::too_many_arguments)]
    fn solve(&self, cs: &mut [Contact], h: f64, fr: &Frame, mut reaction: Option<&mut Reaction>, bodies: &RigidBodySet, drives: &mut Drives, sts: &mut [St], frame_w: V3) {
        let twist = self.sub > 1;
        for ct in cs.iter_mut() { prepare(ct, h, fr, reaction.as_deref_mut()); }
        let rel_vel = |ct: &Contact, sts: &[St], drives: &Drives| -> V3 {
            let st = &sts[ct.st];
            let (v, w, rb) = (st.v, st.w, ct.rb);
            let mut r0 = v[0] + (w[1] * rb[2] - w[2] * rb[1]);
            let mut r1 = v[1] + (w[2] * rb[0] - w[0] * rb[2]);
            let mut r2 = v[2] + (w[0] * rb[1] - w[1] * rb[0]);
            if let Some(s2) = ct.st2 {
                let (v2, w2, q) = (sts[s2].v, sts[s2].w, ct.rb2);
                r0 = r0 - (v2[0] + (w2[1] * q[2] - w2[2] * q[1]));
                r1 = r1 - (v2[1] + (w2[2] * q[0] - w2[0] * q[2]));
                r2 = r2 - (v2[2] + (w2[0] * q[1] - w2[1] * q[0]));
            } else {
                let om = ct.shaft.map_or(0.0, |s| drives.omega(s));
                let (fv, ca) = (ct.fv, ct.ca);
                let (mut w0, mut w1, mut w2) = (fv[0] + ca[0] * om, fv[1] + ca[1] * om, fv[2] + ca[2] * om);
                if let Some(pv) = ct.piv { let po = drives.omega(pv.shaft); let cp = ct.cp; w0 = w0 + cp[0] * po; w1 = w1 + cp[1] * po; w2 = w2 + cp[2] * po; }
                r0 = r0 - w0; r1 = r1 - w1; r2 = r2 - w2;
            }
            [r0, r1, r2]
        };
        let along = |rv: V3, u: V3| rv[0] * u[0] + rv[1] * u[1] + rv[2] * u[2];
        let apply = |ct: &Contact, d: &Dir, lam: f64, sts: &mut [St], drives: &mut Drives, reaction: &mut Option<&mut Reaction>| {
            let (u, cb, kv, kw) = (d.u, d.cb, lam / MB, lam / IB);
            {
                let st = &mut sts[ct.st];
                st.v[0] = st.v[0] + u[0] * kv; st.v[1] = st.v[1] + u[1] * kv; st.v[2] = st.v[2] + u[2] * kv;
                st.w[0] = st.w[0] + cb[0] * kw; st.w[1] = st.w[1] + cb[1] * kw; st.w[2] = st.w[2] + cb[2] * kw;
            }
            if let Some(s2) = ct.st2 {
                let (kv2, kw2, cb2) = (-lam / MB, -lam / IB, d.cb2);
                let st2 = &mut sts[s2];
                st2.v[0] = st2.v[0] + u[0] * kv2; st2.v[1] = st2.v[1] + u[1] * kv2; st2.v[2] = st2.v[2] + u[2] * kv2;
                st2.w[0] = st2.w[0] + cb2[0] * kw2; st2.w[1] = st2.w[1] + cb2[1] * kw2; st2.w[2] = st2.w[2] + cb2[2] * kw2;
                return;
            }
            let l = -lam * d.lk;
            if let Some(s) = ct.shaft { drives.impulse(s, l); }
            if let Some(pv) = ct.piv { drives.impulse(pv.shaft, -lam * d.pk); }
            if let Some(rc) = reaction.as_deref_mut() { rc.add(u[0] * -lam, u[1] * -lam, u[2] * -lam, ct.rx, ct.ry, ct.rz, bodies, self.body); }
        };
        for _ in 0..self.iterations {
            for ci in 0..cs.len() {
                let rv = rel_vel(&cs[ci], sts, drives);
                let vn = along(rv, cs[ci].n);
                let ct = &cs[ci];
                let target = (h * ct.f - ct.hk * vn + ct.g * ct.lam_n) / (1.0 + ct.g);
                let lam_n = m::max(0.0, target);
                let dn = lam_n - ct.lam_n;
                if dn != 0.0 { let d = ct.dn; apply(&cs[ci], &d, dn, sts, drives, &mut reaction); cs[ci].lam_n = lam_n; }
                if cs[ci].plain {
                    let rv = rel_vel(&cs[ci], sts, drives);
                    let (f1, f2) = (cs[ci].fric[0], cs[ci].fric[1]);
                    let mut l1 = f1.lam - along(rv, f1.d.u) * f1.m;
                    let mut l2 = f2.lam - along(rv, f2.d.u) * f2.m;
                    let lim = f1.mu * cs[ci].lam_n;
                    let mm = m::hypot2(l1, l2);
                    if mm > lim { l1 *= lim / mm; l2 *= lim / mm; }
                    apply(&cs[ci], &f1.d, l1 - f1.lam, sts, drives, &mut reaction); cs[ci].fric[0].lam = l1;
                    apply(&cs[ci], &f2.d, l2 - f2.lam, sts, drives, &mut reaction); cs[ci].fric[1].lam = l2;
                } else {
                    for fi in 0..cs[ci].fric.len() {
                        let f = cs[ci].fric[fi];
                        let lim = f.mu * cs[ci].lam_n;
                        let rv = rel_vel(&cs[ci], sts, drives);
                        let l = m::max(-lim, m::min(lim, f.lam - along(rv, f.d.u) * f.m));
                        apply(&cs[ci], &f.d, l - f.lam, sts, drives, &mut reaction);
                        cs[ci].fric[fi].lam = l;
                    }
                }
                // twist (a sub-stepped mechanism's wheels)
                let ct = &cs[ci];
                if twist && ct.depth > 0.0 && !(ct.st2.is_some() || ct.is_surface) {
                    let n = ct.n;
                    let sw = sts[ct.st].w;
                    let om = ct.shaft.map_or(0.0, |s| drives.omega(s));
                    let a = ct.a;
                    let (o0, o1, o2) = (frame_w[0] + a[0] * om, frame_w[1] + a[1] * om, frame_w[2] + a[2] * om);
                    let wr = (sw[0] - o0) * n[0] + (sw[1] - o1) * n[1] + (sw[2] - o2) * n[2];
                    let lim = TWIST_K * ct.fric[0].mu * m::sqrt(R * ct.depth) * ct.lam_n;
                    let im = 1.0 / IB;
                    let l = m::max(-lim, m::min(lim, ct.lam_t - wr / im));
                    let dl = l - ct.lam_t;
                    if dl != 0.0 {
                        let k1 = dl / IB;
                        let w = &mut sts[ct.st].w;
                        w[0] = w[0] + n[0] * k1; w[1] = w[1] + n[1] * k1; w[2] = w[2] + n[2] * k1;
                        cs[ci].lam_t = l;
                    }
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn new_contact(st: usize, st2: Option<usize>, who: Toucher, shaft: Option<usize>, a: V3, n: V3, depth: f64, x: V3, rb: V3, rb2: V3, rw: V3, f: f64, k: f64, c: f64, fric: Vec<Fric>, plain: bool, piv: Option<Piv>, im: Option<f64>, is_surface: bool) -> Contact {
    Contact { st, st2, who, shaft, a, n, depth, x, rb, rb2, rw, f, k, c, lam_n: 0.0, fric, plain, piv, im, is_surface, hk: 0.0, g: 0.0, dn: Dir::default(), fv: [0.0; 3], ca: [0.0; 3], cp: [0.0; 3], rx: 0.0, ry: 0.0, rz: 0.0, lam_t: 0.0 }
}

/// js `prepare(ct, h, fr, reaction)`
fn prepare(ct: &mut Contact, h: f64, fr: &Frame, reaction: Option<&mut Reaction>) {
    ct.hk = h * (ct.k * h + ct.c);
    ct.g = ct.hk * ct.im.unwrap_or(1.0 / MB);
    let (rw, rb, a, has2, piv, rb2) = (ct.rw, ct.rb, ct.a, ct.st2.is_some(), ct.piv, ct.rb2);
    let dir = |d: &mut Dir| {
        d.cb = cross(rb, d.u);
        if has2 { d.cb2 = cross(rb2, d.u); return; }
        d.lk = dot(cross(rw, d.u), a);
        if let Some(pv) = piv { d.pk = dot(cross(pv.r, d.u), pv.a); }
    };
    ct.dn = Dir { u: ct.n, ..Default::default() };
    dir(&mut ct.dn);
    for f in ct.fric.iter_mut() { dir(&mut f.d); }
    if has2 { return; }
    let (w, c, x) = (fr.w, fr.com, ct.x);
    let (dx, dy, dz) = (x[0] - c[0], x[1] - c[1], x[2] - c[2]);
    ct.fv = [fr.v[0] + (w[1] * dz - w[2] * dy), fr.v[1] + (w[2] * dx - w[0] * dz), fr.v[2] + (w[0] * dy - w[1] * dx)];
    ct.ca = cross(a, rw);
    if let Some(pv) = piv { ct.cp = cross(pv.a, pv.r); }
    if let Some(rc) = reaction { rc.at(ct); }
}

/// js `Reaction`: what a mechanism's contacts push back on its body with, added up in 32-bit floats
/// as Rapier's applyImpulseAtPoint adds each, and set once at the end
struct Reaction { com: V3, lv: V3, av: V3, ready: bool, any: bool, m: [f32; 3], e: [f32; 6], l: [f32; 3], a: [f32; 3] }

impl Reaction {
    fn new(fr0: Frame) -> Self { Reaction { com: fr0.com, lv: fr0.v, av: fr0.w, ready: false, any: false, m: [0.0; 3], e: [0.0; 6], l: [0.0; 3], a: [0.0; 3] } }
    fn init(&mut self, b: &RigidBody) {
        let mp = b.mass_properties();
        let im = mp.effective_inv_mass;
        let ii = mp.effective_world_inv_inertia;
        self.m = [im.x, im.y, im.z];
        self.e = [ii.m11, ii.m12, ii.m13, ii.m22, ii.m23, ii.m33];
        self.l = [self.lv[0] as f32, self.lv[1] as f32, self.lv[2] as f32];
        self.a = [self.av[0] as f32, self.av[1] as f32, self.av[2] as f32];
        self.ready = true;
    }
    fn at(&self, ct: &mut Contact) {
        let (c, x) = (self.com, ct.x);
        ct.rx = ((x[0] as f32) as f64 - c[0]) as f32;
        ct.ry = ((x[1] as f32) as f64 - c[1]) as f32;
        ct.rz = ((x[2] as f32) as f64 - c[2]) as f32;
    }
    #[allow(clippy::too_many_arguments)]
    fn add(&mut self, jx: f64, jy: f64, jz: f64, rx: f32, ry: f32, rz: f32, bodies: &RigidBodySet, body: Option<RigidBodyHandle>) {
        if !self.ready { if let Some(b) = body.and_then(|h| bodies.get(h)) { self.init(b); } }
        let (jx, jy, jz) = (jx as f32, jy as f32, jz as f32);
        let tx = ry * jz - rz * jy;
        let ty = rz * jx - rx * jz;
        let tz = rx * jy - ry * jx;
        if jx != 0.0 || jy != 0.0 || jz != 0.0 {
            self.l[0] += jx * self.m[0]; self.l[1] += jy * self.m[1]; self.l[2] += jz * self.m[2];
            self.any = true;
        }
        if tx != 0.0 || ty != 0.0 || tz != 0.0 {
            let e = self.e;
            self.a[0] += e[0] * tx + e[1] * ty + e[2] * tz;
            self.a[1] += e[1] * tx + e[3] * ty + e[4] * tz;
            self.a[2] += e[2] * tx + e[4] * ty + e[5] * tz;
            self.any = true;
        }
    }
    fn apply(&self, b: &mut RigidBody) {
        if !self.any { return; }
        b.set_linvel(Vector::new(self.l[0], self.l[1], self.l[2]), true);
        b.set_angvel(Vector::new(self.a[0], self.a[1], self.a[2]), true);
    }
}
