//! js/nav.js: grid A* navigation for the AI drivers. The only things a robot can't drive over or
//! under are the HUBS, the bump-side TRENCH skirts, the TOWERS and the field perimeter; BUMPS and
//! the TRENCH openings are free space (except for a robot too tall for the TRENCH).
use crate::constants::*;
use crate::field::hub_center;
use jsmath as m;

const CELL: f64 = 0.2;
const SQ2: f64 = std::f64::consts::SQRT_2;
fn nx() -> usize { m::ceil(2.0 * HALF_L / CELL) as usize }
fn nz() -> usize { m::ceil(2.0 * HALF_W / CELL) as usize }

/// An axis-aligned obstacle (center, half extents)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect { pub x: f64, pub z: f64, pub hx: f64, pub hz: f64, pub kind: Kind, pub alliance: Alliance }
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind { Hub, Trench, Tower }

/// js `OBSTACLES`
pub fn obstacles() -> &'static [Rect] {
    static O: std::sync::OnceLock<Vec<Rect>> = std::sync::OnceLock::new();
    O.get_or_init(|| {
        let mut out = Vec::new();
        for a in [Alliance::Blue, Alliance::Red] {
            let s = side_sign(a);
            let hx = hub_center(a).0;
            let hs = HUB::size / 2.0;
            out.push(Rect { x: hx, z: 0.0, hx: hs, hz: hs, kind: Kind::Hub, alliance: a });
            let (z_open, z_post) = (HALF_W - TRENCH::clearWidth, HALF_W - TRENCH::width);
            for sz in [1.0, -1.0] {
                out.push(Rect { x: hx, z: (sz * (z_open + z_post)) / 2.0, hx: TRENCH::depth / 2.0, hz: (z_open - z_post) / 2.0, kind: Kind::Trench, alliance: a });
            }
            out.push(Rect { x: s * (-HALF_L + TOWER::baseDepth / 2.0), z: s * (HALF_W - TOWER::fy), hx: TOWER::baseDepth / 2.0, hz: TOWER::width / 2.0, kind: Kind::Tower, alliance: a });
        }
        out
    })
}

/// js `towerRect(alliance)`
pub fn tower_rect(a: Alliance) -> Rect { *obstacles().iter().find(|o| o.kind == Kind::Tower && o.alliance == a).unwrap() }

/// js `obstacleAt(x, z, pad)`: inside an obstacle grown by pad, or outside the field
pub fn obstacle_at(x: f64, z: f64, pad: f64) -> bool {
    if m::abs(x) > HALF_L - pad || m::abs(z) > HALF_W - pad { return true; }
    obstacles().iter().any(|o| m::abs(x - o.x) < o.hx + pad && m::abs(z - o.z) < o.hz + pad)
}

/// js `TRENCH_OPENINGS`: for a robot too tall to fit under the arm
pub fn trench_openings() -> &'static [Rect] {
    static O: std::sync::OnceLock<Vec<Rect>> = std::sync::OnceLock::new();
    O.get_or_init(|| {
        let mut out = Vec::new();
        for a in [Alliance::Blue, Alliance::Red] {
            let (hx, z_open) = (hub_center(a).0, HALF_W - TRENCH::clearWidth);
            for sz in [1.0, -1.0] {
                out.push(Rect { x: hx, z: (sz * (z_open + HALF_W)) / 2.0, hx: TRENCH::depth / 2.0, hz: (HALF_W - z_open) / 2.0, kind: Kind::Trench, alliance: a });
            }
        }
        out
    })
}

/// js `underTrench(x, z, margin)`
pub fn under_trench(x: f64, z: f64, margin: f64) -> bool {
    trench_openings().iter().any(|o| m::abs(x - o.x) < o.hx + margin && m::abs(z - o.z) < o.hz + margin)
}

/// js `Heap`: a minimal binary heap keyed by f-score (the same swaps, so ties pop in its order)
struct Heap { a: Vec<(f64, usize)> }
impl Heap {
    fn push(&mut self, n: usize, f: f64) {
        let a = &mut self.a;
        a.push((f, n));
        let mut i = a.len() - 1;
        while i > 0 {
            let p = (i - 1) >> 1;
            if a[p].0 <= a[i].0 { break; }
            a.swap(p, i);
            i = p;
        }
    }
    fn pop(&mut self) -> usize {
        let a = &mut self.a;
        let top = a[0];
        let last = a.pop().unwrap();
        if !a.is_empty() {
            a[0] = last;
            let mut i = 0;
            loop {
                let (l, r) = (2 * i + 1, 2 * i + 2);
                let mut mm = i;
                if l < a.len() && a[l].0 < a[mm].0 { mm = l; }
                if r < a.len() && a[r].0 < a[mm].0 { mm = r; }
                if mm == i { break; }
                a.swap(mm, i);
                i = mm;
            }
        }
        top.1
    }
}

/// A point (x, z)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct P { pub x: f64, pub z: f64 }
/// A moving robot to route around (x, z, radius)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Circle { pub x: f64, pub z: f64, pub r: f64 }

/// js `NavGrid`
pub struct NavGrid {
    r: f64,
    nx: usize,
    nz: usize,
    blocked: Vec<u8>,
    /// (Float32Array: its values rounded to f32, as the JS stores them)
    g: Vec<f32>,
    from: Vec<i32>,
    stamp: Vec<u32>,
    closed: Vec<u32>,
    dyn_: Vec<u32>,
    run: u32,
    dyn_count: u32,
    dyn_run: i64,
}

impl NavGrid {
    /// trench: false for a robot that doesn't fit under the TRENCH (it has to use the BUMPS)
    pub fn new(radius: f64, trench: bool) -> NavGrid {
        let (nx, nz) = (nx(), nz());
        let mut blocked = vec![0u8; nx * nz];
        for i in 0..nx {
            for j in 0..nz {
                let x = -HALF_L + (i as f64 + 0.5) * CELL;
                let z = -HALF_W + (j as f64 + 0.5) * CELL;
                let under_arm = !trench && trench_openings().iter().any(|o| m::abs(x - o.x) < o.hx + radius && m::abs(z - o.z) < o.hz + radius);
                blocked[i * nz + j] = if under_arm || obstacle_at(x, z, radius) { 1 } else { 0 };
            }
        }
        NavGrid { r: radius, nx, nz, blocked, g: vec![0.0; nx * nz], from: vec![0; nx * nz], stamp: vec![0; nx * nz], closed: vec![0; nx * nz], dyn_: vec![0; nx * nz], run: 0, dyn_count: 0, dyn_run: -1 }
    }

    fn blocked_(&self, n: usize) -> bool { self.blocked[n] != 0 || self.dyn_[n] as i64 == self.dyn_run }

    fn mark_dyn(&mut self, circles: &[Circle]) {
        self.dyn_count += 1;
        self.dyn_run = self.dyn_count as i64;
        let (nx, nz) = (self.nx as f64, self.nz as f64);
        for c in circles {
            let rr = c.r + self.r;
            let i0 = m::max(0.0, m::floor((c.x - rr + HALF_L) / CELL)) as i64;
            let i1 = m::min(nx - 1.0, m::floor((c.x + rr + HALF_L) / CELL)) as i64;
            let j0 = m::max(0.0, m::floor((c.z - rr + HALF_W) / CELL)) as i64;
            let j1 = m::min(nz - 1.0, m::floor((c.z + rr + HALF_W) / CELL)) as i64;
            for i in i0..=i1 {
                for j in j0..=j1 {
                    let x = -HALF_L + (i as f64 + 0.5) * CELL;
                    let z = -HALF_W + (j as f64 + 0.5) * CELL;
                    if m::hypot2(x - c.x, z - c.z) < rr { self.dyn_[i as usize * self.nz + j as usize] = self.dyn_count; }
                }
            }
        }
    }

    pub fn cell(&self, x: f64, z: f64) -> usize {
        let i = m::min(self.nx as f64 - 1.0, m::max(0.0, m::floor((x + HALF_L) / CELL))) as usize;
        let j = m::min(self.nz as f64 - 1.0, m::max(0.0, m::floor((z + HALF_W) / CELL))) as usize;
        i * self.nz + j
    }

    pub fn center(&self, c: usize) -> P {
        let (i, j) = (c / self.nz, c % self.nz);
        P { x: -HALF_L + (i as f64 + 0.5) * CELL, z: -HALF_W + (j as f64 + 0.5) * CELL }
    }

    pub fn free(&self, x: f64, z: f64) -> bool { !self.blocked_(self.cell(x, z)) }

    /// nearest free cell (breadth-first rings)
    fn nearest_free(&self, c: usize) -> usize {
        if !self.blocked_(c) { return c; }
        let (i0, j0) = ((c / self.nz) as i64, (c % self.nz) as i64);
        for rad in 1..20i64 {
            let (mut best, mut bd) = (-1i64, 1e9f64);
            for di in -rad..=rad {
                for dj in -rad..=rad {
                    if di.abs().max(dj.abs()) != rad { continue; }
                    let (i, j) = (i0 + di, j0 + dj);
                    if i < 0 || j < 0 || i >= self.nx as i64 || j >= self.nz as i64 { continue; }
                    let n = (i * self.nz as i64 + j) as usize;
                    let d = (di * di + dj * dj) as f64;
                    if !self.blocked_(n) && d < bd { bd = d; best = n as i64; }
                }
            }
            if best >= 0 { return best as usize; }
        }
        c
    }

    /// a straight line between two points stays in free cells
    pub fn los(&self, x0: f64, z0: f64, x1: f64, z1: f64) -> bool {
        let d = m::hypot2(x1 - x0, z1 - z0);
        let n = m::ceil(d / (CELL * 0.5)) as u64;
        for k in 0..=n {
            let t = if n != 0 { k as f64 / n as f64 } else { 0.0 };
            if self.blocked_(self.cell(x0 + (x1 - x0) * t, z0 + (z1 - z0) * t)) { return false; }
        }
        true
    }

    /// js `plan(x0, z0, x1, z1, avoid)`: waypoints from (x0, z0) to (x1, z1), the start left out;
    /// a goal inside an obstacle ends at the nearest reachable point. avoid: moving robots.
    pub fn plan(&mut self, x0: f64, z0: f64, x1: f64, z1: f64, avoid: &[Circle]) -> Vec<P> {
        if !avoid.is_empty() { self.mark_dyn(avoid); }
        let out = self.plan_(x0, z0, x1, z1);
        self.dyn_run = -1;
        out
    }

    fn plan_(&mut self, x0: f64, z0: f64, x1: f64, z1: f64) -> Vec<P> {
        if self.los(x0, z0, x1, z1) { return vec![P { x: x1, z: z1 }]; }
        let s = self.nearest_free(self.cell(x0, z0));
        let g_cell = self.cell(x1, z1);
        let goal_blocked = self.blocked_(g_cell);
        let t = self.nearest_free(g_cell);
        self.run += 1;
        let run = self.run;
        let tc = self.center(t);
        let nz = self.nz;
        let h = |this: &NavGrid, c: usize| {
            let p = this.center(c);
            let (dx, dz) = (m::abs(p.x - tc.x) / CELL, m::abs(p.z - tc.z) / CELL);
            m::max(dx, dz) + (SQ2 - 1.0) * m::min(dx, dz)
        };
        let mut open = Heap { a: Vec::new() };
        self.stamp[s] = run; self.g[s] = 0.0; self.from[s] = -1;
        let hs = h(self, s);
        open.push(s, hs);
        let mut found = false;
        let mut iter = 0;
        while !open.a.is_empty() && { let go = iter < 20000; iter += 1; go } {
            let c = open.pop();
            if self.closed[c] == run { continue; }
            self.closed[c] = run;
            if c == t { found = true; break; }
            let (ci, cj) = ((c / nz) as i64, (c % nz) as i64);
            for di in -1i64..=1 {
                for dj in -1i64..=1 {
                    if di == 0 && dj == 0 { continue; }
                    let (i, j) = (ci + di, cj + dj);
                    if i < 0 || j < 0 || i >= self.nx as i64 || j >= nz as i64 { continue; }
                    let n = (i * nz as i64 + j) as usize;
                    if self.blocked_(n) || self.closed[n] == run { continue; }
                    // no corner cutting
                    if di != 0 && dj != 0 && (self.blocked_((ci * nz as i64 + j) as usize) || self.blocked_((i * nz as i64 + cj) as usize)) { continue; }
                    let ng = self.g[c] as f64 + if di != 0 && dj != 0 { SQ2 } else { 1.0 };
                    if self.stamp[n] != run || ng < self.g[n] as f64 {
                        self.stamp[n] = run; self.g[n] = ng as f32; self.from[n] = c as i32;
                        let hn = h(self, n);
                        open.push(n, ng + hn);
                    }
                }
            }
        }
        if !found { return vec![P { x: x1, z: z1 }]; }
        let mut cells = Vec::new();
        let mut c = t as i32;
        while c != -1 { cells.push(c as usize); c = self.from[c as usize]; }
        cells.reverse();
        let mut pts: Vec<P> = cells.iter().map(|&c| self.center(c)).collect();
        if !goal_blocked { let n = pts.len(); pts[n - 1] = P { x: x1, z: z1 }; }
        // string pulling
        let mut out = Vec::new();
        let (mut ax, mut az, mut k) = (x0, z0, 0usize);
        while k < pts.len() {
            let mut far = k;
            let mut mm = pts.len() - 1;
            while mm > k {
                if self.los(ax, az, pts[mm].x, pts[mm].z) { far = mm; break; }
                mm -= 1;
            }
            out.push(pts[far]);
            ax = pts[far].x; az = pts[far].z;
            k = far + 1;
        }
        out
    }
}
