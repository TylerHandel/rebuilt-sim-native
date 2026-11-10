//! js/ballistics.js: FUEL ballistics with quadratic air drag and the Magnus lift of its spin (the
//! same model fuel.rs applies to every airborne FUEL), the shot tables built from it and the
//! shoot-on-the-move solver.
use crate::constants::{FUEL, GRAVITY};
use jsmath as m;

const K: f64 = FUEL::dragK;
const KL: f64 = FUEL::liftK;
const RB: f64 = FUEL::radius;
const SIM_DT: f64 = 1.0 / 240.0;
const DEG: f64 = m::PI / 180.0;

/// js `liftCoef(S)`: the lift coefficient at spin ratio S = R w / v
pub fn lift_coef(s: f64) -> f64 { if s > 0.0 { s / (1.0 + 2.0 * s) } else { 0.0 } }

/// js `magnus(v, w)`: the Magnus acceleration on a FUEL moving at v with spin w (world)
pub fn magnus(v: [f64; 3], w: [f64; 3]) -> [f64; 3] {
    let sp = m::hypot3(v[0], v[1], v[2]);
    if sp < 0.3 { return [0.0; 3]; }
    let a = (w[0] * v[0] + w[1] * v[1] + w[2] * v[2]) / (sp * sp);
    let (px, py, pz) = (w[0] - a * v[0], w[1] - a * v[1], w[2] - a * v[2]);
    let wl = m::hypot3(px, py, pz);
    if wl < 1e-6 { return [0.0; 3]; }
    let f = (KL * lift_coef((RB * wl) / sp) * sp) / wl;
    [f * (py * v[2] - pz * v[1]), f * (pz * v[0] - px * v[2]), f * (px * v[1] - py * v[0])]
}

/// js `spinFor(v, s)`: the spin (world) of a shot along v (x, z) with backspin s
pub fn spin_for(v: [f64; 2], s: f64) -> [f64; 3] {
    let h = m::hypot2(v[0], v[1]);
    if h < 1e-9 || s == 0.0 || s.is_nan() { [0.0; 3] } else { [(-s * v[1]) / h, 0.0, (s * v[0]) / h] }
}

/// js sim2D's result
#[derive(Clone, Copy, Debug)]
pub struct Flight { pub d: f64, pub tof: f64, pub alpha: f64, pub apex: f64, pub clear_y: f64 }

/// js `sim2D(h0, v, th, Ht, clearDist, s)`: a planar shot; where it descends through Ht. (Its
/// path is kept in f32, as the JS game's Float32Arrays keep it: xs, ys, 1802 long.)
#[allow(clippy::too_many_arguments)]
pub fn sim2d(h0: f64, v: f64, th: f64, ht: f64, clear_dist: f64, s: f64, xs: &mut [f32], ys: &mut [f32]) -> Option<Flight> {
    let (mut x, mut y, mut vx, mut vy, mut t, mut apex) = (0.0, h0, v * m::cos(th), v * m::sin(th), 0.0, h0);
    let (as_, sg) = (m::abs(s), m::sign(s));
    for i in 0..1800 {
        xs[i] = x as f32;
        ys[i] = y as f32;
        let sp = m::hypot2(vx, vy);
        let l = if as_ > 0.0 && sp > 0.3 { KL * lift_coef((RB * as_) / sp) * sp * sg } else { 0.0 };
        let ax = -K * sp * vx - l * vy;
        let ay = -GRAVITY - K * sp * vy + l * vx;
        vx += ax * SIM_DT;
        vy += ay * SIM_DT;
        let (nx, ny) = (x + vx * SIM_DT, y + vy * SIM_DT);
        t += SIM_DT;
        if ny > apex { apex = ny; }
        if vy < 0.0 && ny <= ht {
            if y >= ht {
                let f = (y - ht) / (y - ny);
                let d = x + (nx - x) * f;
                let mut clear_y = f64::INFINITY;
                let xc = d - clear_dist;
                if clear_dist > 0.0 && xc > 0.0 {
                    let mut j = i;
                    while j > 0 {
                        let (a, b) = (xs[j - 1] as f64, xs[j] as f64);
                        if a <= xc && b >= xc {
                            let dd = b - a;
                            let g = (xc - a) / (if dd == 0.0 || dd.is_nan() { 1.0 } else { dd });
                            clear_y = ys[j - 1] as f64 + (ys[j] as f64 - ys[j - 1] as f64) * g;
                            break;
                        }
                        j -= 1;
                    }
                }
                return Some(Flight { d, tof: t - SIM_DT + SIM_DT * f, alpha: m::atan2(-vy, vx), apex, clear_y });
            }
            return None;
        }
        x = nx;
        y = ny;
    }
    None
}

/// js `sim3D(p, v, Ht, w)`: the full 3D flight; where it descends through Ht: (x, z, tof)
pub fn sim3d(p: [f64; 3], v: [f64; 3], ht: f64, w: Option<[f64; 3]>) -> Option<(f64, f64, f64)> {
    let (mut x, mut y, mut z) = (p[0], p[1], p[2]);
    let (mut vx, mut vy, mut vz, mut t) = (v[0], v[1], v[2], 0.0);
    for _ in 0..1800 {
        let sp = m::hypot3(vx, vy, vz);
        let a = match w { Some(w) => magnus([vx, vy, vz], w), None => [0.0; 3] };
        vx += (-K * sp * vx + a[0]) * SIM_DT;
        vy += (-GRAVITY - K * sp * vy + a[1]) * SIM_DT;
        vz += (-K * sp * vz + a[2]) * SIM_DT;
        let (nx, ny, nz) = (x + vx * SIM_DT, y + vy * SIM_DT, z + vz * SIM_DT);
        t += SIM_DT;
        if vy < 0.0 && ny <= ht {
            if y >= ht {
                let f = (y - ht) / (y - ny);
                return Some((x + (nx - x) * f, z + (nz - z) * f, t - SIM_DT + SIM_DT * f));
            }
            return None;
        }
        x = nx; y = ny; z = nz;
    }
    None
}

/// js `trajectoryPoints(p, v, maxT, floorY, w)`: x, y, z every 1/60 s
pub fn trajectory_points(p: [f64; 3], v: [f64; 3], max_t: f64, floor_y: f64, w: Option<[f64; 3]>) -> Vec<f64> {
    let mut pts = Vec::new();
    let (mut x, mut y, mut z) = (p[0], p[1], p[2]);
    let (mut vx, mut vy, mut vz) = (v[0], v[1], v[2]);
    let dt = 1.0 / 60.0;
    let mut a = [0.0; 3];
    let mut t = 0.0;
    while t < max_t {
        pts.extend_from_slice(&[x, y, z]);
        for _ in 0..4 {
            let sp = m::hypot3(vx, vy, vz);
            let h = dt / 4.0;
            if let Some(w) = w { a = magnus([vx, vy, vz], w); }
            vx += (-K * sp * vx + a[0]) * h;
            vy += (-GRAVITY - K * sp * vy + a[1]) * h;
            vz += (-K * sp * vz + a[2]) * h;
            x += vx * h; y += vy * h; z += vz * h;
        }
        if y < floor_y { break; }
        t += dt;
    }
    pts
}

/// A shot table's answer (js { theta, v, tof, alpha }); reach false: a pass table's longest throw
#[derive(Clone, Copy, Debug)]
pub struct Sol { pub theta: f64, pub v: f64, pub tof: f64, pub alpha: f64, pub reach: bool }

#[derive(Clone, Copy, Debug)]
struct Pt { v: f64, f: Flight }
#[derive(Clone, Copy, Debug)]
struct Entry { theta: f64, v: f64, tof: f64, alpha: f64 }

/// Where FUEL leave the shooter, by hood angle (js ShotTable's h0: a height, or a function of it)
pub enum H0<'a> { At(f64), Of(&'a dyn Fn(f64) -> f64) }

/// js ShotTable's options
pub struct TableOpts<'a> {
    pub h0: H0<'a>, pub ht: f64, pub hood_min: f64, pub hood_max: f64, pub speed_min: f64, pub speed_max: f64,
    pub d_min: f64, pub d_max: f64, pub step: f64, pub pass: bool, pub pass_theta: f64, pub clear_dist: f64, pub clear_height: f64, pub spin: f64,
}
impl<'a> TableOpts<'a> {
    /// js's defaults: speedMin 2.5, dMin 0.4, dMax 16, step 0.05, 'hub', passTheta 55, no clearance, no spin
    pub fn new(h0: H0<'a>, ht: f64, hood_min: f64, hood_max: f64, speed_max: f64) -> Self {
        TableOpts { h0, ht, hood_min, hood_max, speed_min: 2.5, speed_max, d_min: 0.4, d_max: 16.0, step: 0.05, pass: false, pass_theta: 55.0, clear_dist: 0.0, clear_height: 0.0, spin: 0.0 }
    }
}

/// js `ShotTable`: horizontal distance -> hood angle, exit speed, time of flight
#[derive(Clone, Debug)]
pub struct ShotTable {
    pub ht: f64,
    pub spin: f64,
    d_min: f64,
    step: f64,
    entries: Vec<Option<Entry>>,
    pub min_d: f64,
    pub max_d: f64,
    /// a pass table's longest throw, and how far it goes
    pub far: Option<(Sol, f64)>,
}

impl ShotTable {
    pub fn new(o: &TableOpts) -> Self {
        let mut thetas: Vec<f64> = Vec::new();
        let lob = m::min(m::max(o.pass_theta, o.hood_min), o.hood_max);
        if o.pass {
            thetas.push(lob);
            let mut t = lob - 2.0;
            while t > o.hood_min - 1e-6 { thetas.push(t); t -= 2.0; }
            let r = (lob - o.hood_min) % 2.0;
            if lob > o.hood_min + 1e-6 && r != 0.0 && !r.is_nan() { thetas.push(o.hood_min); }
        } else {
            let mut t = o.hood_min;
            while t <= o.hood_max + 1e-6 { thetas.push(t); t += 1.0; }
        }
        let (mut xs, mut ys) = (vec![0f32; 1802], vec![0f32; 1802]);
        let curves: Vec<(f64, Vec<Pt>)> = thetas.iter().enumerate().map(|(k, &tdeg)| {
            let th = tdeg * DEG;
            let h = match &o.h0 { H0::At(h) => *h, H0::Of(f) => f(tdeg) };
            let mut pts: Vec<Pt> = Vec::new();
            let mut v = if o.pass && k > 0 { m::max(o.speed_min, 0.5 * o.speed_max) } else { o.speed_min };
            while v <= o.speed_max + 1e-6 {
                if let Some(f) = sim2d(h, v, th, o.ht, o.clear_dist, (o.spin * v) / RB, &mut xs, &mut ys) { pts.push(Pt { v, f }); }
                v += 0.1;
            }
            if o.pass { if let Some(last) = pts.last().copied() { if last.v < o.speed_max - 1e-6 {
                if let Some(f) = sim2d(h, o.speed_max, th, o.ht, o.clear_dist, (o.spin * o.speed_max) / RB, &mut xs, &mut ys) { pts.push(Pt { v: o.speed_max, f }); }
            } } }
            (th, pts)
        }).collect();
        let n = m::floor((o.d_max - o.d_min) / o.step) as usize + 1;
        let mut entries: Vec<Option<Entry>> = vec![None; n];
        for (i, e) in entries.iter_mut().enumerate() {
            let dd = o.d_min + i as f64 * o.step;
            let (mut best, mut best_cost) = (None, f64::INFINITY);
            for (th, p) in &curves {
                let mut j = 0;
                while j + 1 < p.len() {
                    let (a, b) = (p[j], p[j + 1]);
                    if (a.f.d - dd) * (b.f.d - dd) > 0.0 { j += 1; continue; }
                    let f = if b.f.d == a.f.d { 0.0 } else { (dd - a.f.d) / (b.f.d - a.f.d) };
                    let cand = Entry { theta: *th, v: a.v + (b.v - a.v) * f, tof: a.f.tof + (b.f.tof - a.f.tof) * f, alpha: a.f.alpha + (b.f.alpha - a.f.alpha) * f };
                    let clear_y = m::min(a.f.clear_y, b.f.clear_y);
                    if o.clear_height != 0.0 && clear_y < o.clear_height { break; }
                    let cost = if !o.pass {
                        let ad = cand.alpha / DEG;
                        cand.tof + (if ad < 52.0 { (52.0 - ad) * 0.06 } else { 0.0 }) + (if ad < 38.0 { 100.0 } else { 0.0 })
                    } else { m::abs(th / DEG - lob) * 100.0 + cand.tof };
                    if cost < best_cost { best_cost = cost; best = Some(cand); }
                    break;
                }
            }
            *e = best;
        }
        let (mut min_d, mut max_d) = (f64::INFINITY, f64::NEG_INFINITY);
        for (i, e) in entries.iter().enumerate() {
            if e.is_none() { continue; }
            let d = o.d_min + i as f64 * o.step;
            min_d = m::min(min_d, d);
            max_d = m::max(max_d, d);
        }
        let mut far: Option<(Sol, f64)> = None;
        if o.pass {
            for (th, p) in &curves {
                if let Some(p) = p.last() {
                    if far.is_none_or(|f| p.f.d > f.1) { far = Some((Sol { theta: *th, v: p.v, tof: p.f.tof, alpha: p.f.alpha, reach: false }, p.f.d)); }
                }
            }
        }
        ShotTable { ht: o.ht, spin: o.spin, d_min: o.d_min, step: o.step, entries, min_d, max_d, far }
    }

    /// js `lookup(d)`
    pub fn lookup(&self, d: f64) -> Option<Sol> {
        if let Some((f, _)) = self.far { if d > self.max_d { return Some(f); } }
        let fi = (d - self.d_min) / self.step;
        let i = m::floor(fi);
        if i.is_nan() || i < 0.0 || i + 1.0 >= self.entries.len() as f64 { return None; }
        let iu = i as usize;
        match (self.entries[iu], self.entries[iu + 1]) {
            (Some(a), Some(b)) => {
                let f = fi - i;
                Some(Sol { theta: a.theta + (b.theta - a.theta) * f, v: a.v + (b.v - a.v) * f, tof: a.tof + (b.tof - a.tof) * f, alpha: a.alpha + (b.alpha - a.alpha) * f, reach: true })
            }
            (a, b) => a.or(b).map(|e| Sol { theta: e.theta, v: e.v, tof: e.tof, alpha: e.alpha, reach: true }),
        }
    }
    /// js `inRange(d)`
    pub fn in_range(&self, d: f64) -> bool { d >= self.min_d && d <= self.max_d && self.lookup(d).is_some() }
}

/// js solveMovingShot's result
#[derive(Clone, Copy, Debug)]
pub struct Moving { pub psi: f64, pub theta: f64, pub v: f64, pub tof: f64, pub dist: f64, pub aim: [f64; 2], pub reach: bool }

/// js `solveMovingShot(table, exit, vel, target)`: exit (world), vel (x, z), target (x, z)
pub fn solve_moving_shot(table: &ShotTable, exit: [f64; 3], vel: [f64; 2], target: [f64; 2]) -> Option<Moving> {
    let (mut ax, mut az) = (target[0], target[1]);
    let d0 = m::hypot2(ax - exit[0], az - exit[2]);
    let straight = || {
        let f = table.far.unwrap().0;
        Some(Moving { psi: m::atan2(-(target[1] - exit[2]), target[0] - exit[0]), theta: f.theta, v: f.v, tof: f.tof, dist: d0, aim: target, reach: false })
    };
    if table.far.is_some() && d0 > table.max_d { return straight(); }
    for _ in 0..5 {
        let d = m::hypot2(ax - exit[0], az - exit[2]);
        let sol = table.lookup(d)?;
        if !sol.reach { return straight(); }
        ax = target[0] - vel[0] * sol.tof;
        az = target[1] - vel[1] * sol.tof;
    }
    // (refined against the full 3D flight: drag couples the launcher's velocity and the shot)
    for _ in 0..3 {
        let (dx, dz) = (ax - exit[0], az - exit[2]);
        let d = m::hypot2(dx, dz);
        let sol = table.lookup(d)?;
        if !sol.reach { return straight(); }
        let ch = m::cos(sol.theta) * sol.v / d;
        let v3 = [dx * ch + vel[0], m::sin(sol.theta) * sol.v, dz * ch + vel[1]];
        let ws = if table.spin != 0.0 { Some(spin_for([dx * ch, dz * ch], (table.spin * sol.v) / RB)) } else { None };
        let Some((hx, hz, _)) = sim3d(exit, v3, table.ht, ws) else { break };
        let (ex, ez) = (hx - target[0], hz - target[1]);
        ax -= ex;
        az -= ez;
        if m::hypot2(ex, ez) < 0.01 { break; }
    }
    let (dx, dz) = (ax - exit[0], az - exit[2]);
    let d = m::hypot2(dx, dz);
    let sol = table.lookup(d)?;
    if !sol.reach { return straight(); }
    Some(Moving { psi: m::atan2(-dz, dx), theta: sol.theta, v: sol.v, tof: sol.tof, dist: d, aim: [ax, az], reach: true })
}
