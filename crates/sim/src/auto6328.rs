//! js/auto6328.js: 6328's own AUTOs run live, their commands from Mechanical-Advantage's
//! RobotCode2026Public (AutoBuilder, DriveToPose, DriveTrajectory, the salesman, ObjectDetection) as
//! one step of the AUTO runner (a 'team6328' step running a program of them in turn). Their poses
//! are in blue field coordinates for the left side; the robot's own pose is read in the same frame
//! for either ALLIANCE (their AllianceFlipUtil: red turned round).
use crate::auto::{AutoRunner, Ctx};
use crate::constants::*;
use crate::fuel::{clamp, State};
use crate::robot::{wrap_angle, Phys, Robot};
use jsmath as m;
use serde_json::Value;

const IN_: f64 = 0.0254;
// their DriveConstants and FieldConstants (Darwin; the welded official layout)
fn d_full_apothem_x() -> f64 { (33.0 * IN_) / 2.0 }
fn d_full_base_radius() -> f64 { m::hypot2((33.0 * IN_) / 2.0, (34.0 * IN_) / 4.0) }
fn d_drive_base_radius() -> f64 { m::hypot2((21.25 * IN_) / 2.0, (22.25 * IN_) / 2.0) }
const D_MAX_LINEAR: f64 = 4.2;
fn d_intake_ref_x() -> f64 { (33.0 * IN_) / 2.0 }
fn d_intake_far_x() -> f64 { (26.5 * IN_) / 2.0 + 12.0 * IN_ }
fn d_frame_width_y() -> f64 { 27.5 * IN_ }
const F_STARTING: f64 = 4.0219;
const F_NZ_NEAR: f64 = 5.2225;
fn f_center() -> f64 { FIELD_L / 2.0 }
const F_HUB_X: f64 = 4.6188;
fn f_mid_y() -> f64 { FIELD_W / 2.0 }
const F_RIGHT_BUMP_START: f64 = 3.4376;
const F_LEFT_BUMP_END: f64 = 4.6314;
const F_POOL_LEFT: [f64; 2] = [8.2705, 6.3446];
const F_POOL_RIGHT: [f64; 2] = [8.2705, 1.7244];
fn launch_pose(k: &str) -> [f64; 3] {
    match k {
        "leftBump" => [3.3219, 5.5585, 130.3973], "rightBump" => [3.3219, FIELD_W - 5.5585, -130.3973],
        "leftTrench" => [3.0504, 7.1297, 116.8714], "rightTrench" => [3.0504, FIELD_W - 7.1297, -116.8714],
        _ => panic!("no launch pose {k}"),
    }
}
const START_INTAKE: f64 = 0.5;
const START_SPIN: f64 = 1.5;
const LAUNCH_TIME: f64 = 3.0;
const DEPOT_LAUNCH_TIME: f64 = 1.5;
fn index_t() -> f64 { 0.35 + 1.2 }

/// js `own(r)`: a robot's pose in their frame (its ALLIANCE's side turned to blue's)
#[derive(Clone, Copy)]
struct Own { x: f64, y: f64, h: f64, vx: f64, vy: f64, w: f64 }
fn own_of(a: Alliance, x: f64, z: f64, yaw: f64, vx: f64, vz: f64, omega: f64) -> Own {
    let b = a == Alliance::Blue;
    Own { x: if b { x + HALF_L } else { HALF_L - x }, y: if b { HALF_W - z } else { HALF_W + z }, h: wrap_angle(if b { yaw } else { yaw - m::PI }), vx: if b { vx } else { -vx }, vy: if b { -vz } else { vz }, w: omega }
}
fn own(r: &Robot) -> Own { own_of(r.alliance, r.pos[0], r.pos[2], r.yaw, r.vel[0], r.vel[2], r.omega) }
/// js `drive(r, vx, vy, w)`: a command in their frame to the sim's
fn drive(r: &mut Robot, vx: f64, vy: f64, w: f64) {
    let b = r.alliance == Alliance::Blue;
    let s = m::hypot2(vx, vy);
    let k = if s > r.max_speed { r.max_speed / s } else { 1.0 };
    r.cmd.vx = (if b { vx } else { -vx }) * k;
    r.cmd.vz = (if b { -vy } else { vy }) * k;
    r.cmd.omega = clamp(w, -r.max_omega, r.max_omega);
}

/// A program step's state (js st: whatever its step keeps)
#[derive(Clone, Debug, Default)]
pub struct St {
    t: f64,
    init: bool, sp_x: f64, sp_y: f64, sp_vx: f64, sp_vy: f64, th: f64, th_v: f64, last_err: f64,
    traj: Option<([f64; 3], [f64; 3], f64)>, solved_t: f64,
    o: Option<[f64; 3]>,
    phase2: Option<f64>, hold: [f64; 2], dtp: Option<Box<St>>,
    idx: Option<f64>, go: Option<f64>,
    at: Option<[f64; 2]>,
}
#[derive(Clone, Copy, Debug, PartialEq)]
enum Key { Init(usize), Ball(usize) }
/// js run.prog: the program's place, its step's state, what its cameras keep, the solver's warm start
#[derive(Clone, Debug, Default)]
pub struct Prog { k: usize, st: St, mirror: bool, coast: bool, no_intake: bool, seen: Option<Vec<(Key, [f64; 3])>>, ax: f64, ay: f64 }

/// js `driveToPose(r, st, tgt, dt, { omega })`: their trapezoid profile on the distance, kP 1.8, and
/// the heading's profiled PID. Returns (distance off, heading off)
fn drive_to_pose(r: &mut Robot, st: &mut St, tgt: [f64; 3], dt: f64) -> (f64, f64) {
    let p = own(r);
    if !st.init { st.init = true; st.sp_x = p.x; st.sp_y = p.y; st.sp_vx = p.vx; st.sp_vy = p.vy; st.th = p.h; st.th_v = p.w; st.last_err = 0.0; }
    let (ex, ey) = (tgt[0] - p.x, tgt[1] - p.y);
    let err = m::hypot2(ex, ey);
    let (dx, dy) = (tgt[0] - st.sp_x, tgt[1] - st.sp_y);
    let dd = m::hypot2(dx, dy);
    let mut v0 = if dd <= 0.01 { m::hypot2(st.sp_vx, st.sp_vy) } else { (st.sp_vx * dx + st.sp_vy * dy) / dd };
    v0 = m::max(v0, -0.5);
    let (a, vv) = (4.0, 4.0);
    let mut v = m::min(vv, v0 + a * dt);
    let v_stop = m::sqrt(2.0 * a * m::max(0.0, dd - v0 * dt));
    v = m::max(m::max(m::min(v, v_stop), v0 - a * dt), 0.0);
    let pos = m::max(0.0, dd - ((v0 + v) / 2.0) * dt);
    let mut vs = 1.8 * (err - pos) + v * clamp((err - 0.01) / 0.04, 0.0, 1.0);
    if err < 0.01 { vs = 0.0; }
    let (ux, uy) = if err > 1e-6 { (ex / err, ey / err) } else { (0.0, 0.0) };
    st.sp_x = tgt[0] - ux * pos; st.sp_y = tgt[1] - uy * pos; st.sp_vx = ux * v; st.sp_vy = uy * v;
    let goal = (tgt[2] * m::PI) / 180.0;
    let (vw, aw) = ((500.0 * m::PI) / 180.0, 8.0);
    let e = wrap_angle(goal - st.th);
    let mut wv = m::min(m::min(vw, m::abs(st.th_v) + aw * dt), m::sqrt(2.0 * aw * m::abs(e)));
    wv = m::sign(e) * m::min(wv, m::abs(e) / dt);
    st.th_v = wv; st.th = wrap_angle(st.th + wv * dt);
    let he = wrap_angle(st.th - p.h);
    let mut w = 5.0 * he + (0.5 * (he - st.last_err)) / dt + st.th_v;
    st.last_err = he;
    if m::abs(wrap_angle(goal - p.h)) < m::PI / 180.0 { w = 0.0; }
    drive(r, ux * vs, uy * vs, w);
    (err, m::abs(wrap_angle(goal - p.h)))
}

// ------------------------------------------------------------------ the salesman
const SA_T_STEP: f64 = 0.1;
const SA_MAX_SQ: f64 = 1.5;
fn sa_range() -> f64 { ((27.5 * IN_) / 2.0) * 0.75 }
const SA_C_DIST: f64 = 6.0;
const SA_C_REWARD: f64 = 8.0;
const SA_HUB_R: f64 = 1.0;
const HUBS: [[f64; 2]; 2] = [[5.2225, 4.0345], [11.3185, 4.0345]];
fn sig(x: f64, c: f64, k: f64) -> f64 { 1.0 / (1.0 + m::exp(-k * (x - c))) }
/// js `reward(...)` (and its gradient in ax, ay)
fn reward(fuel: &[[f64; 2]], x0: f64, y0: f64, vx: f64, vy: f64, ax: f64, ay: f64) -> (f64, f64, f64) {
    let (mut rr, mut gx, mut gy) = (0.0, 0.0, 0.0);
    for &[fx, fy] in fuel {
        let (mut s, mut sx, mut sy) = (0.0, 0.0, 0.0);
        for i in 0..=10 {
            let t = i as f64 * SA_T_STEP;
            let t2 = t * t;
            let (x, y) = (x0 + vx * t + ax * t2, y0 + vy * t + ay * t2);
            let (dx, dy) = (x - fx, y - fy);
            let d = { let h = m::hypot2(dx, dy); if h == 0.0 { 1e-9 } else { h } };
            let g = sig(d, sa_range(), SA_C_DIST);
            s += 1.0 - g;
            let dg = -SA_C_DIST * g * (1.0 - g);
            sx += dg * (dx / d) * t2; sy += dg * (dy / d) * t2;
        }
        let q = sig(s, 1.0, SA_C_REWARD);
        rr += q;
        let dq = SA_C_REWARD * q * (1.0 - q);
        gx += dq * sx; gy += dq * sy;
    }
    (rr, gx, gy)
}
#[derive(Clone, Copy)]
struct Bounds { min_x: f64, max_x: f64, min_y: f64, max_y: f64 }
fn project(mut ax: f64, mut ay: f64, x0: f64, y0: f64, vx: f64, vy: f64, b: Bounds) -> (f64, f64) {
    ax = clamp(ax, -SA_MAX_SQ, SA_MAX_SQ); ay = clamp(ay, -SA_MAX_SQ, SA_MAX_SQ);
    ax = clamp(ax, b.min_x - x0 - vx, b.max_x - x0 - vx); ay = clamp(ay, b.min_y - y0 - vy, b.max_y - y0 - vy);
    for [hx, hy] in HUBS {
        let (ex, ey) = (x0 + vx + ax, y0 + vy + ay);
        let d = m::hypot2(ex - hx, ey - hy);
        if d < SA_HUB_R {
            let k = if d > 1e-6 { SA_HUB_R / d } else { 1.0 };
            ax += (hx + (ex - hx) * k) - ex + (if d > 1e-6 { 0.0 } else { SA_HUB_R });
            ay += (hy + (ey - hy) * k) - ey;
        }
    }
    (ax, ay)
}
fn solve(prog: &mut Prog, fuel: &[[f64; 2]], x0: f64, y0: f64, vx: f64, vy: f64, b: Bounds) -> ([f64; 3], [f64; 3], f64) {
    let near: Vec<[f64; 2]> = fuel.iter().copied().filter(|&[fx, fy]| b.min_x <= fx && fx <= b.max_x && b.min_y <= fy && fy <= b.max_y && m::hypot2(fx - x0, fy - y0) < m::hypot2(vx, vy) + SA_MAX_SQ + 1.0).collect();
    let (mut ax, mut ay) = project(prog.ax, prog.ay, x0, y0, vx, vy, b);
    let (mut rr, mut gx, mut gy) = reward(&near, x0, y0, vx, vy, ax, ay);
    let mut step = 0.5;
    let mut it = 0;
    while it < 12 && step > 1e-3 {
        let g = m::hypot2(gx, gy);
        if g < 1e-6 { break; }
        let (nx, ny) = project(ax + (step * gx) / g, ay + (step * gy) / g, x0, y0, vx, vy, b);
        let r2 = reward(&near, x0, y0, vx, vy, nx, ny);
        if r2.0 > rr { ax = nx; ay = ny; (rr, gx, gy) = r2; step *= 1.3; } else { step *= 0.4; }
        it += 1;
    }
    prog.ax = ax; prog.ay = ay;
    ([x0, vx, ax], [y0, vy, ay], rr)
}

const CAM_RANGE: f64 = 7.0;
const CAM_HUB_R: f64 = 0.6;
const CAM_BOT_R: f64 = 0.45;
const CAM_PERSIST: f64 = 3.0;
fn cam_half() -> f64 { (115.0 * m::PI) / 180.0 }
/// The other robots in their frame (js run.foes, each by its own ALLIANCE's frame, as theirs does)
fn others(ctx: &Ctx) -> Vec<Own> { ctx.foes.iter().map(|o| own_of(o.alliance, o.x, o.z, o.yaw, o.vx, o.vz, 0.0)).collect() }
/// js `seeFuel(run, st, t)`: the FUEL its cameras see, each kept 3 s
fn see_fuel(r: &Robot, prog: &mut Prog, ctx: &Ctx, t: f64) -> Vec<[f64; 2]> {
    let p = own(r);
    if prog.seen.is_none() {
        let mut v = Vec::new();
        let mut k = 0;
        for i in 0..15 { for j in 0..6 { for [sx, sy] in [[1.0, 1.0], [-1.0, 1.0], [1.0, -1.0], [-1.0, -1.0]] {
            v.push((Key::Init(k), [f_center() + sx * (0.076 + 0.152 * j as f64), f_mid_y() + sy * (0.0254 + 0.076 + 0.152 * i as f64), t])); k += 1;
        } } }
        prog.seen = Some(v);
    }
    let oth = others(ctx);
    let hubs = [[F_HUB_X, f_mid_y()], [FIELD_L - F_HUB_X, f_mid_y()]];
    let blocked = |fx: f64, fy: f64| {
        let (sx, sy) = (fx - p.x, fy - p.y);
        let l2 = { let v = sx * sx + sy * sy; if v == 0.0 { 1e-9 } else { v } };
        let near = |cx: f64, cy: f64, rad: f64| { let u = clamp(((cx - p.x) * sx + (cy - p.y) * sy) / l2, 0.0, 1.0); m::hypot2(p.x + sx * u - cx, p.y + sy * u - cy) < rad };
        hubs.iter().any(|h| near(h[0], h[1], CAM_HUB_R)) || oth.iter().any(|o| near(o.x, o.y, CAM_BOT_R))
    };
    let in_view = |fx: f64, fy: f64| { let d = m::hypot2(fx - p.x, fy - p.y); d < CAM_RANGE && d > 0.3 && m::abs(wrap_angle(m::atan2(fy - p.y, fx - p.x) - p.h)) <= cam_half() };
    let seen = prog.seen.as_mut().unwrap();
    let mut now: Vec<usize> = Vec::new();
    for b in &ctx.fuel.balls {
        if b.state != State::Field || b.held || b.in_flight || b.pos[1] > 0.3 { continue; }
        let fx = if r.alliance == Alliance::Blue { b.pos[0] + HALF_L } else { HALF_L - b.pos[0] };
        let fy = if r.alliance == Alliance::Blue { HALF_W - b.pos[2] } else { HALF_W + b.pos[2] };
        if !in_view(fx, fy) || blocked(fx, fy) { continue; }
        now.push(b.id);
        seen.retain(|(k, q)| !(matches!(k, Key::Init(_)) && m::hypot2(q[0] - fx, q[1] - fy) < 0.075));
        match seen.iter_mut().find(|(k, _)| *k == Key::Ball(b.id)) { Some(e) => e.1 = [fx, fy, t], None => seen.push((Key::Ball(b.id), [fx, fy, t])) }
    }
    let (c, s) = (m::cos(p.h), m::sin(p.h));
    seen.retain(|(k, q)| {
        let (lx, ly) = ((q[0] - p.x) * c + (q[1] - p.y) * s, -(q[0] - p.x) * s + (q[1] - p.y) * c);
        let under = lx > -((26.5 * IN_) / 2.0) && lx < d_intake_far_x() && m::abs(ly) < d_frame_width_y() / 2.0;
        let now_has = matches!(k, Key::Ball(id) if now.contains(id));
        !(t - q[2] >= CAM_PERSIST || under || (!now_has && in_view(q[0], q[1]) && !blocked(q[0], q[1])))
    });
    seen.iter().map(|(_, q)| [q[0], q[1]]).collect()
}

/// js `bounds(run, kind, left)`: their zoning limits
fn bounds(ctx: &Ctx, kind: &str, left: bool) -> Bounds {
    let full = Bounds { min_x: F_NZ_NEAR, max_x: f_center() + d_drive_base_radius(), min_y: 0.0, max_y: FIELD_W };
    match kind {
        "full" => return full,
        "left" => return Bounds { min_y: FIELD_W / 2.0 - d_full_apothem_x(), ..full },
        "right" => return Bounds { max_y: FIELD_W / 2.0 + d_full_apothem_x(), ..full },
        _ => {}
    }
    let mid = f_center();
    let oth = others(ctx);
    let home = if left { Bounds { min_x: F_NZ_NEAR, max_x: mid, min_y: f_mid_y(), max_y: FIELD_W } } else { Bounds { min_x: F_NZ_NEAR, max_x: mid, min_y: 0.0, max_y: f_mid_y() } };
    let ally = if left { Bounds { min_y: 0.0, max_y: f_mid_y(), ..home } } else { Bounds { min_y: f_mid_y(), max_y: FIELD_W, ..home } };
    let any = |q: Bounds| oth.iter().any(|o| o.x >= q.min_x && o.x <= q.max_x && o.y >= q.min_y && o.y <= q.max_y);
    let b = if any(ally) { home } else { Bounds { min_x: home.min_x, max_x: home.max_x, min_y: 0.0, max_y: FIELD_W } };
    Bounds { max_x: m::max(b.max_x, f_center() + d_full_apothem_x()), ..b }
}

/// js `salesman(run, st, a, dt)`: SalesmanCommands.autoIntake
fn salesman(run: &mut AutoRunner, r: &mut Robot, ctx: &Ctx, a: &Value) {
    let p = own(r);
    let t = run.elapsed;
    let prog_mirror = run.prog.as_ref().unwrap().mirror;
    let b = bounds(ctx, a["bounds"].as_str().unwrap_or(""), !prog_mirror);
    let (c, s) = (m::cos(p.h), m::sin(p.h));
    let (ix, iy) = (p.x + d_intake_ref_x() * c, p.y + d_intake_ref_x() * s);
    let (ivx, ivy) = (p.vx - p.w * d_intake_ref_x() * s, p.vy + p.w * d_intake_ref_x() * c);
    let prog = run.prog.as_mut().unwrap();
    if prog.st.traj.is_none() || t - prog.st.solved_t >= 0.1 {
        let fuel = see_fuel(r, prog, ctx, t);
        let tr = solve(prog, &fuel, ix, iy, ivx, ivy, b);
        prog.st.traj = Some(tr);
        prog.st.solved_t = t;
    }
    let (tx_, ty_, rew) = prog.st.traj.unwrap();
    let at = |u: f64| [tx_[0] + tx_[1] * u + tx_[2] * u * u, ty_[0] + ty_[1] * u + ty_[2] * u * u];
    let (mut u, mut last) = (0.0, f64::INFINITY);
    while u < 3.0 { let [x, y] = at(u); let d = m::hypot2(x - ix, y - iy); if d >= last { break; } last = d; u += 0.01; }
    u += 0.25;
    let [tx, ty] = at(u);
    let th = m::atan2(ty_[1] + 2.0 * ty_[2] * u, tx_[1] + 2.0 * tx_[2] * u);
    let mm = (30.0 * m::PI) / 180.0;
    let e = wrap_angle(th - p.h);
    let goal_h = if m::abs(e) < mm { p.h } else if e >= mm { th - mm } else { th + mm };
    let mut rx = tx - d_intake_ref_x() * m::cos(goal_h);
    let mut ry = ty - d_intake_ref_x() * m::sin(goal_h);
    rx = clamp(rx, b.min_x + d_drive_base_radius(), b.max_x - d_drive_base_radius());
    ry = clamp(ry, b.min_y + d_drive_base_radius(), b.max_y - d_drive_base_radius());
    let (mut vx, mut vy) = (6.0 * (rx - p.x), 6.0 * (ry - p.y));
    let cap = 3.0 + (1.8 - 3.0) * clamp((rew - 15.0) / (70.0 - 15.0), 0.0, 1.0);
    let sp = m::hypot2(vx, vy);
    if sp > cap { vx *= cap / sp; vy *= cap / sp; }
    drive(r, vx, vy, 8.0 * wrap_angle(goal_h - p.h));
}

fn pt(v: &Value) -> [f64; 5] { let a = v.as_array().unwrap(); [0, 1, 2, 3, 4].map(|i| a[i].as_f64().unwrap()) }

/// js `follow(run, st, a, dt)`: DriveTrajectory, the sample at its time fed forward, kP 8 and 4
fn follow(r: &mut Robot, st: &mut St, a: &Value) -> bool {
    let pts: Vec<[f64; 5]> = a["pts"].as_array().unwrap().iter().map(pt).collect();
    let tt = st.t;
    let p = own(r);
    let mut k = 0;
    while k + 2 < pts.len() && pts[k + 1][4] <= tt { k += 1; }
    let (aa, bq) = (pts[k], pts[(k + 1).min(pts.len() - 1)]);
    let h = bq[4] - aa[4];
    let w = if h > 1e-6 { clamp((tt - aa[4]) / h, 0.0, 1.0) } else { 1.0 };
    let (mut x, mut y) = (aa[0] + (bq[0] - aa[0]) * w, aa[1] + (bq[1] - aa[1]) * w);
    let ha = (aa[2] * m::PI) / 180.0;
    let dh = wrap_angle(((bq[2] - aa[2]) * m::PI) / 180.0);
    let mut head = ha + dh * w;
    let (mut fvx, mut fvy) = if h > 1e-6 { ((bq[0] - aa[0]) / h, (bq[1] - aa[1]) / h) } else { (0.0, 0.0) };
    let fwr = if h > 1e-6 { dh / h } else { 0.0 };
    if a["relative"].as_bool() == Some(true) {
        let o = *st.o.get_or_insert([p.x, p.y, p.h]);
        let (c, s) = (m::cos(o[2]), m::sin(o[2]));
        (x, y) = (o[0] + x * c - y * s, o[1] + x * s + y * c);
        (fvx, fvy) = (fvx * c - fvy * s, fvx * s + fvy * c);
        head += o[2];
    }
    let end = pts[pts.len() - 1][4];
    if tt > end { fvx = 0.0; fvy = 0.0; }
    drive(r, fvx + 8.0 * (x - p.x), fvy + 8.0 * (y - p.y), fwr + 4.0 * wrap_angle(head - p.h));
    tt >= end
}

/// js `returnTarget(run)`: returnToDeterminedLaunchPose
fn return_target(r: &Robot, mirror: bool) -> [f64; 3] {
    let p = own(r);
    let tg = launch_pose(if mirror { "rightBump" } else { "leftBump" });
    let mut t = clamp(m::cbrt((m::abs(p.y - tg[1]) - 0.1) / (0.4 - 0.1)), 0.0, 1.0);
    if p.x < F_NZ_NEAR + 1.0 && (p.y < F_RIGHT_BUMP_START - d_full_base_radius() || p.y > F_LEFT_BUMP_END + d_full_base_radius()) { t = 0.0; }
    let x = tg[0] + (F_NZ_NEAR + 1.5 - tg[0]) * t;
    let deg = if p.x < (F_STARTING + F_NZ_NEAR) / 2.0 { tg[2] } else if p.x < F_NZ_NEAR + 0.838 { 180.0 } else { (m::atan2(tg[1] - p.y, tg[0] - p.x) * 180.0) / m::PI };
    [x, tg[1], deg]
}
fn aimed_at(x: f64, y: f64) -> [f64; 3] { [x, y, (m::atan2(y - f_mid_y(), x - F_HUB_X) * 180.0) / m::PI] }
fn pose(v: &Value) -> [f64; 3] { let a = v.as_array().unwrap(); [a[0].as_f64().unwrap(), a[1].as_f64().unwrap(), a[2].as_f64().unwrap()] }

/// One program step (js STEPS[kind]): true once it's done
fn run_step(run: &mut AutoRunner, r: &mut Robot, ctx: &Ctx, kind: &str, a: &Value, dt: f64) -> bool {
    let num = |k: &str| a[k].as_f64().unwrap_or_else(|| panic!("6328 {kind}: no {k}"));
    let elapsed = run.elapsed;
    let mirror = run.prog.as_ref().unwrap().mirror;
    let coast_x = a["coastX"].as_f64();
    match kind {
        "wait" => run.prog.as_ref().unwrap().st.t >= num("t"),
        "rush" => {
            let p = own(r);
            drive(r, D_MAX_LINEAR * (if a["slow"].as_bool() == Some(true) { 0.6 } else { 1.0 }), 0.0, 0.0);
            let prog = run.prog.as_mut().unwrap();
            if coast_x.is_some_and(|c| p.x > c) { prog.coast = true; }
            prog.st.t >= num("t")
        }
        "traj" => {
            let prog = run.prog.as_mut().unwrap();
            let done = follow(r, &mut prog.st, a);
            let p = own(r);
            if coast_x.is_some_and(|c| p.x > c) { prog.coast = true; }
            if let Some(aim) = a["aim"].as_f64() { if p.x < aim { r.cmd.shoot = true; prog.no_intake = true; } }
            done
        }
        "turn" => {
            let p = own(r);
            let left = !mirror;
            let xt = F_POOL_LEFT[0] - 0.8;
            let k = clamp(m::abs(p.x - xt) / (xt - F_NZ_NEAR), -1.0, 1.0);
            let y0 = if left { F_POOL_LEFT[1] } else { F_POOL_RIGHT[1] };
            let y1 = if left { y0 + 3.5 } else { y0 - 3.5 };
            let prog = run.prog.as_mut().unwrap();
            drive_to_pose(r, &mut prog.st, [xt, y0 + (y1 - y0) * k, if left { -50.0 } else { 50.0 }], dt);
            prog.st.t >= 1.0
        }
        "salesman" => { salesman(run, r, ctx, a); run.prog.as_ref().unwrap().st.t >= num("t") }
        "return" => {
            let tg = return_target(r, mirror);
            drive_to_pose(r, &mut run.prog.as_mut().unwrap().st, tg, dt);
            own(r).x < num("untilX")
        }
        "launch" => {
            let p = own(r);
            let prog = run.prog.as_mut().unwrap();
            if prog.st.phase2.is_none() {
                drive_to_pose(r, &mut prog.st, launch_pose(a["target"].as_str().unwrap()), dt);
                if p.x < F_STARTING - d_full_base_radius() { prog.st.phase2 = Some(elapsed); prog.st.hold = [p.x, p.y]; prog.st.dtp = Some(Box::default()); }
                return false;
            }
            let hold = prog.st.hold;
            drive_to_pose(r, prog.st.dtp.as_mut().unwrap(), aimed_at(hold[0], hold[1]), dt);
            r.cmd.shoot = true; prog.no_intake = true;
            let hasty = a["hasty"].as_bool() == Some(true);
            elapsed - prog.st.phase2.unwrap() >= (if hasty { DEPOT_LAUNCH_TIME } else { LAUNCH_TIME }) && a["leaveAt"].as_f64().is_none_or(|l| elapsed >= l)
        }
        "launchAt" => {
            let prog = run.prog.as_mut().unwrap();
            let (err, herr) = drive_to_pose(r, &mut prog.st, pose(&a["pose"]), dt);
            if prog.st.idx.is_none() && err < 0.1 && herr < (5.0 * m::PI) / 180.0 { prog.st.idx = Some(elapsed); }
            if prog.st.idx.is_some() {
                let launching = matches!(&r.phys, Some(Phys::R6328(p)) if p.launching());
                if launching && prog.st.go.is_none() { prog.st.go = Some(elapsed); }
                let over = prog.st.go.is_some_and(|g| elapsed - g >= index_t());
                if !over { r.cmd.shoot = true; prog.no_intake = true; }
                return if a["until"].as_str() == Some("empty") { over } else { false };
            }
            false
        }
        "pass" => {
            let p = own(r);
            let prog = run.prog.as_mut().unwrap();
            let at = *prog.st.at.get_or_insert([m::min(p.x, f_center() + d_full_apothem_x() - 0.2), p.y]);
            r.cmd.pass = true; prog.no_intake = true;
            drive(r, 3.0 * (at[0] - p.x), 3.0 * (at[1] - p.y), 0.0);
            prog.st.t >= num("t") + 1.0
        }
        "toPose" => {
            let prog = run.prog.as_mut().unwrap();
            drive_to_pose(r, &mut prog.st, pose(&a["pose"]), dt);
            if coast_x.is_some_and(|c| own(r).x > c) { prog.coast = true; }
            false
        }
        _ => panic!("6328: no step {kind}"),
    }
}

/// js `step6328(run, s, dt, next)`: the program's commands in turn; the intake from 0.5 s in (not
/// while it launches), the flywheel tracking the HUB from 1.5 s; coasting, the drive let go
pub fn step(run: &mut AutoRunner, r: &mut Robot, ctx: &Ctx, dt: f64) {
    let (program, mirror) = { let s = &run.steps[run.i]; (s.program.clone().unwrap(), s.prog_mirror) };
    let program = program.as_array().unwrap().clone();
    let prog = run.prog.get_or_insert_with(|| Prog { mirror, ..Default::default() });
    if prog.coast {
        let p = own(r);
        let k = m::exp(-dt / 1.2);
        drive(r, p.vx * k, p.vy * k, p.w * k);
        r.cmd.intake = true;
        return;
    }
    prog.no_intake = false;
    r.cmd.prespin = run.elapsed >= START_SPIN;
    while run.prog.as_ref().unwrap().k < program.len() {
        let k = run.prog.as_ref().unwrap().k;
        let kind = program[k][0].as_str().unwrap().to_string();
        let a = program[k][1].clone();
        let done = run_step(run, r, ctx, &kind, &a, dt);
        let prog = run.prog.as_mut().unwrap();
        prog.st.t += dt;
        if !done { break; }
        prog.k += 1; prog.st = St::default();
        if prog.k >= program.len() { if !prog.coast { run.advance(); } break; }
    }
    let no_intake = run.prog.as_ref().unwrap().no_intake;
    r.cmd.intake = run.elapsed >= START_INTAKE && !no_intake;
}
