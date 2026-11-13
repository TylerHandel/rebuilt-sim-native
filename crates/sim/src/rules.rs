//! js/rules.js: the ROBOT-to-ROBOT rules, checked every physics step for each pair of opposing
//! robots (any number per ALLIANCE).
//!   G403 (MAJOR) AUTO: contacting an opponent while fully across the CENTER LINE.
//!   G415 (MINOR) a deployed over-the-bumper intake reaching inside an opponent's FRAME PERIMETER.
//!   G418 (MINOR) PINNING an opponent against a FIELD element for more than 3 s, plus another MINOR
//!                for every further 3 s; the count resets once the robots are 72 in apart.
//!   G420 (MAJOR) END GAME: contacting an opponent that is touching its own TOWER or climbing.
//! Ramming, even at full speed, is legal.
use crate::constants::*;
use crate::game_match::Match;
use crate::nav::{obstacle_at, tower_rect};
use jsmath as m;

pub const PIN_LIMIT: f64 = 3.0;
pub const PIN_RESET: f64 = 72.0 * IN;
const CONTACT_GAP: f64 = 0.03;

/// What the rules see of a robot (js: the Robot's fields they read)
#[derive(Debug, Clone, Copy)]
pub struct RobotView {
    pub alliance: Alliance,
    pub x: f64,
    pub z: f64,
    pub yaw: f64,
    pub vx: f64,
    pub vz: f64,
    pub half_l: f64,
    pub half_w: f64,
    pub frame_l: f64,
    pub frame_w: f64,
    pub reach: f64,
    pub intake_w: f64,
    pub intake_deploy: f64,
    /// js climbState !== 'none'
    pub climbing: bool,
    pub cmd_vx: f64,
    pub cmd_vz: f64,
}

/// A point (x, z)
type Pt = (f64, f64);

fn rect_poly(r: &RobotView, x: f64, z: f64, hl: f64, hw: f64, ox: f64) -> [Pt; 4] {
    let (c, s) = (m::cos(r.yaw), m::sin(r.yaw));
    let mut out = [(0.0, 0.0); 4];
    for (k, (a, b)) in [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)].into_iter().enumerate() {
        let (lx, lz) = (ox + a * hl, b * hw);
        out[k] = (x + lx * c + lz * s, z - lx * s + lz * c);
    }
    out
}

fn project(p: &[Pt], ax: f64, az: f64) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for q in p { let d = q.0 * ax + q.1 * az; if d < lo { lo = d; } if d > hi { hi = d; } }
    (lo, hi)
}

/// js `polysOverlap(P, Q)`
pub fn polys_overlap(p: &[Pt], q: &[Pt]) -> bool {
    for poly in [p, q] {
        for i in 0..poly.len() {
            let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
            let (ax, az) = (-(b.1 - a.1), b.0 - a.0);
            let (p0, p1) = project(p, ax, az);
            let (q0, q1) = project(q, ax, az);
            if p1 < q0 || q1 < p0 { return false; }
        }
    }
    true
}

fn seg_dist(p: Pt, a: Pt, b: Pt) -> f64 {
    let (dx, dz) = (b.0 - a.0, b.1 - a.1);
    let l2 = { let v = dx * dx + dz * dz; if v == 0.0 || v.is_nan() { 1e-9 } else { v } };
    let t = m::max(0.0, m::min(1.0, ((p.0 - a.0) * dx + (p.1 - a.1) * dz) / l2));
    m::hypot2(p.0 - (a.0 + dx * t), p.1 - (a.1 + dz * t))
}

/// js `polyGap(P, Q)`
pub fn poly_gap(p: &[Pt], q: &[Pt]) -> f64 {
    if polys_overlap(p, q) { return 0.0; }
    let mut d = f64::INFINITY;
    for (aa, bb) in [(p, q), (q, p)] {
        for &pt in aa { for i in 0..bb.len() { d = m::min(d, seg_dist(pt, bb[i], bb[(i + 1) % bb.len()])); } }
    }
    d
}

pub fn bumper_poly(r: &RobotView) -> [Pt; 4] { rect_poly(r, r.x, r.z, r.half_l, r.half_w, 0.0) }
pub fn frame_poly(r: &RobotView) -> [Pt; 4] { rect_poly(r, r.x, r.z, r.frame_l / 2.0, r.frame_w / 2.0, 0.0) }
fn intake_poly_at(r: &RobotView, x: f64, z: f64) -> [Pt; 4] { rect_poly(r, x, z, r.reach / 2.0 + 0.01, r.intake_w / 2.0, r.half_l + r.reach / 2.0 - 0.01) }
fn intake_poly(r: &RobotView) -> [Pt; 4] { intake_poly_at(r, r.x, r.z) }

/// js `holdOffFoes(r, foes, cmd, horizon)` (AUTO's): would its intake reach inside an opponent's
/// FRAME PERIMETER within `horizon` s, the two closing as they are now? Then take out of the
/// command whatever drives it in.
pub fn hold_off_foes(r: &RobotView, foes: &[RobotView], cmd: &mut (f64, f64), horizon: f64) {
    if r.intake_deploy <= 0.85 { return; }
    for p in foes {
        let (rx, rz) = ((r.vx - p.vx) * horizon, (r.vz - p.vz) * horizon);
        let f = frame_poly(p);
        let soon = [0.0, 0.25, 0.5, 0.75, 1.0].iter().any(|&k| polys_overlap(&intake_poly_at(r, r.x + rx * k, r.z + rz * k), &f));
        if !soon { continue; }
        let (dx, dz) = (p.x - r.x, p.z - r.z);
        let d = { let h = m::hypot2(dx, dz); if h == 0.0 { 1.0 } else { h } };
        let into = (cmd.0 * dx + cmd.1 * dz) / d;
        if into > 0.0 { cmd.0 -= (dx / d) * into; cmd.1 -= (dz / d) * into; }
    }
}

/// js `towerProtected(r, pad)`: touching its own TOWER, or hanging on it
pub fn tower_protected(r: &RobotView, pad: f64) -> bool {
    if r.climbing { return true; }
    let t = tower_rect(r.alliance);
    let tw = [(t.x - t.hx - pad, t.z - t.hz - pad), (t.x + t.hx + pad, t.z - t.hz - pad), (t.x + t.hx + pad, t.z + t.hz + pad), (t.x - t.hx - pad, t.z + t.hz + pad)];
    polys_overlap(&bumper_poly(r), &tw)
}

/// Is Y backed against a FIELD element on the far side from X (so X's push traps it)?
fn against_element(y: &RobotView, dx: f64, dz: f64) -> bool {
    let p = bumper_poly(y);
    let mut ext = 0.0;
    for q in &p { ext = m::max(ext, (q.0 - y.x) * dx + (q.1 - y.z) * dz); }
    let (px, pz) = (-dz, dx);
    for lat in [-0.3, 0.0, 0.3] {
        let (x, z) = (y.x + dx * (ext + 0.12) + px * lat, y.z + dz * (ext + 0.12) + pz * lat);
        if obstacle_at(x, z, 0.0) { return true; }
    }
    false
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Pin { pub t: f64, pub fouls: u32, pub active: bool }
#[derive(Debug, Clone, Copy, Default)]
struct G415 { on: bool, off: f64 }

/// One pair's state (A the lower index, B the higher; [0] is A's, [1] B's)
#[derive(Debug, Clone)]
struct PairState {
    contact: bool,
    no_contact_t: f64,
    gap: f64,
    prev_v: [Option<(f64, f64)>; 2],
    pin: [Option<Pin>; 2],
    g415: [Option<G415>; 2],
    g403: bool,
    g420: [bool; 2],
}

/// js `RobotRules`
#[derive(Default)]
pub struct RobotRules { pairs: std::collections::HashMap<(usize, usize), PairState> }

impl RobotRules {
    fn state(&mut self, i: usize, j: usize) -> &mut PairState {
        let k = if i < j { (i, j) } else { (j, i) };
        self.pairs.entry(k).or_insert(PairState { contact: false, no_contact_t: 99.0, gap: 99.0, prev_v: [None; 2], pin: [None; 2], g415: [None; 2], g403: false, g420: [false; 2] })
    }

    /// js `pinState(X, Y)`: X pinning Y
    pub fn pin_state(&mut self, x: usize, y: usize) -> Pin {
        let side = if x < y { 0 } else { 1 };
        *self.state(x, y).pin[side].get_or_insert(Pin::default())
    }
    pub fn gap(&mut self, a: usize, b: usize) -> f64 { self.state(a, b).gap }
    pub fn in_contact(&mut self, a: usize, b: usize) -> bool { self.state(a, b).contact }

    /// js `update(dt, robots)`
    pub fn update(&mut self, dt: f64, robots: &[RobotView], m_: &mut Match) {
        for i in 0..robots.len() {
            for j in i + 1..robots.len() {
                if robots[i].alliance != robots[j].alliance { self.pair(dt, i, j, robots, m_); }
            }
        }
    }

    fn pair(&mut self, dt: f64, ia: usize, ib: usize, robots: &[RobotView], mt: &mut Match) {
        let (a, b) = (&robots[ia], &robots[ib]);
        let live = mt.robot_enabled();
        let gap = poly_gap(&bumper_poly(a), &bumper_poly(b));
        let st = self.state(ia, ib);
        st.gap = gap;
        let contact = gap < CONTACT_GAP;
        if contact { st.no_contact_t = 0.0; } else { st.no_contact_t += dt; }
        // (a contact "episode" ends after half a second apart)
        if st.no_contact_t > 0.5 { st.g403 = false; st.g420 = [false; 2]; }
        let (mut dx, mut dz) = (b.x - a.x, b.z - a.z);
        let dd = { let h = m::hypot2(dx, dz); if h == 0.0 { 1.0 } else { h } };
        dx /= dd; dz /= dd;
        let sides: [(usize, &RobotView, &RobotView, f64, f64); 2] = [(0, a, b, dx, dz), (1, b, a, -dx, -dz)];

        if live {
            // G403 AUTO: contact while fully across the CENTER LINE
            if mt.is_auto() && contact && !st.g403 {
                for x in [a, b] {
                    let s = if x.alliance == Alliance::Blue { 1.0 } else { -1.0 };
                    if bumper_poly(x).iter().all(|p| s * p.0 > 0.0) {
                        st.g403 = true;
                        mt.add_foul(x.alliance, true, "G403", "Contacted an opponent in AUTO while fully across the CENTER LINE");
                    }
                }
            }
            // G415 deployed intake inside the opponent's FRAME PERIMETER
            for &(k, x, y, ux, uz) in &sides {
                let g = st.g415[k].get_or_insert(G415::default());
                let inside = x.intake_deploy > 0.85 && gap < 0.4 && polys_overlap(&intake_poly(x), &frame_poly(y));
                if inside {
                    // only the robot that initiated the contact (was driving into the other) is at fault
                    let vx = st.prev_v[k].unwrap_or((x.vx, x.vz));
                    let vy = st.prev_v[1 - k].unwrap_or((y.vx, y.vz));
                    let (x_in, y_in) = (vx.0 * ux + vx.1 * uz, -(vy.0 * ux + vy.1 * uz));
                    if !g.on && x_in > 0.3 && x_in >= y_in { mt.add_foul(x.alliance, false, "G415", "Deployed intake reached inside the opponent FRAME PERIMETER"); }
                    g.on = true; g.off = 0.0;
                } else if g.on { g.off += dt; if g.off > 1.0 { g.on = false; } }
            }
            // G420 END GAME tower protection
            let end_game = mt.is_teleop() && mt.phase_time >= TIMING::teleop - TIMING::endgame;
            if end_game && contact {
                for &(k, x, y, _, _) in &sides {
                    if tower_protected(y, 0.05) && !tower_protected(x, 0.05) && !st.g420[k] {
                        st.g420[k] = true;
                        mt.add_foul(x.alliance, true, "G420", "Contacted an opponent touching its TOWER during END GAME");
                    }
                }
            }
        }

        // G418 pinning (X pins Y)
        for &(k, x, y, ux, uz) in &sides {
            let p = st.pin[k].get_or_insert(Pin::default());
            if gap >= PIN_RESET { p.t = 0.0; p.fouls = 0; }
            let push = x.cmd_vx * ux + x.cmd_vz * uz;
            let y_speed = m::hypot2(y.vx, y.vz);
            p.active = live && contact && !x.climbing && push > 0.5 && y_speed < 0.35 && against_element(y, ux, uz);
            if p.active {
                p.t += dt;
                if p.t >= PIN_LIMIT * (p.fouls as f64 + 1.0) {
                    p.fouls += 1;
                    mt.add_foul(x.alliance, false, "G418", format!("PINNED an opponent for more than {} s", PIN_LIMIT * p.fouls as f64));
                }
            }
        }
        st.contact = contact;
        st.prev_v = [Some((a.vx, a.vz)), Some((b.vx, b.vz))];
    }
}
