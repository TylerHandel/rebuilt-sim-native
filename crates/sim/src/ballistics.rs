//! js/ballistics.js (so far: the Magnus lift every airborne FUEL feels, fuel.js preStep).
use crate::constants::FUEL;
use jsmath as m;

const KL: f64 = FUEL::liftK;
const RB: f64 = FUEL::radius;

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
