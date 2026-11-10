//! Parity with the browser game: digests of the sim's state that its tools write too
//! (rebuilt-sim-2026 tools/native-parity.mjs), to compare bit for bit. See PLAN.md.

use crate::Sim;

/// FNV-1a (64 bit).
#[derive(Clone, Copy)]
pub struct Fnv(pub u64);

impl Default for Fnv {
    fn default() -> Self { Fnv(0xcbf29ce484222325) }
}

impl Fnv {
    pub fn bytes(&mut self, b: &[u8]) {
        for x in b { self.0 = (self.0 ^ *x as u64).wrapping_mul(0x100000001b3); }
    }
    pub fn f32(&mut self, x: f32) { self.bytes(&x.to_le_bytes()); }
    pub fn f64(&mut self, x: f64) { self.bytes(&x.to_le_bytes()); }
    pub fn hex(&self) -> String { format!("{:016x}", self.0) }
}

/// Every FUEL's position, rotation (x, y, z, w), velocity and spin as f32, in the order they were made.
pub fn fuel_digest(sim: &Sim) -> String {
    let mut h = Fnv::default();
    for b in sim.fuel.iter().map(|f| &sim.physics.bodies[*f]) {
        let (t, q, v, w) = (b.translation(), *b.rotation(), b.linvel(), b.angvel());
        for x in [t.x, t.y, t.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z] { h.f32(x); }
    }
    h.hex()
}
