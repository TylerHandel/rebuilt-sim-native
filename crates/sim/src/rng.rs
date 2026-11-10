//! The headless game's `Math.random`: mulberry32, seeded (rebuilt-sim-2026 tools/headless.mjs).
//! Every draw the JS game makes during a step is made here in the same order (PLAN.md).

#[derive(Debug, Clone)]
pub struct Mulberry32 { a: u32, /// how many draws so far (in the parity digests)
    pub draws: u64 }

impl Mulberry32 {
    pub fn new(seed: u32) -> Self { Mulberry32 { a: seed, draws: 0 } }
    /// The next draw in [0, 1), as JS's `Math.random()` returns it.
    pub fn random(&mut self) -> f64 {
        self.draws += 1;
        self.a = self.a.wrapping_add(0x6d2b79f5);
        let mut t = self.a;
        t = (t ^ (t >> 15)).wrapping_mul(t | 1);
        t ^= t.wrapping_add((t ^ (t >> 7)).wrapping_mul(t | 61));
        (t ^ (t >> 14)) as f64 / 4294967296.0
    }
}
