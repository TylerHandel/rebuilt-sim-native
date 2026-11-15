//! A small seeded random stream (mulberry32, as the game's), so a season replays the same from its
//! seed and a save holds where it is.
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Rng { pub state: u32 }

impl Rng {
    pub fn new(seed: u32) -> Self { Rng { state: seed } }
    /// in [0, 1)
    pub fn next(&mut self) -> f64 {
        self.state = self.state.wrapping_add(0x6D2B79F5);
        let mut t = self.state;
        t = (t ^ (t >> 15)).wrapping_mul(t | 1);
        t ^= t.wrapping_add((t ^ (t >> 7)).wrapping_mul(t | 61));
        ((t ^ (t >> 14)) as f64) / 4294967296.0
    }
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 { lo + self.next() * (hi - lo) }
    pub fn below(&mut self, n: usize) -> usize { ((self.next() * n as f64) as usize).min(n.saturating_sub(1)) }
    /// a standard normal (Box-Muller)
    pub fn gauss(&mut self) -> f64 {
        let u = self.next().max(1e-12);
        let v = self.next();
        (-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()
    }
    pub fn shuffle<T>(&mut self, v: &mut [T]) {
        for i in (1..v.len()).rev() { let j = self.below(i + 1); v.swap(i, j); }
    }
}
