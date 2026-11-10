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

/// A parity script (parity/scripts/*.json; rebuilt-sim-2026 tools/native-trace.mjs runs it there).
#[derive(Debug, Clone, serde::Deserialize)]
pub struct Script {
    pub name: String,
    #[serde(default = "one")]
    pub seed: u32,
    pub slots: Vec<Slot>,
    /// the match jumps straight to this phase ('auto' or 'teleop'), robots enabled
    pub phase: Option<String>,
    pub steps: usize,
    #[serde(default)]
    pub commands: Vec<Command>,
    #[serde(default)]
    pub launches: Vec<Launch>,
}
fn one() -> u32 { 1 }

#[derive(Debug, Clone, serde::Deserialize)]
pub struct Slot { pub robot: String, pub alliance: String, pub pose: [f64; 3] }

/// A robot's command from step `from` on (fields left out are zero / false)
#[derive(Debug, Clone, serde::Deserialize)]
pub struct Command { pub from: usize, pub robot: usize, pub cmd: Cmd }

#[derive(Debug, Clone, Copy, Default, serde::Deserialize)]
#[serde(default)]
pub struct Cmd { pub vx: f64, pub vz: f64, pub omega: f64, pub intake: bool, pub shoot: bool, pub pass: bool, pub outtake: bool }

/// Before step `step`, FUEL `ball` (fuel.balls index) put at `pos` moving at `vel`, spinning at `spin`
#[derive(Debug, Clone, serde::Deserialize)]
pub struct Launch { pub step: usize, pub ball: usize, pub pos: [f64; 3], pub vel: [f64; 3], pub spin: Option<[f64; 3]> }

/// What the JS game did with a script: per step, a digest after each part of it ("match",
/// "drivers", "robotsPre", "fuelPre", "physics", "fuelPost", "robotsPost", "rules", and "frame"
/// on odd steps). See tools/native-trace.mjs for what a digest covers.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct Trace { pub version: u32, pub script: String, pub steps: Vec<std::collections::BTreeMap<String, String>> }

/// The parts of a step, in the order the JS game's stepGame runs them.
pub const PHASES: [&str; 9] = ["match", "drivers", "robotsPre", "fuelPre", "physics", "fuelPost", "robotsPost", "rules", "frame"];

/// Every script with its trace (parity/scripts, assets/parity).
pub fn scripts() -> Vec<(Script, Trace)> {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");
    let mut names: Vec<_> = std::fs::read_dir(format!("{root}/parity/scripts")).unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap()).filter(|f| f.ends_with(".json")).collect();
    names.sort();
    names.iter().map(|f| {
        let s: Script = serde_json::from_str(&std::fs::read_to_string(format!("{root}/parity/scripts/{f}")).unwrap()).unwrap();
        let t: Trace = serde_json::from_str(&std::fs::read_to_string(format!("{root}/assets/parity/{f}")).unwrap()).unwrap();
        (s, t)
    }).collect()
}

/// The first step and part where `got` (step, part, digest) differs from the trace, if any.
pub fn first_mismatch(trace: &Trace, got: &[(usize, &str, String)]) -> Option<String> {
    for (step, part, d) in got {
        let want = trace.steps.get(*step).and_then(|s| s.get(*part));
        if want != Some(d) { return Some(format!("{}: step {step}, after {part}: JS {}, here {d}", trace.script, want.map_or("(none)", |w| w))); }
    }
    None
}
