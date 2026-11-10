//! Where every FUEL is after given numbers of steps, as JSON: to compare against the browser
//! game's Rapier on the same world (PLAN.md: parity checks). After the last count, 120 FUEL are
//! dropped as a heap (examples/bench.rs's) and the counts are run again, as "heap+<n>".
//!   cargo run --release -p sim --example dump -- out.json 1 10 60 120 360
use sim::{Sim, Vector, WorldData};

fn main() {
    let mut args = std::env::args().skip(1);
    let out = args.next().expect("an output file");
    let mut steps: Vec<usize> = args.map(|a| a.parse().expect("a step count")).collect();
    steps.sort();
    let mut s = Sim::new(WorldData::bundled());
    let mut rows: Vec<String> = Vec::new();
    for pass in 0..2 {
        let mut at = 0;
        for n in &steps {
            while at < *n { s.step(); at += 1; }
            // (f32 as f64: every digit, as JavaScript prints the same value)
            let pts: Vec<String> = s.fuel_poses().map(|(p, _)| format!("[{},{},{}]", p.x as f64, p.y as f64, p.z as f64)).collect();
            rows.push(format!("\"{}{}\":[{}]", if pass == 1 { "heap+" } else { "" }, n, pts.join(",")));
        }
        for i in 0..120 {
            let (a, b) = ((i % 10) as f32, (i / 10) as f32);
            s.add_fuel(Vector::new(-1.0 + a * 0.16, 1.0 + (i / 40) as f32 * 0.16, -0.9 + (b % 4.0) * 0.16 + 2.0));
        }
    }
    std::fs::write(out, format!("{{{}}}", rows.join(","))).expect("write");
}
