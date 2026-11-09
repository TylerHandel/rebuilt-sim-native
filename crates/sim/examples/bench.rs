//! How long the physics takes per simulated second: the field with its FUEL settling, then with a
//! pile of FUEL dropped onto it (what the browser game spends ~200-250 ms/s on in a 3v3 match).
//!   cargo run --release -p sim --example bench
use sim::{Sim, Vector, WorldData};
use std::time::Instant;

fn run(s: &mut Sim, secs: f32) -> f64 {
    let n = (secs / s.dt()) as usize;
    let t = Instant::now();
    for _ in 0..n { s.step(); }
    t.elapsed().as_secs_f64() * 1000.0 / secs as f64
}

fn main() {
    let mut s = Sim::new(WorldData::bundled());
    println!("{} FUEL, {} field colliders, dt {:.4} s", s.fuel.len(), s.data.field.len(), s.dt());
    println!("settling (first 2 s): {:.1} ms per simulated s", run(&mut s, 2.0));
    println!("at rest (next 3 s):   {:.1} ms per simulated s", run(&mut s, 3.0));
    // wake everything: drop 120 FUEL as a heap in the middle of the field
    for i in 0..120 {
        let (a, b) = ((i % 10) as f32, (i / 10) as f32);
        s.add_fuel(Vector::new(-1.0 + a * 0.16, 1.0 + (i / 40) as f32 * 0.16, -0.9 + (b % 4.0) * 0.16 + 2.0));
    }
    println!("heap of 120 dropped:  {:.1} ms per simulated s", run(&mut s, 2.0));
}
