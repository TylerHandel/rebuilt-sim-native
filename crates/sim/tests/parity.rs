//! The engine check: this sim's Rapier and the browser game's step the same world to the same
//! bits (assets/parity-engine.json, written by rebuilt-sim-2026 tools/native-parity.mjs).
use serde::Deserialize;
use sim::{parity::fuel_digest, Sim, Vector, WorldData};

#[derive(Deserialize)]
struct Heap { n: usize, step: f32, at: [f32; 3], thrown: [f32; 6] }
#[derive(Deserialize)]
struct Row { part: String, step: usize, fuel: usize, digest: String }
#[derive(Deserialize)]
struct Parity { heap: Heap, digests: Vec<Row> }

#[test]
fn rapier_steps_the_field_and_fuel_as_the_browser_does() {
    let p: Parity = serde_json::from_str(include_str!("../../../assets/parity-engine.json")).unwrap();
    let mut s = Sim::new(WorldData::bundled());
    let (mut part, mut at) = (String::from("start"), 0);
    for row in &p.digests {
        if row.part != part {
            part = row.part.clone();
            at = 0;
            let h = &p.heap;
            for i in 0..h.n {
                let b = s.add_fuel(Vector::new(h.at[0] + (i % 10) as f32 * h.step, h.at[1] + (i / 40) as f32 * h.step, h.at[2] + ((i / 10) % 4) as f32 * h.step));
                if i % 7 == 0 {
                    let body = &mut s.physics.bodies[b];
                    body.set_linvel(Vector::new(h.thrown[0], h.thrown[1], h.thrown[2]), true);
                    body.set_angvel(Vector::new(h.thrown[3], h.thrown[4], h.thrown[5]), true);
                }
            }
        }
        while at < row.step { s.step(); at += 1; }
        assert_eq!(s.fuel.len(), row.fuel);
        assert_eq!(fuel_digest(&s), row.digest, "{} step {}: the FUEL differ from the browser game's", row.part, row.step);
    }
}
