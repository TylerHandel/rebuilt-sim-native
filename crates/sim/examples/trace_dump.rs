//! A parity script's state at one step, after each part of it, as tools/native-trace.mjs --dump prints it:
//!   cargo run --release -p sim --example trace_dump -- parity/scripts/<script>.json <step>
//! LOC=1: each step's pose estimate too, as native-trace --loc prints it
use sim::game::{Settings, World};
use sim::game_match::Phase;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let s: sim::parity::Script = serde_json::from_str(&std::fs::read_to_string(&a[1]).unwrap()).unwrap();
    let at: usize = a[2].parse().unwrap();
    let mut slots: Vec<_> = s.slots.iter().filter(|x| x.alliance == "blue").collect();
    slots.extend(s.slots.iter().filter(|x| x.alliance == "red"));
    let robots = slots.iter().map(|x| (x.robot.clone(), if x.alliance == "blue" { sim::constants::Alliance::Blue } else { sim::constants::Alliance::Red }, x.pose)).collect();
    let mut w = World::new(sim::WorldData::bundled(), &Settings { seed: s.seed, human_players: s.human_players.unwrap_or(true), preload: s.preload, robots, aim_truth: s.aim_truth });
    for (i, x) in slots.iter().enumerate() { w.robots[i].spawn(&mut w.physics, x.pose[0], x.pose[1], x.pose[2]); }
    if let Some(p) = &s.phase { w.jump_to(Phase::parse(p)); }
    let mut cmds: Vec<_> = s.commands.iter().collect();
    cmds.sort_by_key(|c| c.from);
    let p = |b: &rapier3d::prelude::RigidBody| { let (t, v) = (b.translation(), b.linvel()); format!("({}, {}, {}) v ({}, {}, {})", t.x as f64, t.y as f64, t.z as f64, v.x as f64, v.y as f64, v.z as f64) };
    for step in 0..=at {
        for c in cmds.iter().filter(|c| c.from == step) { w.robots[c.robot].cmd = c.cmd; }
        w.step(&mut |w, phase| {
            if step != at { return }
            eprintln!("--- step {step} {phase}: draws {}", w.rng.draws);
            for (i, b) in w.fuel.balls.iter().enumerate() { let bd = &w.physics.bodies[b.body]; if bd.is_enabled() { eprintln!("fuel {i} {} {}", b.state.name(), p(bd)); } }
            for (i, r) in w.robots.iter().enumerate() { eprintln!("robot {i}  {}", p(&w.physics.bodies[r.body])); }
        });
        if step % 2 == 1 { w.frame(2.0 * sim::constants::PHYSICS_DT); }
        if std::env::var("LOC").is_ok() { for r in &w.robots { eprintln!("{step} {}", r.loc.log_line()); } }
    }
}
