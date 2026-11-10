//! What a parity script's robots did: FUEL held, intaked, shot, passed, and the score, every second.
//!   cargo run --release -p sim --example script_stats -- parity/scripts/<script>.json
use sim::constants::Alliance;
use sim::game::{Settings, World};
use sim::game_match::Phase;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let s: sim::parity::Script = serde_json::from_str(&std::fs::read_to_string(&a[1]).unwrap()).unwrap();
    let mut slots: Vec<_> = s.slots.iter().filter(|x| x.alliance == "blue").collect();
    slots.extend(s.slots.iter().filter(|x| x.alliance == "red"));
    let robots = slots.iter().map(|x| (x.robot.clone(), if x.alliance == "blue" { Alliance::Blue } else { Alliance::Red }, x.pose)).collect();
    let mut w = World::new(sim::WorldData::bundled(), &Settings { seed: s.seed, human_players: s.human_players.unwrap_or(true), preload: s.preload, robots, aim_truth: s.aim_truth });
    for (i, x) in slots.iter().enumerate() { w.robots[i].spawn(&mut w.physics, x.pose[0], x.pose[1], x.pose[2]); }
    if let Some(p) = &s.phase { w.jump_to(Phase::parse(p)); }
    let mut cmds: Vec<_> = s.commands.iter().collect();
    cmds.sort_by_key(|c| c.from);
    for step in 0..s.steps {
        for c in cmds.iter().filter(|c| c.from == step) { w.robots[c.robot].cmd = c.cmd; }
        w.step(&mut |_, _| {});
        if step % 2 == 1 { w.frame(2.0 * sim::constants::PHYSICS_DT); }
        if (step + 1) % 120 == 0 {
            if w.robots.is_empty() { println!("{:5.1}s HUMAN PLAYERS thrown {} / {} | blue {} red {}", (step + 1) as f64 * sim::constants::PHYSICS_DT, w.hps[0].thrown, w.hps[1].thrown, w.match_.total(Alliance::Blue), w.match_.total(Alliance::Red)); }
            for r in &w.robots {
                println!("{:5.1}s {} held {:2} intaked {:2} shots {:2} passes {:2} | {} | at ({:.2}, {:.2}) yaw {:.2} arm {:.2} | blue {} red {}", (step + 1) as f64 * sim::constants::PHYSICS_DT, r.key, r.stored.len(), r.stats.intaked, r.stats.shots, r.stats.passes, r.status, r.pos[0], r.pos[2], r.yaw, r.intake_deploy, w.match_.total(Alliance::Blue), w.match_.total(Alliance::Red));
            }
        }
    }
}
