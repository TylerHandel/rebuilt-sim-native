//! A generic robot's held FUEL (its hopper, robot frame) after its preload and after given steps of
//! a parity script, as a debugging aid against the browser game's (HOP_AT in a logging trace):
//!   cargo run --release -p sim --example hop_dump -- parity/scripts/<script>.json 0 59
use sim::game::{Settings, World};
use sim::game_match::Phase;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let s: sim::parity::Script = serde_json::from_str(&std::fs::read_to_string(&a[1]).unwrap()).unwrap();
    let at: Vec<usize> = a[2..].iter().map(|x| x.parse().unwrap()).collect();
    let mut slots: Vec<_> = s.slots.iter().filter(|x| x.alliance == "blue").collect();
    slots.extend(s.slots.iter().filter(|x| x.alliance == "red"));
    let robots = slots.iter().map(|x| (x.robot.clone(), if x.alliance == "blue" { sim::constants::Alliance::Blue } else { sim::constants::Alliance::Red }, x.pose.unwrap())).collect();
    let mut w = World::new(sim::WorldData::bundled(), &Settings { seed: s.seed, human_players: s.human_players.unwrap_or(true), preload: s.preload, robots, aim_truth: s.aim_truth, units: Vec::new() });
    for (i, x) in slots.iter().enumerate() { w.robots[i].spawn(&mut w.physics, x.pose.unwrap()[0], x.pose.unwrap()[1], x.pose.unwrap()[2]); }
    if let Some(p) = &s.phase { w.jump_to(Phase::parse(p)); }
    let hop = |w: &World, tag: &str| { if let Some(g) = &w.robots[0].generic { for e in &g.hopper.list { eprintln!("hop {tag} {} {} {} {}", e.b, e.p[0], e.p[1], e.p[2]); } } };
    hop(&w, "pre");
    let mut cmds: Vec<_> = s.commands.iter().collect();
    cmds.sort_by_key(|c| c.from);
    for step in 0..=*at.iter().max().unwrap_or(&0) {
        for c in cmds.iter().filter(|c| c.from == step) { w.robots[c.robot].cmd = c.cmd; }
        w.step(&mut |_, _| {});
        if step % 2 == 1 { w.frame(2.0 * sim::constants::PHYSICS_DT); }
        if at.contains(&step) { hop(&w, &step.to_string()); }
    }
}
