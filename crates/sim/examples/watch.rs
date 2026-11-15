//! Watch a headless match: every second, each robot's place, heading, load, shooter status and
//! what its AI is doing (diagnosing a robot that scores low).
//! ROBOTS="1706:red:scorer,4930:blue:scorer" SEED=1 cargo run --release -p sim --example watch
use sim::constants::*;
use sim::game::{Settings, UnitSpec, World};

fn main() {
    let spec = std::env::var("ROBOTS").unwrap_or("1706:red:scorer,4930:blue:scorer".into());
    let seed: u32 = std::env::var("SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(1);
    let every: f64 = std::env::var("EVERY").ok().and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let mut robots = Vec::new();
    let mut units = Vec::new();
    for s in spec.split(',') {
        let p: Vec<&str> = s.split(':').collect();
        let a = if p[1] == "blue" { Alliance::Blue } else { Alliance::Red };
        let (auto, start) = sim::ai::opponent_auto(p[2]);
        robots.push((p[0].to_string(), a, [f64::NAN; 3]));
        units.push(UnitSpec { driver: p[2].into(), auto: auto.into(), start: start.into(), skill: p.get(3).copied().unwrap_or("champs").into() });
    }
    let mut w = World::new(sim::WorldData::bundled(), &Settings { seed, human_players: true, preload: 8, robots, aim_truth: false, units });
    let mut next = 0.0;
    while !w.match_.over() {
        w.step(&mut |_, _| {});
        if w.steps % 2 == 0 { w.frame(2.0 * PHYSICS_DT); }
        w.match_.events.clear();
        if w.match_.t >= next {
            next += every;
            for (i, r) in w.robots.iter().enumerate() {
                let label = w.units[i].ai.as_ref().map_or(String::new(), |a| a.label.clone());
                let tw: Vec<String> = r.turret_yaws.iter().map(|y| format!("{:.0}", y.to_degrees())).collect();
                let mut extra = String::new();
                if let Some(sim::robot::Phys::R1706(p)) = &r.phys {
                    let (cy, sy) = (r.yaw.cos(), r.yaw.sin());
                    let mut side = [0, 0];
                    let mut at = Vec::new();
                    for &b in &p.p.held {
                        let t = w.physics.bodies[w.fuel.balls[b].body].translation();
                        let (dx, dz) = (t.x as f64 - r.pos[0], t.z as f64 - r.pos[2]);
                        let lz = dx * sy + dz * cy;
                        let lx = dx * cy - dz * sy;
                        at.push(format!("({:.2},{:.2},{:.2})", lx, t.y as f64, lz));
                        side[if lz < 0.0 { 0 } else { 1 }] += 1;
                    }
                    let psi: Vec<String> = p.aim_psis.iter().map(|a| a.map_or("-".into(), |a| format!("{:.0}", wrap(a - r.yaw).to_degrees()))).collect();
                    extra = format!(" side {:?} rdy {:?} want [{}] arm {:.0} shoot {} intake {} feed {} at {}", side, p.ready_t, psi.join(","), p.arm_deg, r.cmd.shoot, r.cmd.intake, r.want_shoot, if at.len() <= 5 { at.join(" ") } else { String::new() });
                }
                println!("{:6.1} {} {:>5} ({:6.2},{:6.2}) yaw {:5.0} w {:5.2} load {:3} shots {:3} tur [{}] {:<22} {}",
                    w.match_.t, r.key, format!("{:?}", r.alliance), r.pos[0], r.pos[2], r.yaw.to_degrees(), r.omega, r.stored.len(), r.stats.shots, tw.join(","), r.status, label);
                if !extra.is_empty() { println!("        {extra}"); }
            }
        }
    }
    let s = &w.match_.score;
    println!("score blue {:.0} red {:.0}", w.match_.total(Alliance::Blue), w.match_.total(Alliance::Red));
    let _ = s;
}

fn wrap(a: f64) -> f64 { let mut a = a % std::f64::consts::TAU; if a > std::f64::consts::PI { a -= std::f64::consts::TAU } if a < -std::f64::consts::PI { a += std::f64::consts::TAU } a }
