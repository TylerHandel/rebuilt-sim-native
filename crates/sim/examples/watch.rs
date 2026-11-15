//! Watch a headless match: every second, each robot's place, heading, load, shooter status and
//! what its AI is doing (diagnosing a robot that scores low).
//! ROBOTS="1706:red:scorer,4930:blue:scorer" SEED=1 cargo run --release -p sim --example watch
//! (robot:alliance:strategy[:skill[:noauto]]; EVERY=s between lines. 1706: each turret's readiness
//! and what holds it back, where its FUEL sit; 1690: its intake, the FUEL at its mouth (PTS=1 every
//! FUEL near its front, TUBE=1 them against the kicker bar's tube, CMD=1 its command))
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
        let auto = if p.get(4) == Some(&"noauto") { "none" } else { auto };
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
                if let Some(sim::robot::Phys::R1690(p)) = &r.phys {
                    let near = w.fuel.balls.iter().filter(|b| b.state == sim::fuel::State::Field && ((b.pos[0] - r.pos[0]).powi(2) + (b.pos[2] - r.pos[2]).powi(2)).sqrt() < 1.0).count();
                    let ahead = w.fuel.balls.iter().filter(|b| { let (dx, dz) = (b.pos[0] - r.pos[0], b.pos[2] - r.pos[2]); let (c, s) = (r.yaw.cos(), r.yaw.sin()); let lx = dx * c - dz * s; let lz = dx * s + dz * c; b.state == sim::fuel::State::Field && lx > 0.3 && lx < 0.8 && lz.abs() < 0.35 && b.pos[1] < 0.2 }).count();
                    let field = w.fuel.balls.iter().filter(|b| b.state == sim::fuel::State::Field && b.pos[0].abs() < 3.5).count();
                    let path = &p.p.path;
                    let mut touch: Vec<String> = Vec::new();
                    for c in &path.contacts {
                        let nm = match c.wheel { sim::mech::mechanism::Toucher::Wheel(i) => path.wheels[i].name.clone(), sim::mech::mechanism::Toucher::Surface(i) => p.p.surfaces[i].name.clone(), _ => "fuel".into() };
                        if nm.contains("intake") || nm.contains("kick") || nm.contains("carriage") || nm.contains("bumper") { touch.push(format!("{}:{:.0}", nm.replace("intake roller ", "ir-").chars().take(14).collect::<String>(), c.force)); }
                    }
                    let iw = p.p.drives.trains[p.t.intake].omega;
                    let mut pts: Vec<(f64, String)> = w.fuel.balls.iter().filter(|b| b.state == sim::fuel::State::Field).filter_map(|b| { let (dx, dz) = (b.pos[0] - r.pos[0], b.pos[2] - r.pos[2]); let (c, s) = (r.yaw.cos(), r.yaw.sin()); let lx = dx * c - dz * s; let lz = dx * s + dz * c; (lx > -0.1 && lx < 0.9 && lz.abs() < 0.45).then(|| (lx, format!("#{}({:.2},{:.3},{:.2})", b.id, lx, b.pos[1], lz))) }).collect();
                    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
                    if std::env::var("TUBE").is_ok() {
                        let q = r.quat; // (x, y, z, w): world -> robot frame by the inverse rotation
                        let inv = |v: [f64; 3]| { let (x, y, z, w) = (-q[0], -q[1], -q[2], q[3]); let ix = w * v[0] + y * v[2] - z * v[1]; let iy = w * v[1] + z * v[0] - x * v[2]; let iz = w * v[2] + x * v[1] - y * v[0]; let iw = -x * v[0] - y * v[1] - z * v[2]; [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x] };
                        let tube = &p.p.surfaces[sim::mech::r1690::S_KICK_TUBE];
                        let carriage = &p.p.surfaces[sim::mech::r1690::S_CARRIAGE];
                        let mut o = Vec::new();
                        for b in w.fuel.balls.iter().filter(|b| b.state == sim::fuel::State::Field) {
                            let l = inv([b.pos[0] - r.pos[0], b.pos[1] - r.pos[1], b.pos[2] - r.pos[2]]);
                            if l[0] > 0.35 && l[0] < 0.65 && l[2].abs() < 0.33 && l[1] < 0.2 { let t = tube.to_local(l); o.push(format!("#{} robot({:.3},{:.3},{:.2}) tube({:.3},{:.3}) r{:.3}", b.id, l[0], l[1], l[2], t[0], t[1], (t[0] * t[0] + t[1] * t[1]).sqrt())); }
                        }
                        println!("        tube@{:?} carriage@{:?}: {}", tube.pos, carriage.pos, o.join(" | "));
                    }
                    if std::env::var("PTS").is_ok() { println!("        balls: {}", pts.iter().map(|x| x.1.clone()).collect::<Vec<_>>().join(" ")); }
                    let extra2 = format!(" y {:.3} pitch {:.1}", r.pos[1], { let q = r.quat; (2.0 * (q[3] * q[2] + q[0] * q[1])).asin().to_degrees() }); let extra2 = format!("{extra2} kick2 {:.1} iw {:.0} n {}", p.kick.to_degrees(), iw, touch.len());
                    let extra2 = if std::env::var("CMD").is_ok() { format!("{extra2} cmd {:?}", r.cmd) } else { extra2 };
                    extra = format!("{extra2} intake {} deploy {:.2} pos {:.3} full {} near {} ahead {} neutral {} vel {:.2} dir {:.0} cmdv {:.1},{:.1}", r.cmd.intake, p.deploy, p.pos, r.full, near, ahead, field, (r.vel[0].powi(2) + r.vel[2].powi(2)).sqrt(), { let (c, s) = (r.yaw.cos(), r.yaw.sin()); let lx = r.vel[0] * c - r.vel[2] * s; let lz = r.vel[0] * s + r.vel[2] * c; lz.atan2(lx).to_degrees() }, r.cmd.vx, r.cmd.vz);
                }
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
                    extra = format!(" why {:?} side {:?} rdy {:?} want [{}] arm {:.0} shoot {} intake {} feed {} at {}", p.why, side, p.ready_t, psi.join(","), p.arm_deg, r.cmd.shoot, r.cmd.intake, r.want_shoot, if at.len() <= 5 { at.join(" ") } else { String::new() });
                }
                println!("{:6.1} {} {:>5} ({:6.2},{:6.2}) yaw {:5.0} w {:5.2} load {:3} in {:3} shots {:3} tur [{}] {:<22} {}",
                    w.match_.t, r.key, format!("{:?}", r.alliance), r.pos[0], r.pos[2], r.yaw.to_degrees(), r.omega, r.stored.len(), r.stats.intaked, r.stats.shots, tw.join(","), r.status, label);
                if !extra.is_empty() { println!("        {extra}"); }
            }
        }
    }
    let s = &w.match_.score;
    println!("score blue {:.0} red {:.0}", w.match_.total(Alliance::Blue), w.match_.total(Alliance::Red));
    let _ = s;
}

fn wrap(a: f64) -> f64 { let mut a = a % std::f64::consts::TAU; if a > std::f64::consts::PI { a -= std::f64::consts::TAU } if a < -std::f64::consts::PI { a += std::f64::consts::TAU } a }
