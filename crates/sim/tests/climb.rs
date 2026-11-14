//! The climber worked by hand (js robot.js _manualClimb): 1678 backed up to its TOWER raises its
//! climber (its lid with it), the hooks go over the RUNG, lowering it climbs to LEVEL 1, raising it
//! lets the robot back down. And away from the TOWER, it only raises and lowers the lid.
use sim::constants::{Alliance, PHYSICS_DT};
use sim::game::{Settings, World};
use sim::game_match::Phase;
use sim::robot::ClimbState;

fn world() -> World {
    let s = Settings { seed: 3, human_players: false, preload: 0, robots: vec![("1678".into(), Alliance::Blue, [0.0, 0.0, 0.0])], aim_truth: false, units: Vec::new() };
    let mut w = World::new(sim::WorldData::bundled(), &s);
    w.robots[0].human_driven = true;
    w.jump_to(Phase::Teleop);
    w
}
fn step(w: &mut World, n: usize) {
    for _ in 0..n {
        w.step(&mut |_, _| {});
        if w.steps % 2 == 0 { w.frame(2.0 * PHYSICS_DT); }
    }
}
fn lid(w: &World) -> f64 {
    let r = &w.robots[0];
    match &r.phys { Some(sim::robot::Phys::R1678(p)) => p.lift_now(r.cfg["bay"]["lift"]["h"].as_f64().unwrap()), _ => panic!() }
}

#[test]
fn climbs_by_hand_at_its_tower() {
    let mut w = world();
    let p = w.robots[0].climb_pose();
    w.robots[0].spawn(&mut w.physics, p[0] + 0.8, p[1] + 0.3, p[2] + 0.2);
    for _ in 0..480 {
        let r = &mut w.robots[0];
        let (ex, ez) = (p[0] - r.pos[0], p[1] - r.pos[2]);
        let ey = (p[2] - r.yaw).sin().atan2((p[2] - r.yaw).cos());
        r.cmd = sim::parity::Cmd { vx: ex * 3.0, vz: ez * 3.0, omega: ey * 4.0, ..Default::default() };
        step(&mut w, 1);
    }
    w.robots[0].cmd = Default::default();
    assert_eq!(w.robots[0].toggle_climber(), "Climber up");
    step(&mut w, 300);
    assert!(w.robots[0].hooked, "hooked at the TOWER");
    assert!(lid(&w) > 0.99, "the lid up with the climber");
    assert_eq!(w.robots[0].toggle_climber(), "Climber down: climbing");
    step(&mut w, 360);
    let r = &w.robots[0];
    assert_eq!((r.climb_state, r.climb_level), (ClimbState::Hanging, 1));
    assert!(r.pos[1] > 0.15, "up off the carpet: {}", r.pos[1]);
    w.robots[0].toggle_climber();
    step(&mut w, 360);
    let r = &w.robots[0];
    assert_eq!((r.climb_state, r.climb_level), (ClimbState::None, 0));
    assert!(r.pos[1].abs() < 0.01, "back down: {}", r.pos[1]);
}

#[test]
fn raises_only_the_lid_away_from_its_tower() {
    let mut w = world();
    w.robots[0].spawn(&mut w.physics, -2.0, 0.0, 0.0);
    w.robots[0].toggle_climber();
    step(&mut w, 300);
    assert!(!w.robots[0].hooked && lid(&w) > 0.99, "ext {} lid {} y {}", w.robots[0].climber_ext, lid(&w), w.robots[0].pos[1]);
    w.robots[0].toggle_climber();
    step(&mut w, 300);
    assert!(lid(&w) < 0.01 && w.robots[0].climb_state == ClimbState::None);
}
