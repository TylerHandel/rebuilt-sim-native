//! The mechanism engine against the browser game's (rebuilt-sim-2026 tools/native-mech.mjs, its
//! scene: a box frame driving on the field, a ball path and a sub-stepped shooter on it, 12 FUEL),
//! every step to the bit: assets/parity-mech.json.
use rapier3d::prelude::*;
use sim::mech::drives::{Drives, Motor, Shaft};
use sim::mech::mechanism::{Kind, Mechanism, Surface, Wheel};
use sim::parity::Fnv;

#[test]
fn mechanism_matches_the_browser_game() {
    let data = sim::WorldData::bundled();
    let want: serde_json::Value = serde_json::from_str(include_str!("../../../assets/parity-mech.json")).unwrap();
    let (mut physics, _) = sim::field_physics(&data);
    let f = &data.fuel;
    // the frame (js SCENE.frame)
    let frame = physics.bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::new(-6.0, 0.12, 0.0)));
    physics.colliders.insert_with_parent(ColliderBuilder::cuboid(0.35, 0.1, 0.35), frame, &mut physics.bodies);
    physics.bodies[frame].set_linvel(Vector::new(0.4, 0.0, 0.1), true);
    physics.bodies[frame].set_angvel(Vector::new(0.0, 0.3, 0.0), true);
    let mut fuel = Vec::new();
    for i in 0..12usize {
        let (x0, y0, z0, sx, sy, sz) = (-6.25f64, 0.45f64, -0.2f64, 0.15625f64, 0.15625f64, 0.125f64);
        let p = Vector::new((x0 + (i % 4) as f64 * sx) as f32, (y0 + (i / 4) as f64 * sy) as f32, (z0 + (i % 3) as f64 * sz) as f32);
        let body = RigidBodyBuilder::dynamic().translation(p).linear_damping(f.body.linear_damping).angular_damping(f.body.angular_damping).ccd_enabled(f.body.ccd).can_sleep(true);
        let col = ColliderBuilder::ball(f.body.radius).mass(f.body.mass).friction(f.collider.friction).restitution(f.collider.restitution).collision_groups(sim::groups(f.collider.collision_groups));
        let (h, _) = physics.insert(body, col);
        fuel.push(h);
    }
    let mut drives = Drives::default();
    let roller = drives.shaft(Shaft::new("roller", Some(Motor::new("krakenX60", 1.0, 3.0, 0.9, 60.0, f64::INFINITY)), 5e-5, 2e-4, 0.0));
    drives.shafts[roller].cmd = Some(1.0);
    let idler = drives.shaft(Shaft::new("idler", None, 5e-5, 2e-4, 0.0));
    let drum = drives.shaft(Shaft::new("drum", Some(Motor::new("krakenX60", 2.0, 1.0, 0.9, 80.0, f64::INFINITY)), 2e-3, 2e-4, 0.0));
    drives.shafts[drum].cmd = Some(0.8);
    let mut surfaces = vec![
        Surface::mesh("plate", &[0.0, 0.1, -0.3, 0.6, 0.1, -0.3, 0.6, 0.1, 0.3, 0.0, 0.1, 0.3, 0.0, 0.3, 0.3], &[0, 2, 1, 0, 3, 2, 3, 4, 0], 0.3, [-0.3, 0.05, 0.0], 0.2, 0.1, 0.08, f64::INFINITY),
        Surface::block("bumper", 0.35, 0.43, 0.03, 0.16, 0.4, 0.02, 0.35, f64::INFINITY),
    ];
    let mut path = Mechanism::new(Some(frame), vec![roller, idler], vec![
        Wheel::new("roller", roller, [0.25, 0.2, 0.0], [0.0, 0.0, 1.0], 0.05, 0.5).mu(1.0),
        Wheel::new("omni", idler, [0.05, 0.38, 0.0], [0.0, 0.0, 2.0], 0.03, 0.4).kind(Kind::Omni).mu(0.8).tread(4000.0),
    ], vec![0, 1], 4, 1);
    let mut shooter = Mechanism::new(Some(frame), vec![drum], vec![Wheel::new("drum", drum, [-0.2, 0.3, 0.0], [0.0, 0.0, 1.0], 0.0508, 0.6).mu(1.1)], vec![], 4, 8);
    let dt = physics.integration_parameters.dt as f64;
    let digests = want["digests"].as_array().unwrap();
    let (mut seen, mut sub_steps) = (std::collections::BTreeMap::new(), 0usize);
    for (i, d) in digests.iter().enumerate() {
        path.step(dt, &mut physics.bodies, &mut drives, &mut surfaces, &fuel, Some(&fuel), None);
        shooter.step(dt, &mut physics.bodies, &mut drives, &mut surfaces, &fuel, None, None);
        for t in path.contacts.iter() { *seen.entry(format!("{:?}", t.wheel)).or_insert(0usize) += 1; }
        for _ in shooter.contacts.iter() { *seen.entry("drum".to_string()).or_insert(0usize) += 1; }
        if shooter.n_sub > 1 { sub_steps += 1; }
        physics.step();
        let mut h = Fnv::default();
        for &b in fuel.iter().chain([frame].iter()) {
            let bd = &physics.bodies[b];
            let (t, q, v, w) = (bd.translation(), *bd.rotation(), bd.linvel(), bd.angvel());
            for x in [t.x, t.y, t.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z] { h.f32(x); }
        }
        for s in [roller, idler, drum] { h.f64(drives.omega(s)); }
        assert_eq!(h.hex(), d.as_str().unwrap(), "step {} (of {})", i + 1, digests.len());
    }
    // (the scene must use the engine: every wheel and surface touched FUEL, FUEL touched each other)
    eprintln!("contacts over the run: {seen:?}; shooter sub-stepped {sub_steps} steps; drum {:.1} rad/s, roller {:.1}", drives.omega(drum), drives.omega(roller));
    for k in ["Wheel(0)", "Wheel(1)", "Surface(0)", "Surface(1)", "drum"] { assert!(seen.get(k).copied().unwrap_or(0) > 0, "nothing touched {k}"); }
}
