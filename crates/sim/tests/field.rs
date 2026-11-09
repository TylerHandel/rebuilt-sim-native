use sim::{Sim, Vector, WorldData};

#[test]
fn field_loads_as_exported() {
    let d = WorldData::bundled();
    let (field, fuel) = (d.field.len(), d.fuel.start.len());
    let s = Sim::new(d);
    // the fixed body holds every field collider; each FUEL has one collider
    assert_eq!(s.physics.colliders.len(), field + fuel);
    assert_eq!(s.fuel.len(), fuel);
}

#[test]
fn fuel_settles_on_the_field() {
    let mut s = Sim::new(WorldData::bundled());
    for _ in 0..(3.0 / s.dt()) as usize {
        s.step();
    }
    for (i, (p, _)) in s.fuel_poses().enumerate() {
        assert!(p.is_finite(), "FUEL {i} at {p}");
        assert!(p.y > -0.02 && p.y < 2.0, "FUEL {i} fell or flew: {p}");
        assert!(p.x.abs() < 8.5 && p.z.abs() < 4.2, "FUEL {i} left the field: {p}");
    }
}

#[test]
fn a_dropped_fuel_bounces_and_rests_on_the_carpet() {
    let mut s = Sim::new(WorldData::bundled());
    let h = s.add_fuel(Vector::new(0.0, 2.0, 3.0));
    let r = s.data.fuel.body.radius;
    let mut top_after_bounce: f32 = 0.0;
    let mut hit = false;
    for _ in 0..(4.0 / s.dt()) as usize {
        s.step();
        let b = &s.physics.bodies[h];
        if b.linvel().y > 0.0 { hit = true; }
        if hit { top_after_bounce = top_after_bounce.max(b.translation().y); }
    }
    let y = s.physics.bodies[h].translation().y;
    assert!((y - r).abs() < 0.01, "rests on the carpet: y {y}, r {r}");
    // restitution 0.45 against the carpet's: it comes back up, well under where it started
    assert!(top_after_bounce > 0.2 && top_after_bounce < 1.2, "first bounce to {top_after_bounce}");
}

#[test]
fn the_fixed_body_holds_the_field() {
    let d = WorldData::bundled();
    let n = d.field.len();
    let s = Sim::new(d);
    assert_eq!(s.physics.bodies[s.fixed].colliders().len(), n);
}
