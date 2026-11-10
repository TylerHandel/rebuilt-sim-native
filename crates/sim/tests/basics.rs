//! The ported basics against the JS game: its random draws, its constants and their functions,
//! and the engine settings it runs.
use sim::constants::*;
use sim::rng::Mulberry32;
use sim::{Sim, WorldData};

#[test]
fn mulberry32_draws_what_the_js_game_draws() {
    // (tools/headless.mjs mulberry32: draws 1, 2 and 1000 for three seeds, as node printed them)
    for (seed, want) in [
        (1u32, [0.6270739405881613, 0.002735721180215478, 0.12366653862409294]),
        (42, [0.6011037519201636, 0.44829055899754167, 0.6425111389253289]),
        (4294967295, [0.8964226141106337, 0.189478256739676, 0.39633008907549083]),
    ] {
        let mut r = Mulberry32::new(seed);
        let d: Vec<f64> = (0..1000).map(|_| r.random()).collect();
        assert_eq!([d[0], d[1], d[999]].map(f64::to_bits), want.map(f64::to_bits), "seed {seed}");
        assert_eq!(r.draws, 1000);
    }
}

#[test]
fn constants_are_the_js_games() {
    assert_eq!(FUEL::radius.to_bits(), ((5.91 * IN) / 2.0).to_bits());
    assert_eq!(PHYSICS_DT.to_bits(), (1.0f64 / 120.0).to_bits());
    assert_eq!(HALF_L.to_bits(), (FIELD_L / 2.0).to_bits());
    assert_eq!(GROUP::TRENCH_ARM, 256);
    assert_eq!(POINTS::tower, [0.0, 10.0, 20.0, 30.0]);
    assert_eq!(BLUE, "blue");
    // groups(): the field's static colliders as js/physics.js makes them (65710, as exported), and a
    // filter that takes STATIC takes TRENCH_ARM too
    assert_eq!(groups(GROUP::STATIC, GROUP::BALL | GROUP::HELD | GROUP::ROBOT | GROUP::WHEEL | GROUP::INTAKE), 65710);
    assert_eq!(groups(GROUP::ROBOT, GROUP::STATIC | GROUP::BALL), (4 << 16) | (1 | 2 | 256));
    assert_eq!(fw(1.0, 2.0, Alliance::Red), (-(1.0 - HALF_L), -(HALF_W - 2.0)));
    assert_eq!(trench_arm_parts(false).len(), 4);
}

#[test]
fn rapier_settings_are_the_js_games() {
    let d = WorldData::bundled();
    let s = Sim::new(d.clone());
    let (ip, js) = (&s.physics.integration_parameters, &d.physics.integration);
    let f = |k: &str| js[k].as_f64().unwrap();
    assert_eq!(ip.dt, f("dt") as f32);
    assert_eq!(ip.length_unit, f("lengthUnit") as f32);
    assert_eq!(ip.normalized_allowed_linear_error, f("normalizedAllowedLinearError") as f32);
    assert_eq!(ip.normalized_prediction_distance, f("normalizedPredictionDistance") as f32);
    assert_eq!(ip.num_solver_iterations, f("numSolverIterations") as usize);
    assert_eq!(ip.num_internal_pgs_iterations, f("numInternalPgsIterations") as usize);
    assert_eq!(ip.max_ccd_substeps, f("maxCcdSubsteps") as usize);
    assert_eq!(ip.contact_softness.erp(ip.dt), f("contact_erp") as f32);
}
