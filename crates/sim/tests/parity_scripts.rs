//! The parity scripts and the JS game's traces of them. Each script's run here is added as the
//! modules it needs are ported (PORT.md); until then this checks every script has its trace.
use sim::parity::{scripts, PHASES};
#[allow(unused_imports)] use sim::parity::play;

#[test]
fn every_script_has_its_trace() {
    let all = scripts();
    assert!(!all.is_empty());
    for (s, t) in &all {
        assert_eq!(t.version, 1, "{}", s.name);
        assert_eq!(t.script, s.name);
        assert_eq!(t.steps.len(), s.steps, "{}: steps", s.name);
        for (i, st) in t.steps.iter().enumerate() {
            for p in PHASES { if p != "frame" || i % 2 == 1 { assert!(st.contains_key(p), "{} step {i}: no {p}", s.name); } }
        }
    }
}

/// The scripts whose modules are ported (PORT.md), played here and matched against the JS game's
/// traces, step by step, part by part.
const PORTED: [&str; 4] = ["fuel-start", "fuel-launch", "drive-2910", "intake-2910"];

#[test]
fn ported_scripts_play_as_in_the_js_game() {
    for (s, t) in scripts().iter().filter(|(s, _)| PORTED.contains(&s.name.as_str())) {
        let got = sim::parity::play(s);
        if let Some(m) = sim::parity::first_mismatch(t, &got.iter().map(|(a, b, c)| (*a, *b, c.clone())).collect::<Vec<_>>()) { panic!("{m}"); }
    }
}
