//! The parity scripts and the JS game's traces of them. Each script's run here is added as the
//! modules it needs are ported (PORT.md); until then this checks every script has its trace.
use sim::parity::{scripts, PHASES};

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
