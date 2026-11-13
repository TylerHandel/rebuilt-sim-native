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
const PORTED: [&str; 46] = ["fuel-start", "fuel-launch", "drive-2910", "intake-2910", "shoot-2910", "hp-throws", "intake-4414", "shoot-4414", "intake-8793", "shoot-8793", "trench-8793", "intake-1678", "shoot-1678", "intake-1690", "shoot-1690", "intake-7769", "shoot-7769", "intake-341", "shoot-341", "intake-1114", "shoot-1114", "intake-6328", "shoot-6328", "intake-604", "shoot-604", "intake-581", "shoot-581", "intake-2056", "shoot-2056", "intake-1706", "shoot-1706", "intake-971", "shoot-971", "intake-4946", "shoot-4946", "intake-3928", "shoot-3928", "intake-4930", "shoot-4930", "fold-341", "fold-581", "fold-604", "fold-6328", "rules-pin", "rules-intake", "rules-auto"];

#[test]
fn ported_scripts_play_as_in_the_js_game() {
    // (PARITY_ONLY=<name> plays that one alone)
    let only = std::env::var("PARITY_ONLY").ok();
    for (s, t) in scripts().iter().filter(|(s, _)| PORTED.contains(&s.name.as_str()) && only.as_deref().is_none_or(|o| s.name == o)) {
        let (got, fouls) = sim::parity::play_fouls(s);
        if let Some(m) = sim::parity::first_mismatch(t, &got.iter().map(|(a, b, c)| (*a, *b, c.clone())).collect::<Vec<_>>()) { panic!("{m}"); }
        if let Some(want) = &t.fouls { assert_eq!(&fouls, want, "{}: the fouls called", s.name); }
    }
}
