//! Whole seasons played headless (your matches decided as a Champs driver's would be): every stage
//! reachable, nothing stuck, a save that loads back the same.
use campaign::model::RobotRating;
use campaign::season::{Difficulty, Length, Season, Stage};
use campaign::Ratings;

fn ratings() -> Ratings {
    let keys = ["2910", "4414", "8793", "1678", "971", "1690", "341", "581"];
    Ratings { robots: keys.iter().enumerate().map(|(i, k)| RobotRating { key: k.to_string(), opr: 300.0 - 25.0 * i as f64, opr_auto: 40.0, opr_fuel: 270.0 - 25.0 * i as f64, opr_tower: 30.0, elo: 1500.0 }).collect(), residual_sd: 70.0 }
}

/// play your event to its end, deciding your matches too
fn play_event(s: &mut Season, r: &Ratings) {
    let e = s.event_mut();
    while let Some(i) = e.next_for(0) {
        e.decide_quals_until(i, r);
        let (red, blue) = (e.quals[i].red.clone(), e.quals[i].blue.clone());
        let (a, b) = e.decide(&red, &blue, r);
        e.record_qual(i, a, b);
    }
    let q = e.quals.len();
    e.decide_quals_until(q, r);
    if !e.quals.is_empty() {
        e.start_selection();
        while e.picking().is_some() { let t = e.ai_pick(r); e.pick(t, true); }
    }
    e.seed_bracket();
    while let Some(i) = e.next_bout() { e.decide_bout(i, r); }
    assert!(e.playoffs_done());
}

#[test]
fn seasons_play_through() {
    let r = ratings();
    let mut reached = std::collections::HashMap::new();
    for seed in 1..60u32 {
        let d = [Difficulty::Casual, Difficulty::Standard, Difficulty::HallOfFame][seed as usize % 3];
        let robot = ["2910", "971", "581"][(seed as usize / 3) % 3];
        let mut s = Season::new(seed, d, "Our Team", robot, &r);
        let mut guard = 0;
        let mut furthest = s.stage;
        while s.stage != Stage::Over {
            play_event(&mut s, &r);
            // (a save mid-season loads back to the same)
            let saved = s.save();
            assert_eq!(Season::load(&saved).unwrap().save(), saved);
            furthest = s.stage;
            s.advance(&r);
            guard += 1;
            assert!(guard < 10, "a season that doesn't end");
        }
        *reached.entry(format!("{furthest:?}")).or_insert(0) += 1;
        assert!(s.score(&r) >= 0.0);
        assert_eq!(s.events.iter().filter(|e| e.level == campaign::Level::District).count(), 4);
    }
    eprintln!("furthest stage reached: {reached:?}");
    assert!(reached.contains_key("DistrictChampionship"), "nobody made the District Championship");
}

#[test]
fn short_seasons_and_the_next_season() {
    let r = ratings();
    let mut reached = std::collections::HashMap::new();
    for seed in 1..40u32 {
        let d = [Difficulty::Casual, Difficulty::Standard, Difficulty::HallOfFame][seed as usize % 3];
        let robot = ["2910", "971", "581"][(seed as usize / 3) % 3];
        let mut s = Season::new_with(seed, d, Length::Short, "Our Team", robot, &r);
        let mut furthest = s.stage;
        let mut guard = 0;
        while s.stage != Stage::Over {
            play_event(&mut s, &r);
            furthest = s.stage;
            s.advance(&r);
            guard += 1;
            assert!(guard < 6, "a short season that doesn't end");
        }
        // (one district event, then the Championship or not: never the second or the DCMP)
        assert!(!s.events.iter().any(|e| e.level == campaign::Level::DistrictChampionship));
        assert_eq!(s.events.iter().filter(|e| e.you().is_some() && e.level == campaign::Level::District).count(), 1);
        *reached.entry(format!("{furthest:?}")).or_insert(0) += 1;
        // the season after: the same team, a year on, this one in its history
        let n = s.next_season(&r);
        assert_eq!((n.year, n.history.len(), n.length, n.stage), (2, 1, Length::Short, Stage::District1));
        assert_eq!(n.history[0].score, s.score(&r));
        let saved = n.save();
        assert_eq!(Season::load(&saved).unwrap().save(), saved);
    }
    eprintln!("short seasons, furthest stage: {reached:?}");
    assert!(reached.contains_key("Division"), "nobody made the Championship in a short season");
}

#[test]
fn old_saves_load_as_full_first_seasons() {
    let r = ratings();
    let s = Season::new(3, Difficulty::Standard, "Our Team", "2910", &r);
    let mut v: serde_json::Value = serde_json::from_str(&s.save()).unwrap();
    for k in ["length", "year", "history"] { v.as_object_mut().unwrap().remove(k); }
    let old = Season::load(&v.to_string()).unwrap();
    assert_eq!((old.length, old.year, old.history.len()), (Length::Full, 1, 0));
}
