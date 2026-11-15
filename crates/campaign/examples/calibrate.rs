//! How hard the campaign is with the real ratings: many seasons with your matches decided as a
//! Champs driver's would be, your win rate at each level and how far seasons get, by robot and
//! difficulty. cargo run --release -p campaign --example calibrate -- [robot] [seasons]
use campaign::season::{Difficulty, Season, Stage};
use campaign::{Level, Ratings};
use std::collections::BTreeMap;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let r = Ratings::from_json(&std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/ratings.json")).unwrap());
    let robots: Vec<String> = if a.len() > 1 && a[1] != "all" { vec![a[1].clone()] } else { let o = r.order(); vec![o[0].clone(), o[o.len() / 2].clone(), o[o.len() - 1].clone()] };
    let n: u32 = a.get(2).and_then(|x| x.parse().ok()).unwrap_or(200);
    for robot in &robots {
        for d in [Difficulty::Casual, Difficulty::Standard, Difficulty::HallOfFame] {
            let mut wins: BTreeMap<String, (f64, f64)> = BTreeMap::new();
            let mut reached: BTreeMap<String, u32> = BTreeMap::new();
            for seed in 0..n {
                let mut s = Season::new(seed * 7 + 1, d, "Us", robot, &r);
                let mut furthest = Stage::District1;
                while s.stage != Stage::Over {
                    furthest = s.stage;
                    let lvl = s.event().level;
                    let e = s.event_mut();
                    let mut tally = |w: bool| { let x = wins.entry(format!("{lvl:?}")).or_default(); x.1 += 1.0; if w { x.0 += 1.0; } };
                    while let Some(i) = e.next_for(0) {
                        e.decide_quals_until(i, &r);
                        let (red, blue) = (e.quals[i].red.clone(), e.quals[i].blue.clone());
                        let (x, y) = e.decide(&red, &blue, &r);
                        let red_us = red.contains(&0);
                        tally(if red_us { x.total() > y.total() } else { y.total() > x.total() });
                        e.record_qual(i, x, y);
                    }
                    let q = e.quals.len();
                    e.decide_quals_until(q, &r);
                    if !e.quals.is_empty() { e.start_selection(); while e.picking().is_some() { let t = e.ai_pick(&r); e.pick(t, true); } }
                    e.seed_bracket();
                    while let Some(i) = e.next_bout() { e.decide_bout(i, &r); }
                    if lvl == Level::Einstein { furthest = Stage::Einstein; }
                    s.advance(&r);
                }
                let won_all = s.events.iter().any(|e| e.level == Level::Einstein && e.bracket.last().unwrap().winner.is_some_and(|w| e.selection.alliances[w].contains(&0)));
                *reached.entry(if won_all { "Champion".into() } else { format!("{furthest:?}") }).or_default() += 1;
            }
            let w: Vec<String> = wins.iter().map(|(k, (a, b))| format!("{k} {:.0}%", 100.0 * a / b)).collect();
            println!("{robot:>5} {:<12} quals won: {}  | furthest: {:?}", d.name(), w.join(", "), reached);
        }
    }
}
