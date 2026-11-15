//! A whole event decided headless: the schedule's fairness, the rankings, alliance selection, the
//! bracket, the district points.
use campaign::model::{team_names, RobotRating};
use campaign::rng::Rng;
use campaign::{Event, Level, Ratings, Skill, Team};

fn ratings() -> Ratings {
    let keys = ["a", "b", "c", "d", "e", "f"];
    Ratings { robots: keys.iter().enumerate().map(|(i, k)| RobotRating { key: k.to_string(), opr: 100.0 + 40.0 * i as f64, opr_auto: 20.0, opr_fuel: 80.0 + 40.0 * i as f64, opr_tower: 20.0, elo: 1500.0 }).collect(), residual_sd: 60.0 }
}

fn teams(n: usize, seed: u32) -> Vec<Team> {
    let mut rng = Rng::new(seed);
    let names = team_names(n, &mut rng);
    let skills = [Skill::Rookie, Skill::Regional, Skill::Champs, Skill::Trained];
    (0..n).map(|i| Team { id: i, name: names[i].clone(), robot: ["a", "b", "c", "d", "e", "f"][i % 6].into(), skill: skills[(i / 6) % 4], you: i == 0 }).collect()
}

fn play(n: usize, seed: u32) -> Event {
    let r = ratings();
    let mut e = Event::new("Test District", Level::District, teams(n, seed), seed);
    let q = e.quals.len();
    e.decide_quals_until(q, &r);
    assert!(e.quals_done());
    e.start_selection();
    while e.picking().is_some() { let t = e.ai_pick(&r); e.pick(t, true); }
    e.seed_bracket();
    while let Some(i) = e.next_bout() { e.decide_bout(i, &r); }
    e
}

#[test]
fn schedule_is_fair() {
    for (n, seed) in [(24, 1), (29, 2), (36, 3), (40, 4)] {
        let e = Event::new("t", Level::District, teams(n, seed), seed);
        let mut count = vec![0usize; n];
        let mut counted = vec![0usize; n];
        for m in &e.quals {
            assert_eq!(m.red.len() + m.blue.len(), 6);
            let mut all: Vec<usize> = m.red.iter().chain(m.blue.iter()).copied().collect();
            all.sort(); all.dedup();
            assert_eq!(all.len(), 6, "a team twice in one match");
            for &t in &all { count[t] += 1; if !m.surrogates.contains(&t) { counted[t] += 1; } }
        }
        for t in 0..n { assert_eq!(counted[t], 8, "{n} teams: team {t} counted {}", counted[t]); assert!(count[t] <= 9); }
    }
}

#[test]
fn an_event_plays_through() {
    for seed in 1..20 {
        let e = play(32, seed);
        // eight alliances of three, nobody twice
        assert_eq!(e.selection.alliances.len(), 8);
        let mut all: Vec<usize> = e.selection.alliances.iter().flatten().copied().collect();
        assert_eq!(all.len(), 24);
        all.sort(); all.dedup();
        assert_eq!(all.len(), 24, "a team on two alliances");
        // every alliance finishes somewhere, one winner, one finalist
        let fin: Vec<usize> = (0..8).map(|a| e.finish(a).expect("a finish")).collect();
        assert_eq!(fin.iter().filter(|&&f| f == 1).count(), 1);
        assert_eq!(fin.iter().filter(|&&f| f == 2).count(), 1);
        assert_eq!(fin.iter().filter(|&&f| f == 7).count(), 2);
        // rankings sorted by ranking score
        for w in e.ranks.windows(2) { assert!(w[0].rs() >= w[1].rs()); }
        // district points: 4-22 for quals, the winning captain the most
        for t in &e.teams { let p = e.district_points(t.id); assert!((4.0..=22.0 + 16.0 + 30.0).contains(&p), "{p}"); }
    }
}

#[test]
fn stronger_teams_rank_higher() {
    // over many events, the strongest robots (f, Trained) average a better rank than the weakest (a, Rookie)
    let (mut strong, mut weak, mut ns, mut nw) = (0.0, 0.0, 0.0, 0.0);
    let r = ratings();
    for seed in 1..40 {
        let e = play(30, seed);
        for t in &e.teams {
            let s = t.strength(&r);
            if s > 230.0 { strong += e.rank_of(t.id) as f64; ns += 1.0; }
            if s < 80.0 { weak += e.rank_of(t.id) as f64; nw += 1.0; }
        }
    }
    assert!(strong / ns + 5.0 < weak / nw, "strong {} weak {}", strong / ns, weak / nw);
}

#[test]
fn declining_and_picking_a_captain() {
    let r = ratings();
    let mut e = Event::new("t", Level::District, teams(30, 7), 7);
    let q = e.quals.len();
    e.decide_quals_until(q, &r);
    e.start_selection();
    // alliance 1 invites alliance 2's captain: the captains below move up, the 9th seed captains 8
    let second = e.selection.alliances[1][0];
    let ninth = e.ranks[8].team;
    e.pick(second, true);
    assert_eq!(e.selection.alliances[0], vec![e.ranks[0].team, second]);
    assert_eq!(e.selection.alliances[7][0], ninth);
    // a team that declines can't be picked
    let t = e.available(1)[0];
    e.pick(t, false);
    assert!(!e.available(1).contains(&t));
}
