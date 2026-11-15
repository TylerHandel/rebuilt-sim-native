//! Road to Einstein in the game: the season (crates/campaign), saved beside the settings after every
//! match, what happens between your matches (decided as the pits screen asks: ui/campaign.rs), and
//! your matches set up and played in the game, their results recorded.
use bevy::prelude::*;
use campaign::season::{Difficulty, Season, Stage};
use campaign::{Line, Ratings};
use sim::constants::Alliance;
use sim::game::{UnitSpec, World};
use sim::game_match::ai;

use crate::game::MatchSetup;

/// The robots' ratings (examples/ratings.rs)
pub fn ratings() -> &'static Ratings {
    static R: std::sync::OnceLock<Ratings> = std::sync::OnceLock::new();
    R.get_or_init(|| Ratings::from_json(include_str!("../../../assets/ratings.json")))
}

fn save_path() -> Option<std::path::PathBuf> { crate::settings::path().map(|p| p.with_file_name("season.json")) }

/// whether there's a saved season (without reading it)
pub fn has_save() -> bool { save_path().is_some_and(|p| p.exists()) }

/// The saved season, if there is one
pub fn saved() -> Option<Season> { save_path().and_then(|p| std::fs::read_to_string(p).ok()).and_then(|t| Season::load(&t)) }

/// What you're about to do at your event (or doing)
#[derive(Clone, Debug, PartialEq)]
pub enum Next {
    /// your next qualification match (its index)
    Qual(usize),
    /// you're a captain: your pick
    YouPick,
    /// an alliance captain (that alliance's index) invites you
    Invite(usize),
    /// your next playoff match (its bout)
    Bout(usize),
    /// the event's over: where you finished
    EventOver,
    /// the season's over
    Recap,
}

/// The season being played, and the match of it being played
#[derive(Resource)]
pub struct Campaign {
    pub season: Season,
    /// the match you're in (a qualification match's index, or a bout's) while it's played
    pub playing: Option<Next>,
}

/// An AI team's strategy: most score; some, by their number, defend or play hybrid
pub fn strategy(id: usize, skill: campaign::Skill) -> &'static str {
    if skill == campaign::Skill::Rookie { return "scorer"; }
    match id % 7 { 3 => "defense", 5 => "hybrid", _ => "scorer" }
}

impl Campaign {
    pub fn new(seed: u32, d: Difficulty, name: &str, robot: &str) -> Self {
        Campaign { season: Season::new(seed, d, name, robot, ratings()), playing: None }
    }

    pub fn save(&self) {
        let Some(p) = save_path() else { return };
        if let Some(dir) = p.parent() { let _ = std::fs::create_dir_all(dir); }
        let _ = std::fs::write(p, self.season.save());
    }

    /// Run the event on until you're needed: the matches before yours decided, the AI captains'
    /// picks made, the playoff matches without you decided
    pub fn next(&mut self) -> Next {
        let r = ratings();
        let s = &mut self.season;
        if s.stage == Stage::Over { return Next::Recap; }
        let e = s.event_mut();
        if !e.quals_done() {
            if let Some(i) = e.next_for(0) { e.decide_quals_until(i, r); return Next::Qual(i); }
            let q = e.quals.len();
            e.decide_quals_until(q, r);
        }
        if !e.quals.is_empty() && e.selection.alliances.is_empty() { e.start_selection(); }
        while let Some(a) = e.picking() {
            if e.selection.alliances[a][0] == 0 { return Next::YouPick; }
            let t = e.ai_pick(r);
            if t == 0 { return Next::Invite(a); }
            e.pick(t, true);
        }
        e.seed_bracket();
        while let Some(i) = e.next_bout() {
            let mine = e.alliance_of(0);
            let b = &e.bracket[i];
            if mine.is_some() && (b.red == mine || b.blue == mine) { return Next::Bout(i); }
            e.decide_bout(i, r);
        }
        Next::EventOver
    }

    /// The two alliances of a match you're in: (red, blue) team ids
    pub fn sides(&self, n: &Next) -> (Vec<usize>, Vec<usize>) {
        let e = self.season.event();
        match *n {
            Next::Qual(i) => (e.quals[i].red.clone(), e.quals[i].blue.clone()),
            Next::Bout(i) => {
                let b = &e.bracket[i];
                let al = &e.selection.alliances;
                (al[b.red.unwrap()].iter().take(3).copied().collect(), al[b.blue.unwrap()].iter().take(3).copied().collect())
            }
            _ => (Vec::new(), Vec::new()),
        }
    }

    /// Your match as the game plays it: you on your side, the AI teams with their robots, skills
    /// and strategies
    pub fn setup(&self, n: &Next, s: &crate::settings::Settings) -> MatchSetup {
        let (red, blue) = self.sides(n);
        let e = self.season.event();
        let mut entries = Vec::new();
        let mut you = None;
        let mut alliance = Alliance::Red;
        for (ids, a) in [(&red, Alliance::Red), (&blue, Alliance::Blue)] {
            for &t in ids {
                let tm = e.team(t);
                if tm.you {
                    you = Some(entries.len());
                    alliance = a;
                    let driver = if s.auto == "best" { "you" } else { "external" };
                    entries.push((tm.robot.clone(), a, "hub".to_string(), UnitSpec { driver: driver.into(), auto: "best".into(), start: "hub".into(), skill: "champs".into() }));
                } else {
                    let st = strategy(t, tm.skill);
                    let (auto, start) = sim::ai::opponent_auto(st);
                    entries.push((tm.robot.clone(), a, start.into(), UnitSpec { driver: st.into(), auto: auto.into(), start: start.into(), skill: tm.skill.key().into() }));
                }
            }
        }
        MatchSetup { entries, you, alliance, skip_auto: s.auto == "skip", still_auto: s.auto == "none", hp_auto: s.hp == "auto" }
    }

    /// The match you just played: its result into the event and the season, saved
    pub fn record(&mut self, w: &World) {
        let Some(n) = self.playing.take() else { return };
        let line = |a: Alliance| {
            let s = &w.match_.score[ai(a)];
            Line { fuel: s.auto_fuel + s.tele_fuel, auto: s.auto_fuel + s.auto_tower, tower: s.auto_tower + s.tele_tower, fouls: s.foul_pts }
        };
        let (red, blue) = (line(Alliance::Red), line(Alliance::Blue));
        let (r_ids, _) = self.sides(&n);
        let you_red = r_ids.contains(&0);
        let e = self.season.event_mut();
        let (id, rp_before) = match n {
            Next::Qual(i) => (e.quals[i].id.clone(), e.ranks.iter().find(|x| x.team == 0).map_or(0.0, |x| x.rp)),
            Next::Bout(i) => (e.bracket[i].id.clone(), 0.0),
            _ => return,
        };
        match n {
            Next::Qual(i) => e.record_qual(i, red, blue),
            Next::Bout(i) => e.record_bout(i, red, blue),
            _ => {}
        }
        let rp = e.ranks.iter().find(|x| x.team == 0).map_or(0.0, |x| x.rp) - rp_before;
        self.season.note_match(&id, you_red, red, blue, rp);
        self.save();
    }
}
