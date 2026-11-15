//! A season: a district of teams across four district events (you at two of them), the District
//! Championship, a Championship division, and Einstein. Who's where, what's decided and what you
//! play, who goes on, the rival, awards, and the season score. It saves as JSON after every match.
use crate::event::{Event, Level, Line};
use crate::model::{team_names, Ratings, Skill, Team};
use crate::rng::Rng;
use serde::{Deserialize, Serialize};

/// How hard the season is (on top of the robots and the drivers' skills, which get stronger as it
/// goes): it shifts the AI drivers' skills down or up a step
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Difficulty { Casual, Standard, HallOfFame }
impl Difficulty {
    pub fn name(self) -> &'static str { match self { Difficulty::Casual => "Casual", Difficulty::Standard => "Standard", Difficulty::HallOfFame => "Hall of Fame" } }
    fn shift(self) -> i32 { match self { Difficulty::Casual => -1, Difficulty::Standard => 0, Difficulty::HallOfFame => 1 } }
    /// the season score's multiplier
    pub fn mult(self) -> f64 { match self { Difficulty::Casual => 0.75, Difficulty::Standard => 1.0, Difficulty::HallOfFame => 1.5 } }
}

/// Where the season is
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Stage { District1, District2, DistrictChampionship, Division, Einstein, Over }

/// An award at an event
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Award { pub event: String, pub name: String, pub team: usize }

/// Something that happened, for the coach, the recap and the screens
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Beat {
    Match { event: String, id: String, won: Option<bool>, us: f64, them: f64, rp: f64 },
    Ranked { event: String, rank: usize, of: usize },
    Picked { event: String, alliance: usize, captain: bool },
    Finished { event: String, place: Option<usize> },
    Award(Award),
    Advanced { to: Stage },
    Eliminated { at: Stage },
}

/// The season
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Season {
    pub seed: u32,
    pub difficulty: Difficulty,
    pub your_name: String,
    pub your_robot: String,
    /// every team in the season (the district's, then the Championship's from elsewhere); you're 0
    pub teams: Vec<Team>,
    /// the team that keeps turning up: at both your district events, and wherever you go after
    pub rival: usize,
    /// which two district events (0-3) each district team attends
    pub district_events: Vec<[usize; 2]>,
    /// the events so far (the four districts, as each week comes; then the others)
    pub events: Vec<Event>,
    /// the event you're at now (an index into events)
    pub current: usize,
    pub stage: Stage,
    pub awards: Vec<Award>,
    pub beats: Vec<Beat>,
    pub rng: Rng,
}

pub const DISTRICT_TEAMS: usize = 48;
pub const DCMP_TEAMS: usize = 32;
pub const DIVISION_TEAMS: usize = 36;
/// how many go on from the District Championship to the Championship
pub const TO_CHAMPS: usize = 10;

/// The AI drivers' skills at a stage (before the difficulty's shift): a weighted mix
fn skill_mix(stage: Stage) -> [f64; 4] {
    match stage {
        Stage::District1 | Stage::District2 => [0.4, 0.45, 0.15, 0.0],
        Stage::DistrictChampionship => [0.05, 0.45, 0.4, 0.1],
        _ => [0.0, 0.1, 0.55, 0.35],
    }
}
const SKILLS: [Skill; 4] = [Skill::Rookie, Skill::Regional, Skill::Champs, Skill::Trained];
fn pick_skill(stage: Stage, d: Difficulty, rng: &mut Rng) -> Skill {
    let w = skill_mix(stage);
    let mut u = rng.next();
    let mut i = 3;
    for (k, x) in w.iter().enumerate() { if u < *x { i = k; break; } u -= x; }
    SKILLS[(i as i32 + d.shift()).clamp(0, 3) as usize]
}

impl Season {
    /// A new season: you (your team's name, the robot you drive all season) and a district of
    /// AI teams fielding the other designs (any of them, more of the strong ones the higher it goes)
    pub fn new(seed: u32, difficulty: Difficulty, your_name: &str, your_robot: &str, ratings: &Ratings) -> Self {
        let mut rng = Rng::new(seed);
        let names = team_names(DISTRICT_TEAMS + DIVISION_TEAMS, &mut rng);
        let robots = ratings.order();
        let mut teams = vec![Team { id: 0, name: your_name.into(), robot: your_robot.into(), skill: Skill::Champs, you: true }];
        for i in 1..DISTRICT_TEAMS {
            let skill = pick_skill(Stage::District1, difficulty, &mut rng);
            teams.push(Team { id: i, name: names[i].clone(), robot: robots[rng.below(robots.len())].clone(), skill, you: false });
        }
        // the rival: a strong design with a driver who's better than most here
        let rival = 1 + rng.below(DISTRICT_TEAMS - 1);
        teams[rival].robot = robots[rng.below(robots.len().min(5))].clone();
        teams[rival].skill = if difficulty == Difficulty::Casual { Skill::Regional } else { Skill::Champs };
        // two district events each: you at weeks 1 and 3, the rival with you; the rest dealt so
        // each event has 24
        let mut district_events = vec![[0usize, 2]; DISTRICT_TEAMS];
        let mut slots: Vec<usize> = Vec::new();
        for e in 0..4 { let taken = if e == 0 || e == 2 { 2 } else { 0 }; for _ in taken..DISTRICT_TEAMS / 2 { slots.push(e); } }
        let others: Vec<usize> = (1..DISTRICT_TEAMS).filter(|&t| t != rival).collect();
        // (deal pairs of different events: shuffle until no team gets the same one twice)
        loop {
            rng.shuffle(&mut slots);
            if others.iter().enumerate().all(|(k, _)| slots[2 * k] != slots[2 * k + 1]) { break; }
        }
        for (k, &t) in others.iter().enumerate() { let (a, b) = (slots[2 * k], slots[2 * k + 1]); district_events[t] = [a.min(b), a.max(b)]; }
        let mut s = Season { seed, difficulty, your_name: your_name.into(), your_robot: your_robot.into(), teams, rival, district_events, events: Vec::new(), current: 0, stage: Stage::District1, awards: Vec::new(), beats: Vec::new(), rng };
        // week 1: the first district event, yours
        s.open_district(0, ratings);
        s
    }

    fn district_name(i: usize) -> &'static str { ["Riverside District Event", "Lakeshore District Event", "Hillcrest District Event", "Northfield District Event"][i] }

    /// Make district event `i` (and decide it outright if you're not there)
    fn open_district(&mut self, i: usize, ratings: &Ratings) {
        let teams: Vec<Team> = (0..DISTRICT_TEAMS).filter(|&t| self.district_events[t].contains(&i)).map(|t| self.teams[t].clone()).collect();
        let seed = self.seed.wrapping_mul(31).wrapping_add(i as u32 + 1);
        let mut e = Event::new(Self::district_name(i), Level::District, teams, seed);
        if e.you().is_none() { decide_all(&mut e, ratings); self.give_awards(&e); }
        self.events.push(e);
        if self.events.last().unwrap().you().is_some() { self.current = self.events.len() - 1; }
    }

    /// Your event now
    pub fn event(&self) -> &Event { &self.events[self.current] }
    pub fn event_mut(&mut self) -> &mut Event { &mut self.events[self.current] }

    /// A team's district points so far (district events: the best two; the District Championship's
    /// count three times, as FRC counts them), awards included
    pub fn district_points(&self, t: usize) -> f64 {
        let mut pts = 0.0;
        for e in &self.events {
            if !e.teams.iter().any(|x| x.id == t) || !e.playoffs_done() { continue; }
            let k = match e.level { Level::District => 1.0, Level::DistrictChampionship => 3.0, _ => 0.0 };
            let award = self.awards.iter().filter(|a| a.event == e.name && a.team == t).count() as f64 * 5.0;
            pts += k * (e.district_points(t) + award);
        }
        pts
    }

    /// The district's standings: (team, points), best first
    pub fn standings(&self) -> Vec<(usize, f64)> {
        let mut v: Vec<(usize, f64)> = (0..DISTRICT_TEAMS).map(|t| (t, self.district_points(t))).collect();
        v.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap().then(a.0.cmp(&b.0)));
        v
    }

    /// Awards at an event, from what happened there: the highest-scoring alliance's best robot,
    /// the best AUTO, the cleanest play among the playoff alliances, and the winning captain's
    /// Excellence in Engineering (district points: 5 each)
    fn give_awards(&mut self, e: &Event) {
        let add = |name: &str, team: usize, aw: &mut Vec<Award>, beats: &mut Vec<Beat>| {
            let a = Award { event: e.name.clone(), name: name.into(), team };
            if team == 0 { beats.push(Beat::Award(a.clone())); }
            aw.push(a);
        };
        // highest score: the match, and the robot on that alliance with the most points per match
        let mut best = (f64::MIN, Vec::new());
        for m in e.quals.iter().chain(e.bracket.iter().flat_map(|b| b.games.iter())) {
            if let Some((r, b)) = m.result { if r.total() > best.0 { best = (r.total(), m.red.clone()); } if b.total() > best.0 { best = (b.total(), m.blue.clone()); } }
        }
        let per = |t: usize| e.ranks.iter().find(|r| r.team == t).map_or(0.0, |r| if r.played > 0 { r.score / r.played as f64 } else { 0.0 });
        if let Some(&t) = best.1.iter().max_by(|a, b| per(**a).partial_cmp(&per(**b)).unwrap()) { add("Highest Score", t, &mut self.awards, &mut self.beats); }
        let auto = |t: usize| e.ranks.iter().find(|r| r.team == t).map_or(0.0, |r| if r.played > 0 { r.auto / r.played as f64 } else { 0.0 });
        if let Some(r) = e.ranks.iter().max_by(|a, b| auto(a.team).partial_cmp(&auto(b.team)).unwrap()) { add("Excellence in AUTO", r.team, &mut self.awards, &mut self.beats); }
        // clean play: the playoff robot whose alliances gave away the fewest foul points
        let mut fouls: Vec<(usize, f64)> = e.selection.alliances.iter().flatten().map(|&t| {
            let given: f64 = e.quals.iter().filter(|m| m.has(t)).filter_map(|m| m.result.map(|(r, b)| if m.red.contains(&t) { b.fouls } else { r.fouls })).sum();
            (t, given)
        }).collect();
        fouls.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap().then(a.0.cmp(&b.0)));
        if let Some(&(t, _)) = fouls.first() { add("Clean Play", t, &mut self.awards, &mut self.beats); }
        if let Some(w) = e.bracket.last().unwrap().winner { add("Excellence in Engineering", e.selection.alliances[w][0], &mut self.awards, &mut self.beats); }
    }

    /// Once your event's playoffs are done: the next week (and anything decided before it), or the
    /// next level if you made it, or the end of the season
    pub fn advance(&mut self, ratings: &Ratings) {
        assert!(self.event().playoffs_done(), "your event isn't over");
        let e = self.event().clone();
        self.give_awards(&e);
        let place = e.alliance_of(0).and_then(|a| e.finish(a));
        self.beats.push(Beat::Finished { event: e.name.clone(), place });
        match self.stage {
            Stage::District1 => {
                // week 2 (decided), then week 3, yours
                self.open_district(1, ratings);
                self.open_district(2, ratings);
                self.stage = Stage::District2;
            }
            Stage::District2 => {
                self.open_district(3, ratings);
                let st = self.standings();
                let you = st.iter().position(|x| x.0 == 0).unwrap();
                if you >= DCMP_TEAMS { self.end(Stage::DistrictChampionship); return; }
                let teams: Vec<Team> = st.iter().take(DCMP_TEAMS).map(|&(t, _)| {
                    // (drivers get better through the season)
                    let mut x = self.teams[t].clone();
                    if !x.you && t != self.rival { x.skill = better(x.skill, pick_skill(Stage::DistrictChampionship, self.difficulty, &mut self.rng)); }
                    x
                }).collect();
                for t in &teams { self.teams[t.id] = t.clone(); }
                let seed = self.seed.wrapping_mul(31).wrapping_add(11);
                self.events.push(Event::new("District Championship", Level::DistrictChampionship, teams, seed));
                self.current = self.events.len() - 1;
                self.stage = Stage::DistrictChampionship;
                self.beats.push(Beat::Advanced { to: self.stage });
            }
            Stage::DistrictChampionship => {
                // the Championship: the district's top TO_CHAMPS (and the DCMP's winners) go on
                let st = self.standings();
                let winners: Vec<usize> = e.bracket.last().unwrap().winner.map(|w| e.selection.alliances[w].clone()).unwrap_or_default();
                let mut go: Vec<usize> = winners.clone();
                for &(t, _) in &st { if go.len() >= TO_CHAMPS.max(winners.len()) { break; } if !go.contains(&t) { go.push(t); } }
                if !go.contains(&0) { self.end(Stage::Division); return; }
                if !go.contains(&self.rival) { go.push(self.rival); }
                // the division: the district's and strong teams from elsewhere
                let robots = ratings.order();
                let mut teams: Vec<Team> = go.iter().map(|&t| self.teams[t].clone()).collect();
                let mut k = DISTRICT_TEAMS;
                while teams.len() < DIVISION_TEAMS {
                    if self.teams.len() <= k {
                        let names = team_names(DISTRICT_TEAMS + DIVISION_TEAMS, &mut Rng::new(self.seed));
                        let skill = pick_skill(Stage::Division, self.difficulty, &mut self.rng);
                        // (the stronger designs more often: one of the top half most of the time)
                        let pool = if self.rng.next() < 0.7 { robots.len().div_ceil(2) } else { robots.len() };
                        self.teams.push(Team { id: k, name: names[k].clone(), robot: robots[self.rng.below(pool)].clone(), skill, you: false });
                    }
                    teams.push(self.teams[k].clone());
                    k += 1;
                }
                for t in teams.iter_mut() { if !t.you && t.id != self.rival { t.skill = better(t.skill, pick_skill(Stage::Division, self.difficulty, &mut self.rng)); } }
                let seed = self.seed.wrapping_mul(31).wrapping_add(21);
                self.events.push(Event::new("Championship Division", Level::Division, teams, seed));
                self.current = self.events.len() - 1;
                self.stage = Stage::Division;
                self.beats.push(Beat::Advanced { to: self.stage });
            }
            Stage::Division => {
                if place != Some(1) { self.end(Stage::Einstein); return; }
                // Einstein: your alliance against three other division winners, a round robin and
                // the finals (an event of its own: the four alliances as its teams, seeded as they are)
                let mine = e.selection.alliances[e.alliance_of(0).unwrap()].clone();
                let mut teams: Vec<Team> = mine.iter().take(3).map(|&t| e.team(t).clone()).collect();
                let robots = ratings.order();
                let names = team_names(DISTRICT_TEAMS + DIVISION_TEAMS + 9, &mut Rng::new(self.seed ^ 0x5eed));
                for j in 0..9 {
                    let id = 1000 + j;
                    let skill = if self.rng.next() < 0.5 + 0.2 * self.difficulty.shift() as f64 { Skill::Trained } else { Skill::Champs };
                    let t = Team { id, name: names[DISTRICT_TEAMS + DIVISION_TEAMS + j].clone(), robot: robots[self.rng.below(robots.len().div_ceil(3))].clone(), skill, you: false };
                    self.teams.push(t.clone());
                    teams.push(t);
                }
                let seed = self.seed.wrapping_mul(31).wrapping_add(31);
                let mut ev = Event::new("Einstein", Level::Einstein, teams, seed);
                ev.einstein_setup();
                self.events.push(ev);
                self.current = self.events.len() - 1;
                self.stage = Stage::Einstein;
                self.beats.push(Beat::Advanced { to: self.stage });
            }
            Stage::Einstein | Stage::Over => { self.stage = Stage::Over; }
        }
    }

    fn end(&mut self, at: Stage) {
        self.beats.push(Beat::Eliminated { at });
        self.stage = Stage::Over;
    }

    /// The season score: each event's finish and awards (its district points; the Championship's
    /// and Einstein's counted as the District Championship's), times your robot's multiplier (the
    /// weaker its rating, the more) and the difficulty's
    pub fn score(&self, ratings: &Ratings) -> f64 {
        let mut pts = 0.0;
        for e in &self.events {
            if e.you().is_none() || !e.playoffs_done() { continue; }
            let k = match e.level { Level::District => 1.0, Level::DistrictChampionship => 3.0, Level::Division => 4.0, Level::Einstein => 6.0 };
            let award = self.awards.iter().filter(|a| a.event == e.name && a.team == 0).count() as f64 * 5.0;
            pts += k * (e.district_points(0) + award);
        }
        (pts * self.robot_mult(ratings) * self.difficulty.mult()).round()
    }

    /// The weaker your robot, the more its season counts: the best robot's OPR over yours (1 to 2)
    pub fn robot_mult(&self, ratings: &Ratings) -> f64 {
        let best = ratings.robots.iter().map(|r| r.opr).fold(f64::MIN, f64::max);
        let mine = ratings.get(&self.your_robot).opr.max(1.0);
        (best / mine).clamp(1.0, 2.0)
    }

    pub fn save(&self) -> String { serde_json::to_string(self).unwrap() }
    pub fn load(text: &str) -> Option<Self> { serde_json::from_str(text).ok() }

    /// Note a match you played (for the coach and the recap)
    pub fn note_match(&mut self, id: &str, red: bool, r: Line, b: Line, rp: f64) {
        let (us, them) = if red { (r.total(), b.total()) } else { (b.total(), r.total()) };
        let won = if us > them { Some(true) } else if us < them { Some(false) } else { None };
        self.beats.push(Beat::Match { event: self.event().name.clone(), id: id.into(), won, us, them, rp });
    }
}

/// the better of two skills
fn better(a: Skill, b: Skill) -> Skill { if b > a { b } else { a } }

/// Decide a whole event you're not at
pub fn decide_all(e: &mut Event, r: &Ratings) {
    let q = e.quals.len();
    e.decide_quals_until(q, r);
    e.start_selection();
    while e.picking().is_some() { let t = e.ai_pick(r); e.pick(t, true); }
    e.seed_bracket();
    while let Some(i) = e.next_bout() { e.decide_bout(i, r); }
}
