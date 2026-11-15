//! The teams at the events: their robots' ratings (assets/ratings.json, examples/ratings.rs), their
//! drivers' skills, and what a match between them is likely to score.
use crate::rng::Rng;
use serde::{Deserialize, Serialize};

/// A robot's rating, from the headless matches its design played (the Champs AI driving)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotRating {
    pub key: String,
    /// its OPR in 3 v 3 (points it adds to its alliance's score, fouls left out), and that split
    /// into AUTO (FUEL and TOWER), FUEL (both periods) and TOWER points
    pub opr: f64,
    pub opr_auto: f64,
    pub opr_fuel: f64,
    pub opr_tower: f64,
    /// its 1 v 1 strength on the Elo scale (1500: an even robot)
    pub elo: f64,
}

/// Every robot's rating, and how far an alliance's score falls from its robots' OPRs (the luck)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Ratings { pub robots: Vec<RobotRating>, pub residual_sd: f64 }

impl Ratings {
    /// assets/ratings.json
    pub fn from_json(text: &str) -> Self {
        let v: serde_json::Value = serde_json::from_str(text).expect("ratings.json");
        let robots = v["order"].as_array().unwrap().iter().map(|k| {
            let k = k.as_str().unwrap();
            let r = &v["robots"][k];
            let f = |n: &str| r[n].as_f64().unwrap_or(0.0);
            RobotRating { key: k.into(), opr: f("opr"), opr_auto: f("oprAuto"), opr_fuel: f("oprFuel"), opr_tower: f("oprTower"), elo: f("elo") }
        }).collect();
        Ratings { robots, residual_sd: v["residualSd3v3"].as_f64().unwrap_or(60.0) }
    }
    pub fn get(&self, key: &str) -> &RobotRating { self.robots.iter().find(|r| r.key == key).unwrap_or_else(|| panic!("no rating for {key}")) }
    /// the robots from strongest to weakest (by OPR)
    pub fn order(&self) -> Vec<String> {
        let mut v: Vec<&RobotRating> = self.robots.iter().collect();
        v.sort_by(|a, b| b.opr.partial_cmp(&a.opr).unwrap());
        v.into_iter().map(|r| r.key.clone()).collect()
    }
}

/// A driver's skill (the game's AI skills; the ratings are the Champs AI's)
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Hash, PartialOrd, Ord)]
pub enum Skill { Rookie, Regional, Champs, Trained }

impl Skill {
    pub fn key(self) -> &'static str { match self { Skill::Rookie => "rookie", Skill::Regional => "regional", Skill::Champs => "champs", Skill::Trained => "trained" } }
    pub fn name(self) -> &'static str { match self { Skill::Rookie => "Rookie", Skill::Regional => "Regional", Skill::Champs => "Champs", Skill::Trained => "Trained" } }
    /// how much of its robot's rating a driver of this skill gets out of it (FUEL; its TOWER
    /// points come with the climb, a little less often for the weaker drivers). Estimates until
    /// the ratings are played at every skill.
    pub fn factor(self) -> f64 { match self { Skill::Rookie => 0.55, Skill::Regional => 0.8, Skill::Champs => 1.0, Skill::Trained => 1.03 } }
    pub fn tower_factor(self) -> f64 { match self { Skill::Rookie => 0.6, Skill::Regional => 0.85, _ => 1.0 } }
}

/// A team at an event: its name, the robot design it fields, its driver, and whether it's you
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Team {
    /// its place in the season's list of teams (the same at every event)
    pub id: usize,
    pub name: String,
    pub robot: String,
    pub skill: Skill,
    pub you: bool,
}

/// What a team is expected to add to its alliance in a match (points): FUEL in AUTO and TELEOP,
/// TOWER points, and the AUTO part of those
#[derive(Clone, Copy, Debug, Default)]
pub struct Expect { pub fuel: f64, pub tower: f64, pub auto: f64 }

impl Team {
    pub fn expect(&self, r: &Ratings) -> Expect {
        let g = r.get(&self.robot);
        let k = self.skill.factor();
        Expect { fuel: g.opr_fuel.max(0.0) * k, tower: g.opr_tower.max(0.0) * self.skill.tower_factor(), auto: g.opr_auto.max(0.0) * k }
    }
    /// one number for how strong it is (its expected points), for picking and seeding
    pub fn strength(&self, r: &Ratings) -> f64 { let e = self.expect(r); e.fuel + e.tower }
}

/// The fictional teams (no real team's name or number: the robot designs are the real teams',
/// credited as such; the teams fielding them here are made up)
const ADJ: [&str; 24] = ["Iron", "Copper", "Golden", "Silver", "Rusty", "Electric", "Quantum", "Thunder", "Crimson", "Cobalt", "Velvet", "Atomic",
    "Rapid", "Midnight", "Solar", "Lunar", "Granite", "Neon", "Turbo", "Hyper", "Static", "Prairie", "Harbor", "Summit"];
const NOUN: [&str; 24] = ["Otters", "Foxes", "Falcons", "Gearheads", "Robins", "Coyotes", "Owls", "Pistons", "Lynxes", "Herons", "Badgers", "Ravens",
    "Sprockets", "Comets", "Bison", "Wolves", "Hornets", "Marmots", "Pelicans", "Rams", "Voltage", "Beavers", "Kestrels", "Moose"];

/// `n` distinct team names from the seed
pub fn team_names(n: usize, rng: &mut Rng) -> Vec<String> {
    let mut all: Vec<String> = ADJ.iter().flat_map(|a| NOUN.iter().map(move |b| format!("{a} {b}"))).collect();
    rng.shuffle(&mut all);
    all.truncate(n);
    all
}
