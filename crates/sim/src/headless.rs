//! A whole match played headless, every robot driven by its AI (tools/headless.mjs's playMatch):
//! for the robot ratings (examples/ratings.rs) and anything else that wants a result, not a picture.
use crate::constants::*;
use crate::game::{Settings, UnitSpec, World};
use crate::game_match::{ai, Score};

/// One robot in the match: its robots.json key, its alliance, and who drives it
#[derive(Debug, Clone)]
pub struct Entry { pub robot: String, pub alliance: Alliance, pub unit: UnitSpec }

impl Entry {
    /// An AI at `skill` playing `strategy`, its AUTO the one it asks for (js opponentAuto)
    pub fn ai(robot: &str, alliance: Alliance, strategy: &str, skill: &str) -> Self {
        let (auto, start) = crate::ai::opponent_auto(strategy);
        Entry { robot: robot.into(), alliance, unit: UnitSpec { driver: strategy.into(), auto: auto.into(), start: start.into(), skill: skill.into() } }
    }
}

/// What one robot did
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct RobotResult { pub robot: String, pub blue: bool, pub shots: u32, pub intaked: u32, pub passes: u32, pub climb_level: u8 }

/// How a match went
#[derive(Debug, Clone)]
pub struct MatchResult {
    pub score: [Score; 2],
    /// each alliance's total points (FUEL, TOWER, and the fouls the other committed)
    pub totals: [f64; 2],
    pub robots: Vec<RobotResult>,
    /// simulated seconds, and wall-clock seconds it took
    pub sim_s: f64,
    pub wall_s: f64,
}

impl MatchResult {
    pub fn total(&self, a: Alliance) -> f64 { self.totals[ai(a)] }
    /// points from FUEL (AUTO and TELEOP) and the TOWER, without fouls
    pub fn earned(&self, a: Alliance) -> f64 { let s = &self.score[ai(a)]; s.auto_fuel + s.tele_fuel + s.auto_tower + s.tele_tower }
}

/// Play a match from GET READY to the end. `human_players`: both HUMAN PLAYERS throw at their HUBS
/// while active, as in a match. `preload`: FUEL each robot starts with (8 in a match).
pub fn play(entries: &[Entry], seed: u32, human_players: bool, preload: usize) -> MatchResult {
    let t0 = std::time::Instant::now();
    let robots = entries.iter().map(|e| (e.robot.clone(), e.alliance, [f64::NAN; 3])).collect();
    let units = entries.iter().map(|e| e.unit.clone()).collect();
    let mut w = World::new(crate::WorldData::bundled(), &Settings { seed, human_players, preload, robots, aim_truth: false, units });
    // (a match is 2:40 and a few seconds either side; never longer than this)
    let cap = ((TIMING::preMatch + TIMING::auto + TIMING::autoGap + TIMING::teleop + TIMING::post + 5.0) / PHYSICS_DT) as u64;
    while !w.match_.over() && w.steps < cap {
        w.step(&mut |_, _| {});
        if w.steps % 2 == 0 { w.frame(2.0 * PHYSICS_DT); }
        // (nobody reads them: keep them from piling up)
        w.match_.events.clear();
    }
    let robots = w.robots.iter().map(|r| RobotResult { robot: r.key.clone(), blue: r.alliance == Alliance::Blue, shots: r.stats.shots, intaked: r.stats.intaked, passes: r.stats.passes, climb_level: r.climb_level }).collect();
    MatchResult {
        score: w.match_.score.clone(),
        totals: [w.match_.total(Alliance::Blue), w.match_.total(Alliance::Red)],
        robots, sim_s: w.match_.t, wall_s: t0.elapsed().as_secs_f64(),
    }
}
