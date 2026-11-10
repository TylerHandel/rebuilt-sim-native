//! js/game.js: one match (its world, FUEL, robots, rules and HUMAN PLAYERS), stepped at the
//! physics rate, and tools/headless.mjs's createWorld. Robots arrive with M2c.
use crate::constants::*;
use crate::fuel::Fuel;
use crate::game_match::{Match, Phase, ALLIANCES};
use crate::human_player::{HpControls, HumanPlayer};
use crate::rng::Mulberry32;
use crate::world::WorldData;
use rapier3d::prelude::*;

/// What a match is set up with (the parts of the JS game's settings the sim reads so far)
#[derive(Debug, Clone)]
pub struct Settings {
    pub seed: u32,
    /// the HUMAN PLAYERS throw at their HUBS while active (js hps[a].auto)
    pub human_players: bool,
    /// FUEL preloaded in robots, in all
    pub preload: usize,
}

pub struct World {
    pub physics: PhysicsWorld,
    pub fixed: RigidBodyHandle,
    pub data: WorldData,
    pub fuel: Fuel,
    pub match_: Match,
    pub hps: [HumanPlayer; 2],
    pub rng: Mulberry32,
    pub steps: u64,
}

/// The parts of a step, as the JS game's stepGame names them (its probe): see parity.rs PHASES
pub type Probe<'a> = &'a mut dyn FnMut(&World, &'static str);

impl World {
    /// js `createWorld()` and `createGame(world, settings)`: the field, every FUEL, staged
    pub fn new(data: WorldData, s: &Settings) -> Self {
        let (mut physics, fixed) = crate::field_physics(&data);
        let outposts = data.outposts.clone().expect("world.json without outposts: re-run tools/sync-assets.mjs");
        let fuel = Fuel::new(&mut physics, outposts);
        let mut w = World {
            physics, fixed, data, fuel, match_: Match::new(Alliance::Blue),
            hps: [HumanPlayer::new(Alliance::Blue), HumanPlayer::new(Alliance::Red)],
            rng: Mulberry32::new(s.seed), steps: 0,
        };
        w.fuel.stage(&mut w.physics, s.preload, &mut w.rng);
        for hp in &mut w.hps { hp.auto = s.human_players; }
        w
    }

    pub fn dt(&self) -> f64 { PHYSICS_DT }

    /// js `match._go(phase)` with the robots enabled to match (a script's jump to a phase)
    pub fn jump_to(&mut self, phase: Phase) { self.match_.go(phase); }

    /// js `stepGame(game, world, dt, probe)`: one physics step
    pub fn step(&mut self, probe: Probe) {
        let dt = PHYSICS_DT;
        self.match_.update(dt, &[], &mut self.rng);
        probe(self, "match");
        probe(self, "drivers");
        probe(self, "robotsPre");
        self.fuel.pre_step(&mut self.physics, dt);
        probe(self, "fuelPre");
        self.physics.step();
        probe(self, "physics");
        let t = self.match_.t;
        self.fuel.post_step(&mut self.physics, dt, t);
        probe(self, "fuelPost");
        probe(self, "robotsPost");
        probe(self, "rules");
        self.steps += 1;
    }

    /// js `frameGame(game, dt)`: per drawn frame (the headless game runs it every second step)
    pub fn frame(&mut self, dt: f64) {
        let t = self.match_.t;
        self.fuel.update(&mut self.physics, dt, t, &mut self.match_, &mut self.rng);
        for a in ALLIANCES {
            let i = crate::game_match::ai(a);
            self.hps[i].update(dt, t, HpControls::default(), &mut self.fuel, &mut self.physics, &self.match_, &mut self.rng);
        }
    }
}
