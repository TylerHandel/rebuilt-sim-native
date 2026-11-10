//! js/game.js: one match (its world, FUEL, robots, rules and HUMAN PLAYERS), stepped at the
//! physics rate, and tools/headless.mjs's createWorld. Robots arrive with M2c.
use crate::constants::*;
use crate::fuel::Fuel;
use crate::game_match::{Match, Phase, ALLIANCES};
use crate::human_player::{HpControls, HumanPlayer};
use crate::rng::Mulberry32;
use crate::robot::{Robot, UnderArm};
use crate::world::WorldData;
use rapier3d::prelude::*;

/// What a match is set up with (the parts of the JS game's settings the sim reads so far)
#[derive(Debug, Clone)]
pub struct Settings {
    pub seed: u32,
    /// the HUMAN PLAYERS throw at their HUBS while active (js hps[a].auto)
    pub human_players: bool,
    /// FUEL preloaded in each robot (js settings.preload, 8 in a match)
    pub preload: usize,
    /// the robots, made in this order (blue's first, as createGame makes them): robots.json key,
    /// alliance, and where each starts (x, z, yaw)
    pub robots: Vec<(String, Alliance, [f64; 3])>,
    /// the robots aim from their true poses, not their pose estimates (js AIMTRUTH=1)
    pub aim_truth: bool,
}

pub struct World {
    pub physics: PhysicsWorld,
    pub fixed: RigidBodyHandle,
    pub data: WorldData,
    pub fuel: Fuel,
    pub match_: Match,
    pub hps: [HumanPlayer; 2],
    pub rng: Mulberry32,
    pub robots: Vec<Robot>,
    /// js physics.underArm: the colliders the TRENCH arm rule watches
    pub under: Vec<UnderArm>,
    pub steps: u64,
    /// what the driver tells their HUMAN PLAYER this frame (js game.hpControls), cleared by frame
    pub hp_controls: HpControls,
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
            rng: Mulberry32::new(s.seed), robots: Vec::new(), under: Vec::new(), steps: 0, hp_controls: HpControls::default(),
        };
        let pre = w.fuel.stage(&mut w.physics, s.preload * s.robots.len(), &mut w.rng);
        let all: serde_json::Value = serde_json::from_str(include_str!("../../../assets/robots.json")).unwrap();
        for (k, (key, alliance, [x, z, yaw])) in s.robots.iter().enumerate() {
            let cfg = all["robots"][key.as_str()].clone();
            assert!(!cfg.is_null(), "no robot {key}");
            let mut r = Robot::new(key, cfg, *alliance, &mut w.physics, &mut w.under);
            r.spawn(&mut w.physics, *x, *z, *yaw);
            r.load_fuel(&mut w.physics, &mut w.fuel, &pre[k * s.preload..(k + 1) * s.preload]);
            r.loc.truth = s.aim_truth;
            w.robots.push(r);
        }
        // (js createGame: each robot's pose estimate, its own seeded draws, by its slot)
        let seed = s.seed.to_string();
        for (i, r) in w.robots.iter_mut().enumerate() {
            let view = crate::localize::RobotView { pos: r.pos, quat: r.quat, yaw: r.yaw, vel: r.vel, omega: r.omega, swerve: &r.swerve, ready_shoot: r.ready && r.want_shoot, others: &[] };
            let (key, a) = (r.key.clone(), r.alliance);
            r.loc.init(&seed, i, &key, a, &view);
        }
        for hp in &mut w.hps { hp.auto = s.human_players; }
        w
    }

    pub fn dt(&self) -> f64 { PHYSICS_DT }

    /// js `match._go(phase)` with the robots enabled to match (a script's jump to a phase)
    pub fn jump_to(&mut self, phase: Phase) {
        self.match_.go(phase);
        let on = self.match_.robot_enabled();
        for r in &mut self.robots { r.enabled = on; }
    }

    /// js `stepGame(game, world, dt, probe)`: one physics step
    pub fn step(&mut self, probe: Probe) {
        let dt = PHYSICS_DT;
        let climbs: Vec<_> = self.robots.iter().map(|r| (r.alliance, 0u8)).collect();
        self.match_.update(dt, &climbs, &mut self.rng);
        let on = self.match_.robot_enabled();
        for r in &mut self.robots { r.enabled = on; }
        probe(self, "match");
        probe(self, "drivers");
        let teleop = self.match_.is_teleop();
        for r in &mut self.robots { r.pre_step(&mut self.physics, &mut self.fuel, dt, teleop); }
        probe(self, "robotsPre");
        self.fuel.pre_step(&mut self.physics, dt);
        probe(self, "fuelPre");
        if let Some(arms) = &self.data.trench_arms { crate::robot::under_arms(&mut self.physics, &mut self.under, arms); }
        self.physics.step();
        probe(self, "physics");
        let t = self.match_.t;
        self.fuel.post_step(&mut self.physics, dt, t);
        probe(self, "fuelPost");
        for i in 0..self.robots.len() {
            // (the others as they are: those before it already moved on to this step's poses)
            let others: Vec<[f64; 4]> = self.robots.iter().enumerate().filter(|(j, _)| *j != i).map(|(_, o)| [o.pos[0], o.pos[2], o.half_l, o.half_w]).collect();
            self.robots[i].post_step(&mut self.physics, &mut self.fuel, dt, t, &others);
        }
        probe(self, "robotsPost");
        assert!(self.robots.len() < 2, "the rules between robots arrive with M4");
        probe(self, "rules");
        self.steps += 1;
    }

    /// js `frameGame(game, dt)`: per drawn frame (the headless game runs it every second step)
    pub fn frame(&mut self, dt: f64) {
        let t = self.match_.t;
        self.fuel.update(&mut self.physics, dt, t, &mut self.match_, &mut self.rng);
        for a in ALLIANCES {
            let i = crate::game_match::ai(a);
            let c = if a == self.match_.player { self.hp_controls } else { HpControls::default() };
            self.hps[i].update(dt, t, c, &mut self.fuel, &mut self.physics, &self.match_, &mut self.rng);
        }
        self.hp_controls = HpControls::default();
    }
}
