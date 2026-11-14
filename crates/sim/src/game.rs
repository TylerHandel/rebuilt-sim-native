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
    /// who drives each robot (js createGame's entries: its driver, AUTO, start and skill), in the
    /// robots' order; none: every robot driven from outside (robot.cmd set before each step)
    pub units: Vec<UnitSpec>,
}

/// One robot's driver as a match is set up with it (js matchEntries' driver, auto, start, skill)
#[derive(Debug, Clone)]
pub struct UnitSpec {
    /// "external" (its command set from outside, AUTO too), "you" (a person: its AUTO routine runs in
    /// AUTO, then their command), or an AI's strategy ("scorer", "defense", "hybrid")
    pub driver: String,
    /// its AUTO routine ("best", "sweep", "defendStage", ...) and the start it asks for
    pub auto: String,
    pub start: String,
    /// an AI's skill ("rookie", "regional", "champs", "trained")
    pub skill: String,
}
impl UnitSpec {
    pub fn external() -> Self { UnitSpec { driver: "external".into(), auto: "best".into(), start: "hub".into(), skill: "champs".into() } }
}

/// One robot's drivers in the match (js game.units: its AUTO runner and its AI)
#[derive(Clone)]
pub struct Unit {
    pub auto: Option<crate::auto::AutoRunner>,
    pub ai: Option<crate::ai::OpponentAI>,
    /// driven from outside all match long
    pub external: bool,
    /// a person drives it: its AUTO routine in AUTO, then robot.cmd as they set it
    pub human: bool,
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
    /// js game.rules: the rules between robots
    pub rules: crate::rules::RobotRules,
    /// js game.units: who drives each robot
    pub units: Vec<Unit>,
    /// js game.you: the robot a person drives (the camera's and the HUD's), if any
    pub you: Option<usize>,
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
            rng: Mulberry32::new(s.seed), robots: Vec::new(), under: Vec::new(), steps: 0, hp_controls: HpControls::default(), rules: Default::default(), units: Vec::new(), you: None,
        };
        let pre_balls = w.fuel.stage(&mut w.physics, s.preload * s.robots.len(), &mut w.rng);
        let all = crate::auto::data();
        // (js createGame: the starting spots, alliance by alliance, in driver station order)
        let specs: Vec<UnitSpec> = if s.units.is_empty() { vec![UnitSpec::external(); s.robots.len()] } else { s.units.clone() };
        assert_eq!(specs.len(), s.robots.len(), "a unit for each robot");
        let mut assigned: Vec<Option<crate::auto::Assigned>> = vec![None; s.robots.len()];
        for a in ALLIANCES {
            let idx: Vec<usize> = (0..s.robots.len()).filter(|&k| s.robots[k].1 == a).collect();
            let entries: Vec<crate::auto::Entry> = idx.iter().map(|&k| crate::auto::Entry { robot: s.robots[k].0.clone(), auto: specs[k].auto.clone(), start: specs[k].start.clone(), plan: None }).collect();
            for (j, st) in crate::auto::assign_starts(&entries).into_iter().enumerate() { assigned[idx[j]] = Some(st); }
        }
        let mut autos = Vec::new();
        for (k, (key, alliance, [x, z, yaw])) in s.robots.iter().enumerate() {
            let mut cfg = all["robots"][key.as_str()].clone();
            // (a generic robot's hopper as the JS measured it: robots.json geoCap)
            if let Some(g) = all["geoCap"].get(key.as_str()) { cfg["geoCap"] = g.clone(); }
            assert!(!cfg.is_null(), "no robot {key}");
            let mut r = Robot::new(key, cfg, *alliance, &mut w.physics, &mut w.under);
            let st = assigned[k].clone().unwrap();
            // (the team's own AUTO, from its side: js/teamAutos.js)
            let custom = st.team.map(|t| (t, false));
            // (a pose of NaN: where its AUTO starts, js startPose)
            let [x, z, yaw] = if x.is_nan() { crate::auto::start_pose(&st.start, &r, *alliance, custom) } else { [*x, *z, *yaw] };
            r.spawn(&mut w.physics, x, z, yaw);
            autos.push(crate::auto::AutoRunner::new(&r, &specs[k].auto, &st.start, *alliance, custom, st.plan.clone()));
            // (a generic robot drops its preload into its hopper a little at random: from a stream of
            // its own, as native-trace.mjs has it, not the match's)
            let mut pre = Mulberry32::new(s.seed ^ 0x2545F491 ^ k as u32);
            r.load_fuel(&mut w.physics, &mut w.fuel, &pre_balls[k * s.preload..(k + 1) * s.preload], &mut pre);
            r.loc.truth = s.aim_truth;
            w.robots.push(r);
        }
        // (js createGame: each robot's pose estimate, its own seeded draws, by its slot)
        let seed = s.seed.to_string();
        for (i, r) in w.robots.iter_mut().enumerate() {
            let view = crate::localize::RobotView { pos: r.pos, quat: r.quat, yaw: r.yaw, vel: r.vel, omega: r.omega, swerve: &r.swerve, ready_shoot: r.ready && r.want_shoot, others: &[], turrets: &r.turrets, turret_yaws: &r.turret_yaws };
            let (key, a) = (r.key.clone(), r.alliance);
            r.loc.init(&seed, i, &key, a, &view);
        }
        // (js createGame: the AIs, once every robot's made)
        let n = w.robots.len();
        for (k, auto) in autos.into_iter().enumerate() {
            let d = specs[k].driver.as_str();
            let (external, human) = (d == "external", d == "you");
            let ai = if external || human { None } else {
                let a = w.robots[k].alliance;
                let foes: Vec<usize> = (0..n).filter(|&j| w.robots[j].alliance != a).collect();
                let mates: Vec<usize> = (0..n).filter(|&j| j != k && w.robots[j].alliance == a).collect();
                Some(crate::ai::OpponentAI::new(k, &mut w.robots, foes, mates, d, &specs[k].skill))
            };
            if (human || external) && w.you.is_none() && d != "external" { w.you = Some(k); }
            w.units.push(Unit { auto: Some(auto), ai, external, human });
        }
        for hp in &mut w.hps { hp.auto = s.human_players; }
        w
    }

    pub fn dt(&self) -> f64 { PHYSICS_DT }

    /// The robot a person drives (none: watching the AIs)
    pub fn mine(&self) -> Option<&Robot> { self.you.and_then(|i| self.robots.get(i)) }
    pub fn mine_mut(&mut self) -> Option<&mut Robot> { self.you.and_then(|i| self.robots.get_mut(i)) }

    /// js `match._go(phase)` with the robots enabled to match (a script's jump to a phase)
    pub fn jump_to(&mut self, phase: Phase) {
        self.match_.go(phase);
        let on = self.match_.robot_enabled();
        for r in &mut self.robots { r.enabled = on; }
    }

    /// js `stepGame(game, world, dt, probe)`: one physics step
    pub fn step(&mut self, probe: Probe) {
        let dt = PHYSICS_DT;
        let climbs: Vec<_> = self.robots.iter().map(|r| (r.alliance, r.climb_level)).collect();
        let prev = self.match_.phase;
        self.match_.update(dt, &climbs, &mut self.rng);
        let on = self.match_.robot_enabled();
        let (auto, pt) = (self.match_.is_auto(), self.match_.phase_time);
        let starts = prev == Phase::Pre && self.match_.phase == Phase::Auto;
        for r in &mut self.robots {
            // (a latched intake comes down as AUTO starts)
            if starts && r.cfg["intake"]["latched"].as_bool() == Some(true) { r.force_deploy = true; }
            if r.force_deploy && r.intake_deploy >= 1.0 { r.force_deploy = false; }
            r.enabled = on; r.is_auto = auto; r.phase_time = pt;
        }
        probe(self, "match");
        self.drive_units(dt);
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
            self.robots[i].post_step(&mut self.physics, &mut self.fuel, dt, t, &others, &mut self.rng);
            // (its fouls: a generic robot's intake catching FUEL fresh out of a HUB, G408)
            let a = self.robots[i].alliance;
            for (rule, desc) in std::mem::take(&mut self.robots[i].fouls) { self.match_.add_foul(a, false, rule, desc); }
        }
        probe(self, "robotsPost");
        if self.robots.len() > 1 {
            let views: Vec<_> = self.robots.iter().map(|r| r.rules_view()).collect();
            self.rules.update(dt, &views, &mut self.match_);
        }
        probe(self, "rules");
        self.steps += 1;
    }

    /// js stepGame's drivers: each robot's AUTO runner in AUTO, its AI otherwise
    fn drive_units(&mut self, dt: f64) {
        let auto = self.match_.is_auto();
        let teleop = self.match_.is_teleop();
        let views: Vec<_> = self.robots.iter().map(|r| r.rules_view()).collect();
        for i in 0..self.units.len() {
            if self.units[i].external { continue; }
            if auto {
                let foes: Vec<_> = views.iter().filter(|v| v.alliance != views[i].alliance).copied().collect();
                let ctx = crate::auto::Ctx { foes: &foes, others: &foes, fuel: &self.fuel };
                if let Some(a) = &mut self.units[i].auto { a.update(&mut self.robots[i], &ctx, dt); }
                if let Some(ai) = &mut self.units[i].ai { ai.label = "AUTO routine".into(); }
            } else if let Some(ai) = &mut self.units[i].ai {
                ai.update(&mut self.robots, &self.match_, &self.fuel, &mut self.rules, &mut self.rng, dt);
            } else if self.units[i].human && teleop {
                // (its command, as the person driving it set it)
            } else {
                let c = &mut self.robots[i].cmd;
                c.vx = 0.0; c.vz = 0.0; c.omega = 0.0; c.intake = false; c.shoot = false; c.pass = false; c.outtake = false;
            }
        }
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
