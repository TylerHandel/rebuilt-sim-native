//! js/opponent.js: the robot AI (the PvE opponents, your partners, watch mode). In AUTO a robot runs
//! its AUTO routine (auto.rs); in TELEOP it drives with one of these strategies:
//!   scorer  - collects FUEL, stages in its ALLIANCE ZONE and shoots while its HUB is active
//!   defense - guards the lane into an opponent's ALLIANCE ZONE and rams it back, shoves it if it
//!             gets in, and backs off before a PIN turns into a foul
//!   hybrid  - plays defense while only the other HUB is active, scores the rest of the time
//! How well it drives: its skill (handicaps: speed, accuracy, reaction time...) and its brain
//! (strategy choices, the numbers tools/train.mjs tunes by self-play).
use crate::constants::*;
use crate::field::{alliance_line_x, hub_center};
use crate::fuel::{clamp, rand, Fuel, State};
use crate::game_match::Match;
use crate::nav::{obstacle_at, under_trench, Circle, NavGrid, P};
use crate::parity::Cmd;
use crate::rng::Mulberry32;
use crate::robot::{fits_trench, wrap_angle, ClimbState, Phys, Robot};
use crate::rules::{tower_protected, RobotRules, PIN_RESET};
use jsmath as m;
use std::collections::HashMap;

/// js BRAIN_SPEC's values: the strategy's numbers
#[derive(Clone, Copy, Debug)]
pub struct Brain {
    pub fill: f64, pub stage_margin: f64, pub top_up: f64, pub spot_fx: f64, pub spot_z: f64, pub shoot_speed: f64, pub shoot_accel: f64, pub push_through: f64,
    pub shuttle: f64, pub pass_batch: f64, pub drop_fx: f64, pub collect_speed: f64, pub density: f64, pub own_zone: f64, pub intake_dist: f64, pub stockpile: f64,
    pub steal: f64, pub zone_min: f64, pub pin_limit: f64, pub push_speed: f64, pub ram_dist: f64, pub block_lead: f64, pub engage: f64, pub hybrid_load: f64,
}
/// js DEFAULT_BRAIN
pub const DEFAULT_BRAIN: Brain = Brain {
    fill: 0.85, stage_margin: 2.0, top_up: 6.0, spot_fx: 2.7, spot_z: 0.9, shoot_speed: 0.45, shoot_accel: 3.0, push_through: 1.5, shuttle: 1.0, pass_batch: 45.0, drop_fx: 2.6,
    collect_speed: 0.7, density: 0.6, own_zone: 0.4, intake_dist: 8.0, stockpile: 1.5, steal: 2.0, zone_min: 6.0, pin_limit: 1.8, push_speed: 1.0, ram_dist: 2.5, block_lead: 1.2,
    engage: 0.0, hybrid_load: 0.0,
};
/// js TRAINED_BRAIN (js/trainedBrain.js: the self-play result)
pub const TRAINED_BRAIN: Brain = Brain {
    fill: 0.799, stage_margin: 2.334, top_up: 4.434, spot_fx: 2.919, spot_z: 1.11, shoot_speed: 0.367, shoot_accel: 3.447, push_through: 1.99, shuttle: 0.97, pass_batch: 12.665,
    drop_fx: 2.557, collect_speed: 0.778, density: 0.356, own_zone: -0.121, intake_dist: 2.937, stockpile: 1.859, steal: 2.208, zone_min: 6.747, pin_limit: 1.63, push_speed: 0.935,
    ram_dist: 2.262, block_lead: 1.266, engage: -0.353, hybrid_load: 0.101,
};

/// js OPP_SKILLS[k]: the handicaps
#[derive(Clone, Copy, Debug)]
pub struct Skill { pub name: &'static str, pub speed: f64, pub load: f64, pub noise: f64, pub hesitate: f64, pub sotm: bool, pub think: f64, pub lag: f64, pub desc: &'static str }
pub const SKILL_ORDER: [&str; 4] = ["rookie", "regional", "champs", "trained"];
pub fn skill(k: &str) -> Skill {
    match k {
        "rookie" => Skill { name: "Rookie", speed: 0.6, load: 0.35, noise: 3.0, hesitate: 2.5, sotm: false, think: 0.6, lag: 0.7, desc: "Slow, small loads, misses more, stops to shoot, and sometimes holds a PIN too long." },
        "champs" => Skill { name: "Champs", speed: 1.0, load: 1.0, noise: 1.0, hesitate: 0.0, sotm: true, think: 0.2, lag: 0.25, desc: "Full speed, full hoppers, tight cycles, clean defense (hand-tuned strategy)." },
        "trained" => Skill { name: "Trained", speed: 1.0, load: 1.0, noise: 1.0, hesitate: 0.0, sotm: true, think: 0.2, lag: 0.25, desc: "Champs-level robot running the strategy learned by AI-vs-AI self-play." },
        _ => Skill { name: "Regional", speed: 0.8, load: 0.65, noise: 1.7, hesitate: 1.0, sotm: true, think: 0.35, lag: 0.45, desc: "A solid district/regional robot." },
    }
}
/// js `brainFor(skill)`
pub fn brain_for(k: &str) -> Brain {
    match k {
        "rookie" => Brain { fill: 0.3, collect_speed: 0.35, pin_limit: 3.6, ..DEFAULT_BRAIN },
        "champs" => DEFAULT_BRAIN,
        "trained" => TRAINED_BRAIN,
        _ => Brain { fill: 0.5, collect_speed: 0.5, pin_limit: 2.4, ..DEFAULT_BRAIN },
    }
}
/// js OPP_STRATEGIES (the ones a slot can take)
pub const STRATEGIES: [&str; 3] = ["scorer", "defense", "hybrid"];
pub fn strategy_name(k: &str) -> &'static str { match k { "scorer" => "Scorer", "defense" => "Defense", "hybrid" => "Hybrid", _ => "?" } }
pub fn strategy_desc(k: &str) -> &'static str {
    match k {
        "scorer" => "Runs its own cycles: collects FUEL, stages when its HUB is inactive and shoots on the move when it is active.",
        "defense" => "Blocks the other alliance's path to its HUB and pushes its robots while they shoot. Backs off before a 3 s PIN (G418).",
        "hybrid" => "Shift-aware: plays defense while only the other HUB is active and scores whenever its own HUB is active.",
        _ => "",
    }
}
/// js `opponentAuto(strategy)`: the AUTO routine and start an AI opponent runs
pub fn opponent_auto(strategy: &str) -> (&'static str, &'static str) { if strategy == "defense" { ("defendStage", "rightBump") } else { ("best", "rightTrench") } }

/// js phys1706.js FOLD_TO_SHOOT: 1706's intake folds in to feed its shots (the user)
const FOLD_TO_SHOOT: bool = true;

/// js Field.inAllianceZone(alliance, x)
fn in_zone(a: Alliance, x: f64) -> bool { let lx = alliance_line_x(a); if a == Alliance::Blue { x <= lx } else { x >= lx } }

#[derive(Clone, Copy, Debug, PartialEq)]
enum St { Collect, Score, Feed }
#[derive(Clone, Copy, Debug, PartialEq)]
enum Mode { Score, Defend }
#[derive(Clone, Copy, Debug, PartialEq)]
enum Face { Travel, None, Yaw(f64) }
#[derive(Clone, Copy, Debug, PartialEq)]
enum ShotIntake { Until(f64), LetGo }
/// js `_shotHold()`: how long a robot whose shot compacts its load keeps intaking as the shot starts
#[derive(Clone, Copy, Debug, PartialEq)]
enum Hold { NoCompact, Never, Secs(f64) }

/// js `OpponentAI`
#[derive(Clone, Debug)]
pub struct OpponentAI {
    pub me: usize,
    pub foes: Vec<usize>,
    pub mates: Vec<usize>,
    others: Vec<usize>,
    pub foe: Option<usize>,
    focus_t: f64,
    pub strategy: String,
    pub skill: Skill,
    pub brain: Brain,
    nav_fit: NavGrid,
    nav_tall: Option<NavGrid>,
    tall: bool,
    state: St,
    mode: Mode,
    pub label: String,
    path: Option<Vec<P>>,
    goal: Option<[f64; 2]>,
    replan_t: f64,
    think_t: f64,
    ball: Option<usize>,
    ball_best: f64,
    ball_progress_t: f64,
    blacklist: HashMap<usize, f64>,
    collect_t: f64,
    stuck_t: f64,
    unstick: f64,
    unstick_dir: [f64; 2],
    backoff: bool,
    cooldown: f64,
    t: f64,
    dt: f64,
    hesitate_t: f64,
    seen: Vec<[f64; 3]>,
    rate: f64,
    cycle_start: Option<(f64, u32)>,
    max_load: f64,
    fill_target: f64,
    natural: bool,
    stop_at: f64,
    flow: bool,
    last_gain_t: f64,
    intaked_seen: u32,
    shots_seen: u32,
    last_out_t: f64,
    was_shooting: bool,
    prev_shooting: bool,
    rethink: bool,
    shooting_mode: bool,
    was_firing: bool,
    last_cmd: Option<[f64; 2]>,
    zone_fuel: Option<(f64, usize)>,
    intake_state: Option<St>,
    intake_down: bool,
    shot_intake: Option<ShotIntake>,
    shot_gain: (u32, f64),
    progress: Option<(f64, f64)>,
    passing: bool,
    tall_hold: bool,
    lane_lock: Option<f64>,
    lane_t: f64,
}

impl OpponentAI {
    /// js `new OpponentAI({ robot, foes, mates, match, fuel, rules, strategy, skill })`
    pub fn new(me: usize, robots: &mut [Robot], foes: Vec<usize>, mates: Vec<usize>, strategy: &str, skill_key: &str) -> Self {
        let r = &mut robots[me];
        let sk = skill(skill_key);
        let brain = brain_for(skill_key);
        let nav_r = m::min(r.half_l, r.half_w) + 0.07;
        let fits = fits_trench(&r.cfg);
        let nav_fit = NavGrid::new(nav_r, fits);
        let nav_tall = if fits { Some(NavGrid::new(nav_r, false)) } else { None };
        let cap = r.max_capacity();
        r.noise_scale = sk.noise;
        let max_load = if cap < 20.0 { cap } else { m::max(10.0, m::round(sk.load * cap)) };
        let mut fill_target = if cap < 20.0 || sk.load >= 1.0 { max_load } else { m::min(max_load, m::max(8.0, m::round(brain.fill * cap))) };
        let natural = r.phys.as_ref().is_some_and(|p| p.physical().natural_full.is_some());
        let stop_at;
        if natural && sk.load >= 1.0 { stop_at = f64::INFINITY; fill_target = f64::INFINITY; } else { stop_at = max_load; }
        let flow = cap < 20.0 && r.cfg["shooter"]["type"].as_str() == Some("turret");
        let others = foes.iter().chain(mates.iter()).copied().collect();
        let foe = foes.first().copied();
        OpponentAI {
            me, foes, mates, others, foe, focus_t: 0.0, strategy: strategy.into(), skill: sk, brain, nav_fit, nav_tall, tall: false, state: St::Collect, mode: Mode::Score,
            label: "Waiting".into(), path: None, goal: None, replan_t: 0.0, think_t: 0.0, ball: None, ball_best: 1e9, ball_progress_t: 0.0, blacklist: HashMap::new(),
            collect_t: 0.0, stuck_t: 0.0, unstick: 0.0, unstick_dir: [0.0; 2], backoff: false, cooldown: 0.0, t: 0.0, dt: 0.0, hesitate_t: 0.0, seen: Vec::new(), rate: 2.5,
            cycle_start: None, max_load, fill_target, natural, stop_at, flow, last_gain_t: 0.0, intaked_seen: 0, shots_seen: 0, last_out_t: 0.0, was_shooting: false,
            prev_shooting: false, rethink: false, shooting_mode: false, was_firing: false, last_cmd: None, zone_fuel: None, intake_state: None, intake_down: false,
            shot_intake: None, shot_gain: (0, 0.0), progress: None, passing: false, tall_hold: false, lane_lock: None, lane_t: 0.0,
        }
    }

    fn own(&self, robots: &[Robot]) -> Alliance { robots[self.me].alliance }
    fn turret(&self, robots: &[Robot]) -> bool { robots[self.me].cfg["shooter"]["type"].as_str() == Some("turret") }
    /// js `_hasRoom()`
    fn has_room(&self, r: &Robot) -> bool { (r.stored.len() as f64) < self.stop_at && !(self.natural && r.full) }
    fn dist(a: &Robot, b: &Robot) -> f64 { m::hypot2(b.pos[0] - a.pos[0], b.pos[2] - a.pos[2]) }

    /// js `update(dt)`: this step's command for its robot
    pub fn update(&mut self, robots: &mut [Robot], mt: &Match, fuel: &Fuel, rules: &mut RobotRules, rng: &mut Mulberry32, dt: f64) {
        let i = self.me;
        self.t += dt;
        self.dt = dt;
        self.rethink = false;
        self.pick_focus(robots, dt);
        if let Some(f) = self.foe {
            let p = robots[f].pos;
            self.seen.push([self.t, p[0], p[2]]);
            while self.seen.len() > 2 && self.seen[1][0] < self.t - self.skill.lag { self.seen.remove(0); }
        }
        let keep = robots[i].cmd;
        robots[i].cmd = Cmd { spin: keep.spin, stow: keep.stow, ..Cmd::default() };
        self.prev_shooting = self.was_shooting; self.was_shooting = false;
        {
            let r = &mut robots[i];
            if r.cfg["intake"]["fold"].is_object() && fits_trench(&r.cfg) && under_trench(r.pos[0], r.pos[2], r.half_l + 1.2) { r.cmd.lower = true; }
        }
        if !mt.is_teleop() { self.label = if mt.is_auto() { "AUTO".into() } else { "Waiting".into() }; return; }

        let own = self.own(robots);
        match self.foe {
            None => self.mode = Mode::Score,
            Some(f) => {
                self.mode = if self.strategy == "defense" { Mode::Defend }
                else if self.strategy == "hybrid" {
                    let s = mt.shift_index();
                    let loaded = robots[i].stored.len() as f64 >= self.brain.hybrid_load * self.max_load;
                    if (1..=4).contains(&s) && !mt.hub_active(own) && mt.hub_active(robots[f].alliance) && !self.in_grace(mt, own) && loaded { Mode::Defend } else { Mode::Score }
                } else { Mode::Score };
            }
        }
        if self.unstick > 0.0 {
            self.unstick -= dt;
            let r = &mut robots[i];
            let sp = r.max_speed * 0.5;
            r.cmd.vx = self.unstick_dir[0] * sp; r.cmd.vz = self.unstick_dir[1] * sp;
            self.label = "Unsticking".into();
            return;
        }
        if matches!(&robots[i].phys, Some(Phys::R6328(p)) if p.back_off()) {
            let r = &mut robots[i];
            let (f, sp) = (r.forward(), r.max_speed * 0.3);
            r.cmd.vx = -f[0] * sp; r.cmd.vz = -f[1] * sp; r.cmd.intake = true;
            self.label = "Clearing its intake".into();
            return;
        }
        robots[i].ai_pass = None;
        self.shooting_mode = false;
        if self.climb(robots, mt) { return; }
        if self.mode == Mode::Defend { self.defend(robots, mt, rules, dt); }
        else { self.score(robots, mt, fuel, dt); self.yield_to_foes(robots); self.was_shooting = self.state == St::Score && robots[i].cmd.shoot; }
        self.smooth(robots, dt);
        {
            let r = &mut robots[i];
            let holding = !r.stored.is_empty();
            let let_go = match &mut r.phys { Some(Phys::R341(p)) => p.let_go(holding, &mut r.stats), _ => false };
            if let_go { r.cmd.shoot = false; r.cmd.pass = false; }
            if matches!(r.phys, Some(Phys::R1706(_))) && FOLD_TO_SHOOT && r.cmd.shoot { r.cmd.intake = false; }
        }
        // stuck detection (pushing against another robot isn't being stuck)
        let (want, have) = { let r = &robots[i]; (m::hypot2(r.cmd.vx, r.cmd.vz), m::hypot2(r.vel[0], r.vel[2])) };
        let pushing = self.others.iter().any(|&o| if self.foes.contains(&o) { rules.in_contact(i, o) } else { Self::dist(&robots[i], &robots[o]) < robots[i].half_l + robots[o].half_l + 0.15 });
        if want > 1.0 && have < 0.15 && !pushing { self.stuck_t += dt; } else { self.stuck_t = m::max(0.0, self.stuck_t - dt); }
        if self.stuck_t > 1.2 {
            self.stuck_t = 0.0;
            self.unstick = 0.7;
            let c = robots[i].cmd;
            let a = m::atan2(-c.vz, -c.vx) + rand(rng, -1.0, 1.0);
            self.unstick_dir = [m::cos(a), m::sin(a)];
            if let Some(b) = self.ball { self.blacklist.insert(b, self.t + 8.0); }
            self.ball = None;
            self.path = None;
        }
    }

    /// js `_climb()`: END GAME, a robot with a climber heads for its TOWER just in time to get up to
    /// LEVEL 1 before the buzzer, one per ALLIANCE
    fn climb(&mut self, robots: &mut [Robot], mt: &Match) -> bool {
        let i = self.me;
        {
            let r = &robots[i];
            if r.climber.is_none() || r.cfg["climber"].is_null() { return false; }
            if r.climb_state != ClimbState::None {
                self.label = if r.climb_state == ClimbState::Hanging { format!("Hanging (L{})", r.climb_level) } else { "Climbing".into() };
                return true;
            }
        }
        let left = TIMING::teleop - mt.phase_time;
        let p = robots[i].climb_pose();
        let d = m::hypot2(p[0] - robots[i].pos[0], p[2] - robots[i].pos[2]);
        if !robots[i].climb_claim {
            let t1 = robots[i].climber.as_ref().unwrap().times.get(1).copied().unwrap_or(2.0);
            let need = d / (0.55 * robots[i].max_speed * self.skill.speed) + t1 + 2.5;
            if left > need || left < 1.5 { return false; }
            if self.mates.iter().any(|&o| robots[o].climb_claim || robots[o].climb_state != ClimbState::None) { return false; }
            robots[i].climb_claim = true;
        }
        { let r = &mut robots[i]; r.cmd.shoot = r.last_in_zone && !r.stored.is_empty(); }
        if d > 1.0 { self.drive(robots, p[0], p[2], 1.0, Face::Yaw(p[2]), 0.08, true); } else { robots[i].request_climb(); }
        self.label = "Going to climb".into();
        true
    }

    /// js `_pickFocus(dt)`: which opponent to watch
    fn pick_focus(&mut self, robots: &[Robot], dt: f64) {
        if self.foes.len() < 2 { self.foe = self.foes.first().copied(); return; }
        let r = &robots[self.me];
        let defending = self.strategy == "defense" || self.strategy == "hybrid";
        let dist = |o: usize| m::hypot2(robots[o].pos[0] - r.pos[0], robots[o].pos[2] - r.pos[2]);
        let pick = if defending {
            self.focus_t -= dt;
            if self.focus_t > 0.0 && self.foe.is_some() { return; }
            self.focus_t = 2.5;
            let threat = |o: usize| robots[o].stored.len() as f64 + (if robots[o].last_in_zone { 25.0 } else { 0.0 }) - 3.0 * dist(o);
            self.foes.iter().copied().reduce(|a, b| if threat(b) > threat(a) { b } else { a }).unwrap()
        } else { self.foes.iter().copied().reduce(|a, b| if dist(b) < dist(a) { b } else { a }).unwrap() };
        if Some(pick) != self.foe { self.foe = Some(pick); self.seen.clear(); }
    }

    /// js `_smooth(dt)`: driving while shooting or passing, its speed capped and its changes eased
    fn smooth(&mut self, robots: &mut [Robot], dt: f64) {
        let r = &mut robots[self.me];
        let b = self.brain;
        let firing = ((r.cmd.shoot || r.cmd.pass) && !r.stored.is_empty()) || self.shooting_mode;
        if firing {
            let last = if self.was_firing && self.last_cmd.is_some() { self.last_cmd.unwrap() } else { [r.vel[0], r.vel[2]] };
            let (mut vx, mut vz) = (r.cmd.vx, r.cmd.vz);
            let vmax = r.max_speed * b.shoot_speed;
            let v = m::hypot2(vx, vz);
            if v > vmax { vx *= vmax / v; vz *= vmax / v; }
            let (mut dvx, mut dvz) = (vx - last[0], vz - last[1]);
            let (dv, max_dv) = (m::hypot2(dvx, dvz), b.shoot_accel * dt);
            if dv > max_dv { dvx *= max_dv / dv; dvz *= max_dv / dv; }
            r.cmd.vx = last[0] + dvx; r.cmd.vz = last[1] + dvz;
            r.cmd.omega = clamp(r.cmd.omega, -2.5, 2.5);
        }
        self.was_firing = firing;
        self.last_cmd = Some([r.cmd.vx, r.cmd.vz]);
    }

    fn in_grace(&self, mt: &Match, own: Alliance) -> bool { !mt.hub_active(own) && mt.t - mt.last_deactivate[crate::game_match::ai(own)] < 1.5 }

    /// js `_drive(x, z, { speed, face, arrive, avoid })`: along its planned path; returns how far is left
    #[allow(clippy::too_many_arguments)]
    fn drive(&mut self, robots: &mut [Robot], x: f64, z: f64, speed: f64, face: Face, arrive: f64, avoid: bool) -> f64 {
        let i = self.me;
        self.replan_t -= self.dt;
        let tall = !robots[i].fits_trench_lowered() && self.nav_tall.is_some();
        if tall != self.tall { self.tall = tall; self.path = None; }
        let (rx, rz) = (robots[i].pos[0], robots[i].pos[2]);
        if self.path.is_none() || self.goal.is_none() || self.replan_t <= 0.0 || m::hypot2(self.goal.unwrap()[0] - x, self.goal.unwrap()[1] - z) > 0.3 {
            let circles: Vec<Circle> = if avoid { self.others.iter().filter(|&&o| m::hypot2(robots[o].pos[0] - rx, robots[o].pos[2] - rz) < 5.0).map(|&o| Circle { x: robots[o].pos[0], z: robots[o].pos[2], r: m::max(robots[o].half_l, robots[o].half_w) + 0.1 }).collect() } else { Vec::new() };
            let nav = if self.tall { self.nav_tall.as_mut().unwrap() } else { &mut self.nav_fit };
            self.path = Some(nav.plan(rx, rz, x, z, &circles));
            self.goal = Some([x, z]);
            self.replan_t = 0.3;
        }
        let path = self.path.as_mut().unwrap();
        while path.len() > 1 && m::hypot2(path[0].x - rx, path[0].z - rz) < 0.35 { path.remove(0); }
        let next = path[0];
        let mut remain = m::hypot2(next.x - rx, next.z - rz);
        for k in 0..path.len() - 1 { remain += m::hypot2(path[k + 1].x - path[k].x, path[k + 1].z - path[k].z); }
        let r = &robots[i];
        let vmax = r.max_speed * self.skill.speed * speed;
        let sp = m::min(vmax, m::sqrt(2.0 * 5.0 * m::max(0.0, remain - arrive)));
        let (dx, dz) = (next.x - rx, next.z - rz);
        let dn = { let h = m::hypot2(dx, dz); if h == 0.0 { 1.0 } else { h } };
        let (mut vx, mut vz) = ((dx / dn) * sp, (dz / dn) * sp);
        if avoid {
            for &o in &self.others {
                let (ox, oz) = (rx - robots[o].pos[0], rz - robots[o].pos[2]);
                let od = m::hypot2(ox, oz);
                let rr = 1.4;
                if od < rr && od > 1e-3 {
                    let k = ((rr - od) / rr) * vmax * 0.8;
                    let side = if ox * dz - oz * dx > 0.0 { 1.0 } else { -1.0 };
                    vx += (ox / od) * k + side * (-oz / od) * k * 0.6;
                    vz += (oz / od) * k + side * (ox / od) * k * 0.6;
                }
            }
        }
        robots[i].cmd.vx = vx; robots[i].cmd.vz = vz;
        let mut heading = match face { Face::Travel => if sp > 0.3 { Some(m::atan2(-vz, vx)) } else { None }, Face::Yaw(h) => Some(h), Face::None => None };
        if let Some(h) = heading { heading = Some(self.lane_heading(&robots[i], h)); }
        if let Some(h) = heading { let r = &mut robots[i]; r.cmd.omega = clamp(5.0 * wrap_angle(h - r.yaw), -r.max_omega, r.max_omega); }
        remain
    }

    /// js `_laneHeading(heading)`: under a TRENCH's arm a robot too long to turn round holds a square
    /// heading until it's out
    fn lane_heading(&mut self, r: &Robot, heading: f64) -> f64 {
        let reach = r.cfg["intake"]["reach"].as_f64().unwrap_or(0.0) * r.intake_deploy;
        if m::hypot2(r.half_l + reach, r.half_w) <= TRENCH::clearWidth / 2.0 || !under_trench(r.pos[0], r.pos[2], 0.3) {
            self.lane_lock = None; self.lane_t = 0.0;
            return heading;
        }
        let along = m::round(r.yaw / m::PI) * m::PI;
        let wy = wrap_angle(r.yaw);
        let across = m::sign(if wy == 0.0 { 1.0 } else { wy }) * m::PI / 2.0;
        if self.lane_lock.is_none() {
            let a = m::abs(wrap_angle(r.yaw));
            let off = m::min(a, m::PI - a);
            self.lane_lock = Some(if off < m::atan2(2.0 * r.half_l + reach, 2.0 * r.half_w) { along } else { across });
            self.lane_t = 0.0;
        }
        let lock = self.lane_lock.unwrap();
        let stuck = m::abs(wrap_angle(lock - r.yaw)) > 0.2 && m::abs(r.omega) < 0.3;
        self.lane_t = if stuck { self.lane_t + self.dt } else { 0.0 };
        if self.lane_t > 1.0 { self.lane_lock = Some(if m::abs(m::cos(lock)) > 0.5 { across } else { along }); self.lane_t = 0.0; }
        self.lane_lock.unwrap()
    }

    /// js `_spot()`: where a chassis-aimed shooter goes to shoot from
    fn spot(&self, r: &Robot) -> [f64; 2] {
        let s = if r.alliance == Alliance::Blue { 1.0 } else { -1.0 };
        let z_min = self.brain.spot_z;
        let z = clamp(r.pos[2], -2.6, 2.6);
        [s * (-HALF_L + self.brain.spot_fx), if m::abs(z) < z_min { m::sign(if z == 0.0 { 1.0 } else { z }) * z_min } else { z }]
    }
    /// js `_entry()`: a turret shoots from anywhere in the zone: the nearest point just inside
    fn entry(&self, r: &Robot) -> [f64; 2] {
        let own = r.alliance;
        let line_x = alliance_line_x(own);
        let inside = if own == Alliance::Blue { -1.0 } else { 1.0 };
        let sz = m::sign(r.pos[2]);
        let z = clamp(m::abs(r.pos[2]), 1.3, 2.6) * (if sz == 0.0 || sz.is_nan() { 1.0 } else { sz });
        let x = if in_zone(own, r.pos[0]) && m::abs(r.pos[0] - line_x) > 0.8 { r.pos[0] } else { line_x + inside * 0.9 };
        [x, if in_zone(own, r.pos[0]) { r.pos[2] } else { z }]
    }
    fn home(&self, robots: &[Robot]) -> [f64; 2] { let r = &robots[self.me]; if self.turret(robots) { self.entry(r) } else { self.spot(r) } }

    /// js `_zoneFuel()`: how much FUEL is lying in our own ALLIANCE ZONE
    fn zone_fuel(&mut self, own: Alliance, fuel: &Fuel) -> usize {
        if let Some((t, n)) = self.zone_fuel { if t == self.t { return n; } }
        let n = fuel.balls.iter().filter(|b| b.state == State::Field && !b.in_flight && !b.held && b.pos[1] < 0.32 && in_zone(own, b.pos[0]) && m::abs(b.pos[0]) < HALF_L - 0.2).count();
        self.zone_fuel = Some((self.t, n));
        n
    }

    /// js `_score(dt)`
    fn score(&mut self, robots: &mut [Robot], mt: &Match, fuel: &Fuel, dt: f64) {
        let i = self.me;
        let own = self.own(robots);
        let b = self.brain;
        let stored = robots[i].stored.len() as f64;
        let cap = self.max_load;
        { let r = &robots[i];
            if r.stats.intaked != self.intaked_seen || self.state != St::Collect { self.intaked_seen = r.stats.intaked; self.last_gain_t = self.t; }
            if r.stats.shots != self.shots_seen || !self.prev_shooting { self.shots_seen = r.stats.shots; self.last_out_t = self.t; }
        }
        let active = mt.hub_active(own);
        let nc = mt.hub_next_change(own);
        let until_active = if active { 0.0 } else { nc.unwrap_or(f64::INFINITY) };
        let home = self.home(robots);
        let travel = m::hypot2(home[0] - robots[i].pos[0], home[1] - robots[i].pos[2]) / (robots[i].max_speed * self.skill.speed * 0.7) + 1.0;
        let left = TIMING::teleop - mt.phase_time;
        let can_shoot = active || self.in_grace(mt, own);
        let load_time = m::max(0.0, cap - stored) / m::max(0.8, self.rate);
        let feed = b.shuttle >= 0.5 && !can_shoot && until_active > travel + load_time + b.stage_margin;
        let turret = self.turret(robots);

        self.think_t -= dt;
        if self.think_t <= 0.0 {
            self.think_t = self.skill.think;
            self.rethink = true;
            if feed {
                if self.state != St::Feed { self.state = St::Feed; self.path = None; self.ball = None; }
            } else if self.state == St::Feed {
                self.state = St::Collect; self.collect_t = 0.0; self.path = None; self.ball = None;
            } else if self.state == St::Collect {
                let window_left = if active { nc.unwrap_or(left) + 1.5 } else { f64::INFINITY };
                let bps = robots[i].cfg["shooter"]["bps"].as_f64().unwrap();
                let shoot_time = |n: f64| 0.8 + n / bps;
                let last_call = active && stored >= 3.0 && window_left <= travel + shoot_time(stored) + 1.0;
                let stale = self.t - self.last_gain_t;
                let full = robots[i].full;
                let go = stored >= self.stop_at || (full && stored >= 0.5 * cap)
                    || (self.natural && ((stored >= cap && stale > 2.0) || (stored >= 0.75 * cap && stale > 3.0)))
                    || (stored >= self.fill_target && active)
                    || last_call
                    || (stored >= 3.0 && !active && until_active <= travel + b.stage_margin)
                    || (stored > 0.0 && left < travel + 4.0);
                let stay = turret && robots[i].last_in_zone && can_shoot && (self.flow || (self.zone_worth_it(robots, mt, fuel) && self.has_room(&robots[i])));
                if go && !stay { self.state = St::Score; self.path = None; self.progress = None; }
            } else if stored == 0.0 || (stored <= 2.0 && self.t - self.last_out_t > 3.0) {
                self.state = St::Collect; self.collect_t = 0.0; self.path = None;
                self.hesitate_t = self.skill.hesitate;
            } else if !can_shoot && until_active > travel + b.top_up && self.has_room(&robots[i]) {
                self.state = St::Collect; self.path = None;
            } else if turret && can_shoot && robots[i].last_in_zone && (self.flow || (self.zone_worth_it(robots, mt, fuel) && self.has_room(&robots[i]))) {
                self.state = St::Collect; self.path = None;
            }
        }

        if Some(self.state) != self.intake_state { self.intake_state = Some(self.state); self.intake_down = false; self.shot_intake = None; }
        if self.state == St::Feed { self.feed(robots, mt, fuel, dt); return; }
        if self.state == St::Collect {
            if self.hesitate_t > 0.0 { self.hesitate_t -= dt; self.label = "Deciding…".into(); return; }
            if self.cycle_start.is_none() { self.cycle_start = Some((self.t, robots[i].stats.intaked)); }
            self.collect_t += dt;
            self.collect(robots, mt, fuel, dt);
            let in_z = robots[i].last_in_zone;
            if turret && can_shoot && in_z && self.skill.sotm {
                self.shooting_mode = stored > 0.0;
                robots[i].cmd.shoot = stored > 0.0;
                self.label = format!("Shooting while collecting ({stored})");
            } else if self.flow && can_shoot && !in_z && stored > 0.0 && self.pass_on(robots, fuel, left, travel) {
                let tgt = self.drop_point(&robots[i]);
                let clear = self.pass_clear(own, [robots[i].pos[0], robots[i].pos[2]], tgt);
                let r = &mut robots[i];
                r.ai_pass = Some(tgt);
                r.cmd.pass = clear;
                r.cmd.prespin = true;
                self.label = format!("Passing while collecting ({} passed)", r.stats.passes);
            }
            return;
        }
        if let Some((t0, n0)) = self.cycle_start {
            let tt = self.t - t0;
            let n = robots[i].stats.intaked as f64 - n0 as f64;
            if tt > 2.0 { self.rate = 0.7 * self.rate + 0.3 * (n / tt); }
            self.cycle_start = None;
        }
        if turret && robots[i].last_in_zone && can_shoot && self.skill.sotm {
            robots[i].cmd.shoot = true;
            self.label = if robots[i].ready { "Shooting".into() } else { "Aiming".into() };
            return;
        }
        robots[i].cmd.lineup = !robots[i].stored.is_empty();
        let rp = [robots[i].pos[0], robots[i].pos[2]];
        let d_f = self.others.iter().fold(f64::INFINITY, |a, &o| m::min(a, m::hypot2(robots[o].pos[0] - rp[0], robots[o].pos[2] - rp[1])));
        let to_home = m::hypot2(home[0] - rp[0], home[1] - rp[1]);
        if self.progress.is_none() || to_home < self.progress.unwrap().0 - 0.3 || d_f > 3.0 || to_home < 0.8 { self.progress = Some((to_home, self.t)); }
        let bully = self.t - self.progress.unwrap().1 > b.push_through;
        let mut face = Face::Travel;
        if !turret && (to_home < 3.0 || robots[i].last_in_zone) {
            let c = hub_center(own);
            face = Face::Yaw(m::atan2(-(c.1 - rp[1]), c.0 - rp[0]) - robots[i].aim_offset);
        }
        let remain = self.drive(robots, home[0], home[1], 1.0, face, 0.08, !bully);
        let settled = self.skill.sotm || (remain < 0.3 && m::hypot2(robots[i].vel[0], robots[i].vel[2]) < 0.3);
        robots[i].cmd.shoot = can_shoot && robots[i].last_in_zone && settled;
        let lid_up = { let r = &robots[i]; r.deploy_raises_top() && under_trench(r.pos[0], r.pos[2], m::max(r.half_l, r.half_w) + 2.0) };
        let hold = self.shot_hold(&robots[i]);
        if hold != Hold::Never && self.has_room(&robots[i]) && !self.foe_ahead(robots) && !self.hub_fuel_near(&robots[i], fuel) && !lid_up { robots[i].cmd.intake = true; }
        if let Hold::Secs(h) = hold {
            if !robots[i].cmd.shoot { self.shot_intake = None; }
            else if robots[i].cmd.intake {
                let n = robots[i].stats.intaked;
                if self.shot_intake.is_none() { self.shot_intake = Some(ShotIntake::Until(self.t + h)); self.shot_gain = (n, self.t); }
                if n != self.shot_gain.0 { self.shot_gain = (n, self.t); }
                if let Some(ShotIntake::Until(u)) = self.shot_intake { if self.t - self.shot_gain.1 < 1.0 && self.fuel_ahead(&robots[i], fuel) { self.shot_intake = Some(ShotIntake::Until(m::max(u, self.t + 0.3))); } }
                if let Some(ShotIntake::Until(u)) = self.shot_intake { if self.t > u { self.shot_intake = Some(ShotIntake::LetGo); } }
                if self.shot_intake == Some(ShotIntake::LetGo) { robots[i].cmd.intake = false; }
            }
        }
        let r = &robots[i];
        self.label = if r.cmd.shoot { if r.ready { "Shooting".into() } else { "Aiming".into() } } else if bully { "Pushing through".into() } else if remain > 0.3 { "Returning to score".into() } else { "Staged — waiting for HUB".into() };
    }

    /// js `_passOn(left, travel)`
    fn pass_on(&self, robots: &[Robot], fuel: &Fuel, left: f64, travel: f64) -> bool {
        let own = self.own(robots);
        self.ball.is_none_or(|b| !in_zone(own, fuel.balls[b].pos[0])) && left > travel + 8.0
    }
    /// js `_dropPoint()`: where our passes land
    fn drop_point(&self, r: &Robot) -> [f64; 2] {
        let s = if r.alliance == Alliance::Blue { 1.0 } else { -1.0 };
        let sz = m::sign(r.pos[2]);
        let side = if sz == 0.0 || sz.is_nan() { 1.0 } else { sz };
        [s * (-HALF_L + self.brain.drop_fx), side * clamp(m::abs(r.pos[2]), 1.4, 2.6)]
    }
    /// js `_passClear(p, q)`: a pass that doesn't fly over our HUB
    fn pass_clear(&self, own: Alliance, p: [f64; 2], q: [f64; 2]) -> bool {
        let hx = hub_center(own).0;
        if (p[0] - hx) * (q[0] - hx) > 0.0 { return true; }
        let t = (hx - p[0]) / (q[0] - p[0]);
        m::abs(p[1] + (q[1] - p[1]) * t) > HUB::size / 2.0 + 0.55
    }

    /// js `_feed(dt)`: off shift, collect in the NEUTRAL ZONE and pass it all into our zone
    fn feed(&mut self, robots: &mut [Robot], mt: &Match, fuel: &Fuel, dt: f64) {
        let i = self.me;
        let own = self.own(robots);
        let b = self.brain;
        let target = self.drop_point(&robots[i]);
        robots[i].ai_pass = Some(target);
        if self.ball.is_none_or(|x| fuel.balls[x].state != State::Field || fuel.balls[x].held) || self.rethink {
            let nb = self.pick_ball(robots, mt, fuel, true);
            if nb != self.ball { self.ball = nb; self.ball_best = 1e9; self.ball_progress_t = 0.0; }
        }
        let line_x = alliance_line_x(own);
        let rp = [robots[i].pos[0], robots[i].pos[2]];
        let can_pass = !robots[i].last_in_zone && self.pass_clear(own, rp, target);
        let stored = robots[i].stored.len() as f64;
        let full = !self.has_room(&robots[i]) || robots[i].full;
        let batch = if self.flow { 1.0 } else { m::min(self.max_load, m::max(2.0, m::round(b.pass_batch))) };
        if can_pass && stored > 0.0 && (stored >= batch || self.ball.is_none() || full) { self.passing = true; }
        if !can_pass || stored == 0.0 { self.passing = false; }
        self.shooting_mode = self.passing && !self.flow;
        robots[i].cmd.prespin = can_pass && (self.flow || stored >= 0.7 * batch || full);
        let bl = if full { None } else { self.ball };
        let turret = self.turret(robots);
        if self.passing && !turret {
            if let Some(x) = bl { let p = fuel.balls[x].pos; self.drive(robots, p[0], p[2], 0.3, Face::Travel, 0.08, true); }
        } else if let Some(x) = bl {
            let p = fuel.balls[x].pos;
            let d = m::hypot2(p[0] - rp[0], p[2] - rp[1]);
            if d < self.ball_best - 0.1 { self.ball_best = d; self.ball_progress_t = 0.0; }
            else { self.ball_progress_t += dt; if self.ball_progress_t > 2.5 { self.blacklist.insert(x, self.t + 8.0); self.ball = None; } }
            self.drive(robots, p[0], p[2], if d < 1.6 { b.collect_speed } else { 1.0 }, Face::Travel, -0.3, d >= 1.6);
            let it = self.intake_for(robots, fuel, d);
            robots[i].cmd.intake = it;
        } else if stored == 0.0 {
            let h = self.home(robots);
            self.drive(robots, h[0], h[1], 1.0, Face::Travel, 0.08, true);
        }
        if bl.is_none() && stored > 0.0 && !can_pass {
            let tx = line_x + (if own == Alliance::Blue { 2.5 } else { -2.5 });
            self.drive(robots, tx, target[1], 1.0, Face::Travel, 0.08, true);
        }
        robots[i].cmd.pass = self.passing;
        self.label = format!("Feeding its zone ({} passed)", robots[i].stats.passes);
    }

    /// js `_foeAhead()`: an opponent right in front of the intake (G415), allowing for how fast
    /// they're closing and how long the intake takes to come up
    fn foe_ahead(&self, robots: &[Robot]) -> bool {
        let r = &robots[self.me];
        let f = r.forward();
        let ic = &r.cfg["intake"];
        let up = if ic["latched"].as_bool() == Some(true) { 0.4 } else { ic["retractTime"].as_f64().or(ic["deployTime"].as_f64()).unwrap_or(0.0) };
        let reach = ic["reach"].as_f64().unwrap_or(0.0);
        self.foes.iter().any(|&o| {
            let p = &robots[o];
            let to = [p.pos[0] - r.pos[0], p.pos[2] - r.pos[2]];
            let dp = { let h = m::hypot2(to[0], to[1]); if h == 0.0 { 1.0 } else { h } };
            if (to[0] * f[0] + to[1] * f[1]) / dp < 0.3 { return false; }
            let closing = m::max(0.0, ((r.vel[0] - p.vel[0]) * to[0] + (r.vel[2] - p.vel[2]) * to[1]) / dp);
            dp < r.half_l + p.half_l + reach + 0.25 + closing * up
        })
    }

    /// js `_yieldToFoes()`: with an opponent in front of an intake that's out, don't drive into it
    fn yield_to_foes(&self, robots: &mut [Robot]) {
        let i = self.me;
        if robots[i].intake_deploy < 0.3 || !self.foe_ahead(robots) { return; }
        for &o in &self.foes {
            let (dx, dz) = (robots[o].pos[0] - robots[i].pos[0], robots[o].pos[2] - robots[i].pos[2]);
            let d = { let h = m::hypot2(dx, dz); if h == 0.0 { 1.0 } else { h } };
            let hl = robots[i].half_l + robots[o].half_l;
            let c = &mut robots[i].cmd;
            let into = (c.vx * dx + c.vz * dz) / d;
            if into > 0.2 && d < hl + 1.2 { let k = (into - 0.2) / d; c.vx -= dx * k; c.vz -= dz * k; }
        }
    }

    /// js `_pickBall({ feed })`: the FUEL that gets the most FUEL per second of driving
    fn pick_ball(&mut self, robots: &[Robot], mt: &Match, fuel: &Fuel, feed: bool) -> Option<usize> {
        let r = &robots[self.me];
        let b = self.brain;
        let own = r.alliance;
        let opp = other(own);
        let active = mt.hub_active(own) || self.in_grace(mt, own);
        let line_x = alliance_line_x(own);
        let mut bins: HashMap<i64, i32> = HashMap::new();
        let mut cands: Vec<(usize, i64, i64)> = Vec::new();
        const CELL: f64 = 0.5;
        let tall = !r.fits_trench_lowered();
        for (k, ball) in fuel.balls.iter().enumerate() {
            if ball.state != State::Field || ball.in_flight || ball.hub_fresh || ball.in_corral.is_some() || ball.held { continue; }
            let p = ball.pos;
            if p[1] > 0.32 || m::abs(p[0]) > HALF_L - 0.2 || m::abs(p[2]) > HALF_W - 0.2 { continue; }
            if obstacle_at(p[0], p[2], 0.12) { continue; }
            if tall && under_trench(p[0], p[2], 0.3) { continue; }
            if m::hypot2(p[0] - r.pos[0], p[2] - r.pos[2]) < r.half_l + r.half_w {
                let l = r.world_to_local(p[0], p[2]);
                if m::abs(l[0]) < r.half_l - 0.05 && m::abs(l[1]) < r.half_w - 0.05 { continue; }
            }
            if self.blacklist.get(&k).is_some_and(|&bl| bl > self.t) { continue; }
            let (ci, cj) = (m::floor(p[0] / CELL) as i64, m::floor(p[2] / CELL) as i64);
            *bins.entry(ci * 1000 + cj).or_insert(0) += 1;
            if feed && in_zone(own, p[0]) { continue; }
            cands.push((k, ci, cj));
        }
        let around = |ci: i64, cj: i64| { let mut n = 0; for a in -1..=1 { for c in -1..=1 { n += bins.get(&((ci + a) * 1000 + cj + c)).copied().unwrap_or(0); } } n as f64 };
        let sparse = (self.zone_fuel(own, fuel) as f64) < b.zone_min;
        let r = &robots[self.me];
        let room = m::max(1.0, self.max_load - r.stored.len() as f64);
        let f = r.forward();
        let (mut best, mut bc) = (None, f64::INFINITY);
        for (k, ci, cj) in cands {
            let p = fuel.balls[k].pos;
            let d = m::hypot2(p[0] - r.pos[0], p[2] - r.pos[2]);
            let ownb = in_zone(own, p[0]);
            if !feed && ownb && sparse && d > 1.0 { continue; }
            let mut gain = 1.0 + b.density * (m::min(room, around(ci, cj)) - 1.0);
            for &o in &self.others { if m::hypot2(p[0] - robots[o].pos[0], p[2] - robots[o].pos[2]) < 1.3 { gain *= 0.5; } }
            if in_zone(opp, p[0]) { gain *= 1.0 + 0.3 * b.steal; }
            let mut dist = d + (if d > 0.3 { 0.5 * (1.0 - ((p[0] - r.pos[0]) * f[0] + (p[2] - r.pos[2]) * f[1]) / d) } else { 0.0 });
            if feed {
                let from_line = if own == Alliance::Blue { p[0] - line_x } else { line_x - p[0] };
                if !in_zone(opp, p[0]) { dist += 0.35 * m::max(0.0, from_line - 2.0); }
                if m::abs(p[2]) < 1.1 { dist += 1.5; }
            } else if ownb {
                gain *= m::max(0.2, 1.0 + 0.4 * (b.own_zone + (if active { b.stockpile } else { 0.0 })));
            } else {
                dist += m::abs(p[0] - line_x) * 0.5;
            }
            let c = m::max(0.2, dist) / gain;
            if c < bc { bc = c; best = Some(k); }
        }
        best
    }

    /// js `_zoneWorthIt()`
    fn zone_worth_it(&mut self, robots: &[Robot], mt: &Match, fuel: &Fuel) -> bool {
        let own = self.own(robots);
        if (self.zone_fuel(own, fuel) as f64) < self.brain.zone_min { return false; }
        self.pick_ball(robots, mt, fuel, false).is_some_and(|b| in_zone(own, fuel.balls[b].pos[0]))
    }

    /// js `_intakeFor(d)`: the intake down once the FUEL it's going for is within reach
    fn intake_for(&mut self, robots: &[Robot], fuel: &Fuel, d: f64) -> bool {
        let r = &robots[self.me];
        if d < self.brain.intake_dist { self.intake_down = true; }
        let near = under_trench(r.pos[0], r.pos[2], 1.0);
        if r.load_near_trench() { self.tall_hold = true; } else if r.load_top() < TRENCH::clearHeight - 0.08 { self.tall_hold = false; }
        let wedge = near && (self.tall_hold || r.deploy_raises_top());
        self.intake_down && self.has_room(r) && !wedge && !self.foe_ahead(robots) && !self.hub_fuel_near(r, fuel)
    }

    /// js `_collect(dt)`
    fn collect(&mut self, robots: &mut [Robot], mt: &Match, fuel: &Fuel, dt: f64) {
        let i = self.me;
        let b = self.brain;
        if self.ball.is_none_or(|x| fuel.balls[x].state != State::Field || fuel.balls[x].held) || self.rethink {
            let nb = self.pick_ball(robots, mt, fuel, false);
            if nb != self.ball { self.ball = nb; self.ball_best = 1e9; self.ball_progress_t = 0.0; }
        }
        let Some(bi) = self.ball else {
            self.label = "No FUEL in reach".into();
            let h = self.home(robots);
            self.drive(robots, h[0], h[1], 1.0, Face::Travel, 0.08, true);
            return;
        };
        let bp = fuel.balls[bi].pos;
        let own = self.own(robots);
        let d = m::hypot2(bp[0] - robots[i].pos[0], bp[2] - robots[i].pos[2]);
        let line_x = alliance_line_x(own);
        let inside = if own == Alliance::Blue { -1.0 } else { 1.0 };
        let hold = self.flow && robots[i].last_in_zone && !robots[i].stored.is_empty() && (mt.hub_active(own) || self.in_grace(mt, own)) && (bp[0] - line_x) * inside < 0.6;
        if hold || d < self.ball_best - 0.1 { self.ball_best = d; self.ball_progress_t = 0.0; }
        else { self.ball_progress_t += dt; if self.ball_progress_t > 2.5 { self.blacklist.insert(bi, self.t + 8.0); self.ball = None; return; } }
        let near = d < 1.6;
        if hold { self.drive(robots, line_x + inside * 0.6, clamp(bp[2], -HALF_W + 0.6, HALF_W - 0.6), b.collect_speed, Face::Travel, 0.08, true); }
        else { self.drive(robots, bp[0], bp[2], if near { b.collect_speed } else { 1.0 }, Face::Travel, -0.3, !near); }
        let it = self.intake_for(robots, fuel, d);
        let r = &mut robots[i];
        r.cmd.intake = it;
        self.label = if r.full { format!("Collecting ({}, full)", r.stored.len()) } else { format!("Collecting ({}/~{})", r.stored.len(), r.capacity()) };
    }

    /// js `_shotHold()`
    fn shot_hold(&self, r: &Robot) -> Hold {
        match &r.phys {
            Some(Phys::R1114(_)) => Hold::Secs(0.65),
            Some(Phys::R1678(_)) => Hold::Secs(1.0),
            Some(Phys::R1706(_)) => Hold::Never,
            Some(Phys::R581(_)) => Hold::Secs(0.7),
            Some(Phys::R6328(_)) => Hold::Secs(0.35),
            _ => if r.cfg["intake"]["compacts"].as_bool() == Some(true) { Hold::Secs(0.65) } else { Hold::NoCompact },
        }
    }

    /// js `_fuelAhead()`: loose FUEL on the carpet in front of the intake
    fn fuel_ahead(&self, r: &Robot, fuel: &Fuel) -> bool {
        let ic = &r.cfg["intake"];
        let f = r.forward();
        let v = m::max(0.0, r.vel[0] * f[0] + r.vel[2] * f[1]);
        let far = r.half_l + ic["reach"].as_f64().unwrap_or(0.0) + 0.3 + 0.5 * v;
        let w = ic["width"].as_f64().unwrap_or(0.0);
        for b in &fuel.balls {
            if b.state != State::Field || b.held || b.in_flight || b.pos[1] > 0.3 { continue; }
            if m::abs(b.pos[0] - r.pos[0]) > far + 0.5 || m::abs(b.pos[2] - r.pos[2]) > far + 0.5 { continue; }
            let l = r.world_to_local(b.pos[0], b.pos[2]);
            if l[0] > r.half_l - 0.1 && l[0] < far && m::abs(l[1]) < w / 2.0 + 0.05 { return true; }
        }
        false
    }

    /// js `_hubFuelNear()`: FUEL just released by a HUB about to come down into our intake (G408)
    fn hub_fuel_near(&self, r: &Robot, fuel: &Fuel) -> bool {
        let ic = &r.cfg["intake"];
        let (reach, w) = (ic["reach"].as_f64().unwrap_or(0.0), ic["width"].as_f64().unwrap_or(0.0));
        let g = 9.81;
        for b in &fuel.balls {
            if b.state != State::Field || !b.hub_fresh { continue; }
            if m::abs(b.pos[0] - r.pos[0]) > 2.5 || m::abs(b.pos[2] - r.pos[2]) > 2.5 { continue; }
            let v = b.prev_vel;
            let h = b.pos[1] - r.pos[1] - 0.33;
            let t = if h <= 0.0 { 0.0 } else { (v[1] + m::sqrt(v[1] * v[1] + 2.0 * g * h)) / g };
            let l = r.world_to_local(b.pos[0] + (v[0] - r.vel[0]) * t, b.pos[2] + (v[2] - r.vel[2]) * t);
            if l[0] > r.half_l - 0.2 && l[0] < r.half_l + reach + 0.35 && m::abs(l[1]) < w / 2.0 + 0.3 { return true; }
        }
        false
    }

    /// js `_defend(dt)`
    fn defend(&mut self, robots: &mut [Robot], mt: &Match, rules: &mut RobotRules, dt: f64) {
        let i = self.me;
        let f = self.foe.unwrap();
        let b = self.brain;
        let seen = self.seen[0];
        let (palliance, ppos) = (robots[f].alliance, [seen[1], seen[2]]);
        let pin = rules.pin_state(i, f);
        let gap = rules.gap(i, f);
        let mut away = [robots[i].pos[0] - robots[f].pos[0], robots[i].pos[2] - robots[f].pos[2]];
        let ad = { let h = m::hypot2(away[0], away[1]); if h == 0.0 { 1.0 } else { h } };
        away[0] /= ad; away[1] /= ad;
        self.cooldown = m::max(0.0, self.cooldown - dt);
        robots[i].cmd.intake = false;
        let rp = [robots[i].pos[0], robots[i].pos[2]];
        if !self.backoff && pin.t >= b.pin_limit + 3.0 * pin.fouls as f64 { self.backoff = true; }
        if self.backoff {
            if gap >= PIN_RESET + 0.1 { self.backoff = false; self.cooldown = 0.8; }
            let tx = clamp(rp[0] + away[0] * 2.5, -HALF_L + 0.6, HALF_L - 0.6);
            let tz = clamp(rp[1] + away[1] * 2.5, -HALF_W + 0.6, HALF_W - 0.6);
            self.drive(robots, tx, tz, 1.0, Face::None, 0.08, false);
            self.label = "Backing off (pin)".into();
            return;
        }
        let face_p = m::atan2(-(ppos[1] - rp[1]), ppos[0] - rp[0]);
        let face = Face::Yaw(if robots[i].cfg["intake"]["latched"].as_bool() == Some(true) { face_p + m::PI } else { face_p });
        let end_game = mt.phase_time >= TIMING::teleop - TIMING::endgame;
        if end_game && tower_protected(&robots[f].rules_view(), 1.0) {
            self.drive(robots, ppos[0] + away[0] * 2.2, ppos[1] + away[1] * 2.2, 1.0, face, 0.08, false);
            self.label = "Holding off (TOWER)".into();
            return;
        }
        if self.cooldown > 0.0 {
            self.drive(robots, rp[0] + away[0] * 0.5, rp[1] + away[1] * 0.5, 1.0, face, 0.08, false);
            self.label = "Resetting".into();
            return;
        }
        let line_x = alliance_line_x(palliance);
        let outside = if palliance == Alliance::Blue { ppos[0] - line_x } else { line_x - ppos[0] };
        let d = m::hypot2(ppos[0] - rp[0], ppos[1] - rp[1]);
        if outside <= b.engage || robots[f].last_in_zone {
            self.drive(robots, ppos[0], ppos[1], if d < 1.8 { b.push_speed } else { 1.0 }, face, -1.0, false);
            self.label = if pin.active { format!("Pinning {:.1} s", pin.t) } else { "Shoving".into() };
            return;
        }
        let hub_x = hub_center(palliance).0;
        let to_zone = if palliance == Alliance::Blue { -1.0 } else { 1.0 };
        let lanes = [-3.39, -1.52, 1.52, 3.39];
        let lane_z = lanes.iter().copied().reduce(|a, z| if m::abs(z - ppos[1]) < m::abs(a - ppos[1]) { z } else { a }).unwrap();
        let gate = [hub_x - to_zone * 1.5, lane_z];
        let (mut gx, mut gz) = (gate[0] - ppos[0], gate[1] - ppos[1]);
        let gd = { let h = m::hypot2(gx, gz); if h == 0.0 { 1.0 } else { h } };
        gx /= gd; gz /= gd;
        if d < b.ram_dist {
            let lead = m::min(0.5, d / 6.0);
            let (fp, fv) = (robots[f].pos, robots[f].vel);
            let (tx, tz) = (fp[0] + fv[0] * lead - gx * 0.6, fp[2] + fv[2] * lead - gz * 0.6);
            self.drive(robots, tx, tz, b.push_speed, face, -2.0, false);
            let contact = rules.in_contact(i, f);
            self.label = if pin.active { format!("Pinning {:.1} s", pin.t) } else if contact { "Driving it back".into() } else { "Ramming".into() };
        } else {
            let lead = m::min(b.block_lead, gd * 0.7);
            self.drive(robots, ppos[0] + gx * lead, ppos[1] + gz * lead, 1.0, face, 0.08, false);
            self.label = "Guarding its lane".into();
        }
    }
}
