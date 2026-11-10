//! js/match.js: the match's flow and score (2026 Game Manual, Section 6).
use crate::constants::*;
use crate::rng::Mulberry32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase { Pre, Auto, AutoGap, Teleop, Post, Done }

impl Phase {
    pub fn parse(s: &str) -> Phase {
        match s { "pre" => Phase::Pre, "auto" => Phase::Auto, "autoGap" => Phase::AutoGap, "teleop" => Phase::Teleop, "post" => Phase::Post, "done" => Phase::Done, _ => panic!("no phase {s}") }
    }
}

#[derive(Debug, Clone, Default)]
pub struct Score { pub auto_fuel: f64, pub tele_fuel: f64, pub inactive_fuel: f64, pub auto_tower: f64, pub tele_tower: f64, pub foul_pts: f64, pub fouls: Vec<Foul> }

#[derive(Debug, Clone)]
pub struct Foul { pub t: f64, pub committed_by: Alliance, pub major: bool, pub rule: &'static str, pub desc: &'static str, pub pts: f64 }

/// What the JS game's onEvent hears (the HUD and sounds)
#[derive(Debug, Clone)]
pub enum Event { Phase(Phase), Shift { index: i32, active: bool }, GameData { first_inactive: Alliance, random: bool }, Score { alliance: Alliance, pts: f64 }, Inactive(Alliance), Foul(Foul), Tower { alliance: Alliance, pts: f64, level: u8, auto: bool } }

/// Who launched a FUEL (fuel.js b.launch)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaunchInfo { pub by: LaunchBy, pub alliance: Option<Alliance>, pub legal: bool, pub t: Option<f64> }
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaunchBy { Robot, Hp, Test }

pub const fn ai(a: Alliance) -> usize { match a { Alliance::Blue => 0, Alliance::Red => 1 } }
pub const ALLIANCES: [Alliance; 2] = [Alliance::Blue, Alliance::Red];

#[derive(Debug, Clone)]
pub struct Match {
    pub player: Alliance,
    pub phase: Phase,
    pub phase_time: f64,
    pub t: f64,
    pub first_inactive: Option<Alliance>,
    pub tiebreak_random: bool,
    pub score: [Score; 2],
    pub last_deactivate: [f64; 2],
    pub prev_active: [bool; 2],
    pub last_shift: i32,
    pub foul_log: Vec<Foul>,
    pub auto_tower_done: bool,
    pub events: Vec<Event>,
}

impl Match {
    pub fn new(player: Alliance) -> Self {
        Match { player, phase: Phase::Pre, phase_time: 0.0, t: 0.0, first_inactive: None, tiebreak_random: false, score: Default::default(),
            last_deactivate: [-1e9, -1e9], prev_active: [false, false], last_shift: -1, foul_log: Vec::new(), auto_tower_done: false, events: Vec::new() }
    }

    pub fn robot_enabled(&self) -> bool { self.phase == Phase::Auto || self.phase == Phase::Teleop }
    pub fn is_auto(&self) -> bool { self.phase == Phase::Auto }
    pub fn is_teleop(&self) -> bool { self.phase == Phase::Teleop }
    pub fn over(&self) -> bool { self.phase == Phase::Done }

    pub fn shift_index(&self) -> i32 {
        if self.phase != Phase::Teleop { return -1; }
        let tt = self.phase_time;
        if tt < TIMING::transition { return 0; }
        if tt >= TIMING::teleop - TIMING::endgame { return 5; }
        1 + jsmath::floor((tt - TIMING::transition) / TIMING::shift) as i32
    }

    pub fn hub_active(&self, a: Alliance) -> bool {
        match self.phase {
            Phase::Auto => true,
            Phase::Teleop => {
                let s = self.shift_index();
                if s == 0 || s == 5 { return true; }
                let odd = s % 2 == 1; // SHIFT 1, 3
                if odd { Some(a) != self.first_inactive } else { Some(a) == self.first_inactive }
            }
            _ => false,
        }
    }

    /// js `update(dt, robots)`; climb_levels: each robot's (alliance, climbLevel), for the TOWER
    pub fn update(&mut self, dt: f64, climb_levels: &[(Alliance, u8)], rng: &mut Mulberry32) {
        self.t += dt;
        self.phase_time += dt;
        let p = self.phase;
        if p == Phase::Pre && self.phase_time >= TIMING::preMatch { self.go(Phase::Auto); }
        else if p == Phase::Auto && self.phase_time >= TIMING::auto {
            for &(a, l) in climb_levels { self.assess_auto_tower(a, l); }
            self.go(Phase::AutoGap);
        } else if p == Phase::AutoGap && self.phase_time >= TIMING::autoGap {
            self.decide_shifts(rng);
            self.go(Phase::Teleop);
        } else if p == Phase::Teleop && self.phase_time >= TIMING::teleop {
            for &(a, l) in climb_levels { self.assess_teleop_tower(a, l); }
            self.go(Phase::Post);
        } else if p == Phase::Post && self.phase_time >= TIMING::post {
            self.go(Phase::Done);
        }
        for a in ALLIANCES {
            let act = self.hub_active(a);
            if self.prev_active[ai(a)] && !act { self.last_deactivate[ai(a)] = self.t; }
            self.prev_active[ai(a)] = act;
        }
        if self.phase == Phase::Teleop {
            let s = self.shift_index();
            if s != self.last_shift {
                self.last_shift = s;
                let active = self.hub_active(self.player);
                self.events.push(Event::Shift { index: s, active });
            }
        }
    }

    /// js `_go(phase)`
    pub fn go(&mut self, phase: Phase) {
        self.phase = phase;
        self.phase_time = 0.0;
        self.events.push(Event::Phase(phase));
    }

    fn decide_shifts(&mut self, rng: &mut Mulberry32) {
        let (b, r) = (self.score[0].auto_fuel, self.score[1].auto_fuel);
        if b > r { self.first_inactive = Some(Alliance::Blue); }
        else if r > b { self.first_inactive = Some(Alliance::Red); }
        else {
            self.first_inactive = Some(if rng.random() < 0.5 { Alliance::Blue } else { Alliance::Red });
            self.tiebreak_random = true;
        }
        self.events.push(Event::GameData { first_inactive: self.first_inactive.unwrap(), random: self.tiebreak_random });
    }

    /// js `fuelEnteredHub(a, ball, t)`: launch is the FUEL's b.launch
    pub fn fuel_entered_hub(&mut self, a: Alliance, launch: Option<LaunchInfo>) {
        // G407: a ROBOT may only launch FUEL into its HUB with its BUMPERS in its ALLIANCE ZONE
        if let Some(l) = launch {
            if l.by == LaunchBy::Robot && l.alliance == Some(a) && !l.legal {
                self.add_foul(a, true, "G407", "Launched FUEL into the HUB from outside the ALLIANCE ZONE");
            }
        }
        let active = self.hub_active(a);
        let grace = !active && self.t - self.last_deactivate[ai(a)] <= HUB::scoreGrace;
        let counts = active || grace;
        let auto_period = self.phase == Phase::Auto || (self.phase == Phase::AutoGap && grace);
        if self.phase == Phase::Pre || self.phase == Phase::Done { return; }
        let s = &mut self.score[ai(a)];
        if counts {
            if auto_period { s.auto_fuel += POINTS::fuel; } else { s.tele_fuel += POINTS::fuel; }
            self.events.push(Event::Score { alliance: a, pts: POINTS::fuel });
        } else {
            s.inactive_fuel += 1.0;
            self.events.push(Event::Inactive(a));
        }
    }

    /// js `addFoul(committedBy, type, rule, desc)` (major: type 'major')
    pub fn add_foul(&mut self, committed_by: Alliance, major: bool, rule: &'static str, desc: &'static str) {
        if self.phase == Phase::Pre || self.phase == Phase::Done { return; }
        let pts = if major { POINTS::majorFoul } else { POINTS::minorFoul };
        self.score[ai(other(committed_by))].foul_pts += pts;
        let f = Foul { t: self.t, committed_by, major, rule, desc, pts };
        self.score[ai(committed_by)].fouls.push(f.clone());
        self.foul_log.push(f.clone());
        self.events.push(Event::Foul(f));
    }

    fn assess_auto_tower(&mut self, a: Alliance, level: u8) {
        if level >= 1 {
            self.score[ai(a)].auto_tower += POINTS::towerAutoL1;
            self.events.push(Event::Tower { alliance: a, pts: POINTS::towerAutoL1, level: 1, auto: true });
        }
    }
    fn assess_teleop_tower(&mut self, a: Alliance, level: u8) {
        if level >= 1 {
            let pts = POINTS::tower[level as usize];
            self.score[ai(a)].tele_tower += pts;
            self.events.push(Event::Tower { alliance: a, pts, level, auto: false });
        }
    }

    pub fn fuel_points(&self, a: Alliance) -> f64 { let s = &self.score[ai(a)]; s.auto_fuel + s.tele_fuel }
    pub fn total(&self, a: Alliance) -> f64 { let s = &self.score[ai(a)]; s.auto_fuel + s.tele_fuel + s.auto_tower + s.tele_tower + s.foul_pts }
}
