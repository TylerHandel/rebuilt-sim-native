//! The player's settings (js ui.js DEFAULT_SETTINGS, the ones this version uses), kept between runs
//! in settings.json in the user's app data folder, and the robots the menus offer.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Resource, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub robot: String,
    /// "blue" or "red"
    pub alliance: String,
    /// js DRIVER_STATIONS index
    pub ds: usize,
    /// js START_POSITIONS key
    pub start: String,
    /// "best": its AUTO runs (its team's own, or its best plan); "drive": you drive it through AUTO
    /// (practice); "skip": straight to TELEOP; "none": AUTO with the robot standing still
    pub auto: String,
    /// "practice", "1v1" or "3v3" (js matchMode)
    pub mode: String,
    /// 1 v 1: the AI opponent's robot, strategy ("scorer", "defense", "hybrid") and skill
    pub opp_robot: String,
    pub opponent: String,
    pub opp_skill: String,
    /// 3 v 3: the six slots, blue 1-3 then red 1-3 (js slots)
    pub slots: Vec<Slot>,
    pub fps: bool,
    /// "high", "medium" or "low"
    pub graphics: String,
    /// the starting camera (js cameras.js CAMERA_MODES)
    pub camera: String,
    /// your HUMAN PLAYER: "auto" throws whenever your HUB is active, "manual" when you say (js hp)
    pub hp: String,
    /// motion effects: "off", "trails" (behind shot FUEL), "blur" (motion blur) or "both"
    pub motion: String,
    /// where the match is played: "arena" (indoors) or "stadium" (outdoors: sky and weather)
    pub venue: String,
    /// "day", "sunset" (indoors: evening, the house lights half down) or "night" (the house lights
    /// down, the stage lights up)
    pub time: String,
    /// outdoors: "clear", "cloudy", "rain" or "fog"
    pub weather: String,
    /// volumetric fog: the lights' beams in the air ("off", "low", "high")
    pub haze: String,
    /// the film look: filmic tone mapping, a touch of lens fringing, film grain, eyes that adapt
    pub film: bool,
    /// NVIDIA DLSS (where the build has it and the GPU's an RTX): "off", "dlaa" (full resolution,
    /// DLSS's anti-aliasing only), "quality", "balanced" or "performance" (fewer pixels drawn, then
    /// upscaled); takes effect on restart, as it runs on Vulkan
    pub upscaler: String,
}

/// One 3 v 3 slot (js DEFAULT_SLOTS[i]): its robot, who drives it ("you", "scorer", "defense",
/// "hybrid", "empty"), the start its AUTO asks for, and an AI's skill
#[derive(Clone, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Slot { pub robot: String, pub driver: String, pub start: String, pub skill: String }
impl Default for Slot { fn default() -> Self { Slot { robot: "2910".into(), driver: "scorer".into(), start: "hub".into(), skill: "champs".into() } } }
/// js DEFAULT_SLOTS
pub fn default_slots() -> Vec<Slot> {
    let s = |robot: &str, driver: &str, start: &str| Slot { robot: robot.into(), driver: driver.into(), start: start.into(), skill: "champs".into() };
    vec![s("2910", "you", "leftTrench"), s("4414", "scorer", "hub"), s("8793", "defense", "rightBump"), s("1678", "scorer", "leftTrench"), s("971", "scorer", "hub"), s("1690", "hybrid", "rightBump")]
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            robot: "2910".into(), alliance: "blue".into(), ds: 1, start: "rightTrench".into(), auto: "best".into(),
            mode: "practice".into(), opp_robot: "4414".into(), opponent: "scorer".into(), opp_skill: "regional".into(), slots: default_slots(),
            fps: false, graphics: "high".into(), camera: "driver".into(), hp: "auto".into(), motion: "off".into(),
            venue: "arena".into(), time: "day".into(), weather: "clear".into(), haze: "off".into(), film: false, upscaler: "off".into(),
        }
    }
}

fn path() -> Option<PathBuf> {
    let base = std::env::var_os("APPDATA").map(|p| PathBuf::from(p).join("RebuiltSim")).or_else(|| {
        std::env::var_os("XDG_CONFIG_HOME").map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
            .map(|p| p.join("rebuilt-sim"))
    })?;
    Some(base.join("settings.json"))
}

impl Settings {
    pub fn load() -> Self {
        let s: Settings = path().and_then(|p| std::fs::read_to_string(p).ok()).and_then(|t| serde_json::from_str(&t).ok()).unwrap_or_default();
        s.valid()
    }

    /// what was saved, with anything this version doesn't know put back to its default
    fn valid(mut self) -> Self {
        let d = Settings::default();
        let r = Roster::get();
        if !r.robots.iter().any(|x| x.key == self.robot) { self.robot = d.robot.clone(); }
        if self.alliance != "blue" && self.alliance != "red" { self.alliance = d.alliance.clone(); }
        if self.ds > 2 { self.ds = d.ds; }
        if !r.starts.iter().any(|(k, _, _)| *k == self.start) { self.start = d.start.clone(); }
        if !["best", "drive", "skip", "none"].contains(&self.auto.as_str()) { self.auto = d.auto.clone(); }
        if !["practice", "1v1", "3v3"].contains(&self.mode.as_str()) { self.mode = d.mode.clone(); }
        if !r.robots.iter().any(|x| x.key == self.opp_robot) { self.opp_robot = d.opp_robot.clone(); }
        if !sim::ai::STRATEGIES.contains(&self.opponent.as_str()) { self.opponent = d.opponent.clone(); }
        if !sim::ai::SKILL_ORDER.contains(&self.opp_skill.as_str()) { self.opp_skill = d.opp_skill.clone(); }
        if self.slots.len() != 6 { self.slots = d.slots.clone(); }
        for (sl, ds) in self.slots.iter_mut().zip(d.slots.iter()) {
            if !r.robots.iter().any(|x| x.key == sl.robot) { sl.robot = ds.robot.clone(); }
            if !["you", "scorer", "defense", "hybrid", "empty"].contains(&sl.driver.as_str()) { sl.driver = ds.driver.clone(); }
            if !r.starts.iter().any(|(k, _, _)| *k == sl.start) { sl.start = ds.start.clone(); }
            if !sim::ai::SKILL_ORDER.contains(&sl.skill.as_str()) { sl.skill = ds.skill.clone(); }
        }
        // (one slot at most is you)
        let mut you = false;
        for sl in &mut self.slots { if sl.driver == "you" { if you { sl.driver = "scorer".into(); } you = true; } }
        if !["ultra", "high", "medium", "low"].contains(&self.graphics.as_str()) { self.graphics = d.graphics; }
        if !["off", "trails", "blur", "both"].contains(&self.motion.as_str()) { self.motion = d.motion.clone(); }
        if self.hp != "auto" && self.hp != "manual" { self.hp = d.hp.clone(); }
        if !["arena", "stadium"].contains(&self.venue.as_str()) { self.venue = d.venue.clone(); }
        if !["day", "sunset", "night"].contains(&self.time.as_str()) { self.time = d.time.clone(); }
        if !["clear", "cloudy", "rain", "fog"].contains(&self.weather.as_str()) { self.weather = d.weather.clone(); }
        if !["off", "low", "high"].contains(&self.haze.as_str()) { self.haze = d.haze.clone(); }
        if !["off", "dlaa", "quality", "balanced", "performance"].contains(&self.upscaler.as_str()) || !cfg!(feature = "dlss") { self.upscaler = d.upscaler.clone(); }
        if crate::camera::CamMode::from_key(&self.camera).key() != self.camera { self.camera = d.camera; }
        self
    }

    pub fn save(&self) {
        let Some(p) = path() else { return };
        if let Some(dir) = p.parent() { let _ = std::fs::create_dir_all(dir); }
        if let Ok(t) = serde_json::to_string_pretty(self) { let _ = std::fs::write(p, t); }
    }

    pub fn alliance(&self) -> sim::constants::Alliance {
        if self.alliance == "red" { sim::constants::Alliance::Red } else { sim::constants::Alliance::Blue }
    }
}

/// One robot as the menus show it (js robotConfigs ROBOTS[k], its modeled capacity)
pub struct RobotInfo {
    pub key: String,
    pub team: u32,
    pub team_name: String,
    pub robot_name: String,
    pub archetype: String,
    pub blurb: String,
    /// (name, value): "Capacity" and the config's stats
    pub stats: Vec<(String, String)>,
    /// whether this version's sim can build it yet
    pub ported: bool,
}

impl RobotInfo {
    /// js ui.js robotName(k)
    pub fn short(&self) -> String { format!("{} {}", self.team, self.archetype) }
    /// the picker card's name line
    pub fn long_name(&self) -> String {
        if self.robot_name.is_empty() { self.team_name.clone() }
        else if self.robot_name.starts_with(&self.team_name) { self.robot_name.clone() }
        else { format!("{} · {}", self.team_name, self.robot_name) }
    }
}

/// The robots this version offers (the physical ones, in the game's order) and the starting spots
pub struct Roster {
    pub robots: Vec<RobotInfo>,
    /// (key, name, fy)
    pub starts: Vec<(String, String, f64)>,
}

impl Roster {
    pub fn get() -> &'static Roster {
        static R: std::sync::OnceLock<Roster> = std::sync::OnceLock::new();
        R.get_or_init(|| {
            let all: serde_json::Value = serde_json::from_str(include_str!("../../../assets/robots.json")).expect("robots.json");
            let physical: Vec<&str> = all["physical"].as_array().unwrap().iter().filter_map(|x| x.as_str()).collect();
            let mut order: Vec<String> = all["order"].as_array().unwrap().iter().filter_map(|x| x.as_str().map(String::from)).collect();
            for k in all["robots"].as_object().unwrap().keys() { if !order.contains(k) { order.push(k.clone()); } }
            // (the physical robots, and the generic ones whose hoppers hold their FUEL: sim generic.rs)
            let robots = order.iter().filter(|k| physical.contains(&k.as_str()) || sim::robot::GENERIC.contains(&k.as_str())).map(|k| {
                let c = &all["robots"][k];
                let s = |f: &str| c[f].as_str().unwrap_or("").to_string();
                let capacity = all["capacity"][k].as_u64().unwrap_or(0) as u32;
                let mut stats = vec![("Capacity".to_string(), capacity.to_string())];
                if let Some(o) = c["stats"].as_object() {
                    // (in robotConfigs.js's order: this serde_json keeps object keys sorted)
                    const ORDER: [&str; 5] = ["Shot rate", "Aiming", "Top speed", "Trench", "Climb"];
                    let mut keys: Vec<&String> = o.keys().collect();
                    keys.sort_by_key(|k| ORDER.iter().position(|x| x == k).unwrap_or(ORDER.len()));
                    for a in keys { let b = &o[a]; stats.push((a.clone(), b.as_str().map(String::from).unwrap_or_else(|| b.to_string()))); }
                }
                RobotInfo {
                    key: k.clone(), team: c["team"].as_u64().unwrap_or(0) as u32, team_name: s("teamName"), robot_name: s("robotName"),
                    archetype: s("archetype"), blurb: s("blurb"), stats, ported: sim::robot::ported(c),
                }
            }).collect();
            let starts = all["startOrder"].as_array().unwrap().iter().filter_map(|k| k.as_str()).map(|k| {
                let p = &all["startPositions"][k];
                (k.to_string(), p["name"].as_str().unwrap_or(k).to_string(), p["fy"].as_f64().unwrap_or(0.0))
            }).collect();
            Roster { robots, starts }
        })
    }

    pub fn robot(&self, key: &str) -> &RobotInfo { self.robots.iter().find(|r| r.key == key).unwrap_or(&self.robots[0]) }
}

/// js auto.js startPose(key, robot, alliance) for a built-in spot: BUMPERS overlapping the ROBOT
/// STARTING LINE, facing the NEUTRAL ZONE. (x, z, yaw)
pub fn start_pose(robot: &str, start: &str, alliance: sim::constants::Alliance) -> [f64; 3] {
    use sim::constants::*;
    let all: serde_json::Value = serde_json::from_str(include_str!("../../../assets/robots.json")).expect("robots.json");
    let half_l = all["robots"][robot]["frame"]["length"].as_f64().unwrap_or(0.7) / 2.0 + BUMPER_T;
    let fy = Roster::get().starts.iter().find(|(k, _, _)| k == start).map(|(_, _, fy)| *fy).unwrap_or(FIELD_W / 2.0);
    let fx = ALLIANCE_ZONE_DEPTH - 0.02 - half_l;
    let (x, z) = fw(fx, fy, alliance);
    // (js wrapAngle(yaw + PI) on red)
    [x, z, if alliance == Alliance::Blue { 0.0 } else { std::f64::consts::PI }]
}
