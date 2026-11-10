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
    /// "skip": straight to TELEOP; "none": AUTO with the robot standing still (no routines yet)
    pub auto: String,
    pub fps: bool,
    /// "high", "medium" or "low"
    pub graphics: String,
    /// the starting camera (js cameras.js CAMERA_MODES)
    pub camera: String,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            robot: "2910".into(), alliance: "blue".into(), ds: 1, start: "rightTrench".into(), auto: "skip".into(),
            fps: false, graphics: "high".into(), camera: "driver".into(),
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
        if self.auto != "skip" && self.auto != "none" { self.auto = d.auto.clone(); }
        if !["high", "medium", "low"].contains(&self.graphics.as_str()) { self.graphics = d.graphics; }
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
    pub capacity: u32,
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
            let robots = order.iter().filter(|k| physical.contains(&k.as_str())).map(|k| {
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
                    archetype: s("archetype"), blurb: s("blurb"), stats, capacity, ported: sim::robot::ported(c),
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
