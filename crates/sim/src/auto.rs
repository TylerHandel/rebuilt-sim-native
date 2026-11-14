//! js/auto.js: the AUTO routines. G402: drive teams may not control the ROBOT during AUTO, so each
//! robot runs a pre-programmed routine: a built-in one, a plan found for it (BEST_AUTOS), or its
//! team's own (js/teamAutos.js, run as js/customAutos.js's steps; 6328's live: auto6328.rs).
//! Waypoints are in BLUE field coordinates (fx from the blue ALLIANCE WALL, fy from the blue
//! drivers' right), mirrored for the red ALLIANCE and for left-side starts.
use crate::constants::*;
use crate::fuel::{clamp, Fuel};
use crate::nav::under_trench;
use crate::robot::{fits_trench, wrap_angle, ClimbState, Robot};
use crate::rules::{hold_off_foes, RobotView};
use jsmath as m;
use serde_json::Value;
use std::sync::OnceLock;

/// robots.json: startPositions, bestAutos, teamAutos
pub fn data() -> &'static Value {
    static D: OnceLock<Value> = OnceLock::new();
    D.get_or_init(|| serde_json::from_str(include_str!("../../../assets/robots.json")).unwrap())
}

pub const START_ORDER: [&str; 5] = ["leftTrench", "leftBump", "hub", "rightBump", "rightTrench"];
/// js START_POSITIONS[key].fy
pub fn start_fy(key: &str) -> f64 {
    match key {
        "leftTrench" => FIELD_W - 0.64,
        "leftBump" => HUB::fy + HUB::size / 2.0 + 0.93,
        "hub" => HUB::fy,
        "rightBump" => HUB::fy - HUB::size / 2.0 - 0.93,
        "rightTrench" => 0.64,
        _ => panic!("no start {key}"),
    }
}
pub fn start_name(key: &str) -> &'static str {
    match key { "leftTrench" => "Left Trench", "leftBump" => "Left Bump", "hub" => "Hub", "rightBump" => "Right Bump", "rightTrench" => "Right Trench", _ => "?" }
}

/// js `startPose(key, robot, alliance, custom)`: BUMPERS overlapping the ROBOT STARTING LINE; a team's
/// own AUTO where theirs starts, facing as theirs does
pub fn start_pose(key: &str, r: &Robot, alliance: Alliance, custom: Option<(&Value, bool)>) -> [f64; 3] {
    let mut fx = ALLIANCE_ZONE_DEPTH - 0.02 - r.half_l;
    let (mut yaw, fy);
    yaw = 0.0;
    if let Some((a, mirror)) = custom {
        let sf = num(a, "startFy");
        let f = if mirror { FIELD_W - sf } else { sf };
        fy = clamp(f, r.half_w + 0.02, FIELD_W - r.half_w - 0.02);
        if let Some(x) = a["startFx"].as_f64() { fx = clamp(x, ALLIANCE_ZONE_DEPTH - r.half_l - 0.02, ALLIANCE_ZONE_DEPTH + r.half_l - 0.05); }
        if let Some(y) = a["startYaw"].as_f64() { yaw = ((if mirror { -1.0 } else { 1.0 }) * y * m::PI) / 180.0; }
    } else { fy = start_fy(key); }
    let (x, z) = fw(fx, fy, alliance);
    [x, z, wrap_angle(yaw + (if alliance == Alliance::Blue { 0.0 } else { m::PI }))]
}

fn num(v: &Value, k: &str) -> f64 { v[k].as_f64().unwrap_or_else(|| panic!("auto: no number {k}")) }

/// js `yawOf(deg, alliance)`: a heading in blue field coordinates (deg) as the robot's yaw
fn yaw_of(deg: f64, a: Alliance) -> f64 { wrap_angle((deg * m::PI) / 180.0 + (if a == Alliance::Blue { 0.0 } else { m::PI })) }

/// js `nearestSpot(fy)`
pub fn nearest_spot(fy: f64) -> &'static str {
    let mut a = START_ORDER[0];
    for k in START_ORDER { if m::abs(start_fy(k) - fy) < m::abs(start_fy(a) - fy) { a = k; } }
    a
}

/// js `teamAutoFor(robotKey, key, avoid)`: the team's own AUTO from the side nearest the spot
pub fn team_auto_for(robot: &str, key: &str, avoid: &[&str]) -> Option<&'static Value> {
    let list = data()["teamAutos"].get(robot)?.as_array()?;
    let want = if START_ORDER.contains(&key) { start_fy(key) } else { FIELD_W / 2.0 };
    let mut free: Vec<&Value> = list.iter().filter(|a| !avoid.contains(&nearest_spot(num(a, "startFy")))).collect();
    // (a stable sort, as Array.prototype.sort is)
    free.sort_by(|a, b| (m::abs(num(a, "startFy") - want)).partial_cmp(&m::abs(num(b, "startFy") - want)).unwrap());
    free.first().copied()
}

/// One BEST_AUTOS trip
#[derive(Clone, Debug)]
pub struct Trip { pub depot: bool, pub out: String, pub fx: f64, pub a: f64, pub b: f64, pub speed: Option<f64>, pub home: String, pub shoot_at: [f64; 2] }
/// A BEST_AUTOS plan
#[derive(Clone, Debug)]
pub struct Plan { pub start: String, pub preload: String, pub trips: Vec<Trip> }

/// js BEST_AUTOS[key]
pub fn best_auto(key: &str) -> Option<Plan> {
    let p = data()["bestAutos"].get(key)?;
    let trips = p["trips"].as_array()?.iter().map(|t| {
        if t["depot"].as_bool() == Some(true) { return Trip { depot: true, out: String::new(), fx: 0.0, a: 0.0, b: 0.0, speed: t["speed"].as_f64(), home: String::new(), shoot_at: [0.0; 2] }; }
        Trip { depot: false, out: t["out"].as_str().unwrap().into(), fx: num(t, "fx"), a: num(t, "a"), b: num(t, "b"), speed: t["speed"].as_f64(), home: t["home"].as_str().unwrap().into(), shoot_at: [t["shootAt"][0].as_f64().unwrap(), t["shootAt"][1].as_f64().unwrap()] }
    }).collect();
    Some(Plan { start: p["start"].as_str()?.into(), preload: p["preload"].as_str().unwrap_or("").into(), trips })
}

fn flip_start(k: &str) -> &'static str {
    match k { "leftTrench" => "rightTrench", "rightTrench" => "leftTrench", "leftBump" => "rightBump", "rightBump" => "leftBump", _ => "hub" }
}
/// js `mirrorPlan(plan)`: the same plan from the other side (the DEPOT runs dropped: one DEPOT)
pub fn mirror_plan(p: &Plan) -> Plan {
    Plan { start: flip_start(&p.start).into(), preload: p.preload.clone(), trips: p.trips.iter().filter(|t| !t.depot).map(|t| Trip { a: FIELD_W - t.a, b: FIELD_W - t.b, shoot_at: [t.shoot_at[0], FIELD_W - t.shoot_at[1]], ..t.clone() }).collect() }
}

/// One robot's AUTO choice in a match (js matchEntries' auto, start, plan)
#[derive(Clone, Debug)]
pub struct Entry { pub robot: String, pub auto: String, pub start: String, pub plan: Option<Plan> }
/// What assignStarts gives each: where it starts, the plan it runs, the team AUTO it runs
#[derive(Clone, Debug)]
pub struct Assigned { pub start: String, pub plan: Option<Plan>, pub team: Option<&'static Value> }

/// js `assignStarts(entries)`: one ALLIANCE's robots' starting spots (driver station order), so no two
/// start on top of each other
pub fn assign_starts(entries: &[Entry]) -> Vec<Assigned> {
    let mut taken: Vec<&'static str> = Vec::new();
    let mut out: Vec<Option<Assigned>> = vec![None; entries.len()];
    for (i, e) in entries.iter().enumerate() {
        if e.auto != "best" || e.plan.is_some() { continue; }
        if let Some(team) = team_auto_for(&e.robot, &e.start, &taken) {
            let s = nearest_spot(num(team, "startFy"));
            taken.push(s);
            out[i] = Some(Assigned { start: s.into(), plan: None, team: Some(team) });
        }
    }
    for (i, e) in entries.iter().enumerate() {
        if out[i].is_some() { continue; }
        let plan = if e.auto == "best" { e.plan.clone().or_else(|| best_auto(&e.robot)) } else { None };
        let tries: Vec<(String, Option<Plan>)> = match &plan {
            Some(p) => vec![(p.start.clone(), Some(p.clone())), (flip_start(&p.start).into(), Some(mirror_plan(p)))],
            None => vec![(e.start.clone(), None)],
        };
        let pick = match tries.iter().find(|(k, _)| !taken.contains(&k.as_str())) {
            Some(t) => t.clone(),
            None => {
                let want = start_fy(&tries[0].0);
                let mut free: Vec<&str> = START_ORDER.iter().copied().filter(|k| !taken.contains(k)).collect();
                free.sort_by(|a, b| m::abs(start_fy(a) - want).partial_cmp(&m::abs(start_fy(b) - want)).unwrap());
                (free.first().map(|s| s.to_string()).unwrap_or(tries[0].0.clone()), plan.clone())
            }
        };
        taken.push(START_ORDER.iter().copied().find(|k| *k == pick.0).unwrap_or("hub"));
        out[i] = Some(Assigned { start: pick.0, plan: pick.1, team: None });
    }
    out.into_iter().map(|o| o.unwrap()).collect()
}

/// A waypoint: (fx, fy), its speed (a fraction of the top speed) and, a team AUTO's, whether its
/// flywheel runs on the way there
#[derive(Clone, Copy, Debug, Default)]
pub struct Pt { pub fx: f64, pub fy: f64, pub speed: Option<f64>, pub spin: Option<bool> }
/// A drive step's shooting: no, yes, or once back in our ALLIANCE ZONE
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Shoot { #[default] Off, On, Hub }
/// A timed point's dt (s from the one before), heading (deg, blue) and speed (m/s)
#[derive(Clone, Copy, Debug, Default)]
pub struct Tp { pub dt: f64, pub rot: Option<f64>, pub v: f64 }
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Kind { #[default] Shoot, Wait, Climb, Drive, Dtp, Team6328 }

/// One step of a routine (js: a plain object; the fields its type uses)
#[derive(Clone, Debug, Default)]
pub struct Step {
    pub kind: Kind,
    pub timeout: Option<f64>,
    pub hold: bool,
    pub intake: bool,
    pub rot: Option<f64>,
    pub gains: [f64; 2],
    pub aim: Option<f64>,
    pub spin_pts: bool,
    pub t: f64,
    pub until: Option<f64>,
    pub budget: Option<f64>,
    pub requested: bool,
    pub pts: Vec<Pt>,
    pub shoot: Shoot,
    pub speed: Option<f64>,
    /// js s.mirror !== false
    pub mirror: bool,
    pub face: Option<f64>,
    pub timed: bool,
    pub tp: Vec<Tp>,
    pub flow: bool,
    pub pos_lag: f64,
    pub fx: f64,
    pub fy: f64,
    pub vmax: f64,
    pub tol: f64,
    pub limit: Option<f64>,
    /// 6328's program (js s.program) and its side
    pub program: Option<Value>,
    pub prog_mirror: bool,
}
impl Step {
    fn shoot(timeout: f64) -> Self { Step { kind: Kind::Shoot, timeout: Some(timeout), mirror: true, ..Default::default() } }
    fn drive(pts: &[[f64; 2]], intake: bool, shoot: Shoot, speed: f64) -> Self {
        Step { kind: Kind::Drive, pts: pts.iter().map(|p| Pt { fx: p[0], fy: p[1], ..Default::default() }).collect(), intake, shoot, speed: Some(speed), mirror: true, ..Default::default() }
    }
}

/// js customAutos.js `waitTime(action, t)`
fn wait_time(action: &str, t: Option<f64>) -> f64 {
    if action == "wait" { return t.unwrap_or(1.0); }
    if let Some(n) = action.strip_prefix("wait") { if let Ok(v) = n.parse::<f64>() { return v; } }
    if action == "stop" { 0.05 } else { 0.0 }
}

/// js customAutos.js `customSteps(a, mirror, maxSpeed)`: a team's (or the editor's) AUTO as steps
pub fn custom_steps(a: &Value, mirror: bool, max_speed: f64) -> Vec<Step> {
    let fy = |y: f64| if mirror { FIELD_W - y } else { y };
    let rot = |r: Option<f64>| r.map(|r| if mirror { -r } else { r });
    let timed = a["timed"].as_bool() == Some(true);
    let gains = a["gains"].as_array().map_or([5.0, 5.0], |g| [g[0].as_f64().unwrap(), g[1].as_f64().unwrap()]);
    let spin_pts = a["spinPts"].as_bool() == Some(true);
    let mut steps = Vec::new();
    match &a["shootPreload"] { Value::Bool(true) => steps.push(Step::shoot(3.0)), Value::Number(n) => steps.push(Step::shoot(n.as_f64().unwrap())), _ => {} }
    let mut cur: Option<Step> = None;
    let mut per_point = false;
    let (mut px, mut py) = (a["startFx"].as_f64().unwrap_or(ALLIANCE_ZONE_DEPTH - 0.4), num(a, "startFy"));
    let flush = |cur: &mut Option<Step>, steps: &mut Vec<Step>| { if let Some(c) = cur.take() { steps.push(c); } };
    for p in a["points"].as_array().unwrap() {
        let action = p["action"].as_str().unwrap_or("none");
        let ptt = p["t"].as_f64();
        let intake = p["intake"].as_bool() == Some(true);
        if let Some(st) = p.get("step").filter(|s| s.is_object()) {
            flush(&mut cur, &mut steps);
            assert_eq!(st["type"].as_str(), Some("team6328"), "a team AUTO's own step: only 6328's");
            steps.push(Step { kind: Kind::Team6328, program: Some(st["program"].clone()), prog_mirror: st["mirror"].as_bool() == Some(true), mirror: true, ..Default::default() });
            px = num(p, "fx"); py = num(p, "fy");
            continue;
        }
        if let Some(d) = p.get("dtp").filter(|s| s.is_object()) {
            flush(&mut cur, &mut steps);
            let face = rot(p["rot"].as_f64()).map(|r| (r * m::PI) / 180.0);
            steps.push(Step { kind: Kind::Dtp, fx: num(p, "fx"), fy: fy(num(p, "fy")), vmax: m::min(num(p, "v"), max_speed), tol: num(d, "tol"), limit: d["limit"].as_f64(), intake, face, mirror: true, ..Default::default() });
            px = num(p, "fx"); py = num(p, "fy");
            if action == "shoot" { steps.push(Step::shoot(ptt.unwrap_or(3.0))); }
            else if action == "until" { steps.push(Step { kind: Kind::Wait, until: ptt, intake, mirror: true, ..Default::default() }); }
            else if action != "none" { steps.push(Step { kind: Kind::Wait, t: wait_time(action, ptt), intake, mirror: true, ..Default::default() }); }
            continue;
        }
        let shoot = match p["shoot"].as_str() { Some("hub") => Shoot::Hub, Some("always") => Shoot::On, _ => Shoot::Off };
        let has_v = if timed { p["v"].as_f64().is_some() } else { p["v"].as_f64().is_some_and(|v| v != 0.0) };
        let speed = if has_v { m::min(1.0, num(p, "v") / max_speed) } else { p["speed"].as_f64().unwrap_or(1.0) };
        let same = cur.as_ref().is_some_and(|c| c.intake == intake && c.shoot == shoot && (if has_v { per_point } else { c.speed == Some(speed) }));
        if !same {
            flush(&mut cur, &mut steps);
            let mut c = Step { kind: Kind::Drive, intake, shoot, speed: Some(speed), mirror: false, timeout: Some(3.0), ..Default::default() };
            per_point = has_v;
            if timed { c.timed = true; c.gains = gains; }
            if a["flow"].as_bool() == Some(true) { c.flow = true; }
            if spin_pts { c.spin_pts = true; }
            if let Some(l) = a["posLag"].as_f64() { c.pos_lag = l; }
            cur = Some(c);
        }
        let c = cur.as_mut().unwrap();
        let (fxp, fyp) = (num(p, "fx"), num(p, "fy"));
        c.pts.push(if has_v { Pt { fx: fxp, fy: fy(fyp), speed: Some(speed), spin: if spin_pts { Some(p["spin"].as_bool() == Some(true)) } else { None } } } else { Pt { fx: fxp, fy: fy(fyp), ..Default::default() } });
        let ds = m::hypot2(fxp - px, fyp - py);
        *c.timeout.as_mut().unwrap() += (2.0 * ds) / m::max(0.5, max_speed * speed);
        if c.timed { c.tp.push(Tp { dt: p["dt"].as_f64().unwrap_or(0.0), rot: rot(p["rot"].as_f64()), v: p["v"].as_f64().unwrap_or(0.0) }); }
        px = fxp; py = fyp;
        let hold_rot = if timed { rot(p["rot"].as_f64()) } else { None };
        let with_hold = |mut s: Step| { if let Some(r) = hold_rot { s.rot = Some(r); s.gains = gains; } s };
        if action == "shoot" {
            flush(&mut cur, &mut steps);
            if p["hold"].as_bool() == Some(true) {
                let mut s = with_hold(Step { hold: true, intake, ..Step::shoot(ptt.unwrap_or(3.0)) });
                if let Some(aim) = p["aim"].as_f64() { s.aim = Some(aim); }
                if spin_pts { s.spin_pts = true; }
                steps.push(s);
            } else { steps.push(with_hold(Step::shoot(ptt.unwrap_or(3.0)))); }
        } else if action == "until" { flush(&mut cur, &mut steps); steps.push(with_hold(Step { kind: Kind::Wait, until: ptt, intake, mirror: true, ..Default::default() })); }
        else if action != "none" { flush(&mut cur, &mut steps); steps.push(with_hold(Step { kind: Kind::Wait, t: wait_time(action, ptt), intake, mirror: true, ..Default::default() })); }
    }
    flush(&mut cur, &mut steps);
    steps
}

/// Lanes to and from the NEUTRAL ZONE on the right side (low fy); mirrored for the left
fn route_fy(kind: &str) -> f64 { if kind == "trench" { 0.64 } else { HUB::fy - HUB::size / 2.0 - BUMP::width / 2.0 } }

/// js AutoRunner._chain: a run of timed steps as one trajectory
#[derive(Clone, Debug)]
struct Chain { i0: usize, t0: f64, c: Vec<[f64; 2]>, t: Vec<f64>, v: Vec<f64>, r: Vec<f64>, ends: Vec<usize>, k: usize }

/// What an AUTO step sees of the match besides its robot: the opponents (for G415), the other robots
/// and the FUEL (6328's cameras)
pub struct Ctx<'a> { pub foes: &'a [RobotView], pub others: &'a [RobotView], pub fuel: &'a Fuel }

/// js `AutoRunner`
#[derive(Clone, Debug)]
pub struct AutoRunner {
    pub alliance: Alliance,
    pub plan: Option<Plan>,
    pub left: bool,
    pub steps: Vec<Step>,
    pub i: usize,
    pub step_t: f64,
    pub done: bool,
    pub elapsed: f64,
    pi: Option<usize>,
    seg_t: f64,
    seg_budget: Option<f64>,
    lag: f64,
    aim_t: Option<f64>,
    dtp_v: Option<[f64; 2]>,
    dtp_i: Option<usize>,
    full_hold: Option<usize>,
    ta: Option<(usize, usize, bool)>,
    chain: Option<Chain>,
    /// js setpoint: where a timed step should be by now (x, z, vx, vz, yaw)
    pub setpoint: Option<[f64; 5]>,
    /// 6328's program state (auto6328.rs)
    pub prog: Option<crate::auto6328::Prog>,
    /// its routine's margin from the walls (js P's m)
    margin: f64,
}

impl AutoRunner {
    /// js `new AutoRunner(robot, routine, startKey, alliance, custom, plan)`
    pub fn new(r: &Robot, routine: &str, start_key: &str, alliance: Alliance, custom: Option<(&Value, bool)>, plan: Option<Plan>) -> Self {
        let plan = plan.or_else(|| if routine == "best" { best_auto(&r.key) } else { None });
        let left = custom.is_none() && plan.is_none() && start_fy(start_key) > FIELD_W / 2.0 + 0.01;
        let reach = r.cfg["intake"]["reach"].as_f64().unwrap_or(0.0);
        let margin = m::hypot2(r.half_l + reach, r.half_w * 0.5) + 0.03;
        let mut a = AutoRunner { alliance, plan, left, steps: Vec::new(), i: 0, step_t: 0.0, done: false, elapsed: 0.0, pi: None, seg_t: 0.0, seg_budget: None, lag: 0.0, aim_t: None,
            dtp_v: None, dtp_i: None, full_hold: None, ta: None, chain: None, setpoint: None, prog: None, margin };
        a.steps = match custom { Some((c, mirror)) => custom_steps(c, mirror, r.max_speed), None => a.build(r, routine) };
        a
    }

    /// js `P(fx, fy, sideMirror)`: a waypoint on the field, mirrored for a left start, kept far enough
    /// from the walls to turn there with its intake out
    fn p(&self, fx: f64, fy: f64, side_mirror: bool) -> [f64; 2] {
        let m_ = self.margin;
        let y = if side_mirror && self.left { FIELD_W - fy } else { fy };
        let (x, z) = fw(clamp(fx, m_, 2.0 * HALF_L - m_), clamp(y, m_, FIELD_W - m_), self.alliance);
        [x, z]
    }

    fn build(&self, r: &Robot, routine: &str) -> Vec<Step> {
        let small = r.max_capacity() < 20.0;
        let hl = r.half_l;
        let mut steps = Vec::new();
        let fits = fits_trench(&r.cfg);
        let sweep = |steps: &mut Vec<Step>, fx_line: f64, back: bool| {
            let out = if fits { route_fy("trench") } else { route_fy("bump") };
            steps.push(Step::drive(&[[3.2, out], [5.6, out], [fx_line - 0.2, 1.25]], true, Shoot::Off, 1.0));
            steps.push(Step::drive(&[[fx_line, 1.45], [fx_line, 4.4]], true, if small { Shoot::On } else { Shoot::Off }, if small { 0.75 } else { 0.55 }));
            steps.push(Step::drive(&[[6.3, 3.0], [5.2, 2.55], [3.3, 2.4]], small, if small { Shoot::On } else { Shoot::Hub }, if back { 0.9 } else { 1.0 }));
            steps.push(Step::shoot(2.5));
        };
        if routine == "best" { if let Some(p) = &self.plan { return self.plan_steps(r, p); } }
        let routine = if routine == "best" { "sweep" } else { routine };
        match routine {
            "preload" => steps.push(Step::shoot(3.0)),
            "depot" => {
                steps.push(Step::shoot(3.0));
                let dfy = DEPOT::fy;
                steps.push(Step { mirror: false, face: Some(m::PI), ..Step::drive(&[[2.4, 5.2], [1.5, dfy]], true, Shoot::Off, 1.0) });
                let dx = hl + num(&r.cfg["intake"], "reach") + 0.04;
                steps.push(Step { mirror: false, face: Some(m::PI), ..Step::drive(&[[dx, dfy - 0.3]], true, Shoot::Off, 0.5) });
                steps.push(Step { mirror: false, face: Some(m::PI), ..Step::drive(&[[dx, dfy + 0.3]], true, Shoot::Off, 0.3) });
                steps.push(Step { mirror: false, ..Step::drive(&[[2.6, 5.0]], true, Shoot::Hub, 0.8) });
                steps.push(Step::shoot(4.0));
            }
            "sweep" => { steps.push(Step::shoot(3.0)); sweep(&mut steps, 7.55, false); }
            "sweep2" => { steps.push(Step::shoot(3.0)); sweep(&mut steps, 7.55, false); sweep(&mut steps, 7.85, true); }
            "climb" => { steps.push(Step::shoot(3.0)); steps.push(Step { kind: Kind::Climb, mirror: true, ..Default::default() }); }
            "defendStage" => {
                steps.push(Step::shoot(3.0));
                steps.push(Step::drive(&[[3.3, 2.5], [5.9, 2.5], [6.9, 4.0]], false, Shoot::Off, 0.9));
            }
            _ => {}
        }
        steps
    }

    /// js `_planSteps(plan)`
    fn plan_steps(&self, r: &Robot, plan: &Plan) -> Vec<Step> {
        let small = r.max_capacity() < 20.0;
        let bps = num(&r.cfg["shooter"], "bps");
        let max_fx = HALF_L + r.half_w - 0.08;
        let fits = fits_trench(&r.cfg);
        let lane = |kind: &str, fy: f64| { let k = if fits { kind } else { "bump" }; if fy > FIELD_W / 2.0 { FIELD_W - route_fy(k) } else { route_fy(k) } };
        let shoot_all = r.max_capacity() / bps + 1.2;
        let mut steps = Vec::new();
        if plan.preload == "stand" { steps.push(Step::shoot(1.5)); }
        let mut first = plan.preload != "stand";
        for trip in &plan.trips {
            if trip.depot {
                let (dfy, dx) = (DEPOT::fy, r.half_l + num(&r.cfg["intake"], "reach") + 0.04);
                steps.push(Step::drive(&[[2.4, dfy - 1.0], [dx + 0.3, dfy - 0.7]], true, Shoot::Hub, 1.0));
                steps.push(Step { face: Some(m::PI / 2.0), ..Step::drive(&[[dx, dfy - 0.45], [dx, dfy + 0.5]], true, Shoot::Hub, trip.speed.unwrap_or(0.35)) });
                steps.push(Step::drive(&[[2.2, dfy]], true, Shoot::Hub, 0.6));
                if !small { steps.push(Step::shoot(shoot_all)); }
                first = false;
                continue;
            }
            let fx = m::min(trip.fx, max_fx);
            let (out_y, home_y) = (lane(&trip.out, trip.a), lane(&trip.home, trip.b));
            steps.push(Step::drive(&[[3.1, out_y], [5.9, out_y], [fx - 0.35, trip.a]], true, if first { Shoot::Hub } else { Shoot::Off }, 1.0));
            first = false;
            steps.push(Step { speed: trip.speed, ..Step::drive(&[[fx, trip.a], [fx, trip.b]], true, Shoot::Off, 0.0) });
            steps.push(Step::drive(&[[6.0, home_y], [3.2, home_y], trip.shoot_at], small, Shoot::Hub, 1.0));
            steps.push(Step::shoot(shoot_all));
        }
        steps
    }

    /// js `_trenchAhead()`: whether the route passes under a TRENCH arm before the robot next empties
    fn trench_ahead(&mut self, r: &Robot) -> bool {
        let pi = if self.steps.get(self.i).is_some_and(|s| s.kind == Kind::Drive) { self.pi.unwrap_or(0) } else { 0 };
        if let Some((i, p, v)) = self.ta { if i == self.i && p == pi { return v; } }
        let mut under = false;
        let mut k = self.i;
        while k < self.steps.len() && !under {
            let s = &self.steps[k];
            if s.kind == Kind::Shoot { break; }
            if s.kind != Kind::Drive { k += 1; continue; }
            let mut pts: Vec<[f64; 2]> = s.pts.iter().map(|q| self.p(q.fx, q.fy, s.mirror)).collect();
            if k == self.i { let mut v = vec![[r.pos[0], r.pos[2]]]; v.extend_from_slice(&pts[pi.min(pts.len())..]); pts = v; }
            let mut j = 0;
            while j + 1 < pts.len() && !under {
                let (a, b) = (pts[j], pts[j + 1]);
                let n = m::ceil(m::hypot2(b[0] - a[0], b[1] - a[1]) / 0.25) as i64;
                let mut q = 0;
                while q <= n && !under { under = under_trench(a[0] + ((b[0] - a[0]) * q as f64) / n as f64, a[1] + ((b[1] - a[1]) * q as f64) / n as f64, 0.0); q += 1; }
                j += 1;
            }
            k += 1;
        }
        self.ta = Some((self.i, pi, under));
        under
    }

    fn next(&mut self) { self.i += 1; self.step_t = 0.0; self.pi = Some(0); self.seg_t = 0.0; self.seg_budget = None; self.lag = 0.0; self.aim_t = None; }

    /// js `update(dt)`: this step's command for the robot
    pub fn update(&mut self, r: &mut Robot, ctx: &Ctx, dt: f64) {
        let fold = r.cfg["intake"]["fold"].is_object();
        r.cmd = crate::parity::Cmd { lower: fold && fits_trench(&r.cfg) && under_trench(r.pos[0], r.pos[2], r.half_l + 1.2), ..Default::default() };
        self.elapsed += dt;
        if self.done || self.i >= self.steps.len() { self.done = true; return; }
        self.step_t += dt;
        let max_omega = r.max_omega;
        {
            let s = &self.steps[self.i];
            if (s.kind == Kind::Shoot || s.kind == Kind::Wait) && s.rot.is_some() {
                r.cmd.omega = clamp(s.gains[1] * wrap_angle(yaw_of(s.rot.unwrap(), self.alliance) - r.yaw), -max_omega, max_omega);
            }
        }
        let kind = self.steps[self.i].kind;
        match kind {
            Kind::Team6328 => { crate::auto6328::step(self, r, ctx, dt); }
            Kind::Dtp => self.dtp(r, dt),
            Kind::Shoot => self.shoot_step(r),
            Kind::Wait => {
                let s = &self.steps[self.i];
                r.cmd.intake = s.intake;
                let over = match s.until { Some(u) => self.elapsed >= u, None => self.step_t >= s.t };
                if over { self.next(); }
            }
            Kind::Climb => self.climb(r),
            Kind::Drive if self.steps[self.i].timed => self.timed(r),
            Kind::Drive => self.drive(r, ctx, dt),
        }
    }

    /// on to the next step (6328's program, done)
    pub(crate) fn advance(&mut self) { self.next(); }

    fn dtp(&mut self, r: &mut Robot, dt: f64) {
        let s = self.steps[self.i].clone();
        let (tx, tz) = fw(s.fx, clamp(s.fy, r.half_w + 0.02, FIELD_W - r.half_w - 0.02), self.alliance);
        let (dx, dz) = (tx - r.pos[0], tz - r.pos[2]);
        let d = m::hypot2(dx, dz);
        if self.step_t <= dt { self.seg_budget = Some(2.0 * (d / s.vmax + s.vmax / 6.0) + 1.0); }
        let sp = m::min(3.0 * d, s.vmax);
        let want = if d > 1e-3 { [(dx / d) * sp, (dz / d) * sp] } else { [0.0, 0.0] };
        if self.dtp_v.is_none() || (self.dtp_i != Some(self.i.wrapping_sub(1)) && self.dtp_i != Some(self.i)) { self.dtp_v = Some([r.vel[0], r.vel[2]]); }
        let v = self.dtp_v.as_mut().unwrap();
        let (ex, ez) = (want[0] - v[0], want[1] - v[1]);
        let e = m::hypot2(ex, ez);
        let k = if e > 6.0 * dt { (6.0 * dt) / e } else { 1.0 };
        v[0] += ex * k; v[1] += ez * k;
        self.dtp_i = Some(self.i);
        r.cmd.vx = v[0]; r.cmd.vz = v[1];
        if let Some(f) = s.face { r.cmd.omega = clamp(5.0 * wrap_angle((if self.alliance == Alliance::Blue { f } else { f + m::PI }) - r.yaw), -r.max_omega, r.max_omega); }
        r.cmd.intake = s.intake;
        if d < s.tol || s.limit.is_some_and(|l| self.step_t >= l) || self.step_t >= self.seg_budget.unwrap_or(f64::NAN) { self.next(); }
    }

    fn shoot_step(&mut self, r: &mut Robot) {
        let s = self.steps[self.i].clone();
        if s.spin_pts { r.cmd.spin = Some(true); r.cmd.prespin = true; }
        if let (Some(aim), None) = (s.aim, self.aim_t) {
            let (hx, hz) = fw(HUB::fx, HUB::fy, self.alliance);
            let err = r.aim_err.unwrap_or_else(|| wrap_angle(m::atan2(-(hz - r.pos[2]), hx - r.pos[0]) - r.yaw - r.aim_offset));
            r.cmd.omega = clamp(5.0 * err, -r.max_omega, r.max_omega);
            if (m::abs(err) < (2.75 * m::PI) / 180.0 && m::hypot2(r.vel[0], r.vel[2]) < 0.05) || self.step_t >= aim { self.aim_t = Some(self.step_t); self.step_t = 0.0; }
            return;
        }
        r.cmd.shoot = true;
        if r.status.contains("range") { r.cmd.vx = (if self.alliance == Alliance::Blue { -1.0 } else { 1.0 }) * 1.2; }
        if s.intake { r.cmd.intake = true; }
        if (r.stored.is_empty() && !s.hold) || self.step_t > s.timeout.unwrap() { self.next(); }
    }

    fn climb(&mut self, r: &mut Robot) {
        if r.climb_state != ClimbState::None || self.steps[self.i].requested { return; }
        let p = r.climb_pose();
        let (dx, dz) = (p[0] - r.pos[0], p[2] - r.pos[2]);
        let d = m::hypot2(dx, dz);
        let budget = *self.steps[self.i].budget.get_or_insert((2.0 * d) / (r.max_speed * 0.8) + 2.0);
        if self.step_t > budget { self.next(); return; }
        if d > 1.0 {
            let sp = m::min(r.max_speed * 0.8, d * 2.0);
            r.cmd.vx = (dx / d) * sp; r.cmd.vz = (dz / d) * sp;
            r.cmd.omega = clamp(4.0 * wrap_angle(p[2] - r.yaw), -5.0, 5.0);
        } else {
            r.request_climb();
            self.steps[self.i].requested = true;
        }
    }

    /// Under a TRENCH later on this route: the load kept under the arm (a drive step's and a timed
    /// one's). Returns true when it stopped to spit FUEL out (the step's command done).
    fn keep_under_arm(&mut self, r: &mut Robot, dir: Option<[f64; 2]>) -> bool {
        if !(fits_trench(&r.cfg) && self.trench_ahead(r)) { return false; }
        if r.load_near_trench() { self.full_hold = Some(self.i); }
        if self.full_hold == Some(self.i) { r.cmd.intake = false; }
        let big = m::max(r.half_l, r.half_w);
        if r.deploy_raises_top() && under_trench(r.pos[0], r.pos[2], big + 2.0) { r.cmd.intake = false; }
        let ahead = dir.is_some_and(|[ux, uz]| [0.4, 0.8, 1.2].iter().any(|k| under_trench(r.pos[0] + ux * k, r.pos[2] + uz * k, big)));
        if ahead && !r.fits_trench_lowered() { self.full_hold = Some(self.i); r.cmd.vx = 0.0; r.cmd.vz = 0.0; r.cmd.intake = false; r.cmd.outtake = true; return true; }
        false
    }

    fn drive(&mut self, r: &mut Robot, ctx: &Ctx, dt: f64) {
        let s = self.steps[self.i].clone();
        let pi = *self.pi.get_or_insert(0);
        self.seg_t += dt;
        let accel = r.cfg["drive"]["maxAccel"].as_f64().filter(|a| *a != 0.0).unwrap_or(6.0);
        if self.seg_budget.is_none() {
            let t0 = self.p(s.pts[pi].fx, s.pts[pi].fy, s.mirror);
            let l = m::hypot2(t0[0] - r.pos[0], t0[1] - r.pos[2]);
            let v = m::max(0.3, r.max_speed * s.pts[pi].speed.or(s.speed).unwrap_or(1.0));
            let a = m::min(6.0, accel);
            self.seg_budget = Some(1.5 * (if l < (v * v) / a { 2.0 * m::sqrt(l / a) } else { l / v + v / a }) + 1.0);
        }
        let mut pts: Vec<[f64; 2]> = s.pts.iter().map(|q| self.p(q.fx, q.fy, s.mirror)).collect();
        let n = pts.len();
        let mut after = self.i + 1;
        while s.flow && self.steps.get(after).is_some_and(|x| x.kind == Kind::Drive && x.flow) {
            let st = &self.steps[after];
            pts.extend(st.pts.iter().map(|q| self.p(q.fx, q.fy, st.mirror)));
            after += 1;
        }
        let flows = pts.len() > n;
        let tgt = pts[pi];
        let last = pi == pts.len() - 1;
        let (dx, dz) = (tgt[0] - r.pos[0], tgt[1] - r.pos[2]);
        let d = m::hypot2(dx, dz);
        let mut remain = d;
        for k in pi..pts.len() - 1 { remain += m::hypot2(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]); }
        let vmax = r.max_speed * s.pts[pi.min(n - 1)].speed.or(s.speed).unwrap_or(1.0);
        let mut sp = m::min(vmax, m::sqrt(2.0 * 6.0 * m::max(0.0, remain - 0.02)) + 0.05);
        if !last {
            let (nx, nz) = (pts[pi + 1][0] - tgt[0], pts[pi + 1][1] - tgt[1]);
            let nl = m::hypot2(nx, nz);
            if nl > 1e-3 && d > 1e-3 {
                let turn = m::acos(clamp((dx * nx + dz * nz) / (d * nl), -1.0, 1.0));
                let vc = vmax * m::max(0.2, 1.0 - (0.5 * turn) / (m::PI / 2.0));
                sp = m::min(sp, m::sqrt(vc * vc + 2.0 * 6.0 * m::max(0.0, d - 0.35)));
            }
        }
        let moving = m::hypot2(r.vel[0], r.vel[2]);
        self.lag = clamp(self.lag + (sp - moving) * dt - (if moving > 0.8 * sp { 0.5 * dt } else { 0.0 }), 0.0, 0.6);
        sp = m::min(r.max_speed, sp + 3.0 * self.lag);
        if d > 1e-3 { r.cmd.vx = (dx / d) * sp; r.cmd.vz = (dz / d) * sp; }
        let me = r.rules_view();
        let mut c = (r.cmd.vx, r.cmd.vz);
        hold_off_foes(&me, ctx.foes, &mut c, 0.3);
        r.cmd.vx = c.0; r.cmd.vz = c.1;
        let mut face = match s.face { Some(f) => if self.alliance == Alliance::Blue { f } else { f + m::PI }, None => m::atan2(-dz, dx) };
        if self.left && s.face.is_none() { face = m::atan2(-dz, dx); }
        let fixed = r.cfg["shooter"]["type"].as_str() == Some("fixed");
        let aim_ahead = s.face.is_none() && fixed && self.steps.get(after).is_some_and(|x| x.kind == Kind::Shoot) && remain < 2.5;
        if aim_ahead { let (hx, hz) = fw(HUB::fx, HUB::fy, self.alliance); face = m::atan2(-(hz - r.pos[2]), hx - r.pos[0]) - r.aim_offset; }
        if d > 0.15 || s.face.is_some() || aim_ahead { r.cmd.omega = clamp(5.0 * wrap_angle(face - r.yaw), -r.max_omega, r.max_omega); }
        r.cmd.intake = s.intake;
        let dir = if d > 1e-3 { Some([dx / d, dz / d]) } else { None };
        if self.keep_under_arm(r, dir) { return; }
        r.cmd.shoot = match s.shoot { Shoot::Hub => r.last_in_zone, Shoot::On => true, Shoot::Off => false };
        if s.spin_pts { let sp = s.pts[pi.min(n - 1)].spin == Some(true); r.cmd.spin = Some(sp); r.cmd.prespin = sp; }
        let late = self.seg_t > self.seg_budget.unwrap();
        if (last && d < 0.1) || (!last && d < 0.35) || late {
            if last { self.next(); }
            else if flows && pi == n - 1 { self.next(); }
            else { self.pi = Some(pi + 1); self.seg_t = 0.0; self.seg_budget = None; }
        }
        if self.step_t > s.timeout.unwrap_or(8.0) { self.next(); }
    }

    /// js `_chain()`
    fn make_chain(&self, r: &Robot) -> Chain {
        let mut ch = Chain { i0: self.i, t0: self.elapsed, c: vec![[r.pos[0], r.pos[2]]], t: vec![0.0], v: vec![m::hypot2(r.vel[0], r.vel[2])], r: vec![r.yaw], ends: Vec::new(), k: 0 };
        for k in self.i..self.steps.len() {
            let st = &self.steps[k];
            if st.kind != Kind::Drive || !st.timed { break; }
            for (q, p) in st.pts.iter().enumerate() {
                let tp = st.tp[q];
                ch.c.push(self.p(p.fx, p.fy, st.mirror));
                let tl = *ch.t.last().unwrap();
                ch.t.push(tl + tp.dt);
                ch.v.push(tp.v);
                let rl = *ch.r.last().unwrap();
                ch.r.push(match tp.rot { None => rl, Some(rot) => yaw_of(rot, self.alliance) });
            }
            ch.ends.push(ch.c.len() - 1);
            if !st.flow { break; }
        }
        ch
    }

    /// js `_timed(s, next)`: a team's trajectory run by the clock
    fn timed(&mut self, r: &mut Robot) {
        let need = match &self.chain { Some(c) => self.i < c.i0 || self.i >= c.i0 + c.ends.len(), None => true };
        if need { self.chain = Some(self.make_chain(r)); }
        let tau = self.elapsed - self.chain.as_ref().unwrap().t0;
        loop {
            let c = self.chain.as_ref().unwrap();
            if !(tau >= c.t[c.ends[self.i - c.i0]]) { break; }
            self.next();
            let c = self.chain.as_ref().unwrap();
            if self.i >= c.i0 + c.ends.len() { self.chain = None; self.setpoint = None; return; }
        }
        let st = self.steps[self.i].clone();
        let c = self.chain.as_mut().unwrap();
        let j = self.i - c.i0;
        while c.k + 2 < c.t.len() && c.t[c.k + 1] <= tau { c.k += 1; }
        let k = c.k;
        let h = c.t[k + 1] - c.t[k];
        let (w, f) = along(c, k, tau);
        let (v0, v1) = (c.v[k], c.v[k + 1]);
        let vm = (v0 + v1) / 2.0;
        let (a, b) = (c.c[k], c.c[k + 1]);
        let l = m::hypot2(b[0] - a[0], b[1] - a[1]);
        let v = if h > 1e-6 { if vm > 1e-6 { ((l / h) * (v0 + (v1 - v0) * w)) / vm } else { l / h } } else { 0.0 };
        let (ux, uz) = if l > 1e-6 { ((b[0] - a[0]) / l, (b[1] - a[1]) / l) } else { (0.0, 0.0) };
        let [kp, kr] = st.gains;
        let turn = wrap_angle(c.r[k + 1] - c.r[k]);
        let mut sp0 = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, ux * v, uz * v, wrap_angle(c.r[k] + turn * w)];
        if st.pos_lag != 0.0 && tau > st.pos_lag {
            let t2 = tau - st.pos_lag;
            let mut k2 = k;
            while k2 > 0 && c.t[k2] > t2 { k2 -= 1; }
            let (_, pf) = along(c, k2, t2);
            let (a2, b2) = (c.c[k2], c.c[k2 + 1]);
            sp0[0] = a2[0] + (b2[0] - a2[0]) * pf; sp0[1] = a2[1] + (b2[1] - a2[1]) * pf;
        }
        let first = if j == 0 { 1 } else { c.ends[j - 1] + 1 };
        self.setpoint = Some(sp0);
        let (mut vx, mut vz) = (sp0[2] + kp * (sp0[0] - r.pos[0]), sp0[3] + kp * (sp0[1] - r.pos[2]));
        let spd = m::hypot2(vx, vz);
        if spd > r.max_speed { vx *= r.max_speed / spd; vz *= r.max_speed / spd; }
        r.cmd.vx = vx; r.cmd.vz = vz;
        r.cmd.omega = clamp((if h > 1e-6 { turn / h } else { 0.0 }) + kr * wrap_angle(sp0[4] - r.yaw), -r.max_omega, r.max_omega);
        let pi = clamp((k + 1) as f64 - first as f64, 0.0, (st.pts.len() - 1) as f64) as usize;
        self.pi = Some(pi);
        r.cmd.intake = st.intake;
        let mm = m::hypot2(vx, vz);
        let dir = if mm > 0.05 { Some([vx / mm, vz / mm]) } else { None };
        if self.keep_under_arm(r, dir) { return; }
        r.cmd.shoot = match st.shoot { Shoot::Hub => r.last_in_zone, Shoot::On => true, Shoot::Off => false };
        if st.spin_pts { let sp = st.pts[pi].spin == Some(true); r.cmd.spin = Some(sp); r.cmd.prespin = sp; }
    }
}

/// js `along(c, k, tau)`: (w, how far through the stretch's time; f, how far along its length)
fn along(c: &Chain, k: usize, tau: f64) -> (f64, f64) {
    let h = c.t[k + 1] - c.t[k];
    let w = if h > 1e-6 { clamp((tau - c.t[k]) / h, 0.0, 1.0) } else { 1.0 };
    let (v0, v1) = (c.v[k], c.v[k + 1]);
    let vm = (v0 + v1) / 2.0;
    (w, if vm > 1e-6 { (v0 * w + 0.5 * (v1 - v0) * w * w) / vm } else { w })
}
