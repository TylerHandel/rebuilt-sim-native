//! The robot ratings (docs/CAMPAIGN.md: Difficulty): every robot played headless by the Champs AI,
//!   - a 1 v 1 round robin, every robot against every other from both sides (Bradley-Terry
//!     strengths on the Elo scale, and what each scores);
//!   - 3 v 3 matches of mixed alliances, each robot in about as many as the others (its OPR, as FRC
//!     computes it: the least-squares share of its alliances' scores; AUTO, FUEL and TOWER apart);
//! written to assets/ratings.json. Each match's result is kept as it finishes
//! (target/ratings/*.jsonl), so a run picks up where the last one stopped.
//!   cargo run --release -p sim --example ratings -- [--seeds 1] [--opr 51] [--threads 4] [--only 1v1|3v3|fit]
use sim::constants::Alliance;
use sim::game_match::ai;
use sim::headless::{play, Entry};
use std::collections::{HashMap, HashSet, VecDeque};
use std::io::{BufRead, Write};
use std::sync::{Arc, Mutex};

const SKILL: &str = "champs";

#[derive(Clone, Debug)]
struct Job { kind: &'static str, id: String, blue: Vec<String>, red: Vec<String>, seed: u32 }

fn arg(name: &str, def: &str) -> String {
    let a: Vec<String> = std::env::args().collect();
    a.iter().position(|x| x == name).and_then(|i| a.get(i + 1).cloned()).unwrap_or(def.into())
}

fn root() -> std::path::PathBuf { std::path::PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")) }

fn robots() -> Vec<String> {
    let d = sim::auto::data();
    d["order"].as_array().unwrap().iter().map(|k| k.as_str().unwrap().to_string()).collect()
}

/// a small seeded shuffle (mulberry32, as the game's)
fn shuffle<T>(v: &mut [T], rng: &mut sim::rng::Mulberry32) {
    for i in (1..v.len()).rev() { let j = (rng.random() * (i + 1) as f64) as usize; v.swap(i, j); }
}

fn jobs(seeds: u32, opr: usize) -> Vec<Job> {
    let rs = robots();
    let mut out = Vec::new();
    for s in 0..seeds {
        for (i, a) in rs.iter().enumerate() {
            for (j, b) in rs.iter().enumerate() {
                if i == j { continue; }
                out.push(Job { kind: "1v1", id: format!("1v1-{a}-{b}-{s}"), blue: vec![a.clone()], red: vec![b.clone()], seed: 1000 + (i * 31 + j) as u32 * 7 + s * 7919 });
            }
        }
    }
    // 3 v 3: each match the six who've played least (ties broken at random), split at random
    let mut rng = sim::rng::Mulberry32::new(2026);
    let mut played: HashMap<String, usize> = rs.iter().map(|r| (r.clone(), 0)).collect();
    for k in 0..opr {
        let mut pool = rs.clone();
        shuffle(&mut pool, &mut rng);
        pool.sort_by_key(|r| played[r]);
        let mut six: Vec<String> = pool[..6].to_vec();
        shuffle(&mut six, &mut rng);
        for r in &six { *played.get_mut(r).unwrap() += 1; }
        out.push(Job { kind: "3v3", id: format!("3v3-{k}"), blue: six[..3].to_vec(), red: six[3..].to_vec(), seed: 50_000 + k as u32 });
    }
    out
}

fn raw_path(kind: &str) -> std::path::PathBuf { root().join("target/ratings").join(format!("{kind}.jsonl")) }

fn done(kind: &str) -> Vec<serde_json::Value> {
    let Ok(f) = std::fs::File::open(raw_path(kind)) else { return Vec::new() };
    std::io::BufReader::new(f).lines().filter_map(|l| serde_json::from_str(&l.ok()?).ok()).collect()
}

fn run(all: Vec<Job>, threads: usize) {
    std::fs::create_dir_all(root().join("target/ratings")).unwrap();
    let have: HashSet<String> = ["1v1", "3v3"].iter().flat_map(|k| done(k)).map(|v| v["id"].as_str().unwrap().to_string()).collect();
    let todo: VecDeque<Job> = all.into_iter().filter(|j| !have.contains(&j.id)).collect();
    let n = todo.len();
    eprintln!("{n} matches to play on {threads} threads");
    let q = Arc::new(Mutex::new(todo));
    let files: Arc<Mutex<HashMap<&'static str, std::fs::File>>> = Arc::new(Mutex::new(HashMap::new()));
    let count = Arc::new(Mutex::new(0usize));
    let hs: Vec<_> = (0..threads).map(|_| {
        let (q, files, count) = (q.clone(), files.clone(), count.clone());
        std::thread::spawn(move || loop {
            let Some(j) = q.lock().unwrap().pop_front() else { break };
            let mut es: Vec<Entry> = j.blue.iter().map(|r| Entry::ai(r, Alliance::Blue, "scorer", SKILL)).collect();
            es.extend(j.red.iter().map(|r| Entry::ai(r, Alliance::Red, "scorer", SKILL)));
            let r = play(&es, j.seed, true, 8);
            let s = |a: Alliance| { let x = &r.score[ai(a)]; serde_json::json!({ "total": r.total(a), "autoFuel": x.auto_fuel, "teleFuel": x.tele_fuel, "autoTower": x.auto_tower, "teleTower": x.tele_tower, "fouls": x.foul_pts }) };
            let line = serde_json::json!({ "id": j.id, "kind": j.kind, "seed": j.seed, "blue": j.blue, "red": j.red, "blueScore": s(Alliance::Blue), "redScore": s(Alliance::Red), "robots": r.robots, "wall": r.wall_s });
            let mut fs = files.lock().unwrap();
            let f = fs.entry(j.kind).or_insert_with(|| std::fs::OpenOptions::new().create(true).append(true).open(raw_path(j.kind)).unwrap());
            writeln!(f, "{line}").unwrap();
            let mut c = count.lock().unwrap();
            *c += 1;
            eprintln!("[{}/{n}] {} {:?} {} - {} {:?} ({:.0} s)", *c, j.id, j.blue, r.totals[0], r.totals[1], j.red, r.wall_s);
        })
    }).collect();
    for h in hs { h.join().unwrap(); }
}

/// Solve A x = b (Gaussian elimination with partial pivoting)
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Vec<f64> {
    let n = b.len();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().partial_cmp(&a[j][c].abs()).unwrap()).unwrap();
        a.swap(c, p); b.swap(c, p);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            for k in c..n { a[r][k] -= f * a[c][k]; }
            b[r] -= f * b[c];
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() { x[r] = (b[r] - (r + 1..n).map(|k| a[r][k] * x[k]).sum::<f64>()) / a[r][r]; }
    x
}

fn mean(v: &[f64]) -> f64 { if v.is_empty() { 0.0 } else { v.iter().sum::<f64>() / v.len() as f64 } }
fn sd(v: &[f64]) -> f64 { let m = mean(v); (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / (v.len().max(2) - 1) as f64).sqrt() }
fn r1(x: f64) -> f64 { (x * 10.0).round() / 10.0 }

fn fit() {
    let rs = robots();
    let idx: HashMap<&str, usize> = rs.iter().enumerate().map(|(i, r)| (r.as_str(), i)).collect();
    let n = rs.len();
    // ---- 1 v 1: Bradley-Terry (a tie half a win each), by its MM iteration; what each scores
    let m1 = done("1v1");
    let mut wins = vec![vec![0.0; n]; n];
    let mut games = vec![vec![0.0; n]; n];
    let mut pts: Vec<Vec<f64>> = vec![Vec::new(); n];
    let mut part: Vec<[Vec<f64>; 4]> = (0..n).map(|_| Default::default()).collect();
    let mut won = vec![0.0; n];
    for m in &m1 {
        let (b, r) = (idx[m["blue"][0].as_str().unwrap()], idx[m["red"][0].as_str().unwrap()]);
        let (sb, sr) = (m["blueScore"]["total"].as_f64().unwrap(), m["redScore"]["total"].as_f64().unwrap());
        let wb = if sb > sr { 1.0 } else if sb < sr { 0.0 } else { 0.5 };
        wins[b][r] += wb; wins[r][b] += 1.0 - wb; games[b][r] += 1.0; games[r][b] += 1.0;
        won[b] += wb; won[r] += 1.0 - wb;
        for (k, key) in [(b, "blueScore"), (r, "redScore")] {
            let s = &m[key];
            pts[k].push(s["total"].as_f64().unwrap());
            part[k][0].push(s["autoFuel"].as_f64().unwrap());
            part[k][1].push(s["teleFuel"].as_f64().unwrap());
            part[k][2].push(s["autoTower"].as_f64().unwrap() + s["teleTower"].as_f64().unwrap());
            part[k][3].push(s["fouls"].as_f64().unwrap());
        }
    }
    let mut p = vec![1.0; n];
    for _ in 0..500 {
        let mut q = vec![0.0; n];
        for i in 0..n {
            let w: f64 = (0..n).map(|j| wins[i][j]).sum::<f64>() + 0.5; // (a half win of prior, so no one's 0)
            let d: f64 = (0..n).filter(|&j| j != i).map(|j| (games[i][j] + 1.0 / (n as f64 - 1.0)) / (p[i] + p[j])).sum();
            q[i] = w / d;
        }
        let g = q.iter().map(|x| x.ln()).sum::<f64>() / n as f64;
        p = q.iter().map(|x| (x.ln() - g).exp()).collect();
    }
    // ---- 3 v 3: OPR (least squares, a robot's share of its alliances' scores)
    let m3 = done("3v3");
    let opr_of = |field: &dyn Fn(&serde_json::Value) -> f64| -> Vec<f64> {
        let mut ata = vec![vec![0.0; n]; n];
        let mut atb = vec![0.0; n];
        for m in &m3 {
            for (side, key) in [("blue", "blueScore"), ("red", "redScore")] {
                let ids: Vec<usize> = m[side].as_array().unwrap().iter().map(|x| idx[x.as_str().unwrap()]).collect();
                let y = field(&m[key]);
                for &i in &ids { for &j in &ids { ata[i][j] += 1.0; } atb[i] += y; }
            }
        }
        // (a touch of ridge so a robot in too few matches doesn't run away)
        for i in 0..n { ata[i][i] += 0.05; }
        solve(ata, atb)
    };
    let (opr, opr_auto, opr_fuel, opr_tower) = if m3.is_empty() { (vec![0.0; n], vec![0.0; n], vec![0.0; n], vec![0.0; n]) } else {(
        opr_of(&|s| s["total"].as_f64().unwrap() - s["fouls"].as_f64().unwrap()),
        opr_of(&|s| s["autoFuel"].as_f64().unwrap() + s["autoTower"].as_f64().unwrap()),
        opr_of(&|s| s["autoFuel"].as_f64().unwrap() + s["teleFuel"].as_f64().unwrap()),
        opr_of(&|s| s["autoTower"].as_f64().unwrap() + s["teleTower"].as_f64().unwrap()),
    )};
    // the residual spread: how far an alliance's score falls from its OPRs' sum (the luck the event
    // engine adds to a decided match)
    let mut res = Vec::new();
    let mut apps = vec![0usize; n];
    for m in &m3 {
        for (side, key) in [("blue", "blueScore"), ("red", "redScore")] {
            let ids: Vec<usize> = m[side].as_array().unwrap().iter().map(|x| idx[x.as_str().unwrap()]).collect();
            for &i in &ids { apps[i] += 1; }
            let s = &m[key];
            res.push(s["total"].as_f64().unwrap() - s["fouls"].as_f64().unwrap() - ids.iter().map(|&i| opr[i]).sum::<f64>());
        }
    }
    let mut out = serde_json::Map::new();
    for (i, r) in rs.iter().enumerate() {
        let games1: f64 = (0..n).map(|j| games[i][j]).sum();
        out.insert(r.clone(), serde_json::json!({
            "elo": (1500.0 + 400.0 * p[i].log10()).round(),
            "winPct": if games1 > 0.0 { r1(100.0 * won[i] / games1) } else { 0.0 },
            "games1v1": games1, "pts1v1": r1(mean(&pts[i])), "sd1v1": r1(sd(&pts[i])),
            "auto1v1": r1(mean(&part[i][0])), "tele1v1": r1(mean(&part[i][1])), "tower1v1": r1(mean(&part[i][2])), "foulPts1v1": r1(mean(&part[i][3])),
            "games3v3": apps[i], "opr": r1(opr[i]), "oprAuto": r1(opr_auto[i]), "oprFuel": r1(opr_fuel[i]), "oprTower": r1(opr_tower[i]),
        }));
    }
    let mut order: Vec<String> = rs.clone();
    order.sort_by(|a, b| out[b]["opr"].as_f64().unwrap().partial_cmp(&out[a]["opr"].as_f64().unwrap()).unwrap().then(out[b]["elo"].as_f64().unwrap().partial_cmp(&out[a]["elo"].as_f64().unwrap()).unwrap()));
    let doc = serde_json::json!({
        "source": "examples/ratings.rs: headless matches, every robot driven by the Champs AI (scorer), HUMAN PLAYERS throwing, 8 FUEL preloaded",
        "skill": SKILL, "matches1v1": m1.len(), "matches3v3": m3.len(),
        "residualSd3v3": r1(sd(&res)),
        "order": order, "robots": out,
    });
    let path = root().join("assets/ratings.json");
    std::fs::write(&path, serde_json::to_string_pretty(&doc).unwrap()).unwrap();
    eprintln!("{} 1v1 and {} 3v3 matches -> {}", m1.len(), m3.len(), path.display());
    eprintln!("{:>6} {:>5} {:>6} {:>6} {:>6} {:>6} {:>6}", "robot", "elo", "win%", "pts1v1", "OPR", "auto", "tower");
    for r in &doc["order"].as_array().unwrap().clone() {
        let x = &doc["robots"][r.as_str().unwrap()];
        eprintln!("{:>6} {:>5} {:>6} {:>6} {:>6} {:>6} {:>6}", r.as_str().unwrap(), x["elo"], x["winPct"], x["pts1v1"], x["opr"], x["oprAuto"], x["oprTower"]);
    }
}

fn main() {
    let seeds: u32 = arg("--seeds", "1").parse().unwrap();
    let opr: usize = arg("--opr", "51").parse().unwrap();
    let threads: usize = arg("--threads", "4").parse().unwrap();
    let only = arg("--only", "all");
    let all = jobs(seeds, opr);
    let pick: Vec<Job> = all.into_iter().filter(|j| only == "all" || j.kind == only).collect();
    if only != "fit" { run(pick, threads); }
    fit();
}
