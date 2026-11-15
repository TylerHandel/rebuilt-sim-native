//! One event: its teams, the qualification schedule, the matches (yours played in the game, the
//! rest decided here), the rankings, alliance selection, the double-elimination playoff, and the
//! district points it gives out.
use crate::model::{Ratings, Team};
use crate::rng::Rng;
use serde::{Deserialize, Serialize};

/// What kind of event it is
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Level { District, DistrictChampionship, Division, Einstein }

/// The game's rules an event plays by (2026 Game Manual section 6: win 3 RP, tie 1, and the three
/// BONUS RP, their thresholds higher at the District Championship and the Championship, Team Update
/// 19). The manual's FUEL thresholds (100 and 360 at a district, 200 and 500 above) are for real
/// matches; this simulator's alliances score several times the FUEL (a Champs-level 3 v 3 alliance:
/// ~790, examples/ratings.rs), so here they're scaled to it, ENERGIZED for a good alliance and
/// SUPERCHARGED for a great one. TOWER points are as the real game's, and so is TRAVERSAL's.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Rules {
    pub quals_per_team: usize,
    /// ENERGIZED RP: FUEL scored in an active HUB; SUPERCHARGED RP: more; TRAVERSAL RP: TOWER points
    pub energized: f64,
    pub supercharged: f64,
    pub traversal: f64,
}

impl Rules {
    pub fn for_level(l: Level) -> Self {
        match l {
            Level::District => Rules { quals_per_team: 8, energized: 450.0, supercharged: 750.0, traversal: 50.0 },
            _ => Rules { quals_per_team: 10, energized: 650.0, supercharged: 900.0, traversal: 90.0 },
        }
    }
}

/// One alliance's score in a match
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Line {
    /// FUEL scored (AUTO and TELEOP, in an active HUB: the ENERGIZED and SUPERCHARGED count)
    pub fuel: f64,
    /// AUTO points (FUEL and TOWER), part of the others
    pub auto: f64,
    pub tower: f64,
    /// points from the other alliance's fouls
    pub fouls: f64,
}
impl Line {
    pub fn total(&self) -> f64 { self.fuel + self.tower + self.fouls }
}

/// A match: its two alliances (team ids; red then blue, as the field lists them) and its result
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Match {
    /// "Q12", "M3", "F2"
    pub id: String,
    pub red: Vec<usize>,
    pub blue: Vec<usize>,
    /// a team playing an extra match to fill the schedule (its result doesn't count for it)
    pub surrogates: Vec<usize>,
    pub result: Option<(Line, Line)>,
}
impl Match {
    pub fn has(&self, t: usize) -> bool { self.red.contains(&t) || self.blue.contains(&t) }
    /// 1 red won, -1 blue won, 0 tie (by score alone)
    pub fn winner(&self) -> Option<i32> {
        let (r, b) = self.result?;
        Some(if r.total() > b.total() { 1 } else if r.total() < b.total() { -1 } else { 0 })
    }
}

/// A team's line in the qualification rankings
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Rank {
    pub team: usize,
    pub played: usize,
    pub rp: f64,
    pub wins: usize,
    pub losses: usize,
    pub ties: usize,
    pub score: f64,
    pub tower: f64,
    pub auto: f64,
    /// a draw for the last tiebreaker, made once
    pub draw: f64,
}
impl Rank {
    /// ranking score: RP per match
    pub fn rs(&self) -> f64 { if self.played == 0 { 0.0 } else { self.rp / self.played as f64 } }
    pub fn avg(&self, v: f64) -> f64 { if self.played == 0 { 0.0 } else { v / self.played as f64 } }
}

/// Where alliance selection is
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Selection {
    /// the eight alliances: their captain, then their picks
    pub alliances: Vec<Vec<usize>>,
    /// 0: first round (alliance 1 to 8), 1: second (8 to 1), 2: done
    pub round: usize,
    /// whose turn: the alliance index
    pub turn: usize,
    /// teams that declined an invitation (they can't be picked, but can still be a captain)
    pub declined: Vec<usize>,
}

/// A playoff match in the double-elimination bracket: who plays (alliance indices, from the
/// matches before it) and who won
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Bout {
    pub id: String,
    pub red: Option<usize>,
    pub blue: Option<usize>,
    /// where each comes from: ("W", match) the winner of, ("L", match) the loser of, ("A", n) alliance n
    pub from: [(char, usize); 2],
    pub games: Vec<Match>,
    pub winner: Option<usize>,
    pub loser: Option<usize>,
}

/// One event
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Event {
    pub name: String,
    pub level: Level,
    pub rules: Rules,
    pub teams: Vec<Team>,
    pub quals: Vec<Match>,
    pub ranks: Vec<Rank>,
    pub selection: Selection,
    pub bracket: Vec<Bout>,
    pub rng: Rng,
}

/// The FRC double-elimination bracket for eight alliances (2023 on): rounds 1-5 and the finals
fn bracket() -> Vec<Bout> {
    let b = |id: &str, a: (char, usize), c: (char, usize)| Bout { id: id.into(), red: None, blue: None, from: [a, c], games: Vec::new(), winner: None, loser: None };
    vec![
        b("M1", ('A', 0), ('A', 7)), b("M2", ('A', 3), ('A', 4)), b("M3", ('A', 1), ('A', 6)), b("M4", ('A', 2), ('A', 5)),
        b("M5", ('L', 1), ('L', 2)), b("M6", ('L', 3), ('L', 4)), b("M7", ('W', 1), ('W', 2)), b("M8", ('W', 3), ('W', 4)),
        b("M9", ('L', 7), ('W', 6)), b("M10", ('L', 8), ('W', 5)),
        b("M11", ('W', 7), ('W', 8)), b("M12", ('W', 10), ('W', 9)),
        b("M13", ('L', 11), ('W', 12)),
        b("F", ('W', 11), ('W', 13)),
    ]
}

impl Event {
    /// A new event: its teams and the qualification schedule
    pub fn new(name: &str, level: Level, teams: Vec<Team>, seed: u32) -> Self {
        let rules = Rules::for_level(level);
        let mut rng = Rng::new(seed);
        let ranks = teams.iter().map(|t| Rank { team: t.id, draw: rng.next(), ..Default::default() }).collect();
        let mut e = Event { name: name.into(), level, rules, teams, quals: Vec::new(), ranks, selection: Selection::default(), bracket: bracket(), rng };
        e.quals = e.schedule();
        e
    }

    pub fn team(&self, id: usize) -> &Team { self.teams.iter().find(|t| t.id == id).expect("team at this event") }
    pub fn you(&self) -> Option<usize> { self.teams.iter().find(|t| t.you).map(|t| t.id) }

    /// The qualification schedule: every team in `quals_per_team` matches, as evenly spread as it
    /// can (the teams that have played least go next, not twice in a row, few repeat partners);
    /// a last match that needs more teams than are owed one takes surrogates
    fn schedule(&mut self) -> Vec<Match> {
        let ids: Vec<usize> = self.teams.iter().map(|t| t.id).collect();
        let n = ids.len();
        let q = self.rules.quals_per_team;
        let total = (n * q).div_ceil(6);
        let mut played = vec![0usize; n];
        let mut last = vec![usize::MAX; n];
        let mut partners = vec![vec![0u32; n]; n];
        let mut out = Vec::new();
        for m in 0..total {
            // the candidates: who's owed a match, fewest played first, not just off the field
            let mut order: Vec<usize> = (0..n).collect();
            self.rng.shuffle(&mut order);
            order.sort_by_key(|&i| (played[i], (last[i] != usize::MAX && m - last[i] <= 1) as usize));
            let six: Vec<usize> = order[..6].to_vec();
            let surrogates: Vec<usize> = six.iter().copied().filter(|&i| played[i] >= q).collect();
            // the split with the fewest repeat partners, of a few tries
            let mut best = (u32::MAX, six.clone());
            for _ in 0..12 {
                let mut s = six.clone();
                self.rng.shuffle(&mut s);
                let cost: u32 = [&s[..3], &s[3..]].iter().map(|a| { let mut c = 0; for x in 0..3 { for y in x + 1..3 { c += partners[a[x]][a[y]]; } } c }).sum();
                if cost < best.0 { best = (cost, s); }
            }
            let s = best.1;
            for a in [&s[..3], &s[3..]] { for &x in a { for &y in a { if x != y { partners[x][y] += 1; } } } }
            for &i in &s { played[i] += 1; last[i] = m; }
            out.push(Match { id: format!("Q{}", m + 1), red: s[..3].iter().map(|&i| ids[i]).collect(), blue: s[3..].iter().map(|&i| ids[i]).collect(), surrogates: surrogates.iter().map(|&i| ids[i]).collect(), result: None });
        }
        out
    }

    /// Decide a match between two alliances from their robots' ratings and drivers' skills, with luck
    pub fn decide(&mut self, red: &[usize], blue: &[usize], ratings: &Ratings) -> (Line, Line) {
        let sd = ratings.residual_sd;
        let line = |ids: &[usize], rng: &mut Rng| {
            let (mut fuel, mut tower, mut auto) = (0.0, 0.0, 0.0);
            for &id in ids {
                let e = self.teams.iter().find(|t| t.id == id).unwrap().expect(ratings);
                fuel += e.fuel; tower += e.tower; auto += e.auto;
            }
            // (the luck: most of it in the FUEL, some in whether the climbs came off)
            let fuel = (fuel + rng.gauss() * sd * 0.9).max(0.0).round();
            let tower = ((tower + rng.gauss() * sd * 0.15) / 10.0).round().max(0.0) * 10.0;
            let auto = (auto * (1.0 + rng.gauss() * 0.15)).max(0.0).round().min(fuel + tower);
            Line { fuel, auto, tower, fouls: 0.0 }
        };
        let mut rng = self.rng.clone();
        let (mut r, mut b) = (line(red, &mut rng), line(blue, &mut rng));
        // fouls: now and then a MINOR (5) or MAJOR (15) against one side
        for side in [&mut r, &mut b] {
            let u = rng.next();
            side.fouls = if u < 0.18 { 5.0 } else if u < 0.24 { 15.0 } else if u < 0.27 { 10.0 } else { 0.0 };
        }
        self.rng = rng;
        (r, b)
    }

    /// Some of two alliances' teams' part of a match (the rest played it): theirs decided as
    /// `decide` would, the luck scaled to how many of the three they are, and no fouls
    pub fn decide_part(&mut self, red: &[usize], blue: &[usize], ratings: &Ratings) -> (Line, Line) {
        let mut rng = self.rng.clone();
        let mut line = |ids: &[usize]| {
            let (mut fuel, mut tower, mut auto) = (0.0, 0.0, 0.0);
            for &id in ids {
                let e = self.teams.iter().find(|t| t.id == id).unwrap().expect(ratings);
                fuel += e.fuel; tower += e.tower; auto += e.auto;
            }
            if ids.is_empty() { return Line::default(); }
            let sd = ratings.residual_sd * (ids.len() as f64 / 3.0).sqrt();
            let fuel = (fuel + rng.gauss() * sd * 0.9).max(0.0).round();
            let tower = ((tower + rng.gauss() * sd * 0.15) / 10.0).round().max(0.0) * 10.0;
            let auto = (auto * (1.0 + rng.gauss() * 0.15)).max(0.0).round().min(fuel + tower);
            Line { fuel, auto, tower, fouls: 0.0 }
        };
        let (r, b) = (line(red), line(blue));
        self.rng = rng;
        (r, b)
    }

    /// The qualification match `you` play next, if any (its index in quals)
    pub fn next_for(&self, team: usize) -> Option<usize> { self.quals.iter().position(|m| m.result.is_none() && m.has(team)) }

    /// Decide every unplayed qualification match before `upto` (an index into quals)
    pub fn decide_quals_until(&mut self, upto: usize, ratings: &Ratings) {
        for i in 0..upto.min(self.quals.len()) {
            if self.quals[i].result.is_some() { continue; }
            let (red, blue) = (self.quals[i].red.clone(), self.quals[i].blue.clone());
            let res = self.decide(&red, &blue, ratings);
            self.record_qual(i, res.0, res.1);
        }
    }

    /// A qualification match's result (yours from the game, or decided)
    pub fn record_qual(&mut self, i: usize, red: Line, blue: Line) {
        self.quals[i].result = Some((red, blue));
        let m = self.quals[i].clone();
        let rp = |me: &Line, them: &Line, rules: &Rules| {
            let mut p = if me.total() > them.total() { 3.0 } else if me.total() == them.total() { 1.0 } else { 0.0 };
            if me.fuel >= rules.energized { p += 1.0; }
            if me.fuel >= rules.supercharged { p += 1.0; }
            if me.tower >= rules.traversal { p += 1.0; }
            p
        };
        for (ids, me, them) in [(&m.red, &red, &blue), (&m.blue, &blue, &red)] {
            for &t in ids {
                if m.surrogates.contains(&t) { continue; }
                let p = rp(me, them, &self.rules);
                let r = self.ranks.iter_mut().find(|r| r.team == t).unwrap();
                r.played += 1;
                r.rp += p;
                if me.total() > them.total() { r.wins += 1 } else if me.total() < them.total() { r.losses += 1 } else { r.ties += 1 }
                r.score += me.total() - me.fouls;
                r.tower += me.tower;
                r.auto += me.auto;
            }
        }
        self.sort_ranks();
    }

    /// The rankings' order: ranking score, then average match points (fouls left out), TOWER, AUTO,
    /// then the draw
    fn sort_ranks(&mut self) {
        self.ranks.sort_by(|a, b| {
            let k = |r: &Rank| [r.rs(), r.avg(r.score), r.avg(r.tower), r.avg(r.auto), r.draw];
            let (x, y) = (k(a), k(b));
            y.partial_cmp(&x).unwrap()
        });
    }
    /// a team's rank (1 = first)
    pub fn rank_of(&self, team: usize) -> usize { self.ranks.iter().position(|r| r.team == team).unwrap() + 1 }
    pub fn quals_done(&self) -> bool { self.quals.iter().all(|m| m.result.is_some()) }

    // ------------------------------------------------------------------ alliance selection
    /// Start alliance selection: the top eight seeds are captains
    pub fn start_selection(&mut self) {
        let caps: Vec<usize> = self.ranks.iter().take(8).map(|r| r.team).collect();
        self.selection = Selection { alliances: caps.into_iter().map(|c| vec![c]).collect(), round: 0, turn: 0, declined: Vec::new() };
    }
    /// The alliance whose turn it is to pick, or none once selection's over
    pub fn picking(&self) -> Option<usize> { if self.selection.round >= 2 { None } else { Some(self.selection.turn) } }
    fn on_alliance(&self, t: usize) -> bool { self.selection.alliances.iter().any(|a| a.contains(&t)) }
    /// Who's still available to be picked by alliance `a` (not on an alliance, not declined; a
    /// lower captain can be picked by a higher alliance)
    pub fn available(&self, a: usize) -> Vec<usize> {
        self.ranks.iter().map(|r| r.team).filter(|&t| {
            if self.selection.declined.contains(&t) { return false; }
            match self.selection.alliances.iter().position(|al| al.contains(&t)) {
                None => true,
                // (a captain of a lower alliance, before that alliance has picked anyone)
                Some(i) => i > a && self.selection.alliances[i].len() == 1 && self.selection.round == 0,
            }
        }).collect()
    }
    /// The picking alliance invites `t`; `accept`: whether it says yes (an AI always does; you choose)
    pub fn pick(&mut self, t: usize, accept: bool) {
        let a = self.selection.turn;
        if !accept { self.selection.declined.push(t); return; }
        // a lower captain picked: its alliance's captain becomes the next seed down
        if let Some(i) = self.selection.alliances.iter().position(|al| al.len() == 1 && al[0] == t) {
            self.selection.alliances.remove(i);
            // (the highest seed not on an alliance: one that declined an invitation may still captain)
            let next = self.ranks.iter().map(|r| r.team).find(|&x| x != t && !self.on_alliance(x));
            self.selection.alliances.push(vec![next.expect("a team to captain")]);
        }
        self.selection.alliances[a].push(t);
        // the serpentine: 1 to 8, then 8 to 1
        let s = &mut self.selection;
        if s.round == 0 { if s.turn == 7 { s.round = 1; } else { s.turn += 1; } }
        else if s.turn == 0 { s.round = 2; } else { s.turn -= 1; }
    }
    /// An AI captain's pick: the strongest available by what the scouts saw: half its robot's
    /// rating, half what it did here (its share of its alliances' points per match), a little off
    pub fn ai_pick(&mut self, ratings: &Ratings) -> usize {
        let a = self.selection.turn;
        let avail = self.available(a);
        let mut best = (f64::MIN, avail[0]);
        for t in avail {
            let here = self.ranks.iter().find(|r| r.team == t).map_or(0.0, |r| r.avg(r.score) / 3.0);
            let seen = (0.5 * self.team(t).strength(ratings) + 0.5 * here) * (1.0 + self.rng.gauss() * 0.08);
            if seen > best.0 { best = (seen, t); }
        }
        best.1
    }

    // ------------------------------------------------------------------ playoffs
    /// The next playoff match to play (its bout's index), once both alliances in it are known
    pub fn next_bout(&self) -> Option<usize> {
        self.bracket.iter().position(|b| b.winner.is_none() && b.red.is_some() && b.blue.is_some())
    }
    /// Einstein: the four division winners as alliances (the teams in threes, in order), a round
    /// robin, then the finals between the top two (best of three)
    pub fn einstein_setup(&mut self) {
        self.quals.clear();
        let ids: Vec<usize> = self.teams.iter().map(|t| t.id).collect();
        self.selection = Selection { alliances: ids.chunks(3).map(|c| c.to_vec()).collect(), round: 2, turn: 0, declined: Vec::new() };
        let b = |id: &str, a: usize, c: usize| Bout { id: id.into(), red: None, blue: None, from: [('A', a), ('A', c)], games: Vec::new(), winner: None, loser: None };
        self.bracket = vec![b("R1", 0, 3), b("R2", 1, 2), b("R3", 0, 2), b("R4", 1, 3), b("R5", 0, 1), b("R6", 2, 3)];
        self.bracket.push(Bout { id: "F".into(), red: None, blue: None, from: [('R', 1), ('R', 2)], games: Vec::new(), winner: None, loser: None });
        self.seed_bracket();
    }
    /// Einstein's round-robin standings: alliances by wins, then points scored
    pub fn round_robin(&self) -> Vec<usize> {
        let mut v: Vec<(usize, usize, f64)> = (0..self.selection.alliances.len()).map(|a| (a, 0, 0.0)).collect();
        for b in self.bracket.iter().filter(|b| b.id.starts_with('R')) {
            if let Some(w) = b.winner.filter(|&w| w < v.len()) { v[w].1 += 1; }
            for g in &b.games { let (r, bl) = g.result.unwrap(); v[b.red.unwrap()].2 += r.total(); v[b.blue.unwrap()].2 += bl.total(); }
        }
        v.sort_by(|a, b| b.1.cmp(&a.1).then(b.2.partial_cmp(&a.2).unwrap()));
        v.into_iter().map(|x| x.0).collect()
    }
    /// Fill in who plays whom from the alliances and the results so far
    pub fn seed_bracket(&mut self) {
        let rr_done = self.bracket.iter().filter(|b| b.id.starts_with('R')).all(|b| b.winner.is_some());
        let standings = if rr_done && self.level == Level::Einstein { self.round_robin() } else { Vec::new() };
        for i in 0..self.bracket.len() {
            let from = self.bracket[i].from;
            let who = |(k, n): (char, usize), br: &[Bout]| match k {
                'A' => Some(n),
                'R' => standings.get(n - 1).copied(),
                'W' => br[n - 1].winner,
                _ => br[n - 1].loser,
            };
            let (r, b) = (who(from[0], &self.bracket), who(from[1], &self.bracket));
            self.bracket[i].red = r;
            self.bracket[i].blue = b;
        }
    }
    /// Record a playoff game (alliance indices red, blue); a tie goes to the alliance with more foul
    /// points, then AUTO, then TOWER, then it's replayed. The finals go to whoever wins two.
    pub fn record_bout(&mut self, i: usize, red: Line, blue: Line) {
        let (ra, ba) = (self.bracket[i].red.unwrap(), self.bracket[i].blue.unwrap());
        let al = self.selection.alliances.clone();
        let id = format!("{}{}", self.bracket[i].id, if self.bracket[i].id == "F" { (self.bracket[i].games.len() + 1).to_string() } else { String::new() });
        self.bracket[i].games.push(Match { id, red: al[ra].clone(), blue: al[ba].clone(), surrogates: Vec::new(), result: Some((red, blue)) });
        let key = |l: &Line| [l.total(), l.fouls, l.auto, l.tower];
        let w = match key(&red).partial_cmp(&key(&blue)).unwrap() { std::cmp::Ordering::Greater => Some(ra), std::cmp::Ordering::Less => Some(ba), _ => None };
        let need = if self.bracket[i].id == "F" { 2 } else { 1 };
        // (a round-robin game that's tied is a tie: neither wins it)
        if w.is_none() && self.bracket[i].id.starts_with('R') { self.bracket[i].winner = Some(usize::MAX); }
        let wins = |a: usize, b: &Bout| b.games.iter().filter(|g| { let (r, bl) = g.result.unwrap(); let k = key(&r).partial_cmp(&key(&bl)).unwrap(); (k == std::cmp::Ordering::Greater && g.red == al[a]) || (k == std::cmp::Ordering::Less && g.blue == al[a]) }).count();
        if w.is_some() {
            let b = &self.bracket[i];
            if wins(ra, b) >= need { self.bracket[i].winner = Some(ra); self.bracket[i].loser = Some(ba); }
            else if wins(ba, b) >= need { self.bracket[i].winner = Some(ba); self.bracket[i].loser = Some(ra); }
        }
        self.seed_bracket();
    }
    /// Decide the next playoff match (unless it's yours)
    pub fn decide_bout(&mut self, i: usize, ratings: &Ratings) {
        let (r, b) = (self.bracket[i].red.unwrap(), self.bracket[i].blue.unwrap());
        let al = self.selection.alliances.clone();
        // (each alliance plays its three best robots)
        let res = self.decide(&al[r][..3.min(al[r].len())], &al[b][..3.min(al[b].len())], ratings);
        self.record_bout(i, res.0, res.1);
    }
    pub fn playoffs_done(&self) -> bool { self.bracket.last().unwrap().winner.is_some() }
    /// An alliance's finish (1 won, 2 finalist, 3, 4, 5 for 5th-6th, 7 for 7th-8th)
    pub fn finish(&self, alliance: usize) -> Option<usize> {
        let lost = |id: &str| self.bracket.iter().find(|b| b.id == id).and_then(|b| b.loser) == Some(alliance);
        if self.bracket.last().unwrap().winner == Some(alliance) { return Some(1); }
        if lost("F") { return Some(2); }
        if self.level == Level::Einstein { return self.playoffs_done().then(|| 3); }
        if lost("M13") { return Some(3); }
        if lost("M12") { return Some(4); }
        if lost("M9") || lost("M10") { return Some(5); }
        if lost("M5") || lost("M6") { return Some(7); }
        None
    }
    /// the alliance a team's on, if any
    pub fn alliance_of(&self, t: usize) -> Option<usize> { self.selection.alliances.iter().position(|a| a.contains(&t)) }

    // ------------------------------------------------------------------ district points
    /// The district points a team earned here (FRC's district model): qualification performance,
    /// alliance selection, playoff advancement (awards apart)
    pub fn district_points(&self, t: usize) -> f64 {
        let n = self.teams.len() as f64;
        let r = self.rank_of(t) as f64;
        let alpha: f64 = 1.07;
        let qual = (erfinv((n - 2.0 * r + 2.0) / (alpha * n)) * (10.0 / erfinv(1.0 / alpha)) + 12.0).ceil();
        let mut pts = qual.clamp(4.0, 22.0);
        if let Some(a) = self.alliance_of(t) {
            let slot = self.selection.alliances[a].iter().position(|&x| x == t).unwrap();
            pts += if slot <= 1 { 16.0 - a as f64 } else { 1.0 + a as f64 };
            pts += match self.finish(a) { Some(1) => 30.0, Some(2) => 20.0, Some(3) => 13.0, Some(4) => 7.0, _ => 0.0 };
        }
        pts
    }
}

/// The inverse error function (Giles' single-precision approximation, good to ~1e-7)
pub fn erfinv(x: f64) -> f64 {
    let w = -((1.0 - x) * (1.0 + x)).ln();
    if w < 5.0 {
        let w = w - 2.5;
        let mut p = 2.81022636e-08;
        for c in [3.43273939e-07, -3.5233877e-06, -4.39150654e-06, 0.00021858087, -0.00125372503, -0.00417768164, 0.246640727, 1.50140941] { p = c + p * w; }
        p * x
    } else {
        let w = w.sqrt() - 3.0;
        let mut p = -0.000200214257;
        for c in [0.000100950558, 0.00134934322, -0.00367342844, 0.00573950773, -0.0076224613, 0.00943887047, 1.00167406, 2.83297682] { p = c + p * w; }
        p * x
    }
}
