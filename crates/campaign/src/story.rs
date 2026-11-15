//! The story told around the matches (never instead of them): your coach before and after each
//! match, from what happened; the rival; the season recap.
use crate::event::{Event, Level};
use crate::model::Ratings;
use crate::season::{Beat, Season, Stage};

/// one of the lines, picked by `n` (so the same moment says the same thing on a reload)
fn pick<'a>(n: usize, lines: &[&'a str]) -> &'a str { lines[n % lines.len()] }

/// What your coach says before your next match: the plan, who's on your alliance and theirs
pub fn before_match(s: &Season, e: &Event, red: &[usize], blue: &[usize], r: &Ratings) -> String {
    let you_red = red.contains(&0);
    let (ours, theirs) = if you_red { (red, blue) } else { (blue, red) };
    let sum = |ids: &[usize]| ids.iter().map(|&t| e.team(t).strength(r)).sum::<f64>();
    let (us, them) = (sum(ours), sum(theirs));
    let n = s.beats.len();
    let mut out = String::new();
    if theirs.contains(&s.rival) {
        out += pick(n, &[
            &format!("It's {} across the field. They'll be watching what we do in AUTO; let's give them something to watch.", e.team(s.rival).name),
            &format!("{} again. Don't play their game: run our cycles and let the score talk.", e.team(s.rival).name),
        ]);
        out += " ";
    } else if ours.contains(&s.rival) {
        out += &format!("We're with {} this time. Strange, but they're good: let them have the near side and we take the far. ", e.team(s.rival).name);
    }
    let ratio = us / them.max(1.0);
    out += if ratio > 1.25 {
        pick(n / 2, &["On paper we're the stronger alliance. Don't let that make us sloppy: clean cycles, no fouls.", "We should win this one if we just play our game. Stay out of their zone in AUTO."])
    } else if ratio < 0.8 {
        pick(n / 2, &["They're stronger on paper. We need a big AUTO and every FUEL in an active shift.", "This one's uphill. Play smart: score when our HUB is lit and make them earn every point."])
    } else {
        pick(n / 2, &["This one's close. The TOWER at the end could decide it.", "Even match. Whoever wastes fewer seconds wins it."])
    };
    out
}

/// What your coach says after a match: from the result and how you did
pub fn after_match(s: &Season, us: f64, them: f64, rp: f64, fouls_given: f64) -> String {
    let n = s.beats.len();
    let mut out = String::from(if us > them {
        if us - them > 150.0 { pick(n, &["That's how it's done. Nobody's going to forget that one.", "A blowout. Keep that rhythm."]) }
        else { pick(n, &["A win's a win. We'll take it.", "Close, but ours. Good driving at the end."]) }
    } else if us < them {
        if them - us > 150.0 { pick(n, &["Shake it off. We learn from it and go again.", "Rough one. Let's look at where the cycles went."]) }
        else { pick(n, &["So close. One more cycle and that's ours.", "That one hurts. A couple of FUEL the other way and we win it."]) }
    } else { "A tie. Don't see many of those." });
    if fouls_given >= 15.0 { out += " And we gave away too many foul points: watch the contact."; }
    if rp >= 5.0 { out += " Bonus ranking points too: that's a big one for the standings."; }
    out
}

/// After the qualification matches: where you ranked, what's next
pub fn after_quals(e: &Event) -> String {
    let rank = e.rank_of(0);
    let n = e.teams.len();
    if rank <= 8 { format!("We finished {rank} of {n}: we're an alliance captain. Who do we want with us?") }
    else if rank <= 16 { format!("{rank} of {n}. We're in the mix: if a captain's been watching, we'll get a call.") }
    else { format!("{rank} of {n}. Not where we wanted. Our scouting can still get us picked; be ready.") }
}

/// The season recap: your record, the moments that mattered, how far you got, the score
pub fn recap(s: &Season, r: &Ratings) -> Vec<String> {
    let (mut w, mut l, mut t) = (0, 0, 0);
    for b in &s.beats { if let Beat::Match { won, .. } = b { match won { Some(true) => w += 1, Some(false) => l += 1, None => t += 1 } } }
    let mut out = vec![format!("{}: {w}-{l}-{t} in the matches we played, driving {}.", s.your_name, s.your_robot)];
    for b in &s.beats {
        match b {
            Beat::Finished { event, place: Some(p) } => out.push(format!("{event}: {}.", match p { 1 => "won it".into(), 2 => "finalists".into(), p => format!("finished {}", ordinal(*p)) })),
            Beat::Finished { event, place: None } => out.push(format!("{event}: not picked for the playoffs.")),
            Beat::Award(a) => out.push(format!("{}: the {} award.", a.event, a.name)),
            Beat::Eliminated { at } => out.push(format!("The season ended short of the {}.", stage_name(*at))),
            _ => {}
        }
    }
    if s.events.iter().any(|e| e.level == Level::Einstein && e.playoffs_done() && e.bracket.last().unwrap().winner.is_some_and(|w| e.selection.alliances[w].contains(&0))) {
        out.push("World champions.".into());
    }
    out.push(format!("Season {} score: {} ({} difficulty, {}, robot multiplier x{:.2}).", s.year, s.score(r), s.difficulty.name(), s.length.name().to_lowercase(), s.robot_mult(r)));
    if !s.history.is_empty() {
        let best = s.history.iter().map(|p| p.score).fold(s.score(r), f64::max);
        let past = s.history.iter().map(|p| format!("season {} {} ({}, {})", p.year, p.score, p.robot, p.reached)).collect::<Vec<_>>().join("; ");
        out.push(format!("Before this: {past}. Best: {best}."));
    }
    out
}

pub fn stage_name(s: Stage) -> &'static str {
    match s { Stage::District1 | Stage::District2 => "district events", Stage::DistrictChampionship => "District Championship", Stage::Division => "Championship", Stage::Einstein => "Einstein field", Stage::Over => "end" }
}

fn ordinal(n: usize) -> String { match n { 1 => "1st".into(), 2 => "2nd".into(), 3 => "3rd".into(), n => format!("{n}th") } }
