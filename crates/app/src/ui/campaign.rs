//! The pits (Road to Einstein's screen between matches): where you are in the season, your next
//! match with both alliances and what your coach makes of it, alliance selection, the playoffs,
//! the event's end and the season's recap; the rankings beside it. Tabs (left/right, or click) show
//! the event's schedule, its full rankings, its playoffs and every team's scouting card.
use bevy::prelude::*;
use campaign::season::{Beat, Stage};
use campaign::story;

use super::{color, radius, read_nav, Face, Fonts, NavRepeat, Screen};
use crate::campaign::{ratings, Campaign, Next, ROLES};
use crate::game::{GameCmd, PendingSetup};
use crate::settings::{Roster, Settings};

pub struct PitsPlugin;
impl Plugin for PitsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Pits>()
            .add_systems(OnEnter(Screen::Pits), |mut p: ResMut<Pits>| { p.dirty = true; p.next = None; p.focus = 0; p.tab = 0; })
            .add_systems(OnExit(Screen::Pits), |mut commands: Commands, q: Query<Entity, With<PitsRoot>>| { for e in &q { commands.entity(e).despawn(); } })
            .add_systems(Update, (input, draw).chain().run_if(in_state(Screen::Pits)));
    }
}

#[derive(Clone, Debug)]
enum Act { Play, Role(usize), Pick(usize), Accept, Decline, Continue, NextSeason, Menu }

/// the pits' tabs
const TABS: [&str; 5] = ["NEXT", "SCHEDULE", "RANKINGS", "PLAYOFFS", "SCOUTING"];

#[derive(Resource, Default)]
pub struct Pits {
    next: Option<Next>,
    /// the tab shown (TABS)
    pub tab: usize,
    focus: usize,
    dirty: bool,
    acts: Vec<(String, Act)>,
    pub clicked: Option<usize>,
    tab_clicked: Option<usize>,
}

impl Pits {
    /// show a tab (TABS)
    pub fn show_tab(&mut self, k: usize) { self.tab = k.min(TABS.len() - 1); self.dirty = true; }
    /// work out what's next again (after the event's changed underneath)
    pub fn refresh(&mut self) { self.next = None; self.dirty = true; }
}

#[derive(Component)]
struct PitsRoot;
#[derive(Component)]
struct Btn(usize);
#[derive(Component)]
struct TabBtn(usize);

fn input(
    keys: Res<ButtonInput<KeyCode>>,
    pads: Query<&Gamepad>,
    time: Res<Time>,
    mut rep: Local<NavRepeat>,
    mut p: ResMut<Pits>,
    mut camp: Option<ResMut<Campaign>>,
    settings: Res<Settings>,
    mut cmds: MessageWriter<GameCmd>,
    mut commands: Commands,
    mut next_screen: ResMut<NextState<Screen>>,
) {
    let Some(c) = camp.as_mut() else { next_screen.set(Screen::Menu); return };
    if p.next.is_none() { p.next = Some(c.next()); p.dirty = true; }
    let nav = read_nav(&keys, &pads, &mut rep, time.delta_secs());
    let tab = if let Some(t) = p.tab_clicked.take() { t }
        else if nav.left || keys.just_pressed(KeyCode::KeyQ) { (p.tab + TABS.len() - 1) % TABS.len() }
        else if nav.right || keys.just_pressed(KeyCode::KeyE) || keys.just_pressed(KeyCode::Tab) { (p.tab + 1) % TABS.len() }
        else { p.tab };
    if tab != p.tab { p.tab = tab; p.dirty = true; }
    // (the other tabs are to look at: back leaves them for the next match's)
    if p.tab != 0 {
        if nav.back { p.tab = 0; p.dirty = true; }
        return;
    }
    let n = p.acts.len();
    if nav.up && p.focus > 0 { p.focus -= 1; p.dirty = true; }
    if nav.down && p.focus + 1 < n { p.focus += 1; p.dirty = true; }
    let mut act = p.clicked.take().or(if nav.ok { Some(p.focus) } else { None }).and_then(|i| p.acts.get(i).cloned()).map(|a| a.1);
    if nav.back { act = Some(Act::Menu); }
    let Some(act) = act else { return };
    let r = ratings();
    match act {
        Act::Play => {
            let nx = p.next.clone().unwrap();
            commands.insert_resource(PendingSetup(c.setup(&nx, &settings)));
            c.playing = Some(nx);
            c.lite = settings.camp_lite;
            c.after = None;
            cmds.write(GameCmd::Start);
        }
        Act::Role(t) => {
            let now = c.role(t);
            let k = ROLES.iter().position(|r| *r == now).map_or(0, |k| (k + 1) % ROLES.len());
            c.roles.insert(t, ROLES[k]);
        }
        Act::Pick(t) => { let e = c.season.event_mut(); e.pick(t, true); c.save(); p.next = None; }
        Act::Accept => { let e = c.season.event_mut(); e.pick(0, true); c.save(); p.next = None; }
        Act::Decline => { let e = c.season.event_mut(); e.pick(0, false); c.save(); p.next = None; }
        Act::Continue => {
            if c.season.stage == Stage::Over { c.save(); next_screen.set(Screen::Menu); return; }
            c.season.advance(r);
            c.save();
            p.next = None;
            p.focus = 0;
        }
        Act::NextSeason => {
            c.season = c.season.next_season(r);
            c.after = None;
            c.save();
            p.next = None;
            p.focus = 0;
        }
        Act::Menu => { c.save(); next_screen.set(Screen::Menu); }
    }
    p.dirty = true;
}

fn team_line(c: &Campaign, t: usize) -> (String, String) {
    let e = c.season.event();
    let tm = e.team(t);
    let info = Roster::get().robot(&tm.robot);
    let who = if tm.you { "YOU".to_string() } else { tm.name.clone() };
    let what = if tm.you { info.short() } else { format!("{} · {} {}", info.short(), tm.skill.name(), c.role(t)) };
    (who, what)
}

/// a team's name as the tables show it (yours: your team's name)
fn name_of(c: &Campaign, t: usize) -> String { let tm = c.season.event().team(t); if tm.you { c.season.your_name.clone() } else { tm.name.clone() } }
/// a team's color in the tables: you, your rival, everyone else
fn tint_of(c: &Campaign, t: usize) -> Color { if c.season.event().team(t).you { color::FUEL } else if t == c.season.rival { color::WARN } else { color::TEXT } }

fn draw(mut commands: Commands, mut p: ResMut<Pits>, camp: Option<Res<Campaign>>, settings: Res<Settings>, fonts: Res<Fonts>, roots: Query<Entity, With<PitsRoot>>) {
    if !p.dirty { return }
    let Some(c) = camp else { return };
    let Some(nx) = p.next.clone() else { return };
    p.dirty = false;
    for e in &roots { commands.entity(e).despawn(); }
    let f = fonts.clone();
    let r = ratings();
    let s = &c.season;
    let e = s.event();
    // what to do, and what's said about it
    let mut acts: Vec<(String, Act)> = Vec::new();
    let title;
    let mut lines: Vec<String> = Vec::new();
    let mut sides: Option<(Vec<usize>, Vec<usize>)> = None;
    match &nx {
        Next::Qual(i) => {
            let m = &e.quals[*i];
            let mine: Vec<usize> = (0..e.quals.len()).filter(|&k| e.quals[k].has(0) && !e.quals[k].surrogates.contains(&0)).collect();
            let k = mine.iter().position(|&k| k == *i).map_or(0, |k| k + 1);
            title = format!("{} · your match {} of {}", m.id, k, mine.len());
            lines.push(story::before_match(s, e, &m.red, &m.blue, r));
            sides = Some((m.red.clone(), m.blue.clone()));
            acts.push((format!("PLAY {}", m.id), Act::Play));
            if !settings.camp_lite { roles(&c, &m.red, &m.blue, &mut acts); } else { lines.push(lite_line(&c, &nx)); }
        }
        Next::Bout(i) => {
            let b = &e.bracket[*i];
            let (rs, bs) = c.sides(&nx);
            title = if b.id == "F" { format!("Finals, match {}", b.games.len() + 1) } else { format!("Playoffs: {}", b.id) };
            lines.push(story::before_match(s, e, &rs, &bs, r));
            acts.push((format!("PLAY {}", if b.id == "F" { format!("F{}", b.games.len() + 1) } else { b.id.clone() }), Act::Play));
            if !settings.camp_lite { roles(&c, &rs, &bs, &mut acts); } else { lines.push(lite_line(&c, &nx)); }
            sides = Some((rs, bs));
        }
        Next::YouPick => {
            title = format!("Alliance selection: you're the captain of alliance {}", e.alliance_of(0).unwrap() + 1);
            lines.push(story::after_quals(e));
            let mut avail = e.available(e.selection.turn);
            avail.sort_by(|a, b| e.team(*b).strength(r).partial_cmp(&e.team(*a).strength(r)).unwrap());
            for t in avail.into_iter().take(8) {
                let (who, what) = team_line(&c, t);
                acts.push((format!("PICK {who} · {what} · ranked {}", e.rank_of(t)), Act::Pick(t)));
            }
        }
        Next::Invite(a) => {
            let cap = e.selection.alliances[*a][0];
            title = format!("Alliance {} invites you", a + 1);
            lines.push(story::after_quals(e));
            lines.push(format!("{}, the captain of alliance {}, wants you on their alliance. Decline and you can't be picked by anyone else (though you could still captain one if a captain's spot opens).", team_line(&c, cap).0, a + 1));
            acts.push(("ACCEPT".into(), Act::Accept));
            acts.push(("DECLINE".into(), Act::Decline));
        }
        Next::EventOver => {
            title = format!("{} is over", e.name);
            let place = e.alliance_of(0).and_then(|a| e.finish(a));
            lines.push(match place {
                Some(1) => "We won it! Banner's going up in the shop.".into(),
                Some(2) => "Finalists. So close; we'll be back.".into(),
                Some(p) => format!("Out in the playoffs, {}.", match p { 3 => "3rd", 4 => "4th", 5 => "5th-6th", _ => "7th-8th" }),
                None => "Not picked for the playoffs this time.".into(),
            });
            for a in s.awards.iter().filter(|a| a.event == e.name && a.team == 0) { lines.push(format!("Award: {}!", a.name)); }
            if matches!(e.level, campaign::Level::District | campaign::Level::DistrictChampionship) {
                let st = s.standings();
                let pos = st.iter().position(|x| x.0 == 0).unwrap() + 1;
                lines.push(format!("District points: {} ({} in the district).", s.district_points(0), ordinal(pos)));
            }
            acts.push(("CONTINUE".into(), Act::Continue));
        }
        Next::Recap => {
            title = "The season".into();
            lines.extend(story::recap(s, r));
            acts.push((format!("NEXT SEASON (SEASON {})", s.year + 1), Act::NextSeason));
            acts.push(("BACK TO THE MENU".into(), Act::Continue));
        }
    }
    if let Some(a) = &c.after { if matches!(nx, Next::Qual(_) | Next::Bout(_) | Next::EventOver | Next::YouPick | Next::Invite(_)) { lines.insert(0, a.clone()); } }
    acts.push(("SAVE AND QUIT TO MENU".into(), Act::Menu));
    p.focus = p.focus.min(acts.len() - 1);
    let tab = p.tab;
    let focus = p.focus;
    p.acts = acts.clone();

    commands.spawn((
        PitsRoot,
        Node { width: Val::Percent(100.0), height: Val::Percent(100.0), position_type: PositionType::Absolute, align_items: AlignItems::Center, justify_content: JustifyContent::Center, ..default() },
        BackgroundColor(color::rgba(5, 6, 10, 0.78)), GlobalZIndex(10),
    )).with_children(|o| {
        o.spawn((
            Node { width: Val::Px(1180.0), max_width: Val::Percent(96.0), max_height: Val::Percent(94.0), flex_direction: FlexDirection::Column, padding: UiRect::axes(Val::Px(24.0), Val::Px(20.0)), border: UiRect::all(Val::Px(1.0)), border_radius: radius(16.0), overflow: Overflow::clip(), ..default() },
            BackgroundColor(color::BG), BorderColor::all(color::LINE),
        )).with_children(|b| {
            // header: the event, the stage, your record
            b.spawn(Node { flex_direction: FlexDirection::Row, align_items: AlignItems::Baseline, column_gap: Val::Px(14.0), margin: UiRect::bottom(Val::Px(4.0)), ..default() }).with_children(|h| {
                h.spawn(f.text("ROAD TO EINSTEIN", Face::Barlow(800), 18.0, color::FUEL));
                h.spawn(f.text(e.name.clone(), Face::Barlow(800), 34.0, color::TEXT));
            });
            let rk = e.ranks.iter().find(|x| x.team == 0);
            let rec = rk.map(|x| format!("{}-{}-{} · ranked {} of {} · {:.2} RS", x.wins, x.losses, x.ties, e.rank_of(0), e.teams.len(), x.rs())).unwrap_or_default();
            b.spawn((f.text(format!("{} · {} · {} difficulty · {}", s.your_name, Roster::get().robot(&s.your_robot).short(), s.difficulty.name(), rec), Face::Inter(400), 14.0, color::MUTED), Node { margin: UiRect::bottom(Val::Px(10.0)), ..default() }));
            // the tabs
            b.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(6.0), margin: UiRect::bottom(Val::Px(12.0)), ..default() }).with_children(|tb| {
                for (k, name) in TABS.iter().enumerate() {
                    let on = k == tab;
                    tb.spawn((Node { padding: UiRect::axes(Val::Px(12.0), Val::Px(5.0)), border: UiRect::bottom(Val::Px(2.0)), ..default() }, BackgroundColor(if on { color::tint(0.08) } else { Color::NONE }), BorderColor::all(if on { color::FUEL } else { Color::NONE }), TabBtn(k)))
                        .observe(|ev: On<PointerClick>, q: Query<&TabBtn>, mut p: ResMut<Pits>| { if let Ok(b) = q.get(ev.entity) { p.tab_clicked = Some(b.0); } })
                        .with_children(|t| { t.spawn(f.text(*name, Face::Barlow(700), 15.0, if on { color::TEXT } else { color::MUTED })); });
                }
                tb.spawn((f.text("◀ ▶ / Q E", Face::Inter(400), 12.0, color::MUTED), Node { margin: UiRect::left(Val::Px(10.0)), align_self: AlignSelf::Center, ..default() }));
            });
            match tab {
                1 => return schedule(b, &f, &c),
                2 => return rankings(b, &f, &c, r),
                3 => return playoffs(b, &f, &c),
                4 => return scouting(b, &f, &c, r),
                _ => {}
            }
            b.spawn(Node { display: Display::Grid, grid_template_columns: vec![GridTrack::flex(1.4), GridTrack::flex(1.0)], column_gap: Val::Px(18.0), ..default() }).with_children(|g| {
                // left: what's next
                g.spawn(Node { flex_direction: FlexDirection::Column, row_gap: Val::Px(8.0), ..default() }).with_children(|l| {
                    l.spawn(f.text(title.clone(), Face::Barlow(700), 24.0, color::TEXT));
                    if let Some((rs, bs)) = &sides {
                        l.spawn(Node { display: Display::Grid, grid_template_columns: vec![GridTrack::flex(1.0), GridTrack::flex(1.0)], column_gap: Val::Px(10.0), ..default() }).with_children(|al| {
                            for (ids, tint, name) in [(rs, color::RED, "RED"), (bs, color::BLUE, "BLUE")] {
                                al.spawn((Node { flex_direction: FlexDirection::Column, padding: UiRect::all(Val::Px(10.0)), border: UiRect::left(Val::Px(4.0)), border_radius: radius(8.0), row_gap: Val::Px(4.0), ..default() }, BackgroundColor(color::tint(0.04)), BorderColor::all(tint))).with_children(|col| {
                                    col.spawn(f.text(name, Face::Barlow(800), 16.0, tint));
                                    for &t in ids.iter() {
                                        let (who, what) = team_line(&c, t);
                                        col.spawn(f.text(who, Face::Inter(700), 14.0, if t == 0 { color::FUEL } else if t == s.rival { color::WARN } else { color::TEXT }));
                                        col.spawn(f.text(what, Face::Inter(400), 12.0, color::MUTED));
                                    }
                                });
                            }
                        });
                    }
                    for ln in &lines {
                        l.spawn((Node { padding: UiRect::all(Val::Px(10.0)), border_radius: radius(8.0), ..default() }, BackgroundColor(color::tint(0.05)))).with_children(|q| { q.spawn(f.text(ln.clone(), Face::Inter(400), 14.0, color::TEXT)); });
                    }
                    for (i, (label, _)) in acts.iter().enumerate() {
                        let main = i == 0;
                        let (bg, fg) = if main { (color::FUEL, color::DARK) } else { (color::tint(0.08), color::TEXT) };
                        l.spawn((Node { padding: UiRect::axes(Val::Px(14.0), Val::Px(9.0)), border: UiRect::all(Val::Px(2.0)), border_radius: radius(8.0), ..default() }, BackgroundColor(bg), BorderColor::all(if i == focus { color::tint(0.9) } else { Color::NONE }), Btn(i)))
                            .observe(|ev: On<PointerClick>, q: Query<&Btn>, mut p: ResMut<Pits>| { if let Ok(b) = q.get(ev.entity) { p.clicked = Some(b.0); } })
                            .observe(|ev: On<PointerOver>, q: Query<&Btn>, mut p: ResMut<Pits>| { if let Ok(b) = q.get(ev.entity) { if p.focus != b.0 { p.focus = b.0; p.dirty = true; } } })
                            .with_children(|t| { t.spawn(f.text(label.clone(), Face::Barlow(800), if main { 20.0 } else { 16.0 }, fg)); });
                    }
                });
                // right: the rankings, and your matches here
                g.spawn(Node { flex_direction: FlexDirection::Column, row_gap: Val::Px(3.0), ..default() }).with_children(|rt| {
                    rt.spawn(f.text("RANKINGS", Face::Barlow(700), 15.0, color::MUTED));
                    let you = e.rank_of(0);
                    for (k, rkx) in e.ranks.iter().enumerate() {
                        if k >= 10 && k + 1 != you { continue; }
                        let tm = e.team(rkx.team);
                        let col = if tm.you { color::FUEL } else if tm.id == s.rival { color::WARN } else { color::TEXT };
                        rt.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(8.0), ..default() }).with_children(|row| {
                            row.spawn((f.text(format!("{:>2}", k + 1), Face::Inter(600), 13.0, color::MUTED), Node { width: Val::Px(24.0), ..default() }));
                            row.spawn((f.text(if tm.you { s.your_name.clone() } else { tm.name.clone() }, Face::Inter(600), 13.0, col), Node { width: Val::Px(170.0), ..default() }));
                            row.spawn(f.text(format!("{} · {}-{}-{} · {:.2}", Roster::get().robot(&tm.robot).team, rkx.wins, rkx.losses, rkx.ties, rkx.rs()), Face::Inter(400), 12.0, color::MUTED));
                        });
                    }
                    rt.spawn((f.text("YOUR MATCHES", Face::Barlow(700), 15.0, color::MUTED), Node { margin: UiRect::top(Val::Px(10.0)), ..default() }));
                    for bt in s.beats.iter().rev().filter_map(|b| if let Beat::Match { event, id, won, us, them, .. } = b { (event == &e.name).then_some((id, won, us, them)) } else { None }).take(8) {
                        let (txt, col) = match bt.1 { Some(true) => ("W", color::OK), Some(false) => ("L", color::BAD), None => ("T", color::MUTED) };
                        rt.spawn(f.text(format!("{} {txt} {}-{}", bt.0, bt.2, bt.3), Face::Inter(600), 13.0, col));
                    }
                });
            });
        });
    });
}

/// what a 1 v 1 match is (settings.camp_lite)
fn lite_line(c: &Campaign, n: &Next) -> String {
    let (r, b) = c.on_field(n, true);
    let foe = if r.contains(&0) { b[0] } else { r[0] };
    format!("1 v 1: you against {} on the field; your partners' points and the rest of theirs are decided from their ratings.", name_of(c, foe))
}

/// your partners' roles in your next match, each a button that changes it
fn roles(c: &Campaign, red: &[usize], blue: &[usize], acts: &mut Vec<(String, Act)>) {
    let mine = if red.contains(&0) { red } else { blue };
    for &t in mine.iter().filter(|&&t| t != 0) {
        acts.push((format!("{}: {} (change)", name_of(c, t), c.role(t).to_uppercase()), Act::Role(t)));
    }
}

/// a cell of a table: text in a fixed width
fn cell(row: &mut ChildSpawnerCommands, f: &Fonts, s: impl Into<String>, w: f32, size: f32, face: Face, col: Color) {
    row.spawn((f.text(s, face, size, col), TextLayout::new(Justify::Left, LineBreak::NoWrap), Node { width: Val::Px(w), overflow: Overflow::clip(), ..default() }));
}
fn heading(b: &mut ChildSpawnerCommands, f: &Fonts, s: &str) {
    b.spawn((f.text(s, Face::Barlow(700), 15.0, color::MUTED), Node { margin: UiRect::new(Val::ZERO, Val::ZERO, Val::Px(6.0), Val::Px(4.0)), ..default() }));
}
fn short(s: &str, n: usize) -> String { if s.chars().count() <= n { s.to_string() } else { format!("{}…", s.chars().take(n - 1).collect::<String>()) } }

/// The schedule: your matches in full (partners, opponents, results), then every match
fn schedule(b: &mut ChildSpawnerCommands, f: &Fonts, c: &Campaign) {
    let e = c.season.event();
    let names = |ids: &[usize]| ids.iter().map(|&t| short(&name_of(c, t), 16)).collect::<Vec<_>>().join(", ");
    if e.quals.is_empty() { heading(b, f, "NO QUALIFICATION MATCHES HERE"); return; }
    heading(b, f, "YOUR MATCHES");
    for m in e.quals.iter().filter(|m| m.has(0)) {
        b.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(8.0), ..default() }).with_children(|row| {
            cell(row, f, m.id.clone(), 44.0, 13.0, Face::Inter(700), color::TEXT);
            cell(row, f, names(&m.red), 360.0, 12.0, Face::Inter(400), if m.red.contains(&0) { color::FUEL } else { color::RED });
            let (res, col) = match m.result {
                Some((r, bl)) => {
                    let us_red = m.red.contains(&0);
                    let (us, them) = if us_red { (r.total(), bl.total()) } else { (bl.total(), r.total()) };
                    (format!("{:.0} – {:.0}", r.total(), bl.total()), if us > them { color::OK } else if us < them { color::BAD } else { color::MUTED })
                }
                None => ("—".into(), color::MUTED),
            };
            cell(row, f, res, 90.0, 13.0, Face::Inter(700), col);
            cell(row, f, names(&m.blue), 360.0, 12.0, Face::Inter(400), if m.blue.contains(&0) { color::FUEL } else { color::BLUE });
            if m.surrogates.contains(&0) { cell(row, f, "surrogate", 80.0, 11.0, Face::Inter(400), color::MUTED); }
        });
    }
    heading(b, f, "ALL MATCHES");
    b.spawn(Node { display: Display::Grid, grid_template_columns: RepeatedGridTrack::flex(6, 1.0), column_gap: Val::Px(10.0), row_gap: Val::Px(2.0), ..default() }).with_children(|g| {
        for m in &e.quals {
            let (txt, col) = match m.result {
                Some((r, bl)) => (format!("{} {:.0}–{:.0}", m.id, r.total(), bl.total()), if r.total() > bl.total() { color::RED } else if r.total() < bl.total() { color::BLUE } else { color::MUTED }),
                None => (format!("{} —", m.id), color::MUTED),
            };
            g.spawn(f.text(txt, Face::Inter(if m.has(0) { 700 } else { 400 }), 12.0, if m.has(0) && m.result.is_none() { color::FUEL } else { col }));
        }
    });
}

/// The full rankings, in two columns
fn rankings(b: &mut ChildSpawnerCommands, f: &Fonts, c: &Campaign, r: &campaign::Ratings) {
    let e = c.season.event();
    let _ = r;
    if e.ranks.iter().all(|x| x.played == 0) && e.quals.is_empty() { heading(b, f, "NO RANKINGS AT THIS EVENT: ITS ALLIANCES ARE THE DIVISIONS' WINNERS"); return; }
    let half = e.ranks.len().div_ceil(2);
    b.spawn(Node { display: Display::Grid, grid_template_columns: RepeatedGridTrack::flex(2, 1.0), column_gap: Val::Px(24.0), ..default() }).with_children(|g| {
        for part in e.ranks.chunks(half) {
            g.spawn(Node { flex_direction: FlexDirection::Column, row_gap: Val::Px(2.0), ..default() }).with_children(|col| {
                col.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(6.0), ..default() }).with_children(|row| {
                    for (h, w) in [("#", 26.0), ("TEAM", 160.0), ("ROBOT", 130.0), ("W-L-T", 64.0), ("RS", 44.0), ("AVG", 44.0)] { cell(row, f, h, w, 11.0, Face::Inter(700), color::MUTED); }
                });
                for x in part {
                    let k = e.rank_of(x.team);
                    let tc = tint_of(c, x.team);
                    col.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(6.0), ..default() }).with_children(|row| {
                        cell(row, f, k.to_string(), 26.0, 13.0, Face::Inter(600), color::MUTED);
                        cell(row, f, short(&name_of(c, x.team), 21), 160.0, 13.0, Face::Inter(600), tc);
                        cell(row, f, short(&Roster::get().robot(&e.team(x.team).robot).short(), 19), 130.0, 12.0, Face::Inter(400), color::MUTED);
                        cell(row, f, format!("{}-{}-{}", x.wins, x.losses, x.ties), 64.0, 13.0, Face::Inter(400), color::TEXT);
                        cell(row, f, format!("{:.2}", x.rs()), 44.0, 13.0, Face::Inter(400), color::TEXT);
                        cell(row, f, format!("{:.0}", x.avg(x.score)), 44.0, 13.0, Face::Inter(400), color::TEXT);
                    });
                }
            });
        }
    });
}

/// The playoffs: the alliances, then each bout (the double elimination bracket's, or Einstein's
/// round robin and finals) with who's in it and how it went
fn playoffs(b: &mut ChildSpawnerCommands, f: &Fonts, c: &Campaign) {
    let e = c.season.event();
    let al = &e.selection.alliances;
    if al.is_empty() { heading(b, f, "THE PLAYOFFS START AFTER ALLIANCE SELECTION"); return; }
    let al_name = |a: usize| format!("A{}", a + 1);
    b.spawn(Node { display: Display::Grid, grid_template_columns: vec![GridTrack::flex(1.0), GridTrack::flex(1.2)], column_gap: Val::Px(24.0), ..default() }).with_children(|g| {
        g.spawn(Node { flex_direction: FlexDirection::Column, row_gap: Val::Px(3.0), ..default() }).with_children(|col| {
            heading(col, f, "ALLIANCES");
            for (k, a) in al.iter().enumerate() {
                col.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(6.0), ..default() }).with_children(|row| {
                    cell(row, f, al_name(k), 30.0, 13.0, Face::Inter(700), color::TEXT);
                    let txt = a.iter().map(|&t| short(&name_of(c, t), 16)).collect::<Vec<_>>().join(", ");
                    let place = e.finish(k).map(|p| match p { 1 => "WON".to_string(), 2 => "FINALIST".into(), 5 => "5-6".into(), 7 => "7-8".into(), p => ordinal(p) }).unwrap_or_default();
                    cell(row, f, txt, 380.0, 12.0, Face::Inter(if a.contains(&0) { 700 } else { 400 }), if a.contains(&0) { color::FUEL } else { color::TEXT });
                    cell(row, f, place, 70.0, 12.0, Face::Inter(700), color::MUTED);
                });
            }
        });
        g.spawn(Node { flex_direction: FlexDirection::Column, row_gap: Val::Px(3.0), ..default() }).with_children(|col| {
            heading(col, f, if e.level == campaign::Level::Einstein { "ROUND ROBIN AND FINALS" } else { "DOUBLE ELIMINATION" });
            for bt in &e.bracket {
                col.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(8.0), ..default() }).with_children(|row| {
                    let side = |x: Option<usize>| x.map_or("TBD".to_string(), al_name);
                    let mine = e.alliance_of(0).is_some_and(|a| bt.red == Some(a) || bt.blue == Some(a));
                    cell(row, f, bt.id.clone(), 40.0, 13.0, Face::Inter(700), if mine { color::FUEL } else { color::TEXT });
                    cell(row, f, side(bt.red), 40.0, 13.0, Face::Inter(600), color::RED);
                    cell(row, f, "vs", 22.0, 12.0, Face::Inter(400), color::MUTED);
                    cell(row, f, side(bt.blue), 40.0, 13.0, Face::Inter(600), color::BLUE);
                    let games = bt.games.iter().filter_map(|m| m.result).map(|(r, bl)| format!("{:.0}–{:.0}", r.total(), bl.total())).collect::<Vec<_>>().join(", ");
                    cell(row, f, games, 200.0, 12.0, Face::Inter(400), color::TEXT);
                    let w = match bt.winner { Some(usize::MAX) => "tie".to_string(), Some(w) => format!("{} wins", al_name(w)), None => String::new() };
                    cell(row, f, w, 90.0, 12.0, Face::Inter(700), color::OK);
                });
            }
        });
    });
}

/// Every team here: its robot (the robot's rating), its drive team, how it plays, and how it's done
fn scouting(b: &mut ChildSpawnerCommands, f: &Fonts, c: &Campaign, r: &campaign::Ratings) {
    let e = c.season.event();
    let mut ids: Vec<usize> = e.teams.iter().map(|t| t.id).collect();
    ids.sort_by(|a, b| e.team(*b).strength(r).partial_cmp(&e.team(*a).strength(r)).unwrap());
    let half = ids.len().div_ceil(2);
    b.spawn(Node { display: Display::Grid, grid_template_columns: RepeatedGridTrack::flex(2, 1.0), column_gap: Val::Px(24.0), ..default() }).with_children(|g| {
        for part in ids.chunks(half) {
            g.spawn(Node { flex_direction: FlexDirection::Column, row_gap: Val::Px(2.0), ..default() }).with_children(|col| {
                col.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(6.0), ..default() }).with_children(|row| {
                    for (h, w) in [("TEAM", 160.0), ("ROBOT", 130.0), ("OPR", 40.0), ("DRIVERS", 70.0), ("PLAYS", 64.0), ("HERE", 60.0)] { cell(row, f, h, w, 11.0, Face::Inter(700), color::MUTED); }
                });
                for &t in part {
                    let tm = e.team(t);
                    let here = e.ranks.iter().find(|x| x.team == t).filter(|x| x.played > 0).map_or("—".to_string(), |x| format!("{:.0} avg", x.avg(x.score)));
                    col.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(6.0), ..default() }).with_children(|row| {
                        cell(row, f, short(&name_of(c, t), 20), 160.0, 13.0, Face::Inter(600), tint_of(c, t));
                        cell(row, f, short(&Roster::get().robot(&tm.robot).short(), 19), 130.0, 12.0, Face::Inter(400), color::MUTED);
                        cell(row, f, format!("{:.0}", r.get(&tm.robot).opr), 40.0, 13.0, Face::Inter(400), color::TEXT);
                        cell(row, f, if tm.you { "you".into() } else { tm.skill.name().to_string() }, 70.0, 12.0, Face::Inter(400), color::TEXT);
                        cell(row, f, if tm.you { "—".into() } else { c.role(t).to_string() }, 64.0, 12.0, Face::Inter(400), color::TEXT);
                        cell(row, f, here, 60.0, 12.0, Face::Inter(400), color::TEXT);
                    });
                }
            });
        }
    });
}

fn ordinal(n: usize) -> String { match n % 10 { 1 if n % 100 != 11 => format!("{n}st"), 2 if n % 100 != 12 => format!("{n}nd"), 3 if n % 100 != 13 => format!("{n}rd"), _ => format!("{n}th") } }
