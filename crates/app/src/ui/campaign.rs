//! The pits (Road to Einstein's screen between matches): where you are in the season, your next
//! match with both alliances and what your coach makes of it, alliance selection, the playoffs,
//! the event's end and the season's recap; the rankings beside it.
use bevy::prelude::*;
use campaign::season::{Beat, Stage};
use campaign::story;

use super::{color, radius, read_nav, Face, Fonts, NavRepeat, Screen};
use crate::campaign::{ratings, strategy, Campaign, Next};
use crate::game::{GameCmd, PendingSetup};
use crate::settings::{Roster, Settings};

pub struct PitsPlugin;
impl Plugin for PitsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Pits>()
            .add_systems(OnEnter(Screen::Pits), |mut p: ResMut<Pits>| { p.dirty = true; p.next = None; p.focus = 0; })
            .add_systems(OnExit(Screen::Pits), |mut commands: Commands, q: Query<Entity, With<PitsRoot>>| { for e in &q { commands.entity(e).despawn(); } })
            .add_systems(Update, (input, draw).chain().run_if(in_state(Screen::Pits)));
    }
}

#[derive(Clone, Debug)]
enum Act { Play, Pick(usize), Accept, Decline, Continue, Menu }

#[derive(Resource, Default)]
pub struct Pits {
    next: Option<Next>,
    focus: usize,
    dirty: bool,
    acts: Vec<(String, Act)>,
    pub clicked: Option<usize>,
    /// what your coach said after your last match (shown until the next)
    pub after: Option<String>,
}

#[derive(Component)]
struct PitsRoot;
#[derive(Component)]
struct Btn(usize);

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
            p.after = None;
            cmds.write(GameCmd::Start);
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
        Act::Menu => { c.save(); next_screen.set(Screen::Menu); }
    }
    p.dirty = true;
}

fn team_line(e: &campaign::Event, t: usize) -> (String, String) {
    let tm = e.team(t);
    let info = Roster::get().robot(&tm.robot);
    let who = if tm.you { "YOU".to_string() } else { tm.name.clone() };
    let what = if tm.you { info.short() } else { format!("{} · {} {}", info.short(), tm.skill.name(), strategy(t, tm.skill)) };
    (who, what)
}

fn draw(mut commands: Commands, mut p: ResMut<Pits>, camp: Option<Res<Campaign>>, fonts: Res<Fonts>, roots: Query<Entity, With<PitsRoot>>) {
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
        }
        Next::Bout(i) => {
            let b = &e.bracket[*i];
            let (rs, bs) = c.sides(&nx);
            title = if b.id == "F" { format!("Finals, match {}", b.games.len() + 1) } else { format!("Playoffs: {}", b.id) };
            lines.push(story::before_match(s, e, &rs, &bs, r));
            sides = Some((rs, bs));
            acts.push((format!("PLAY {}", if b.id == "F" { format!("F{}", b.games.len() + 1) } else { b.id.clone() }), Act::Play));
        }
        Next::YouPick => {
            title = format!("Alliance selection: you're the captain of alliance {}", e.alliance_of(0).unwrap() + 1);
            lines.push(story::after_quals(e));
            let mut avail = e.available(e.selection.turn);
            avail.sort_by(|a, b| e.team(*b).strength(r).partial_cmp(&e.team(*a).strength(r)).unwrap());
            for t in avail.into_iter().take(8) {
                let (who, what) = team_line(e, t);
                acts.push((format!("PICK {who} · {what} · ranked {}", e.rank_of(t)), Act::Pick(t)));
            }
        }
        Next::Invite(a) => {
            let cap = e.selection.alliances[*a][0];
            title = format!("Alliance {} invites you", a + 1);
            lines.push(story::after_quals(e));
            lines.push(format!("{}, the captain of alliance {}, wants you on their alliance. Decline and you can't be picked by anyone else (though you could still captain one if a captain's spot opens).", team_line(e, cap).0, a + 1));
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
            acts.push(("BACK TO THE MENU".into(), Act::Continue));
        }
    }
    if let Some(a) = &p.after { lines.insert(0, a.clone()); }
    acts.push(("SAVE AND QUIT TO MENU".into(), Act::Menu));
    p.focus = p.focus.min(acts.len() - 1);
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
            b.spawn((f.text(format!("{} · {} · {} difficulty · {}", s.your_name, Roster::get().robot(&s.your_robot).short(), s.difficulty.name(), rec), Face::Inter(400), 14.0, color::MUTED), Node { margin: UiRect::bottom(Val::Px(12.0)), ..default() }));
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
                                        let (who, what) = team_line(e, t);
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

fn ordinal(n: usize) -> String { match n % 10 { 1 if n % 100 != 11 => format!("{n}st"), 2 if n % 100 != 12 => format!("{n}nd"), 3 if n % 100 != 13 => format!("{n}rd"), _ => format!("{n}th") } }
