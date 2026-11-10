//! The overlays over a match or the menu (index.html #pause, #results, #controls; js ui.js
//! renderPause, showResults, _buildControls).

use bevy::prelude::*;
use sim::constants::{other, Alliance};
use sim::game_match::ai;

use super::menu::ControlsOpen;
use super::{color, radius, read_nav, Face, Fonts, NavRepeat, Screen};
use crate::game::{Game, GameCmd};
use crate::settings::Roster;

pub struct OverlaysPlugin;

impl Plugin for OverlaysPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Choice>()
            .add_systems(OnEnter(Screen::Paused), |mut c: ResMut<Choice>| *c = Choice(0, true, false))
            .add_systems(OnEnter(Screen::Results), |mut c: ResMut<Choice>| *c = Choice(0, true, false))
            .add_systems(OnExit(Screen::Paused), despawn::<PauseRoot>)
            .add_systems(OnExit(Screen::Results), despawn::<ResultsRoot>)
            .add_systems(Update, (pause_input, draw_pause).chain().run_if(in_state(Screen::Paused)))
            .add_systems(Update, (results_input, draw_results).chain().run_if(in_state(Screen::Results)))
            .add_systems(Update, controls);
    }
}

/// The focused item of the pause menu or the results, whether to redraw, and whether it was clicked
#[derive(Resource, Default)]
struct Choice(usize, bool, bool);

#[derive(Component)]
struct PauseRoot;
#[derive(Component)]
struct ResultsRoot;
#[derive(Component)]
struct ControlsRoot;
#[derive(Component)]
struct Pick(usize);

fn despawn<C: Component>(mut commands: Commands, q: Query<Entity, With<C>>) {
    for e in &q { commands.entity(e).despawn(); }
}

/// css .overlay + .menuBox
fn overlay(commands: &mut Commands, marker: impl Component, width: f32, body: impl FnOnce(&mut ChildSpawnerCommands)) {
    commands.spawn((
        marker,
        Node { position_type: PositionType::Absolute, width: Val::Percent(100.0), height: Val::Percent(100.0), align_items: AlignItems::Center, justify_content: JustifyContent::Center, ..default() },
        BackgroundColor(color::rgba(5, 6, 10, 0.72)), GlobalZIndex(20),
    )).with_children(|o| {
        o.spawn((
            Node { width: Val::Px(width), max_width: Val::Percent(94.0), max_height: Val::Percent(94.0), flex_direction: FlexDirection::Column, padding: UiRect::axes(Val::Px(26.0), Val::Px(22.0)), border: UiRect::all(Val::Px(1.0)), border_radius: radius(16.0), overflow: Overflow::clip(), ..default() },
            // (solid: an overlay can open over the menu, whose text would show through)
            BackgroundColor(color::over(12, 14, 20, 1.0)), BorderColor::all(color::LINE),
        )).with_children(body);
    });
}

fn heading(b: &mut ChildSpawnerCommands, f: &Fonts, s: &str) {
    b.spawn((f.text(s, Face::Barlow(800), 34.0, color::TEXT), Node { margin: UiRect::bottom(Val::Px(12.0)), ..default() }));
}

/// css .btn (a list's): the first is the main one
fn item(b: &mut ChildSpawnerCommands, f: &Fonts, label: &str, i: usize, focused: bool, main: bool, width: Val) {
    let (bg, fg, size) = if main { (color::FUEL, color::DARK, 24.0) } else { (color::tint(0.08), color::TEXT, 18.0) };
    b.spawn((
        Node { width, justify_content: JustifyContent::Center, padding: UiRect::all(Val::Px(12.0)), border: UiRect::all(Val::Px(3.0)), border_radius: radius(10.0), ..default() },
        BackgroundColor(bg), BorderColor::all(if focused { color::tint(0.9) } else { Color::NONE }), Pick(i),
    )).observe(|ev: On<PointerClick>, q: Query<&Pick>, mut c: ResMut<Choice>| {
        if let Ok(p) = q.get(ev.entity) { c.0 = p.0; c.1 = true; c.2 = true; }
    }).observe(|ev: On<PointerOver>, q: Query<&Pick>, mut c: ResMut<Choice>| {
        if let Ok(p) = q.get(ev.entity) { if c.0 != p.0 { c.0 = p.0; c.1 = true; } }
    }).with_children(|t| { t.spawn(f.text(label, Face::Barlow(800), size, fg)); });
}


// ------------------------------------------------------------------ pause
const PAUSE: [&str; 4] = ["Resume", "Restart match", "Controls", "Main menu"];

fn pause_input(
    keys: Res<ButtonInput<KeyCode>>,
    pads: Query<&Gamepad>,
    time: Res<Time>,
    mut rep: Local<NavRepeat>,
    mut c: ResMut<Choice>,
    mut controls: ResMut<ControlsOpen>,
    mut cmds: MessageWriter<GameCmd>,
) {
    let nav = read_nav(&keys, &pads, &mut rep, time.delta_secs());
    let clicked = std::mem::take(&mut c.2);
    // (and not on the press that closed the Controls)
    if controls.0 || controls.is_changed() { return }
    if nav.up && c.0 > 0 { c.0 -= 1; c.1 = true; }
    if nav.down && c.0 + 1 < PAUSE.len() { c.0 += 1; c.1 = true; }
    let start = pads.iter().any(|p| p.just_pressed(GamepadButton::Start));
    if (nav.back || start) && !keys.just_pressed(KeyCode::Backspace) { cmds.write(GameCmd::Resume); return }
    if nav.ok || clicked {
        match c.0 {
            0 => { cmds.write(GameCmd::Resume); }
            1 => { cmds.write(GameCmd::Restart); }
            2 => controls.0 = true,
            _ => { cmds.write(GameCmd::ToMenu); }
        }
    }
}

fn draw_pause(mut commands: Commands, mut c: ResMut<Choice>, fonts: Res<Fonts>, roots: Query<Entity, With<PauseRoot>>) {
    if !c.1 { return }
    c.1 = false;
    for e in &roots { commands.entity(e).despawn(); }
    let f = fonts.clone();
    let focus = c.0;
    overlay(&mut commands, PauseRoot, 360.0, |b| {
        heading(b, &f, "Paused");
        b.spawn(Node { flex_direction: FlexDirection::Column, row_gap: Val::Px(8.0), ..default() }).with_children(|l| {
            for (i, label) in PAUSE.iter().enumerate() { item(l, &f, label, i, i == focus, i == 0, Val::Percent(100.0)); }
        });
    });
}

// ------------------------------------------------------------------ results
fn results_input(
    keys: Res<ButtonInput<KeyCode>>,
    pads: Query<&Gamepad>,
    time: Res<Time>,
    mut rep: Local<NavRepeat>,
    mut c: ResMut<Choice>,
    mut cmds: MessageWriter<GameCmd>,
) {
    let nav = read_nav(&keys, &pads, &mut rep, time.delta_secs());
    let clicked = std::mem::take(&mut c.2);
    if (nav.left || nav.up) && c.0 > 0 { c.0 = 0; c.1 = true; }
    if (nav.right || nav.down) && c.0 == 0 { c.0 = 1; c.1 = true; }
    // (A: rematch, B: main menu, as their labels say)
    if nav.back { cmds.write(GameCmd::ToMenu); return }
    if nav.ok || clicked { cmds.write(if c.0 == 0 { GameCmd::Restart } else { GameCmd::ToMenu }); }
}

/// js ui.js showResults
fn draw_results(mut commands: Commands, mut c: ResMut<Choice>, fonts: Res<Fonts>, game: Res<Game>, roots: Query<Entity, With<ResultsRoot>>) {
    if !c.1 { return }
    c.1 = false;
    for e in &roots { commands.entity(e).despawn(); }
    let f = fonts.clone();
    let focus = c.0;
    let m = &game.0.match_;
    let me = m.player;
    let (tb, tr) = (m.total(Alliance::Blue), m.total(Alliance::Red));
    let win = if tb == tr { "TIE".to_string() } else { format!("{} WINS", if tb > tr { "BLUE" } else { "RED" }) };
    let s = &m.score[ai(me)];
    let up = |a: Alliance| if a == Alliance::Blue { "BLUE" } else { "RED" };
    overlay(&mut commands, ResultsRoot, 980.0, |b| {
        heading(b, &f, "Match Results");
        b.spawn((f.text(win, Face::Barlow(800), 30.0, color::TEXT), Node { margin: UiRect::bottom(Val::Px(10.0)), ..default() }));
        // css .resgrid: each ALLIANCE's points, VS between
        b.spawn(Node { display: Display::Grid, grid_template_columns: vec![GridTrack::flex(1.0), GridTrack::auto(), GridTrack::flex(1.0)], column_gap: Val::Px(16.0), align_items: AlignItems::Center, ..default() })
            .with_children(|g| {
                let col = |g: &mut ChildSpawnerCommands, a: Alliance| {
                    let sc = &m.score[ai(a)];
                    let tint = if a == Alliance::Blue { color::BLUE } else { color::RED };
                    g.spawn((Node { flex_direction: FlexDirection::Column, padding: UiRect::all(Val::Px(12.0)), border: UiRect::left(Val::Px(4.0)), border_radius: radius(10.0), ..default() }, BackgroundColor(color::tint(0.04)), BorderColor::all(tint)))
                        .with_children(|c| {
                            c.spawn(f.text(format!("{}", m.total(a)), Face::Barlow(800), 56.0, tint));
                            for (k, v) in [("AUTO FUEL", sc.auto_fuel), ("TELEOP FUEL", sc.tele_fuel), ("TOWER", sc.auto_tower + sc.tele_tower), ("FOUL points received", sc.foul_pts)] {
                                c.spawn(Node { flex_direction: FlexDirection::Row, justify_content: JustifyContent::SpaceBetween, ..default() }).with_children(|r| {
                                    r.spawn(f.text(k, Face::Inter(400), 13.0, color::MUTED));
                                    r.spawn(f.text(format!("{v}"), Face::Inter(600), 13.0, color::TEXT));
                                });
                            }
                        });
                };
                col(g, Alliance::Blue);
                g.spawn(f.text("VS", Face::Barlow(800), 28.0, color::MUTED));
                col(g, Alliance::Red);
            });
        // css .stats
        b.spawn(Node { display: Display::Grid, grid_template_columns: RepeatedGridTrack::flex(4, 1.0), column_gap: Val::Px(10.0), margin: UiRect::vertical(Val::Px(12.0)), ..default() })
            .with_children(|g| {
                for (v, k) in [
                    (s.auto_fuel + s.tele_fuel, format!("{} FUEL scored in an active HUB", up(me))),
                    (s.inactive_fuel, format!("{} FUEL into an inactive HUB (0 pts)", up(me))),
                    (m.score[0].fouls.len() as f64, "BLUE fouls".into()),
                    (m.score[1].fouls.len() as f64, "RED fouls".into()),
                ] {
                    g.spawn((Node { flex_direction: FlexDirection::Column, padding: UiRect::all(Val::Px(8.0)), border_radius: radius(8.0), ..default() }, BackgroundColor(color::tint(0.04))))
                        .with_children(|st| {
                            st.spawn(f.text(format!("{v}"), Face::Barlow(800), 28.0, color::TEXT));
                            st.spawn(f.text(k, Face::Inter(400), 12.0, color::MUTED));
                        });
                }
            });
        // every robot's match (css .restable)
        if let Some(r) = game.0.robots.first() {
            let info = Roster::get().robot(&r.key);
            b.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(12.0), padding: UiRect::vertical(Val::Px(4.0)), border: UiRect::vertical(Val::Px(1.0)), ..default() })
                .insert(BorderColor::all(color::LINE))
                .with_children(|row| {
                    row.spawn(f.text(format!("{}1", if r.alliance == Alliance::Blue { "B" } else { "R" }), Face::Inter(600), 13.0, if r.alliance == Alliance::Blue { color::BLUE } else { color::RED }));
                    row.spawn(f.text(format!("{} {}", info.team, info.archetype), Face::Inter(700), 13.0, color::TEXT));
                    row.spawn(f.text("Driven by you · launching, passing and intake counts come with the mechanisms", Face::Inter(400), 13.0, color::MUTED));
                });
        }
        let fouls = |a: Alliance| -> String {
            let fl = &m.score[ai(a)].fouls;
            if fl.is_empty() { return if a == me { "None — clean match!".into() } else { "None".into() } }
            fl.iter().map(|x| format!("{} {} (+{} to {}): {}", x.rule, if x.major { "MAJOR" } else { "MINOR" }, x.pts, up(other(a)), x.desc)).collect::<Vec<_>>().join("\n")
        };
        for a in [me, other(me)] {
            b.spawn((f.text(format!("{} ALLIANCE fouls ({})", if a == me { "Your".to_string() } else { up(a).to_string() }, m.score[ai(a)].fouls.len()), Face::Barlow(700), 15.0, color::MUTED), Node { margin: UiRect::top(Val::Px(8.0)), ..default() }));
            b.spawn(f.text(fouls(a), Face::Inter(400), 13.0, color::TEXT));
        }
        b.spawn(Node { flex_direction: FlexDirection::Row, justify_content: JustifyContent::Center, column_gap: Val::Px(10.0), margin: UiRect::top(Val::Px(16.0)), ..default() })
            .with_children(|l| {
                item(l, &f, "Rematch (A)", 0, focus == 0, true, Val::Px(220.0));
                item(l, &f, "Main menu (B)", 1, focus == 1, false, Val::Px(220.0));
            });
    });
}

// ------------------------------------------------------------------ controls
/// js ui.js CONTROLS: what this version does, then what's still to come
const CONTROLS: [(&str, &str, &str, bool); 15] = [
    ("Drive (field-relative)", "Left stick", "W A S D", true),
    ("Rotate", "Right stick X", "Q / E or ← →", true),
    ("Intake", "LT", "Shift", true),
    ("Slow mode (hold)", "Left stick click", "X", true),
    ("Pause", "Menu button", "Esc", true),
    ("Restart match", "View button", "Backspace", true),
    ("Camera", "D-pad ← / →", "[ / ]", true),
    ("Turn camera around 180°", "Right stick click", "T", true),
    ("Shoot (auto-aim, shoot on the move)", "RT", "Space", true),
    ("Pass FUEL to your ALLIANCE ZONE corner", "RB (hold)", "R", true),
    ("Outtake / eject", "LB", "F", true),
    ("Human player: open/close CHUTE DOOR", "X", "G", false),
    ("Human player: throw FUEL at HUB", "Y (hold)", "H", false),
    ("Climb / cancel / lower (add-on)", "A", "C", false),
    ("Toggle field / robot-relative", "B", "B", false),
];

fn controls(
    mut commands: Commands,
    mut open: ResMut<ControlsOpen>,
    keys: Res<ButtonInput<KeyCode>>,
    pads: Query<&Gamepad>,
    fonts: Res<Fonts>,
    roots: Query<Entity, With<ControlsRoot>>,
) {
    if !open.0 {
        for e in &roots { commands.entity(e).despawn(); }
        return;
    }
    let close = keys.just_pressed(KeyCode::Escape) || keys.just_pressed(KeyCode::Backspace) || pads.iter().any(|p| p.just_pressed(GamepadButton::East));
    if close && !open.is_changed() { open.0 = false; return }
    if !roots.is_empty() { return }
    let f = fonts.clone();
    overlay(&mut commands, ControlsRoot, 860.0, |b| {
        heading(b, &f, "Controls");
        b.spawn(Node { display: Display::Grid, grid_template_columns: vec![GridTrack::flex(1.6), GridTrack::flex(1.0), GridTrack::flex(1.0)], row_gap: Val::Px(3.0), column_gap: Val::Px(12.0), ..default() })
            .with_children(|g| {
                for h in ["Action", "Xbox controller", "Keyboard"] { g.spawn(f.text(h, Face::Inter(600), 13.0, color::MUTED)); }
                for (i, (a, pad, key, works)) in CONTROLS.iter().enumerate() {
                    if i == 8 {
                        g.spawn((Node { grid_column: GridPlacement::span(3), margin: UiRect::top(Val::Px(8.0)), ..default() },)).with_children(|s| {
                            s.spawn(f.text("COMING WITH THE MECHANISMS", Face::Barlow(700), 14.0, color::MUTED));
                        });
                    }
                    let c = if *works { color::TEXT } else { color::MUTED };
                    g.spawn(f.text(*a, Face::Inter(400), 13.0, c));
                    g.spawn(f.text(*pad, Face::Inter(400), 13.0, c));
                    g.spawn(f.text(*key, Face::Inter(600), 13.0, c));
                }
            });
        b.spawn((f.text("Menus: D-pad / left stick (or the arrows) to move, left / right to change, A (Enter) to select, B (Esc) to go back. With a mouse, click the ‹ › buttons, a value, or any team in the robot strip.", Face::Inter(400), 13.0, color::MUTED), Node { margin: UiRect::top(Val::Px(12.0)), ..default() }));
        b.spawn((f.text("Press B / Esc to close", Face::Inter(400), 13.0, color::MUTED), Node { margin: UiRect::top(Val::Px(8.0)), align_self: AlignSelf::Center, ..default() }));
    });
}
