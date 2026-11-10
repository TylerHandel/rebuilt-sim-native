//! The match HUD (index.html #hud, js ui.js updateHUD): the score bar with the clock and the HUBS,
//! your HUB's state, the TELEOP timeline, your robot's panel, the HUMAN PLAYER's panel, the camera
//! name and the controls hint. Shown while playing (and under pause and the results).

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use sim::constants::{Alliance, ALLIANCE_ZONE_DEPTH, HALF_L, TIMING};
use sim::game_match::{ai, HubLight, Phase};

use super::{color, radius, Face, Fonts, Screen};
use crate::game::{Driver, Game, StepTime};
use crate::settings::{Roster, Settings};
use crate::update::UpdateStatus;

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
            .add_systems(OnEnter(Screen::Playing), spawn)
            .add_systems(OnEnter(Screen::Menu), despawn)
            .add_systems(Update, (refresh, blink).run_if(|s: Res<State<Screen>>| *s.get() != Screen::Menu))
            .add_systems(Update, fps_box);
    }
}

#[derive(Component)]
struct HudRoot;
#[derive(Component, Clone, Copy, PartialEq, Eq)]
enum T { Cam, ScoreBlue, ScoreRed, Phase, Clock, MyHubState, MyHubSub, RpTeam, RpName, FuelText, Shooter, Zone, Intake, Drive, Chute, Corral, Door, HpMode, Fouls }
#[derive(Component)]
struct HubPill(Alliance);
#[derive(Component)]
struct MyHub;
#[derive(Component)]
struct Seg(usize);
#[derive(Component)]
struct SegProg(usize);
#[derive(Component)]
struct FuelFill;
/// Pulses (css @keyframes pulse) while its HUB's lights warn
#[derive(Component)]
struct Pulse(bool);

fn despawn(mut commands: Commands, q: Query<Entity, With<HudRoot>>) {
    for e in &q { commands.entity(e).despawn(); }
}

fn spawn(mut commands: Commands, fonts: Res<Fonts>, existing: Query<(), With<HudRoot>>) {
    if !existing.is_empty() { return }
    let f = &*fonts;
    commands.spawn((HudRoot, Node { position_type: PositionType::Absolute, width: Val::Percent(100.0), height: Val::Percent(100.0), ..default() }, Pickable::IGNORE))
        .with_children(|h| {
            // the score bar (css #scorebar), centered at the top
            h.spawn(Node { position_type: PositionType::Absolute, top: Val::Px(12.0), width: Val::Percent(100.0), justify_content: JustifyContent::Center, ..default() })
                .with_children(|row| {
                    row.spawn((Node { flex_direction: FlexDirection::Row, border_radius: radius(10.0), overflow: Overflow::clip(), ..default() }, BoxShadow::new(Color::srgba(0.0, 0.0, 0.0, 0.45), Val::Px(0.0), Val::Px(6.0), Val::Px(0.0), Val::Px(24.0))))
                        .with_children(|bar| {
                            let team = |bar: &mut ChildSpawnerCommands, a: Alliance| {
                                let (c0, c1) = if a == Alliance::Blue { (color::BLUE_D, color::BLUE) } else { (color::RED, color::RED_D) };
                                bar.spawn((
                                    Node { flex_direction: FlexDirection::Row, align_items: AlignItems::Center, column_gap: Val::Px(12.0), padding: UiRect::axes(Val::Px(18.0), Val::Px(0.0)), min_width: Val::Px(150.0), justify_content: if a == Alliance::Blue { JustifyContent::FlexStart } else { JustifyContent::FlexEnd }, ..default() },
                                    BackgroundGradient::from(LinearGradient::to_right(vec![c0.into(), c1.into()])),
                                )).with_children(|t| {
                                    let lbl = |t: &mut ChildSpawnerCommands| { t.spawn(f.text(if a == Alliance::Blue { "BLUE" } else { "RED" }, Face::Barlow(700), 18.0, color::white(0.85))); };
                                    let pts = |t: &mut ChildSpawnerCommands| { t.spawn((f.text("0", Face::Barlow(800), 48.0, Color::WHITE), if a == Alliance::Blue { T::ScoreBlue } else { T::ScoreRed })); };
                                    if a == Alliance::Blue { lbl(t); pts(t); } else { pts(t); lbl(t); }
                                });
                            };
                            team(bar, Alliance::Blue);
                            bar.spawn((Node { flex_direction: FlexDirection::Column, align_items: AlignItems::Center, padding: UiRect::new(Val::Px(22.0), Val::Px(22.0), Val::Px(6.0), Val::Px(8.0)), min_width: Val::Px(210.0), ..default() }, BackgroundColor(color::rgba(0x10, 0x13, 0x1a, 1.0))))
                                .with_children(|c| {
                                    c.spawn((f.text("", Face::Barlow(700), 16.0, color::MUTED), T::Phase));
                                    c.spawn((f.text("", Face::Barlow(800), 44.0, Color::WHITE), T::Clock));
                                    c.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(6.0), margin: UiRect::top(Val::Px(4.0)), ..default() }).with_children(|hubs| {
                                        for a in [Alliance::Blue, Alliance::Red] {
                                            hubs.spawn((Node { padding: UiRect::axes(Val::Px(8.0), Val::Px(1.0)), border: UiRect::all(Val::Px(1.0)), border_radius: radius(99.0), ..default() }, BackgroundColor(Color::NONE), BorderColor::all(color::LINE), HubPill(a), Pulse(false)))
                                                .with_children(|p| { p.spawn(f.text("HUB", Face::Inter(700), 12.0, color::MUTED)); });
                                        }
                                    });
                                });
                            team(bar, Alliance::Red);
                        });
                });
            // your HUB (css #myhub)
            h.spawn(Node { position_type: PositionType::Absolute, top: Val::Px(118.0), width: Val::Percent(100.0), justify_content: JustifyContent::Center, ..default() })
                .with_children(|row| {
                    row.spawn((Node { flex_direction: FlexDirection::Column, align_items: AlignItems::Center, padding: UiRect::new(Val::Px(16.0), Val::Px(16.0), Val::Px(4.0), Val::Px(6.0)), border: UiRect::all(Val::Px(1.0)), border_radius: radius(8.0), ..default() }, BackgroundColor(color::PANEL), BorderColor::all(color::LINE), MyHub, Pulse(false)))
                        .with_children(|m| {
                            m.spawn((f.text("", Face::Barlow(800), 22.0, color::OK), T::MyHubState));
                            m.spawn((f.text("", Face::Barlow(600), 14.0, color::MUTED), T::MyHubSub));
                        });
                });
            // the timeline (css #timeline): AUTO, TRANSITION, the four SHIFTS, END GAME
            h.spawn(Node { position_type: PositionType::Absolute, top: Val::Px(186.0), width: Val::Percent(100.0), justify_content: JustifyContent::Center, ..default() })
                .with_children(|row| {
                    row.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(2.0), ..default() }).with_children(|tl| {
                        let segs = [("AUTO", TIMING::auto), ("TRANS", TIMING::transition), ("SHIFT 1", TIMING::shift), ("SHIFT 2", TIMING::shift), ("SHIFT 3", TIMING::shift), ("SHIFT 4", TIMING::shift), ("END GAME", TIMING::endgame)];
                        for (i, (n, d)) in segs.iter().enumerate() {
                            tl.spawn((Node { width: Val::Px(*d as f32 * 3.3), height: Val::Px(18.0), justify_content: JustifyContent::Center, align_items: AlignItems::Center, border_radius: radius(3.0), overflow: Overflow::clip(), ..default() }, BackgroundColor(color::tint(0.08)), Seg(i)))
                                .with_children(|s| {
                                    s.spawn((Node { position_type: PositionType::Absolute, left: Val::Px(0.0), top: Val::Px(0.0), bottom: Val::Px(0.0), width: Val::Percent(0.0), ..default() }, BackgroundColor(color::white(0.22)), SegProg(i)));
                                    s.spawn((f.text(*n, Face::Barlow(700), 11.0, color::white(0.8)), TextLayout::no_wrap()));
                                });
                        }
                    });
                });
            // your robot (css #robotPanel)
            panel(h, Node { left: Val::Px(14.0), bottom: Val::Px(14.0), width: Val::Px(270.0), ..panel_node() }, |p| {
                p.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(6.0), margin: UiRect::bottom(Val::Px(6.0)), ..default() }).with_children(|t| {
                    t.spawn((f.text("", Face::Barlow(700), 19.0, color::TEXT), T::RpTeam));
                    t.spawn((f.text("", Face::Barlow(700), 19.0, color::MUTED), T::RpName));
                });
                p.spawn((Node { height: Val::Px(22.0), border_radius: radius(6.0), overflow: Overflow::clip(), margin: UiRect::bottom(Val::Px(8.0)), justify_content: JustifyContent::Center, align_items: AlignItems::Center, ..default() }, BackgroundColor(color::white(0.08))))
                    .with_children(|b| {
                        b.spawn((Node { position_type: PositionType::Absolute, left: Val::Px(0.0), top: Val::Px(0.0), bottom: Val::Px(0.0), width: Val::Percent(0.0), ..default() }, BackgroundGradient::from(LinearGradient::to_right(vec![color::rgba(0xd9, 0xb2, 0x1c, 1.0).into(), color::FUEL.into()])), FuelFill));
                        b.spawn((f.text("", Face::Inter(700), 13.0, color::DARK), T::FuelText));
                    });
                for (k, t) in [("Shooter", T::Shooter), ("Zone", T::Zone), ("Intake", T::Intake), ("Drive", T::Drive)] { row(p, f, k, t); }
            });
            // the HUMAN PLAYER (css #hpPanel)
            panel(h, Node { right: Val::Px(14.0), bottom: Val::Px(14.0), width: Val::Px(220.0), ..panel_node() }, |p| {
                p.spawn((f.text("Human Player", Face::Barlow(700), 19.0, color::TEXT), Node { margin: UiRect::bottom(Val::Px(6.0)), ..default() }));
                for (k, t) in [("Chute", T::Chute), ("Corral", T::Corral), ("Chute door", T::Door), ("Mode", T::HpMode)] { row(p, f, k, t); }
                p.spawn((f.text("Fouls", Face::Barlow(700), 15.0, color::MUTED), Node { margin: UiRect::new(Val::Px(0.0), Val::Px(0.0), Val::Px(8.0), Val::Px(2.0)), ..default() }));
                row(p, f, "Committed", T::Fouls);
            });
            // the camera (css #camName)
            h.spawn((Node { position_type: PositionType::Absolute, top: Val::Px(14.0), right: Val::Px(16.0), padding: UiRect::axes(Val::Px(10.0), Val::Px(4.0)), border: UiRect::all(Val::Px(1.0)), border_radius: radius(6.0), ..default() }, BackgroundColor(color::PANEL), BorderColor::all(color::LINE)))
                .with_children(|c| { c.spawn((f.text("", Face::Inter(400), 12.0, color::MUTED), T::Cam)); });
            // the controls (css #hint): what works in this version
            h.spawn(Node { position_type: PositionType::Absolute, bottom: Val::Px(14.0), width: Val::Percent(100.0), justify_content: JustifyContent::Center, ..default() })
                .with_children(|row| {
                    row.spawn((Node { padding: UiRect::axes(Val::Px(12.0), Val::Px(5.0)), border: UiRect::all(Val::Px(1.0)), border_radius: radius(6.0), ..default() }, BackgroundColor(color::PANEL), BorderColor::all(color::LINE)))
                        .with_children(|t| {
                            t.spawn((Text::default(), f.font(Face::Inter(400), 12.0), TextColor(color::MUTED))).with_children(|s| {
                                for (i, (k, v)) in [("WASD", "Drive"), ("Q/E", "Rotate"), ("Shift", "Intake"), ("X", "Slow"), ("[ ]", "Camera"), ("T", "Turn around"), ("Backspace", "Restart"), ("Esc", "Pause")].iter().enumerate() {
                                    s.spawn((TextSpan::new(format!("{}{k}", if i > 0 { " · " } else { "" })), f.font(Face::Inter(700), 12.0), TextColor(color::TEXT)));
                                    s.spawn((TextSpan::new(format!(" {v}")), f.font(Face::Inter(400), 12.0), TextColor(color::MUTED)));
                                }
                            });
                        });
                });
        });
}

fn panel_node() -> Node {
    Node { position_type: PositionType::Absolute, flex_direction: FlexDirection::Column, padding: UiRect::axes(Val::Px(14.0), Val::Px(10.0)), border: UiRect::all(Val::Px(1.0)), border_radius: radius(10.0), ..default() }
}

fn panel(h: &mut ChildSpawnerCommands, node: Node, body: impl FnOnce(&mut ChildSpawnerCommands)) {
    h.spawn((node, BackgroundColor(color::PANEL), BorderColor::all(color::LINE))).with_children(body);
}

/// css .rows > div: a name and its value
fn row(p: &mut ChildSpawnerCommands, f: &Fonts, k: &str, t: T) {
    p.spawn((Node { flex_direction: FlexDirection::Row, justify_content: JustifyContent::SpaceBetween, padding: UiRect::vertical(Val::Px(2.0)), border: UiRect::bottom(Val::Px(1.0)), column_gap: Val::Px(8.0), ..default() }, BorderColor::all(color::white(0.05))))
        .with_children(|r| {
            r.spawn(f.text(k, Face::Inter(400), 13.0, color::MUTED));
            r.spawn((f.text("—", Face::Inter(600), 13.0, color::TEXT), TextLayout::new(Justify::Right, LineBreak::NoWrap), t));
        });
}

#[allow(clippy::too_many_arguments)]
fn refresh(
    game: Res<Game>,
    driver: Res<Driver>,
    rig: Res<crate::camera::CameraRig>,
    mut texts: Query<(&T, &mut Text, &mut TextColor)>,
    mut pills: Query<(&HubPill, &mut BackgroundColor, &mut BorderColor, &mut Pulse, &Children)>,
    mut pill_text: Query<(&mut Text, &mut TextColor), (Without<T>, Without<HubPill>)>,
    mut myhub: Query<&mut Pulse, (With<MyHub>, Without<HubPill>)>,
    mut segs: Query<(&Seg, &mut BackgroundColor), Without<HubPill>>,
    mut progs: Query<(&SegProg, &mut Node), Without<FuelFill>>,
    mut fill: Query<&mut Node, (With<FuelFill>, Without<SegProg>)>,
) {
    let w = &game.0;
    let m = &w.match_;
    let me = m.player;
    let Some(r) = w.robots.first() else { return };
    let info = Roster::get().robot(&r.key);
    let on = m.hub_active(me);
    let nc = if m.phase == Phase::Auto { None } else { m.hub_next_change(me) };
    // js ui.js updateHUD: your HUB's line
    let sub = match m.phase {
        Phase::Pre => "Match starting".to_string(),
        Phase::Auto => "AUTO — both HUBS active".into(),
        Phase::AutoGap => if m.first_inactive.is_some() { String::new() } else { "Waiting for FMS game data…".into() },
        Phase::Teleop => {
            let mut s = match nc {
                Some(n) => if on { format!("goes inactive in {}s", n.ceil()) } else { format!("active in {}s", n.ceil()) },
                None => if on { "active until the end".into() } else { String::new() },
            };
            if m.shift_index() == 0 && m.first_inactive.is_some() {
                s += if m.first_inactive == Some(me) { " · you go inactive first" } else { " · opponent goes inactive first" };
            }
            s
        }
        Phase::Post => "FUEL still being counted (3s)".into(),
        Phase::Done => String::new(),
    };
    // any BUMPER corner on this side of the ALLIANCE ZONE line (js robot.js inAllianceZone)
    let line = -HALF_L + ALLIANCE_ZONE_DEPTH;
    let in_zone = [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)].iter().any(|(sx, sz)| {
        let c = sim::robot::apply_quat([sx * r.half_l, 0.0, sz * r.half_w], r.quat);
        let x = r.pos[0] + c[0];
        if r.alliance == Alliance::Blue { x <= line } else { x >= -line }
    });
    let fouls = &m.score[ai(me)].fouls;
    // (from +0: an empty f64 sum is -0, which prints as "-0")
    let foul_pts = fouls.iter().fold(0.0, |a, x| a + x.pts);
    for (t, mut text, mut col) in &mut texts {
        let (s, c): (String, Option<Color>) = match t {
            T::Cam => (format!("{}{}", rig.mode.name(), if rig.flip { " · turned around" } else { "" }), None),
            T::ScoreBlue => (format!("{}", m.total(Alliance::Blue)), None),
            T::ScoreRed => (format!("{}", m.total(Alliance::Red)), None),
            T::Phase => (m.phase_label().into(), None),
            T::Clock => (m.clock_text(), None),
            T::MyHubState => (if m.phase == Phase::Pre { "GET READY" } else if on { "YOUR HUB ACTIVE" } else { "YOUR HUB INACTIVE" }.into(), Some(if on { color::OK } else { color::BAD })),
            T::MyHubSub => (sub.clone(), None),
            T::RpTeam => (info.team.to_string(), None),
            T::RpName => (if info.robot_name.is_empty() || info.robot_name == info.archetype { format!("{} · {}", info.team_name, info.archetype) } else { format!("{} · {}", info.robot_name, info.archetype) }, None),
            T::FuelText => (if r.full { format!("{} FUEL · FULL", r.stored.len()) } else { format!("{} / ~{} FUEL", r.stored.len(), r.capacity()) }, None),
            T::Shooter => {
                let st = &r.status;
                let c = if r.ready { color::OK } else if st.contains("range") || st.contains("Outside") || st.contains("Disabled") { color::BAD } else if st == "Empty" { color::TEXT } else { color::WARN };
                (st.clone(), Some(c))
            }
            T::Zone => (if in_zone { "ALLIANCE ZONE (can score)" } else { "Outside — pass only" }.into(), Some(if in_zone { color::OK } else { color::WARN })),
            T::Intake => (if r.out.intake_speed > 0.0 { if r.full { "Intaking (full)" } else { "Intaking" } } else if r.out.intake_speed < 0.0 { "Ejecting" } else if r.intake_deploy > 0.5 { "Deployed" } else { "Stowed" }.into(), None),
            T::Drive => (format!("Field-relative{}{}", if driver.1 { " · SLOW" } else { "" }, if m.is_auto() { " · AUTO" } else { "" }), None),
            T::Chute => (w.fuel.chute_count(me).to_string(), None),
            T::Corral => (w.fuel.corral_balls(me).len().to_string(), None),
            T::Door => (if w.hps[ai(me)].door_open { "OPEN" } else { "Closed" }.into(), None),
            T::HpMode => (if w.hps[ai(me)].auto { "Auto-throw" } else { "Manual (Y / H)" }.into(), None),
            T::Fouls => (format!("{} ({} pts to opp.)", fouls.len(), foul_pts), None),
        };
        if text.0 != s { text.0 = s; }
        if let Some(c) = c { if col.0 != c { col.0 = c; } }
    }
    for (p, mut bg, mut border, mut pulse, kids) in &mut pills {
        let on = m.hub_active(p.0);
        let c = if p.0 == Alliance::Blue { color::BLUE } else { color::RED };
        bg.0 = if on { c } else { Color::NONE };
        *border = BorderColor::all(if on { Color::NONE } else { color::LINE });
        pulse.0 = m.hub_light_mode(p.0) == HubLight::Warning;
        for k in kids.iter() {
            if let Ok((mut t, mut tc)) = pill_text.get_mut(k) {
                let s = if on { "HUB ON" } else { "HUB OFF" };
                if t.0 != s { t.0 = s.into(); }
                tc.0 = if on { Color::WHITE } else { color::MUTED };
            }
        }
    }
    for mut p in &mut myhub { p.0 = m.hub_light_mode(me) == HubLight::Warning; }
    // the timeline: how far through each part, and whose HUB is active in each SHIFT
    let auto_frac = match m.phase { Phase::Auto => m.phase_time / TIMING::auto, Phase::Pre => 0.0, _ => 1.0 };
    let tt = m.teleop_elapsed();
    let b = [0.0, TIMING::transition, TIMING::transition + TIMING::shift, TIMING::transition + 2.0 * TIMING::shift, TIMING::transition + 3.0 * TIMING::shift, TIMING::transition + 4.0 * TIMING::shift, TIMING::teleop];
    for (s, mut node) in &mut progs {
        let i = s.0;
        let frac = if i == 0 { auto_frac } else { ((tt - b[i - 1]) / (b[i] - b[i - 1])).clamp(0.0, 1.0) };
        node.width = Val::Percent(frac as f32 * 100.0);
    }
    for (s, mut bg) in &mut segs {
        let i = s.0;
        // (the browser's tints over a dark panel: over the carpet they'd mix with its colors)
        let mine = color::over(55, 214, 122, 0.35);
        bg.0 = if (2..=5).contains(&i) {
            match m.first_inactive {
                Some(fi) => { let odd = (i - 1) % 2 == 1; if if odd { me != fi } else { me == fi } { mine } else { color::over(255, 90, 90, 0.22) } }
                None => color::tint(0.08),
            }
        } else { mine };
    }
    let pct = if r.full { 100.0 } else { (100.0 * r.stored.len() as f32 / r.capacity().max(1) as f32).min(100.0) };
    for mut n in &mut fill { if n.width != Val::Percent(pct) { n.width = Val::Percent(pct); } }
}

/// css @keyframes pulse: a warning HUB blinks
fn blink(time: Res<Time>, mut q: Query<(&Pulse, &mut BackgroundColor)>) {
    let k = (time.elapsed_secs() / 0.35) as i32 % 2 == 0;
    for (p, mut bg) in &mut q {
        if !p.0 { continue }
        bg.0 = bg.0.with_alpha(if k { 1.0 } else { 0.35 });
    }
}

#[derive(Component)]
struct FpsBox;

/// css #fps: the frame rate, physics time and update status in the corner (Settings: Show FPS)
fn fps_box(
    mut commands: Commands,
    settings: Res<Settings>,
    fonts: Res<Fonts>,
    diag: Res<DiagnosticsStore>,
    st: Res<StepTime>,
    upd: Res<UpdateStatus>,
    mut q: Query<(Entity, &mut Text), With<FpsBox>>,
) {
    if !settings.fps {
        for (e, _) in &q { commands.entity(e).despawn(); }
        return;
    }
    let fps = diag.get(&FrameTimeDiagnosticsPlugin::FPS).and_then(|d| d.smoothed()).unwrap_or(0.0);
    let s = format!("{fps:.0} fps · physics {:.2} ms/step · graphics {}\n{}", st.0, settings.graphics, upd.text());
    if let Ok((_, mut t)) = q.single_mut() { if t.0 != s { t.0 = s; } return }
    commands.spawn((
        Text::new(s), fonts.font(Face::Inter(500), 11.0), TextColor(color::rgba(0xcf, 0xe3, 0xff, 1.0)),
        Node { position_type: PositionType::Absolute, left: Val::Px(10.0), top: Val::Px(10.0), max_width: Val::Px(320.0), padding: UiRect::axes(Val::Px(8.0), Val::Px(5.0)), border_radius: radius(6.0), ..default() },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.55)), GlobalZIndex(50), FpsBox, Pickable::IGNORE,
    ));
}
