//! The loading screen: over everything while the field's CAD comes in and the arena's light is
//! worked out (arena.rs), with the update check's word on a new version (update.rs) and the
//! credits. It fades once the game's ready (a key, click or button skips the rest of its time).

use bevy::prelude::*;

use crate::field::FieldScene;
use crate::settings::{Roster, Settings};
use crate::ui::{color, radius, Face, Fonts};
use crate::update::UpdateStatus;

/// The least it's shown, s (long enough to read who made what)
const MIN: f32 = 4.0;
/// How long it takes to fade, s
const FADE: f32 = 0.6;

pub struct LoadingPlugin;

impl Plugin for LoadingPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Loading { ready_at: None, fade: 0.0 })
            .add_systems(Startup, spawn)
            .add_systems(Update, progress.run_if(resource_exists::<Loading>));
    }
}

/// While it's up (the menus wait for it)
#[derive(Resource)]
pub struct Loading {
    /// when everything was in (s since start)
    ready_at: Option<f32>,
    /// how far it's faded (0-1)
    fade: f32,
}

#[derive(Component)]
struct Root;
#[derive(Component)]
struct Step(usize);
#[derive(Component)]
struct Bar;
#[derive(Component)]
struct UpdateLine;
#[derive(Component)]
struct Hint;

/// Who to thank, and the notices that go with them
fn credits() -> Vec<(&'static str, String)> {
    let teams: Vec<String> = Roster::get().robots.iter().map(|r| format!("{} {}", r.team, r.team_name)).collect();
    let mut c = vec![
        ("The robots", format!("Modeled from the public CAD, code and build write-ups of FRC teams {}. Thank you for sharing them.", teams.join(", "))),
        ("The field", "From FIRST's published REBUILT field CAD and drawings.".to_string()),
        ("Built with", "Bevy (game engine), Rapier by Dimforge (physics), Velopack (updates). Fonts: Inter and Barlow Condensed (SIL Open Font License).".to_string()),
    ];
    if cfg!(feature = "dlss") {
        c.push(("NVIDIA DLSS", "Upscaling by NVIDIA DLSS. NVIDIA and the NVIDIA logo are trademarks and/or registered trademarks of NVIDIA Corporation in the U.S. and other countries. Notices: assets/licenses.".to_string()));
    }
    c.push(("", "FIRST® and FIRST® Robotics Competition are trademarks of FIRST. REBUILT Sim is an unofficial fan project, not affiliated with or endorsed by FIRST, NVIDIA or the teams.".to_string()));
    c
}

const STEPS: [&str; 3] = ["Field CAD", "Arena lighting", "Update check"];

fn spawn(mut commands: Commands, fonts: Res<Fonts>) {
    let f = fonts.clone();
    commands.spawn((
        Root,
        Node { width: Val::Percent(100.0), height: Val::Percent(100.0), position_type: PositionType::Absolute, flex_direction: FlexDirection::Column, align_items: AlignItems::Center, justify_content: JustifyContent::Center, row_gap: Val::Px(18.0), padding: UiRect::all(Val::Px(32.0)), ..default() },
        BackgroundColor(Color::srgb(0.03, 0.035, 0.05)),
        GlobalZIndex(1000),
    )).with_children(|o| {
        // the brand
        o.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(12.0), align_items: AlignItems::Baseline, ..default() }).with_children(|l| {
            l.spawn(f.text("REBUILT", Face::Barlow(800), 72.0, color::TEXT));
            l.spawn(f.text("2026", Face::Barlow(800), 72.0, color::FUEL));
            l.spawn(f.text(format!("SIM {}", env!("CARGO_PKG_VERSION")), Face::Barlow(700), 26.0, color::MUTED));
        });
        // what's loading, and a bar
        o.spawn(Node { width: Val::Px(520.0), max_width: Val::Percent(90.0), flex_direction: FlexDirection::Column, row_gap: Val::Px(8.0), ..default() }).with_children(|p| {
            p.spawn((Node { height: Val::Px(6.0), border_radius: radius(3.0), overflow: Overflow::clip(), ..default() }, BackgroundColor(color::tint(0.08)))).with_children(|b| {
                b.spawn((Bar, Node { width: Val::Percent(0.0), height: Val::Percent(100.0), ..default() }, BackgroundColor(color::FUEL)));
            });
            p.spawn(Node { flex_direction: FlexDirection::Row, justify_content: JustifyContent::SpaceBetween, ..default() }).with_children(|r| {
                for (i, s) in STEPS.iter().enumerate() {
                    r.spawn((Step(i), f.text(format!("○ {s}"), Face::Inter(600), 13.0, color::MUTED)));
                }
            });
            p.spawn((UpdateLine, f.text("Checking for updates…", Face::Inter(400), 13.0, color::MUTED)));
        });
        // the credits
        o.spawn(Node { width: Val::Px(760.0), max_width: Val::Percent(94.0), flex_direction: FlexDirection::Column, row_gap: Val::Px(9.0), margin: UiRect::top(Val::Px(14.0)), padding: UiRect::all(Val::Px(18.0)), border: UiRect::all(Val::Px(1.0)), border_radius: radius(12.0), ..default() })
            .insert((BackgroundColor(color::tint(0.03)), BorderColor::all(color::LINE)))
            .with_children(|c| {
                for (head, body) in credits() {
                    c.spawn(Node { flex_direction: FlexDirection::Column, row_gap: Val::Px(2.0), ..default() }).with_children(|e| {
                        if !head.is_empty() { e.spawn(f.text(head.to_uppercase(), Face::Barlow(700), 14.0, color::FUEL)); }
                        e.spawn(f.text(body, Face::Inter(400), 12.5, if head.is_empty() { color::MUTED } else { color::TEXT }));
                    });
                }
            });
        o.spawn((Hint, f.text("", Face::Inter(600), 13.0, color::MUTED)));
    });
}

#[allow(clippy::too_many_arguments)]
fn progress(
    mut commands: Commands,
    mut loading: ResMut<Loading>,
    time: Res<Time<Real>>,
    (assets, field, env, settings, update): (Res<AssetServer>, Option<Res<FieldScene>>, Option<Res<crate::arena::ArenaEnv>>, Res<Settings>, Res<UpdateStatus>),
    (keys, mouse, pads): (Res<ButtonInput<KeyCode>>, Res<ButtonInput<MouseButton>>, Query<&Gamepad>),
    roots: Query<Entity, With<Root>>,
    mut steps: Query<(&Step, &mut Text, &mut TextColor), (Without<UpdateLine>, Without<Hint>)>,
    mut bar: Query<&mut Node, With<Bar>>,
    mut line: Query<&mut Text, (With<UpdateLine>, Without<Step>, Without<Hint>)>,
    mut hint: Query<&mut Text, (With<Hint>, Without<Step>, Without<UpdateLine>)>,
    mut colors: Query<(Option<&mut BackgroundColor>, Option<&mut TextColor>, Option<&mut BorderColor>), Without<Step>>,
    children: Query<&Children>,
) {
    let t = time.elapsed_secs();
    let msg = update.text();
    // what's in: the field's CAD, the arena's light (Low has none to make), the update check's answer
    let done = [
        field.as_ref().is_some_and(|f| assets.is_loaded_with_dependencies(&f.0)),
        env.is_some() || settings.graphics == "low",
        !msg.is_empty() && !msg.contains("Checking"),
    ];
    for (s, mut text, mut c) in &mut steps {
        let want = format!("{} {}", if done[s.0] { "●" } else { "○" }, STEPS[s.0]);
        if text.0 != want { text.0 = want; c.0 = if done[s.0] { color::TEXT } else { color::MUTED }; }
    }
    let n = done.iter().filter(|d| **d).count();
    for mut b in &mut bar { b.width = Val::Percent(100.0 * n as f32 / done.len() as f32); }
    for mut l in &mut line { let want = if msg.is_empty() { "Checking for updates…".to_string() } else { msg.clone() }; if l.0 != want { l.0 = want; } }
    // (the game's ready once the field and its light are; the update check needn't hold it up)
    if done[0] && done[1] && loading.ready_at.is_none() { loading.ready_at = Some(t); }
    let Some(ready) = loading.ready_at else { return };
    for mut h in &mut hint { if h.0.is_empty() { h.0 = "Press any key to start".into(); } }
    let skip = keys.get_just_pressed().next().is_some() || mouse.get_just_pressed().next().is_some()
        || pads.iter().any(|p| p.get_just_pressed().next().is_some());
    // (REBUILT_HOLD_LOADING=1 keeps it up, for a screenshot of it)
    let go = (skip || t >= MIN.max(ready + 1.5)) && !std::env::var("REBUILT_HOLD_LOADING").is_ok_and(|v| !v.is_empty());
    if !go && loading.fade == 0.0 { return }
    loading.fade = (loading.fade + time.delta_secs() / FADE).min(1.0);
    let a = 1.0 - loading.fade;
    for root in &roots {
        for e in std::iter::once(root).chain(children.iter_descendants(root)) {
            let Ok((bg, tc, bc)) = colors.get_mut(e) else { continue };
            if let Some(mut bg) = bg { let c = bg.0; bg.0 = c.with_alpha(c.alpha().min(a)); }
            if let Some(mut tc) = tc { let c = tc.0; tc.0 = c.with_alpha(c.alpha().min(a)); }
            if let Some(mut bc) = bc { let c = bc.top; *bc = BorderColor::all(c.with_alpha(c.alpha().min(a))); }
        }
    }
    if loading.fade >= 1.0 {
        for e in &roots { commands.entity(e).despawn(); }
        commands.remove_resource::<Loading>();
    }
}
