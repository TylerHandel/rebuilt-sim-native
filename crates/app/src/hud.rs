//! Frame rate, physics time per step, FUEL count, and the update status.

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;

use crate::game::{Game, StepTime};
use crate::update::UpdateStatus;

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
            .add_systems(Startup, spawn)
            .add_systems(Update, refresh);
    }
}

#[derive(Component)]
struct HudText;

fn spawn(mut commands: Commands) {
    commands.spawn((
        Text::new(""),
        TextFont { font_size: FontSize::Px(15.0), ..default() },
        Node { position_type: PositionType::Absolute, top: Val::Px(10.0), left: Val::Px(12.0), ..default() },
        HudText,
    ));
}

fn refresh(diag: Res<DiagnosticsStore>, game: Res<Game>, st: Res<StepTime>, upd: Res<UpdateStatus>, mut q: Query<&mut Text, With<HudText>>) {
    use sim::constants::Alliance;
    use sim::game_match::Phase;
    let fps = diag.get(&FrameTimeDiagnosticsPlugin::FPS).and_then(|d| d.smoothed()).unwrap_or(0.0);
    let Ok(mut t) = q.single_mut() else { return };
    let m = &game.0.match_;
    let clock = match m.phase {
        Phase::Teleop => { let left = (sim::constants::TIMING::teleop - m.phase_time).max(0.0); format!("TELEOP {}:{:02}", (left / 60.0) as i32, left as i32 % 60) }
        Phase::Done | Phase::Post => "match over: Backspace (or View) for another".into(),
        p => format!("{p:?}"),
    };
    let speed = game.0.robots.first().map(|r| r.vel[0].hypot(r.vel[2])).unwrap_or(0.0);
    t.0 = format!(
        "{fps:.0} fps   physics {:.2} ms/step\n{clock}   blue {:.0} - {:.0} red   {speed:.1} m/s\n{}\nWASD drive | Q/E turn | Shift intake | X slow | Backspace restart | right-drag camera | wheel zoom | F1 colliders",
        st.0, m.total(Alliance::Blue), m.total(Alliance::Red), upd.text()
    );
}
