//! Frame rate, physics time per step, FUEL count, and the update status.

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;

use crate::fuel::{SimState, StepTime};
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

fn refresh(diag: Res<DiagnosticsStore>, sim: Res<SimState>, st: Res<StepTime>, upd: Res<UpdateStatus>, mut q: Query<&mut Text, With<HudText>>) {
    let fps = diag.get(&FrameTimeDiagnosticsPlugin::FPS).and_then(|d| d.smoothed()).unwrap_or(0.0);
    let Ok(mut t) = q.single_mut() else { return };
    t.0 = format!(
        "{fps:.0} fps   physics {:.2} ms/step   {} FUEL   t {:.1} s\n{}\nRight-drag turn · wheel zoom · WASD move · Space drop FUEL · R reset · F1 colliders",
        st.0, sim.0.fuel.len(), sim.0.time, upd.text()
    );
}
