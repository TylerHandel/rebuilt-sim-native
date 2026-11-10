//! For checking a build without a person at it: with REBUILT_SHOT=<file.png> set, the game waits for
//! the field to load and the FUEL to settle, saves a screenshot there, and exits.
//! REBUILT_SHOT_COLLIDERS=1 shows the colliders in it (F1). REBUILT_SHOT_KEYS=KeyW,KeyE holds those
//! keys down the whole time (Bevy's KeyCode names), to see the robot driven.

use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot, ScreenshotCaptured};

use crate::field::{ColliderView, FieldScene};

pub struct ShotPlugin;

impl Plugin for ShotPlugin {
    fn build(&self, app: &mut App) {
        if let Some(path) = std::env::var_os("REBUILT_SHOT") {
            app.insert_resource(Shot { path: path.into(), frames: 0, taken: false })
                .add_systems(Update, shoot)
                .add_systems(PreUpdate, hold_keys.after(bevy::input::InputSystems));
        }
    }
}

#[derive(Resource)]
struct Shot {
    path: std::path::PathBuf,
    frames: u32,
    taken: bool,
}

fn shoot(
    mut commands: Commands,
    mut shot: ResMut<Shot>,
    assets: Res<AssetServer>,
    field: Res<FieldScene>,
    mut views: Query<&mut Visibility, With<ColliderView>>,
) {
    if shot.taken || !assets.is_loaded_with_dependencies(&field.0) {
        return;
    }
    shot.frames += 1;
    if shot.frames == 1 && std::env::var("REBUILT_SHOT_COLLIDERS").is_ok_and(|v| v == "1") {
        for mut v in &mut views { *v = Visibility::Visible; }
    }
    if shot.frames < 90 {
        return;
    }
    shot.taken = true;
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(shot.path.clone()))
        .observe(|_: On<ScreenshotCaptured>, mut exit: MessageWriter<AppExit>| {
            exit.write(AppExit::Success);
        });
}

fn hold_keys(mut keys: ResMut<ButtonInput<KeyCode>>) {
    let Ok(list) = std::env::var("REBUILT_SHOT_KEYS") else { return };
    for name in list.split(',') {
        let k = match name.trim() {
            "KeyW" => KeyCode::KeyW, "KeyA" => KeyCode::KeyA, "KeyS" => KeyCode::KeyS, "KeyD" => KeyCode::KeyD,
            "KeyQ" => KeyCode::KeyQ, "KeyE" => KeyCode::KeyE, "ShiftLeft" => KeyCode::ShiftLeft, "KeyX" => KeyCode::KeyX,
            _ => continue,
        };
        keys.press(k);
    }
}
