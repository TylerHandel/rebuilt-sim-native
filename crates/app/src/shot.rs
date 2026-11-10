//! For checking a build without a person at it: with REBUILT_SHOT=<file.png> set, the game waits for
//! the field to load and the FUEL to settle, saves a screenshot there, and exits.
//! REBUILT_SHOT_COLLIDERS=1 shows the colliders in it (F1). REBUILT_SHOT_KEYS=KeyW,KeyE holds those
//! keys down the whole time (Bevy's KeyCode names), to see the robot driven.
//! REBUILT_SHOT_WAIT=<frames> waits that long after them (90 if unset). REBUILT_SHOT_FLOW
//! walks the screens first, in order: "practice" (open the Practice page), "settings", "start" (start
//! a match), "pause", "controls", "results" (end the match).

use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot, ScreenshotCaptured};

use crate::field::{ColliderView, FieldScene};
use crate::game::{Game, GameCmd};
use crate::ui::menu::{ControlsOpen, Menu, Page};

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

#[allow(clippy::too_many_arguments)]
fn shoot(
    mut commands: Commands,
    mut shot: ResMut<Shot>,
    assets: Res<AssetServer>,
    field: Res<FieldScene>,
    mut views: Query<&mut Visibility, With<ColliderView>>,
    mut menu: ResMut<Menu>,
    mut cmds: MessageWriter<GameCmd>,
    mut controls: ResMut<ControlsOpen>,
    mut game: ResMut<Game>,
    time: Res<Time<Real>>,
) {
    if shot.taken || !assets.is_loaded_with_dependencies(&field.0) {
        return;
    }
    shot.frames += 1;
    // (how it's going, every 30 frames: a slow machine's run shows where it's spending its time)
    if shot.frames % 30 == 0 { eprintln!("shot: frame {} at {:.1} s, {} steps", shot.frames, time.elapsed_secs(), game.0.steps); }
    if shot.frames == 1 && std::env::var("REBUILT_SHOT_COLLIDERS").is_ok_and(|v| v == "1") {
        for mut v in &mut views { *v = Visibility::Visible; }
    }
    // the screens to walk, one every 15 frames
    let flow: Vec<String> = std::env::var("REBUILT_SHOT_FLOW").map(|v| v.split(',').map(|x| x.trim().to_string()).collect()).unwrap_or_default();
    if shot.frames % 15 == 0 {
        if let Some(step) = flow.get((shot.frames / 15) as usize - 1) {
            match step.as_str() {
                "practice" => { menu.page = Page::Practice; menu.focus = 0; menu.dirty = true; }
                "settings" => { menu.page = Page::Settings; menu.focus = 0; menu.dirty = true; }
                "start" => { cmds.write(GameCmd::Start); }
                "pause" => { cmds.write(GameCmd::Pause); }
                "controls" => controls.0 = true,
                "results" => game.0.match_.go(sim::game_match::Phase::Done),
                _ => {}
            }
        }
    }
    let wait = std::env::var("REBUILT_SHOT_WAIT").ok().and_then(|v| v.parse().ok()).unwrap_or(90u32);
    if shot.frames < wait + 15 * flow.len() as u32 {
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
            "Space" => KeyCode::Space, "KeyR" => KeyCode::KeyR, "KeyF" => KeyCode::KeyF,
            _ => continue,
        };
        keys.press(k);
    }
}
