#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
//! REBUILT Sim, native: the browser game (github.com/TylerHandel/rebuilt-sim-2026) rebuilt in Rust.
//! The simulation runs in the `sim` crate; this draws it, takes input, and keeps itself updated.

mod arena;
mod bevel;
mod camera;
mod field;
mod draw;
mod effects;
mod game;
mod lights;
mod materials;
mod net;
mod robot_model;
mod settings;
mod shot;
mod ui;
mod update;
mod venue;

use bevy::prelude::*;
use bevy::window::PresentMode;

fn main() {
    // Velopack's install/update hooks run first and may exit (on install, update, uninstall).
    velopack::VelopackApp::build().run();

    // Assets sit next to the executable when installed; in a `cargo run` they're the workspace's.
    let assets = if std::env::var_os("CARGO_MANIFEST_DIR").is_some() { "../../assets" } else { "assets" };
    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: format!("REBUILT Sim {}", env!("CARGO_PKG_VERSION")),
                        present_mode: PresentMode::AutoVsync,
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin { file_path: assets.into(), ..default() }),
        )
        .insert_resource(ClearColor(Color::srgb(0.05, 0.06, 0.08)))
        .insert_resource(settings::Settings::load())
        .insert_resource(robot_model::AssetDir(bevy::asset::io::file::FileAssetReader::get_base_path().join(assets)))
        .add_plugins((ui::UiPlugin, game::GamePlugin, robot_model::RobotModelPlugin, camera::OrbitCameraPlugin, field::FieldPlugin, draw::DrawPlugin, update::UpdatePlugin, shot::ShotPlugin, lights::LightsPlugin, arena::ArenaPlugin, venue::VenuePlugin, effects::EffectsPlugin, materials::MaterialsPlugin, bevel::BevelPlugin))
        .run();
}
