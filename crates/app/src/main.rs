#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
//! REBUILT Sim, native: the browser game (github.com/TylerHandel/rebuilt-sim-2026) rebuilt in Rust.
//! The simulation runs in the `sim` crate; this draws it, takes input, and keeps itself updated.

mod camera;
mod field;
mod fuel;
mod hud;
mod shot;
mod update;

use bevy::prelude::*;
use bevy::window::PresentMode;

fn main() {
    // Velopack's install/update hooks run first and may exit (on install, update, uninstall).
    velopack::VelopackApp::build().run();

    // Assets sit next to the executable when installed; in a `cargo run` they're the workspace's.
    let assets = if std::env::var_os("CARGO_MANIFEST_DIR").is_some() { "../../assets" } else { "assets" };
    let sim = sim::Sim::new(sim::WorldData::bundled());
    let dt = sim.dt() as f64;

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
        .insert_resource(Time::<Fixed>::from_seconds(dt))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.06, 0.08)))
        .insert_resource(fuel::SimState(sim))
        .add_plugins((camera::OrbitCameraPlugin, field::FieldPlugin, fuel::FuelPlugin, hud::HudPlugin, update::UpdatePlugin, shot::ShotPlugin))
        .run();
}
