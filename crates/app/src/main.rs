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
mod loading;
mod materials;
mod nature;
mod net;
mod robot_model;
mod scene;
mod settings;
mod showcase;
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
    let settings = settings::Settings::load();
    let mut app = App::new();
    // NVIDIA DLSS: this game's id with NVIDIA's runtime, and Vulkan (DLSS runs only there) when it's on
    #[cfg(feature = "dlss")]
    app.insert_resource(bevy::anti_alias::dlss::DlssProjectId(bevy::asset::uuid::uuid!("d3327edf-e73f-451a-b85a-82e8cc58847b")));
    let mut render = bevy::render::settings::WgpuSettings::default();
    if settings.upscaler != "off" {
        render.backends = Some(bevy::render::settings::Backends::VULKAN);
        camera::DLSS_LAUNCHED.store(true, std::sync::atomic::Ordering::Relaxed);
    }
    app
        .add_plugins(
            DefaultPlugins
                .set(bevy::render::RenderPlugin { render_creation: render.into(), ..default() })
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: format!("REBUILT Sim {}", env!("CARGO_PKG_VERSION")),
                        present_mode: PresentMode::AutoVsync,
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin { file_path: assets.into(), ..default() })
                // (anisotropic filtering: the carpet's markings and the field's textures stay sharp
                // looked at low across the field)
                .set(ImagePlugin { default_sampler: sharp_sampler() })
                .set(bevy::gltf::GltfPlugin { default_sampler: sharp_sampler(), ..default() }),
        )
        .insert_resource(ClearColor(Color::srgb(0.05, 0.06, 0.08)))
        .insert_resource(settings)
        .insert_resource(robot_model::AssetDir(bevy::asset::io::file::FileAssetReader::get_base_path().join(assets)))
        .add_plugins((ui::UiPlugin, game::GamePlugin, robot_model::RobotModelPlugin, camera::OrbitCameraPlugin, field::FieldPlugin, draw::DrawPlugin, update::UpdatePlugin, shot::ShotPlugin, lights::LightsPlugin, arena::ArenaPlugin, venue::VenuePlugin, effects::EffectsPlugin, materials::MaterialsPlugin, bevel::BevelPlugin))
        .add_plugins((scene::ScenePlugin, showcase::ShowcasePlugin, loading::LoadingPlugin, nature::NaturePlugin, bevy::post_process::auto_exposure::AutoExposurePlugin))
        .run();
}

/// Textures sampled with trilinear and 16x anisotropic filtering
fn sharp_sampler() -> bevy::image::ImageSamplerDescriptor {
    use bevy::image::{ImageFilterMode, ImageSamplerDescriptor};
    ImageSamplerDescriptor { mag_filter: ImageFilterMode::Linear, min_filter: ImageFilterMode::Linear, mipmap_filter: ImageFilterMode::Linear, anisotropy_clamp: 16, ..ImageSamplerDescriptor::default() }
}
