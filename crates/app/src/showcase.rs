//! The robot select screen's showcase: the chosen robot on a turntable in a small studio of its
//! own, its mechanisms going through their motions (robot_model::pose_demo), drawn into an image
//! the picker's card shows (ui/menu.rs). It lives on render layer 1, out of the arena's sight and
//! light, far under the field, and is only drawn while the Practice page is open.

use bevy::camera::visibility::RenderLayers;
use bevy::camera::RenderTarget;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;

use crate::robot_model::{self, AssetDir, RobotModel};
use crate::settings::Settings;
use crate::ui::menu::{Menu, Page};
use crate::ui::Screen;

/// The studio: well out of the arena's way (and its sun's shadows)
const AT: Vec3 = Vec3::new(0.0, -400.0, 0.0);
/// The image's size: twice what the card shows it at
pub const W: u32 = 720;
pub const H: u32 = 450;

pub struct ShowcasePlugin;

impl Plugin for ShowcasePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn)
            .add_systems(Update, (choose, layer, active))
            .add_systems(PostUpdate, turn.before(bevy::transform::TransformSystems::Propagate));
    }
}

/// What the card shows
#[derive(Resource)]
pub struct ShowcaseImage(pub Handle<Image>);

/// The robot on the turntable, which robot it is
#[derive(Component)]
pub struct Showcase(String);

#[derive(Component)]
struct StudioCam;

fn spawn(mut commands: Commands, mut images: ResMut<Assets<Image>>, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let image = images.add(Image::new_target_texture(W, H, TextureFormat::Bgra8UnormSrgb, None));
    commands.insert_resource(ShowcaseImage(image.clone()));
    let layer = RenderLayers::layer(1);
    // a three-quarter view from the front, a little above the bumpers
    let eye = AT + Vec3::new(1.55, 0.95, 1.25);
    commands.spawn((
        StudioCam,
        Camera3d::default(),
        Camera { order: -1, is_active: false, clear_color: ClearColorConfig::Custom(Color::NONE), ..default() },
        RenderTarget::Image(image.into()),
        Projection::Perspective(PerspectiveProjection { fov: 32f32.to_radians(), ..default() }),
        Msaa::Sample4,
        bevy::core_pipeline::tonemapping::Tonemapping::AgX,
        Transform::from_translation(eye).looking_at(AT + Vec3::new(0.0, 0.28, 0.0), Vec3::Y),
        layer.clone(),
    ));
    // a key light high to one side, a cool rim from behind, a warm fill low on the other side
    let lights = [
        (Vec3::new(1.6, 2.4, 1.4), Color::srgb(1.0, 0.97, 0.92), 900_000.0),
        (Vec3::new(-1.8, 1.6, -1.2), Color::srgb(0.65, 0.78, 1.0), 700_000.0),
        (Vec3::new(-0.6, 0.6, 2.0), Color::srgb(1.0, 0.85, 0.7), 220_000.0),
    ];
    for (i, (p, c, lm)) in lights.into_iter().enumerate() {
        commands.spawn((
            PointLight { color: c, intensity: lm, range: 8.0, radius: 0.3, shadow_maps_enabled: i == 0, ..default() },
            Transform::from_translation(AT + p),
            layer.clone(),
        ));
    }
    // the turntable: a dark, satin disc with a lit rim
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.78, 0.03))),
        MeshMaterial3d(mats.add(StandardMaterial { base_color: Color::srgb(0.035, 0.038, 0.045), perceptual_roughness: 0.32, reflectance: 0.6, ..default() })),
        Transform::from_translation(AT - Vec3::Y * 0.016),
        layer.clone(),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Torus::new(0.775, 0.79))),
        MeshMaterial3d(mats.add(StandardMaterial { base_color: Color::BLACK, emissive: LinearRgba::rgb(2.4, 1.9, 0.35), ..default() })),
        Transform::from_translation(AT),
        layer,
    ));
}

/// The robot chosen on the turntable (another one when the choice changes)
fn choose(
    mut commands: Commands,
    settings: Res<Settings>,
    assets: Res<AssetServer>,
    dir: Res<AssetDir>,
    shown: Query<(Entity, &Showcase)>,
) {
    let key = &settings.robot;
    if shown.iter().any(|(_, s)| &s.0 == key) { return }
    for (e, _) in &shown { commands.entity(e).despawn(); }
    // (usize::MAX: no robot of the match's under it, so the match's systems leave it be)
    if let Some(e) = robot_model::spawn(&mut commands, &assets, &dir, usize::MAX, key) {
        commands.entity(e).insert((Showcase(key.clone()), Transform::from_translation(AT), RenderLayers::layer(1)));
    }
}

/// Everything in the robot's model on the studio's layer as it comes in
fn layer(
    mut commands: Commands,
    shown: Query<Entity, With<Showcase>>,
    children: Query<&Children>,
    unlayered: Query<(), Without<RenderLayers>>,
) {
    for root in &shown {
        for e in children.iter_descendants(root) {
            if unlayered.contains(e) { commands.entity(e).insert(RenderLayers::layer(1)); }
        }
    }
}

/// Drawn only while the robot select page's open
fn active(screen: Res<State<Screen>>, menu: Option<Res<Menu>>, mut cams: Query<&mut Camera, With<StudioCam>>) {
    let on = *screen.get() == Screen::Menu && menu.is_some_and(|m| m.page == Page::Practice);
    for mut c in &mut cams { if c.is_active != on { c.is_active = on; } }
}

/// The turntable turning and the robot going through its motions
fn turn(time: Res<Time>, mut shown: Query<(&mut RobotModel, &mut Transform), With<Showcase>>, mut parts: Query<&mut Transform, Without<RobotModel>>) {
    let t = time.elapsed_secs();
    for (mut m, mut root) in &mut shown {
        root.translation = AT;
        root.rotation = Quat::from_rotation_y(0.9 + t * 0.35);
        robot_model::pose_demo(&mut m, t, time.delta_secs(), &mut parts, &time);
    }
}
