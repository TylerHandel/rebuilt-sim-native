//! A camera that follows your robot from behind and above: right or middle drag turns it around the
//! robot, the wheel zooms. A match starts with it behind the robot, looking downfield from its
//! ALLIANCE's end; behind the menus it circles the field slowly. The graphics setting is applied
//! here: anti-aliasing on the camera, shadows on the sun.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;

use crate::game::Game;
use crate::settings::Settings;
use crate::ui::Screen;

pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn)
            .add_systems(OnEnter(Screen::Playing), behind_robot)
            .add_systems(Update, graphics)
            .add_systems(PostUpdate, orbit.before(TransformSystems::Propagate));
    }
}

#[derive(Component)]
pub struct Orbit {
    pub focus: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub dist: f32,
}

/// Behind the menus: the whole field, turning slowly
const MENU_VIEW: Orbit = Orbit { focus: Vec3::ZERO, yaw: -0.6, pitch: 0.55, dist: 13.0 };

fn spawn(mut commands: Commands) {
    let mut o = MENU_VIEW;
    // (screenshots: REBUILT_SHOT_CAM=<yaw deg>,<pitch>,<dist> to look from elsewhere)
    if let Ok(v) = std::env::var("REBUILT_SHOT_CAM") {
        let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        if let [y, p, d] = f[..] { (o.yaw, o.pitch, o.dist) = (y.to_radians(), p, d); }
    }
    let t = place(&o);
    commands.spawn((Camera3d::default(), t, o));
}

/// A new match (not a resume): behind the robot, on its ALLIANCE's side
fn behind_robot(mut q: Query<&mut Orbit>, game: Res<Game>, mut last: Local<u32>, world_gen: Res<crate::game::WorldGen>) {
    if *last == world_gen.0 { return }
    *last = world_gen.0;
    let Some(r) = game.0.robots.first() else { return };
    let Ok(mut o) = q.single_mut() else { return };
    o.focus = robot_at(&game, r.body);
    // (yaw -90 degrees: behind a robot facing +x, which is downfield for blue)
    o.yaw = if r.alliance == sim::constants::Alliance::Blue { -std::f32::consts::FRAC_PI_2 } else { std::f32::consts::FRAC_PI_2 };
    (o.pitch, o.dist) = (0.85, 4.5);
    if let Ok(v) = std::env::var("REBUILT_SHOT_CAM") {
        let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        if let [y, p, d] = f[..] { (o.yaw, o.pitch, o.dist) = (y.to_radians(), p, d); }
    }
}

/// High: 4x MSAA and shadows; Medium: shadows; Low: neither
fn graphics(settings: Res<Settings>, mut commands: Commands, cams: Query<Entity, With<Camera3d>>, mut suns: Query<&mut DirectionalLight>) {
    if !settings.is_changed() { return }
    for c in &cams { commands.entity(c).insert(if settings.graphics == "high" { Msaa::Sample4 } else { Msaa::Off }); }
    for mut s in &mut suns { s.shadow_maps_enabled = settings.graphics != "low"; }
}

fn robot_at(game: &Game, body: rapier3d::prelude::RigidBodyHandle) -> Vec3 {
    game.0.physics.bodies.get(body).map(|b| b.translation() + Vec3::Y * 0.3).unwrap_or(Vec3::ZERO)
}

fn place(o: &Orbit) -> Transform {
    let rot = Quat::from_euler(EulerRot::YXZ, o.yaw, -o.pitch, 0.0);
    let eye = o.focus + rot * Vec3::new(0.0, 0.0, o.dist);
    Transform::from_translation(eye).looking_at(o.focus, Vec3::Y)
}

fn orbit(
    mut q: Query<(&mut Orbit, &mut Transform)>,
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    game: Res<Game>,
    time: Res<Time>,
    screen: Res<State<Screen>>,
) {
    let Ok((mut o, mut t)) = q.single_mut() else { return };
    if *screen.get() == Screen::Menu {
        let yaw = o.yaw + 0.04 * time.delta_secs();
        *o = Orbit { yaw, ..MENU_VIEW };
        *t = place(&o);
        return;
    }
    if buttons.pressed(MouseButton::Right) || buttons.pressed(MouseButton::Middle) {
        o.yaw -= motion.delta.x * 0.005;
        o.pitch = (o.pitch + motion.delta.y * 0.005).clamp(0.05, 1.5);
    }
    o.dist = (o.dist * (1.0 - scroll.delta.y * 0.1)).clamp(1.5, 30.0);
    if let Some(r) = game.0.robots.first() {
        let target = robot_at(&game, r.body);
        // (a little lag, so the view doesn't shake with every bump)
        let k = 1.0 - (-12.0 * time.delta_secs()).exp();
        o.focus = o.focus.lerp(target, k);
    }
    *t = place(&o);
}
