//! A camera that follows your robot from behind and above: right or middle drag turns it around the
//! robot, the wheel zooms. It starts behind the robot, looking downfield from the driver station.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;

use crate::game::Game;

pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn).add_systems(PostUpdate, orbit.before(TransformSystems::Propagate));
    }
}

#[derive(Component)]
pub struct Orbit {
    pub focus: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub dist: f32,
}

fn spawn(mut commands: Commands, game: Res<Game>) {
    let focus = game.0.robots.first().map(|r| robot_at(&game, r.body)).unwrap_or(Vec3::ZERO);
    // (yaw -90 degrees: behind a robot facing +x, which is downfield for blue)
    let mut o = Orbit { focus, yaw: -std::f32::consts::FRAC_PI_2, pitch: 0.85, dist: 4.5 };
    // (screenshots: REBUILT_SHOT_CAM=<yaw deg>,<pitch>,<dist> to look from elsewhere)
    if let Ok(v) = std::env::var("REBUILT_SHOT_CAM") {
        let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        if let [y, p, d] = f[..] { (o.yaw, o.pitch, o.dist) = (y.to_radians(), p, d); }
    }
    let t = place(&o);
    commands.spawn((Camera3d::default(), t, o));
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
) {
    let Ok((mut o, mut t)) = q.single_mut() else { return };
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
