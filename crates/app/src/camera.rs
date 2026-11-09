//! A camera that orbits a point on the field: right or middle drag turns it, the wheel zooms,
//! WASD / arrows move the point it looks at.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;

pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn).add_systems(Update, orbit);
    }
}

#[derive(Component)]
pub struct Orbit {
    pub focus: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub dist: f32,
}

fn spawn(mut commands: Commands) {
    let o = Orbit { focus: Vec3::ZERO, yaw: 0.0, pitch: 0.75, dist: 16.0 };
    let t = place(&o);
    commands.spawn((Camera3d::default(), t, o));
}

fn place(o: &Orbit) -> Transform {
    let rot = Quat::from_euler(EulerRot::YXZ, o.yaw, -o.pitch, 0.0);
    let eye = o.focus + rot * Vec3::new(0.0, 0.0, o.dist);
    Transform::from_translation(eye).looking_at(o.focus, Vec3::Y)
}

fn orbit(
    mut q: Query<(&mut Orbit, &mut Transform)>,
    buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    time: Res<Time>,
) {
    let Ok((mut o, mut t)) = q.single_mut() else { return };
    if buttons.pressed(MouseButton::Right) || buttons.pressed(MouseButton::Middle) {
        o.yaw -= motion.delta.x * 0.005;
        o.pitch = (o.pitch + motion.delta.y * 0.005).clamp(0.05, 1.5);
    }
    o.dist = (o.dist * (1.0 - scroll.delta.y * 0.1)).clamp(1.0, 40.0);
    let mut mv = Vec2::ZERO;
    if keys.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]) { mv.y -= 1.0; }
    if keys.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]) { mv.y += 1.0; }
    if keys.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]) { mv.x -= 1.0; }
    if keys.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]) { mv.x += 1.0; }
    if mv != Vec2::ZERO {
        let step = Quat::from_rotation_y(o.yaw) * Vec3::new(mv.x, 0.0, mv.y) * o.dist * 0.6 * time.delta_secs();
        o.focus += step;
    }
    *t = place(&o);
}
