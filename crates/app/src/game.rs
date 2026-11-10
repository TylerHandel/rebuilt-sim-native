//! The game: the ported simulation (sim::game::World) with 2910 on the field, stepped at the
//! physics rate, and driven by the keyboard or a controller the way the browser game drives it
//! (js/main.js driverCommand, js/input.js).
//!
//! Keyboard: WASD drive, Q/E (or the arrows, or J/L) turn, Shift intake, X slow, Backspace restart.
//! Controller: left stick drive, right stick turn, left trigger intake, left stick click slow, View
//! restart. Stick up drives away from the camera (field-relative from wherever the camera is).

use bevy::prelude::*;
use sim::constants::{Alliance, PHYSICS_DT};
use sim::game::{Settings, World};
use sim::game_match::Phase;
use sim::parity::Cmd;
use std::time::Instant;

use crate::update::InMatch;

/// The robot you drive, and where it starts (blue, by its driver station, facing downfield).
const ROBOT: &str = "2910";
const START: [f64; 3] = [-6.2, 2.4, 0.0];

#[derive(Resource)]
pub struct Game(pub World);

/// How long the last physics steps took (ms per step, smoothed).
#[derive(Resource, Default)]
pub struct StepTime(pub f32);

/// Bumped each time the World is replaced (a restart), so the views know to remake themselves.
#[derive(Resource, Default)]
pub struct WorldGen(pub u32);

/// The driver's command, read each frame and applied at each step.
#[derive(Resource, Default)]
pub struct Driver(pub Cmd);

pub fn new_world() -> World {
    let settings = Settings {
        seed: 1,
        // (their throws need the shot solver, which arrives with the mechanisms: M3)
        human_players: false,
        preload: 0,
        robots: vec![(ROBOT.into(), Alliance::Blue, START)],
    };
    let mut w = World::new(sim::WorldData::bundled(), &settings);
    w.jump_to(Phase::Teleop);
    w
}

pub struct GamePlugin;

impl Plugin for GamePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Game(new_world()))
            .insert_resource(Time::<Fixed>::from_seconds(PHYSICS_DT))
            .init_resource::<StepTime>()
            .init_resource::<Driver>()
            .init_resource::<WorldGen>()
            .add_systems(FixedUpdate, step)
            .add_systems(Update, (drive, restart, in_match));
    }
}

fn step(mut game: ResMut<Game>, driver: Res<Driver>, mut st: ResMut<StepTime>) {
    let t = Instant::now();
    let w = &mut game.0;
    if let Some(r) = w.robots.first_mut() { r.cmd = driver.0; }
    w.step(&mut |_, _| {});
    // (frameGame runs every second step, as the headless game and the parity traces run it)
    if w.steps % 2 == 0 { w.frame(2.0 * PHYSICS_DT); }
    let ms = t.elapsed().as_secs_f32() * 1000.0;
    st.0 = if st.0 == 0.0 { ms } else { st.0 * 0.95 + ms * 0.05 };
}

/// js input.js deadband
fn deadband(v: f32, d: f32) -> f32 {
    let a = v.abs();
    if a < d { 0.0 } else { v.signum() * (a - d) / (1.0 - d) }
}

/// js main.js driverCommand: the sticks (or keys) to a field-relative command
fn drive(
    keys: Res<ButtonInput<KeyCode>>,
    pads: Query<&Gamepad>,
    cam: Query<&Transform, With<Camera3d>>,
    game: Res<Game>,
    mut driver: ResMut<Driver>,
) {
    let (mut lx, mut ly, mut rx, mut lt, mut slow) = (0.0f32, 0.0f32, 0.0f32, 0.0f32, false);
    for pad in &pads {
        let (x, y) = (deadband(pad.get(GamepadAxis::LeftStickX).unwrap_or(0.0), 0.1), deadband(pad.get(GamepadAxis::LeftStickY).unwrap_or(0.0), 0.1));
        let r = deadband(pad.get(GamepadAxis::RightStickX).unwrap_or(0.0), 0.1);
        let t = pad.get(GamepadButton::LeftTrigger2).unwrap_or(0.0);
        if x != 0.0 || y != 0.0 || r != 0.0 || t > 0.0 {
            // (Bevy's stick y is up-positive; the browser's is down-positive)
            (lx, ly, rx, lt) = (x, -y, r, t);
        }
        slow |= pad.pressed(GamepadButton::LeftThumb);
    }
    let k = |c: &[KeyCode]| keys.any_pressed(c.iter().copied());
    let kx = k(&[KeyCode::KeyD]) as i32 - k(&[KeyCode::KeyA]) as i32;
    let ky = k(&[KeyCode::KeyS]) as i32 - k(&[KeyCode::KeyW]) as i32;
    let kr = k(&[KeyCode::KeyL, KeyCode::ArrowRight, KeyCode::KeyE]) as i32 - k(&[KeyCode::KeyJ, KeyCode::ArrowLeft, KeyCode::KeyQ]) as i32;
    if kx != 0 || ky != 0 {
        let m = ((kx * kx + ky * ky) as f32).sqrt();
        (lx, ly) = (kx as f32 / m, ky as f32 / m);
    }
    if kr != 0 { rx = kr as f32; }
    if k(&[KeyCode::ShiftLeft, KeyCode::ShiftRight]) { lt = 1.0; }
    slow |= k(&[KeyCode::KeyX]);

    let Some(r) = game.0.robots.first() else { return };
    let (max_speed, max_omega) = (r.max_speed, r.max_omega);
    let (mut up, mut right) = (-ly as f64, lx as f64);
    let mag = up.hypot(right).min(1.0);
    if mag > 0.0 { up *= mag; right *= mag; }
    let scale = if slow { 0.4 } else { 1.0 };
    // stick up is away from the camera
    let f = cam.single().map(|t| { let f = t.forward(); Vec2::new(f.x, f.z).normalize_or(Vec2::X) }).unwrap_or(Vec2::X);
    let (fx, fz) = (f.x as f64, f.y as f64);
    let rx = rx as f64;
    driver.0 = Cmd {
        vx: (fx * up - fz * right) * max_speed * scale,
        vz: (fz * up + fx * right) * max_speed * scale,
        omega: -rx.signum() * rx * rx * max_omega * if slow { 0.5 } else { 1.0 },
        intake: lt > 0.3,
        ..default()
    };
}

/// Backspace or the controller's View button: a new match
fn restart(keys: Res<ButtonInput<KeyCode>>, pads: Query<&Gamepad>, mut game: ResMut<Game>, mut world_gen: ResMut<WorldGen>) {
    if keys.just_pressed(KeyCode::Backspace) || pads.iter().any(|p| p.just_pressed(GamepadButton::Select)) {
        game.0 = new_world();
        world_gen.0 += 1;
    }
}

/// An update waits while a match is being played.
fn in_match(game: Res<Game>, mut m: ResMut<InMatch>) {
    let on = game.0.match_.robot_enabled();
    if m.0 != on { m.0 = on; }
}
