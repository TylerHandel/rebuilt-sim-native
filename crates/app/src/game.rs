//! The game: the ported simulation (sim::game::World), stepped at the physics rate while a match is
//! being played, and driven by the keyboard or a controller the way the browser game drives it
//! (js/main.js driverCommand, js/input.js). Behind the menus it's the field with its FUEL.
//!
//! Keyboard: WASD drive, Q/E (or the arrows, or J/L) turn, Shift intake, Space shoot, R pass, F
//! eject, G CHUTE DOOR, H throw (your HUMAN PLAYER), X slow, Esc pause, Backspace restart. Controller: left stick drive, right stick turn, left
//! trigger intake, right trigger shoot, right bumper pass, left bumper eject, X CHUTE DOOR, Y throw,
//! left stick click slow,
//! Menu pause, View restart. Stick up drives away from the camera (from the
//! driver station, downfield); [ ] or the D-pad change the camera (camera.rs).

use bevy::prelude::*;
use sim::constants::{Alliance, PHYSICS_DT, TIMING};
use sim::game::{Settings as SimSettings, World};
use sim::game_match::Phase;
use sim::parity::Cmd;
use std::time::Instant;

use crate::settings::{start_pose, Settings};
use crate::ui::Screen;
use crate::update::InMatch;

#[derive(Resource)]
pub struct Game(pub World);

/// How long the last physics steps took (ms per step, smoothed).
#[derive(Resource, Default)]
pub struct StepTime(pub f32);

/// Bumped each time the World is replaced, so the views know to remake themselves.
#[derive(Resource, Default)]
pub struct WorldGen(pub u32);

/// The driver's command, read each frame and applied at each step, and whether it's slow mode.
#[derive(Resource, Default)]
pub struct Driver(pub Cmd, pub bool);

/// What the match being played was started with
#[derive(Resource, Clone)]
pub struct MatchSetup {
    pub robot: String,
    pub alliance: Alliance,
    pub start: String,
    pub skip_auto: bool,
    /// your HUMAN PLAYER throws by itself (settings hp "auto")
    pub hp_auto: bool,
}

/// What the screens ask of the game (js ui.js handlers: onStart, onRestart, onResume, onMenu)
#[derive(Message, Clone, Copy, PartialEq, Eq, Debug)]
pub enum GameCmd { Start, Restart, Resume, Pause, ToMenu }

/// The field and its FUEL, and the robot of `setup` (none behind the menus)
pub fn new_world(setup: Option<&MatchSetup>) -> World {
    let robots = setup.map(|m| vec![(m.robot.clone(), m.alliance, start_pose(&m.robot, &m.start, m.alliance))]).unwrap_or_default();
    let settings = SimSettings {
        seed: rand_seed(),
        // (both throw at their HUBS while active; yours as the hp setting says, below)
        human_players: setup.is_some(),
        preload: if setup.is_some() { 8 } else { 0 },
        robots,
        aim_truth: false,
    };
    let mut w = World::new(sim::WorldData::bundled(), &settings);
    if let Some(m) = setup {
        w.match_.player = m.alliance;
        // (js createGame: your HUMAN PLAYER throws by itself unless you've set it to manual)
        w.hps[sim::game_match::ai(m.alliance)].auto = m.hp_auto;
    }
    w
}

fn rand_seed() -> u32 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.subsec_nanos() ^ d.as_secs() as u32).unwrap_or(1)
}

pub struct GamePlugin;

impl Plugin for GamePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Game(new_world(None)))
            .insert_resource(Time::<Fixed>::from_seconds(PHYSICS_DT))
            .init_resource::<StepTime>()
            .init_resource::<Driver>()
            .init_resource::<WorldGen>()
            .add_message::<GameCmd>()
            .add_systems(FixedUpdate, step.run_if(in_state(Screen::Playing)))
            .add_systems(Update, (drive.run_if(in_state(Screen::Playing)), keys, commands, in_match).chain());
    }
}

fn step(mut game: ResMut<Game>, driver: Res<Driver>, setup: Option<Res<MatchSetup>>, mut st: ResMut<StepTime>) {
    let t = Instant::now();
    let w = &mut game.0;
    if let Some(r) = w.robots.first_mut() { r.cmd = driver.0; }
    w.step(&mut |_, _| {});
    // (frameGame runs every second step, as the headless game and the parity traces run it)
    if w.steps % 2 == 0 { w.frame(2.0 * PHYSICS_DT); }
    // no AUTO routines yet: "skip" goes from AUTO straight to where TELEOP is decided
    if setup.is_some_and(|s| s.skip_auto) && w.match_.phase == Phase::Auto {
        w.match_.go(Phase::AutoGap);
        w.match_.phase_time = TIMING::autoGap;
    }
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
    rig: Res<crate::camera::CameraRig>,
    game: Res<Game>,
    mut driver: ResMut<Driver>,
) {
    let (mut lx, mut ly, mut rx, mut lt, mut slow) = (0.0f32, 0.0f32, 0.0f32, 0.0f32, false);
    // (js input.js: the right trigger or Space shoots, the right bumper or R passes, the left
    // bumper or F ejects)
    let (mut rt, mut rb, mut lb) = (0.0f32, false, false);
    for pad in &pads {
        let (x, y) = (deadband(pad.get(GamepadAxis::LeftStickX).unwrap_or(0.0), 0.1), deadband(pad.get(GamepadAxis::LeftStickY).unwrap_or(0.0), 0.1));
        let r = deadband(pad.get(GamepadAxis::RightStickX).unwrap_or(0.0), 0.1);
        let t = pad.get(GamepadButton::LeftTrigger2).unwrap_or(0.0);
        if x != 0.0 || y != 0.0 || r != 0.0 || t > 0.0 {
            // (Bevy's stick y is up-positive; the browser's is down-positive)
            (lx, ly, rx, lt) = (x, -y, r, t);
        }
        slow |= pad.pressed(GamepadButton::LeftThumb);
        rt = rt.max(pad.get(GamepadButton::RightTrigger2).unwrap_or(0.0));
        rb |= pad.pressed(GamepadButton::RightTrigger);
        lb |= pad.pressed(GamepadButton::LeftTrigger);
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
    if k(&[KeyCode::Space]) { rt = 1.0; }
    rb |= k(&[KeyCode::KeyR]);
    lb |= k(&[KeyCode::KeyF]);

    let Some(r) = game.0.robots.first() else { return };
    let (max_speed, max_omega) = (r.max_speed, r.max_omega);
    let (mut up, mut right) = (-ly as f64, lx as f64);
    let mag = up.hypot(right).min(1.0);
    if mag > 0.0 { up *= mag; right *= mag; }
    let scale = if slow { 0.4 } else { 1.0 };
    // (js CameraRig.driveForward: away from the camera; from the driver station, downfield)
    let rp = game.0.physics.bodies.get(r.body).map(|b| b.translation());
    let f = rig.drive_forward(rp);
    let (fx, fz) = (f.x as f64, f.y as f64);
    let rx = rx as f64;
    *driver = Driver(Cmd {
        vx: (fx * up - fz * right) * max_speed * scale,
        vz: (fz * up + fx * right) * max_speed * scale,
        omega: -rx.signum() * rx * rx * max_omega * if slow { 0.5 } else { 1.0 },
        intake: lt > 0.3,
        shoot: rt > 0.3,
        pass: rb,
        outtake: lb,
    }, slow);
}

/// In a match: Esc / Menu pauses, Backspace / View restarts
fn keys(keys: Res<ButtonInput<KeyCode>>, pads: Query<&Gamepad>, screen: Res<State<Screen>>, mut cmds: MessageWriter<GameCmd>, mut game: ResMut<Game>) {
    if *screen.get() != Screen::Playing { return }
    // your HUMAN PLAYER (js input.js: X or G opens / closes the CHUTE DOOR, Y or H held throws)
    let hp = &mut game.0.hp_controls;
    hp.toggle_door |= keys.just_pressed(KeyCode::KeyG) || pads.iter().any(|p| p.just_pressed(GamepadButton::West));
    hp.throw_held = keys.pressed(KeyCode::KeyH) || pads.iter().any(|p| p.pressed(GamepadButton::North));
    if keys.just_pressed(KeyCode::Escape) || pads.iter().any(|p| p.just_pressed(GamepadButton::Start)) { cmds.write(GameCmd::Pause); }
    if keys.just_pressed(KeyCode::Backspace) || pads.iter().any(|p| p.just_pressed(GamepadButton::Select)) { cmds.write(GameCmd::Restart); }
}

/// js main.js startMatch / onResume / onMenu
fn commands(
    mut cmds: MessageReader<GameCmd>,
    mut commands: Commands,
    mut game: ResMut<Game>,
    mut world_gen: ResMut<WorldGen>,
    settings: Res<Settings>,
    setup: Option<Res<MatchSetup>>,
    mut next: ResMut<NextState<Screen>>,
    mut driver: ResMut<Driver>,
) {
    for c in cmds.read() {
        match c {
            GameCmd::Start | GameCmd::Restart => {
                let m = match (&setup, c) {
                    (Some(s), GameCmd::Restart) => (**s).clone(),
                    _ => MatchSetup { robot: settings.robot.clone(), alliance: settings.alliance(), start: settings.start.clone(), skip_auto: settings.auto == "skip", hp_auto: settings.hp == "auto" },
                };
                game.0 = new_world(Some(&m));
                commands.insert_resource(m);
                world_gen.0 += 1;
                *driver = Driver::default();
                next.set(Screen::Playing);
            }
            GameCmd::Resume => next.set(Screen::Playing),
            GameCmd::Pause => next.set(Screen::Paused),
            GameCmd::ToMenu => {
                game.0 = new_world(None);
                commands.remove_resource::<MatchSetup>();
                world_gen.0 += 1;
                next.set(Screen::Menu);
            }
        }
    }
}

/// An update waits while a match is being played; the results come up when it's over.
fn in_match(game: Res<Game>, screen: Res<State<Screen>>, mut m: ResMut<InMatch>, mut next: ResMut<NextState<Screen>>) {
    let s = *screen.get();
    let on = matches!(s, Screen::Playing | Screen::Paused) && !game.0.match_.over();
    if m.0 != on { m.0 = on; }
    if s == Screen::Playing && game.0.match_.over() { next.set(Screen::Results); }
}
