//! The cameras (js cameras.js CameraRig): Driver Station (where your driver stands behind the
//! ALLIANCE WALL, turning to keep the robot in view), Follow, Robot Chase, Overhead and Broadcast;
//! [ ] or the D-pad's left / right cycle them, T or the right stick's click turns the view around.
//! Behind the menus the camera circles the field slowly. The graphics setting is applied here too:
//! anti-aliasing on the camera, shadows on the sun.

use bevy::camera::{PerspectiveProjection, Projection};
use bevy::prelude::*;
use sim::constants::{Alliance, HALF_L, HALF_W};

use crate::game::{Game, WorldGen};
use crate::settings::Settings;
use crate::ui::Screen;

/// js CAMERA_MODES
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CamMode { Driver, Follow, Chase, Overhead, Broadcast }
pub const CAMERA_MODES: [CamMode; 5] = [CamMode::Driver, CamMode::Follow, CamMode::Chase, CamMode::Overhead, CamMode::Broadcast];

impl CamMode {
    /// js CAMERA_NAMES
    pub fn name(self) -> &'static str {
        match self { CamMode::Driver => "Driver Station", CamMode::Follow => "Follow", CamMode::Chase => "Robot Chase", CamMode::Overhead => "Overhead", CamMode::Broadcast => "Broadcast" }
    }
    pub fn key(self) -> &'static str {
        match self { CamMode::Driver => "driver", CamMode::Follow => "follow", CamMode::Chase => "chase", CamMode::Overhead => "overhead", CamMode::Broadcast => "broadcast" }
    }
    pub fn from_key(k: &str) -> CamMode { CAMERA_MODES.into_iter().find(|m| m.key() == k).unwrap_or(CamMode::Driver) }
}

/// js constants.js DRIVER_STATIONS[i].fy (from assets/constants.json: the generated constants
/// carry its item types, not the array)
pub fn driver_station_fy(i: usize) -> f64 {
    static FY: std::sync::OnceLock<Vec<f64>> = std::sync::OnceLock::new();
    let fy = FY.get_or_init(|| {
        let c: serde_json::Value = serde_json::from_str(include_str!("../../../assets/constants.json")).expect("constants.json");
        c["constants"]["DRIVER_STATIONS"].as_array().map(|a| a.iter().filter_map(|d| d["fy"].as_f64()).collect()).unwrap_or_default()
    });
    fy.get(i.min(fy.len().saturating_sub(1))).copied().unwrap_or(HALF_W)
}

/// js cameras.js: the driver's eyes, m behind the FIELD edge and above the carpet
const DS_EYE_BACK: f32 = 0.06 + 0.12;
const DS_EYE_H: f32 = 1.6;

/// js CameraRig
#[derive(Resource)]
pub struct CameraRig {
    pub mode: CamMode,
    pos: Vec3,
    look: Vec3,
    up: Vec3,
    pub alliance: Alliance,
    pub ds: usize,
    snap: bool,
    pub flip: bool,
}

impl Default for CameraRig {
    fn default() -> Self {
        CameraRig { mode: CamMode::Driver, pos: Vec3::new(0.0, 10.0, 10.0), look: Vec3::ZERO, up: Vec3::Y, alliance: Alliance::Blue, ds: 1, snap: true, flip: false }
    }
}

impl CameraRig {
    pub fn cycle(&mut self, dir: i32) {
        let i = CAMERA_MODES.iter().position(|m| *m == self.mode).unwrap_or(0) as i32;
        self.mode = CAMERA_MODES[((i + dir).rem_euclid(5)) as usize];
        self.snap = true;
    }
    pub fn toggle_flip(&mut self) { self.flip = !self.flip; self.snap = true; }

    /// js driveForward(robot): which way "up" on the stick drives, field-relative (a unit x, z)
    pub fn drive_forward(&self, robot: Option<Vec3>) -> Vec2 {
        let s = if self.alliance == Alliance::Blue { 1.0 } else { -1.0 };
        if self.mode == CamMode::Driver { return Vec2::new(s * if self.flip { -1.0 } else { 1.0 }, 0.0); }
        let mut d = if self.mode == CamMode::Overhead { Vec2::new(self.up.x, self.up.z) } else {
            let t = robot.unwrap_or(self.look);
            Vec2::new(t.x - self.pos.x, t.z - self.pos.z)
        };
        // right under the camera: the way it looks instead
        if self.mode != CamMode::Overhead && d.length() < 0.6 { d = Vec2::new(self.look.x - self.pos.x, self.look.z - self.pos.z); }
        d.normalize_or(Vec2::X)
    }

    /// js update(dt, robot): robot is its position and its forward (x, z); returns the fov, degrees
    fn update(&mut self, dt: f32, robot: Option<(Vec3, Vec2)>) -> f32 {
        let s = if self.alliance == Alliance::Blue { 1.0 } else { -1.0 };
        let (hl, hw) = (HALF_L as f32, HALF_W as f32);
        let rp = robot.map(|r| r.0).unwrap_or(Vec3::ZERO);
        let (mut p, mut l, fov) = match self.mode {
            CamMode::Driver => {
                let fy = driver_station_fy(self.ds) as f32;
                let z = s * (hw - fy);
                let p = Vec3::new(-s * (hl + DS_EYE_BACK), DS_EYE_H, z);
                let l = if robot.is_some() { Vec3::new(rp.x + s * 1.0, 0.25, rp.z) } else { Vec3::new(-s * (hl - 8.5), 0.0, z * 0.3) };
                (p, l, 55.0)
            }
            CamMode::Follow => (Vec3::new(rp.x - s * 3.4, 2.9, rp.z), Vec3::new(rp.x + s * 1.5, 0.3, rp.z), 60.0),
            CamMode::Chase => {
                let f = robot.map(|r| r.1).unwrap_or(Vec2::new(s, 0.0));
                (Vec3::new(rp.x - f.x * 2.6, 1.9, rp.z - f.y * 2.6), Vec3::new(rp.x + f.x * 2.5, 0.5, rp.z + f.y * 2.5), 65.0)
            }
            CamMode::Overhead => (Vec3::new(-s * 1.2, 15.5, 0.001), Vec3::ZERO, 52.0),
            CamMode::Broadcast => (Vec3::new(0.0, 7.5, hw + 7.5), Vec3::new(rp.x * 0.35, 0.0, 0.0), 50.0),
        };
        // turned around: the same shot from the other side of the robot
        if self.flip && self.mode != CamMode::Overhead {
            p.x = 2.0 * rp.x - p.x; p.z = 2.0 * rp.z - p.z;
            l.x = 2.0 * rp.x - l.x; l.z = 2.0 * rp.z - l.z;
        }
        if self.snap {
            (self.pos, self.look) = (p, l);
            self.snap = false;
        } else {
            let k = 1.0 - (-dt * if self.mode == CamMode::Chase { 8.0 } else { 6.0 }).exp();
            self.pos = self.pos.lerp(p, k);
            self.look = self.look.lerp(l, k);
        }
        // overhead: "downfield" up the screen
        self.up = if self.mode == CamMode::Overhead { Vec3::new(if self.flip { -s } else { s }, 0.0, 0.0) } else { Vec3::Y };
        fov
    }
}

pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CameraRig>()
            .add_systems(Startup, spawn)
            .add_systems(OnEnter(Screen::Playing), new_match)
            .add_systems(Update, (graphics, keys.run_if(in_state(Screen::Playing))))
            .add_systems(PostUpdate, place.before(TransformSystems::Propagate));
    }
}

fn spawn(mut commands: Commands) {
    commands.spawn((Camera3d::default(), IsDefaultUiCamera, Transform::from_xyz(0.0, 10.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y)));
}

/// A new match (not a resume): the starting camera, on the robot's ALLIANCE's side
fn new_match(mut rig: ResMut<CameraRig>, game: Res<Game>, settings: Res<Settings>, mut last: Local<u32>, world_gen: Res<WorldGen>) {
    if *last == world_gen.0 { return }
    *last = world_gen.0;
    rig.mode = CamMode::from_key(&settings.camera);
    // (screenshots: REBUILT_SHOT_CAMERA=<mode> starts in that one)
    if let Ok(m) = std::env::var("REBUILT_SHOT_CAMERA") { rig.mode = CamMode::from_key(&m); }
    rig.alliance = game.0.robots.first().map(|r| r.alliance).unwrap_or(Alliance::Blue);
    rig.ds = settings.ds;
    rig.flip = false;
    rig.snap = true;
}

/// [ ] or the D-pad cycle the cameras; T or the right stick's click turns it around
fn keys(keys: Res<ButtonInput<KeyCode>>, pads: Query<&Gamepad>, mut rig: ResMut<CameraRig>) {
    let p = |b: GamepadButton| pads.iter().any(|g| g.just_pressed(b));
    if keys.just_pressed(KeyCode::BracketLeft) || p(GamepadButton::DPadLeft) { rig.cycle(-1); }
    if keys.just_pressed(KeyCode::BracketRight) || keys.just_pressed(KeyCode::KeyV) || p(GamepadButton::DPadRight) { rig.cycle(1); }
    if keys.just_pressed(KeyCode::KeyT) || p(GamepadButton::RightThumb) { rig.toggle_flip(); }
}

/// Where the camera is this frame
fn place(
    mut cams: Query<(&mut Transform, &mut Projection), With<Camera3d>>,
    mut rig: ResMut<CameraRig>,
    game: Res<Game>,
    time: Res<Time>,
    screen: Res<State<Screen>>,
    mut menu_yaw: Local<f32>,
) {
    let Ok((mut t, mut proj)) = cams.single_mut() else { return };
    let dt = time.delta_secs();
    let fov = if *screen.get() == Screen::Menu {
        // behind the menus: the whole field, circling slowly
        *menu_yaw += 0.04 * dt;
        let (y, p, d) = (-0.6 + *menu_yaw, 0.55f32, 13.0f32);
        let eye = Quat::from_euler(EulerRot::YXZ, y, -p, 0.0) * Vec3::new(0.0, 0.0, d);
        *t = Transform::from_translation(eye).looking_at(Vec3::ZERO, Vec3::Y);
        rig.snap = true;
        55.0
    } else {
        let robot = game.0.robots.first().and_then(|r| {
            let b = game.0.physics.bodies.get(r.body)?;
            let f = *b.rotation() * Vec3::X;
            Some((b.translation(), Vec2::new(f.x, f.z).normalize_or(Vec2::X)))
        });
        let fov = rig.update(dt, robot);
        *t = Transform::from_translation(rig.pos).looking_at(rig.look, rig.up);
        fov
    };
    // (three.js's fov is vertical, in degrees, as Bevy's is in radians)
    if let Projection::Perspective(pp) = &mut *proj {
        let f = (fov as f32).to_radians();
        if (pp.fov - f).abs() > 1e-4 { *pp = PerspectiveProjection { fov: f, ..pp.clone() }; }
    }
}

/// The graphics preset (js graphics.js, as far as Bevy goes): Low: no shadows or anti-aliasing;
/// Medium: shadows and FXAA; High: 4x MSAA, bigger shadows and bloom (what glows, glows: the HUB
/// lights and the LEDs); Ultra: temporal anti-aliasing, ambient occlusion and temporally filtered
/// soft shadows, 4K cascades out further.
#[allow(clippy::too_many_arguments)]
fn graphics(
    settings: Res<Settings>,
    mut commands: Commands,
    cams: Query<Entity, With<Camera3d>>,
    mut suns: Query<(Entity, &mut DirectionalLight)>,
    env: Option<Res<crate::arena::ArenaEnv>>,
    mut truss: Query<&mut Visibility, With<crate::arena::TrussLight>>,
    mut ambient: ResMut<GlobalAmbientLight>,
) {
    use bevy::anti_alias::{fxaa::Fxaa, taa::TemporalAntiAliasing};
    use bevy::core_pipeline::prepass::{DepthPrepass, MotionVectorPrepass, NormalPrepass};
    use bevy::light::{CascadeShadowConfigBuilder, DirectionalLightShadowMap, ShadowFilteringMethod};
    use bevy::pbr::{ScreenSpaceAmbientOcclusion, ScreenSpaceAmbientOcclusionQualityLevel};
    use bevy::post_process::bloom::{Bloom, BloomCompositeMode, BloomPrefilter};
    use bevy::camera::Hdr;
    use bevy::render::camera::{TemporalJitter, MipBias};
    use bevy::light::{GeneratedEnvironmentMapLight, Skybox};
    use bevy::post_process::effect_stack::Vignette;
    use bevy::render::view::{ColorGrading, ColorGradingGlobal, ColorGradingSection};
    if !settings.is_changed() { return }
    let g = settings.graphics.as_str();
    let (high, ultra) = (g == "high" || g == "ultra", g == "ultra");
    // (the arena's light, arena.rs: from Medium up, the environment's fill and reflections and the
    // trusses' lights; Low keeps the plain sun and ambient)
    let arena = g != "low" && env.is_some();
    ambient.brightness = if arena { 60.0 } else { 400.0 };
    for mut v in &mut truss { *v = if arena { Visibility::Visible } else { Visibility::Hidden }; }
    let blur = g != "low" && matches!(settings.motion.as_str(), "blur" | "both");
    for c in &cams {
        let mut e = commands.entity(c);
        e.remove::<(Fxaa, TemporalAntiAliasing, TemporalJitter, MipBias, ScreenSpaceAmbientOcclusion, Bloom, Hdr)>()
            .remove::<(DepthPrepass, NormalPrepass, MotionVectorPrepass)>()
            .remove::<(GeneratedEnvironmentMapLight, bevy::light::EnvironmentMapLight, Skybox, Vignette, ColorGrading)>()
            .remove::<bevy::post_process::motion_blur::MotionBlur>();
        // (motion blur, as the Motion effects setting asks: half a frame's shutter)
        if blur { e.insert(bevy::post_process::motion_blur::MotionBlur { shutter_angle: 0.5, samples: 3 }); }
        if let (true, Some(env)) = (arena, env.as_ref()) {
            let i = crate::arena::ENV_INTENSITY;
            e.insert((
                GeneratedEnvironmentMapLight { environment_map: env.0.clone(), intensity: i, ..default() },
                Skybox { image: Some(env.0.clone()), brightness: i, ..default() },
                // (an arena's warmer light, a touch more contrast and color)
                ColorGrading {
                    global: ColorGradingGlobal { temperature: 0.01, post_saturation: 1.08, ..default() },
                    midtones: ColorGradingSection { contrast: 1.1, ..default() },
                    shadows: ColorGradingSection { contrast: 1.05, ..default() },
                    ..default()
                },
            ));
        }
        // (a soft vignette; screen-space effects only on Ultra: the time goes to detail instead)
        if high && arena { e.insert(Vignette { intensity: 0.3, radius: 0.95, smoothness: 3.0, ..default() }); }
        e.insert(match g { "high" => Msaa::Sample4, _ => Msaa::Off });
        e.insert(if ultra { ShadowFilteringMethod::Temporal } else { ShadowFilteringMethod::Gaussian });
        if g == "medium" { e.insert(Fxaa::default()); }
        if high {
            // (only what's brighter than a lit panel blooms: the lights, lights.rs glow)
            e.insert((Hdr, Bloom {
                intensity: 0.15,
                low_frequency_boost: 0.35,
                prefilter: BloomPrefilter { threshold: 4.0, threshold_softness: 0.2 },
                composite_mode: BloomCompositeMode::Additive,
                ..Bloom::NATURAL
            }));
        }
        if ultra {
            e.insert((TemporalAntiAliasing::default(), ScreenSpaceAmbientOcclusion { quality_level: ScreenSpaceAmbientOcclusionQualityLevel::Medium, ..default() }));
        }
    }
    commands.insert_resource(DirectionalLightShadowMap { size: if high { 4096 } else { 2048 } });
    for (e, mut s) in &mut suns {
        s.shadow_maps_enabled = g != "low";
        // (with the trusses' lights and the arena's fill, the sun's a key light, half as bright)
        s.illuminance = if arena { 4500.0 } else { 9000.0 };
        s.contact_shadows_enabled = false;
        commands.entity(e).insert(if ultra {
            CascadeShadowConfigBuilder { num_cascades: 4, first_cascade_far_bound: 6.0, maximum_distance: 60.0, ..default() }.build()
        } else {
            CascadeShadowConfigBuilder { maximum_distance: 40.0, ..default() }.build()
        });
    }
}
