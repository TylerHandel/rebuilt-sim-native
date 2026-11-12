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

/// The camera the match is seen through (not the robot showcase's)
#[derive(Component)]
pub struct MainCam;

fn spawn(mut commands: Commands) {
    commands.spawn((Camera3d::default(), MainCam, IsDefaultUiCamera, Transform::from_xyz(0.0, 10.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y)));
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
    mut cams: Query<(&mut Transform, &mut Projection), With<MainCam>>,
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
/// lights and the LEDs); Ultra: temporal anti-aliasing and 4K shadows out further. (No screen-space
/// ambient occlusion: its rays marched through the depth each pixel every frame for little the
/// arena's fill doesn't already give.)
#[allow(clippy::too_many_arguments)]
fn graphics(
    settings: Res<Settings>,
    mut commands: Commands,
    cams: Query<Entity, With<MainCam>>,
    suns: Query<Entity, With<DirectionalLight>>,
    env: Option<Res<crate::arena::ArenaEnv>>,
    mut truss: Query<&mut Visibility, With<crate::arena::TrussLight>>,
    mut curves: ResMut<Assets<bevy::post_process::auto_exposure::AutoExposureCompensationCurve>>,
    mut eye: Local<Vec<(i32, Handle<bevy::post_process::auto_exposure::AutoExposureCompensationCurve>)>>,
    dlss: Upscaler,
) {
    use bevy::anti_alias::{fxaa::Fxaa, taa::TemporalAntiAliasing};
    use bevy::core_pipeline::prepass::{DepthPrepass, MotionVectorPrepass, NormalPrepass};
    use bevy::light::{CascadeShadowConfigBuilder, DirectionalLightShadowMap, ShadowFilteringMethod};
    use bevy::pbr::ScreenSpaceAmbientOcclusion;
    use bevy::post_process::bloom::{Bloom, BloomCompositeMode, BloomPrefilter};
    use bevy::camera::Hdr;
    use bevy::render::camera::{TemporalJitter, MipBias};
    use bevy::light::{GeneratedEnvironmentMapLight, Skybox};
    use bevy::post_process::effect_stack::Vignette;
    use bevy::render::view::{ColorGrading, ColorGradingGlobal, ColorGradingSection};
    use bevy::core_pipeline::tonemapping::Tonemapping;
    use bevy::camera::Exposure;
    use bevy::light::VolumetricFog;
    use bevy::pbr::{DistanceFog, FogFalloff};
    use bevy::post_process::auto_exposure::AutoExposure;
    use bevy::post_process::effect_stack::ChromaticAberration;
    // (again when a new environment's ready: its look's made in the background, arena.rs)
    if !settings.is_changed() && !env.as_ref().is_some_and(|e| e.is_changed()) { return }
    dlss.note();
    let look = crate::arena::Look::of(&settings);
    let g = settings.graphics.as_str();
    let (high, ultra) = (g == "high" || g == "ultra", g == "ultra");
    // (the arena's light, arena.rs: from Medium up, the environment's fill and reflections and the
    // trusses' lights; Low keeps the plain sun and ambient)
    let arena = g != "low" && env.is_some();
    // (outdoors there are no trusses: the light towers instead, scene.rs)
    for mut v in &mut truss { *v = if arena && !look.outdoor { Visibility::Visible } else { Visibility::Hidden }; }
    let blur = g != "low" && matches!(settings.motion.as_str(), "blur" | "both");
    for c in &cams {
        let mut e = commands.entity(c);
        e.remove::<(Fxaa, TemporalAntiAliasing, TemporalJitter, MipBias, ScreenSpaceAmbientOcclusion, Bloom, Hdr)>()
            .remove::<(DepthPrepass, NormalPrepass, MotionVectorPrepass)>()
            .remove::<(GeneratedEnvironmentMapLight, bevy::light::EnvironmentMapLight, Skybox, Vignette, ColorGrading)>()
            .remove::<bevy::post_process::motion_blur::MotionBlur>()
            .remove::<(Tonemapping, ChromaticAberration, AutoExposure, VolumetricFog, DistanceFog, Exposure)>()
            .remove::<bevy::anti_alias::contrast_adaptive_sharpening::ContrastAdaptiveSharpening>();
        dlss.clear(&mut e);
        // (motion blur, as the Motion effects setting asks: half a frame's shutter)
        if blur { e.insert(bevy::post_process::motion_blur::MotionBlur { shutter_angle: 0.5, samples: 3 }); }
        if let (true, Some(env)) = (arena, env.as_ref()) {
            let i = crate::arena::ENV_INTENSITY;
            e.insert((
                // (filtered once at the start, arena.rs: nothing to do for it each frame)
                bevy::light::EnvironmentMapLight { diffuse_map: env.diffuse.clone(), specular_map: env.specular.clone(), intensity: i, ..default() },
                // (outdoors the sky itself brighter than the light it gives: a clear sky's as bright as
                // the sunlit grass under it, and the sun here is scaled down with everything else)
                Skybox { image: Some(env.sky.clone()), brightness: i * if look.outdoor { match look.time { crate::arena::Tod::Day => 5.0, crate::arena::Tod::Sunset => 3.0, crate::arena::Tod::Night => 1.5 } } else { 1.0 }, ..default() },
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
        // (only with the film look: the clear look keeps the corners as bright as the middle)
        if high && arena && settings.film { e.insert(Vignette { intensity: 0.2, radius: 0.95, smoothness: 3.0, ..default() }); }
        e.insert(ShadowFilteringMethod::Gaussian);
        // NVIDIA DLSS, where it's on and the GPU has it: it anti-aliases (and upscales) in place of
        // MSAA, FXAA and TAA
        let upscaled = dlss.apply(&settings, &mut e);
        e.insert(match g { "high" if !upscaled => Msaa::Sample4, _ => Msaa::Off });
        if g == "medium" && !upscaled { e.insert(Fxaa::default()); }
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
        if ultra && !upscaled {
            e.insert(TemporalAntiAliasing::default());
        }
        // how bright the scene's drawn: the sunlit stadium's a stop darker; night as the arena by day, so it's dark
        let ev = match (look.outdoor, look.time) {
            (true, crate::arena::Tod::Day) if look.cloud() < 0.5 => 10.6,
            (true, crate::arena::Tod::Day) => 10.0,
            (_, crate::arena::Tod::Night) => 9.7,
            _ => 9.7,
        };
        e.insert(Exposure { ev100: if arena { ev } else { 9.7 } });
        // the film look: filmic tone mapping and a touch of lens fringing; eyes that adapt (where
        // the image is HDR, High and Ultra), within a stop and a half either way
        if settings.film {
            e.insert((Tonemapping::AgX, ChromaticAberration { intensity: 0.0015, max_samples: 4, ..default() }));
            if high {
                // (the meter reads the image after the camera's exposure: the average the look's
                // tuned to (2^-2.25 by day, darker at dusk and darker still at night, so night
                // stays night) is what the eye corrects toward, by at most a stop and a half either
                // way; the curve's y is that correction plus the average. Quarter stops: exact in
                // binary, as the curve checks its joins with ==)
                let a = match look.time { crate::arena::Tod::Day => -2.25, crate::arena::Tod::Sunset => -2.75, crate::arena::Tod::Night => -4.25 };
                let k = (a * 4.0) as i32;
                let curve = match eye.iter().find(|(j, _)| *j == k) {
                    Some((_, h)) => h.clone(),
                    None => {
                        use bevy::{curve::cubic_splines::LinearSpline, math::vec2};
                        let s = 1.5;
                        let h = curves.add(bevy::post_process::auto_exposure::AutoExposureCompensationCurve::from_curve(LinearSpline::new([
                            vec2(-10.0, -10.0 + s), vec2(a - s, a), vec2(a + s, a), vec2(4.0, 4.0 - s),
                        ])).expect("a monotonic eye curve"));
                        eye.push((k, h.clone()));
                        h
                    }
                };
                e.insert(AutoExposure { range: -10.0..=4.0, speed_brighten: 1.5, speed_darken: 2.0, compensation_curve: curve, ..default() });
            }
        } else {
            // the clear look: a neutral tone mapping that keeps colors as they are, a steady exposure,
            // and the anti-aliasing's softness sharpened back (on Medium up: FXAA, TAA and DLSS all
            // soften a little)
            e.insert(Tonemapping::TonyMcMapface);
            if g != "low" { e.insert(bevy::anti_alias::contrast_adaptive_sharpening::ContrastAdaptiveSharpening { enabled: true, sharpening_strength: 0.4, denoise: false }); }
        }
        // the haze the lights' beams show in (scene.rs's volume over the field): the steps each
        // pixel's ray takes through it; jittered where temporal anti-aliasing smooths the grain
        if arena && settings.haze != "off" {
            e.insert(VolumetricFog { step_count: if settings.haze == "high" { 48 } else { 24 }, jitter: if ultra { 0.6 } else { 0.0 }, ambient_intensity: 0.0, ..default() });
        }
        // outdoors: the distance hazing over (the fog's own, the rain's, a clear day's faint one)
        let (fc, fk) = look.haze();
        if arena && look.outdoor && fk > 0.0 {
            let c = fc * 1.6;
            e.insert(DistanceFog {
                color: Color::linear_rgb(c.x, c.y, c.z),
                directional_light_color: if look.time == crate::arena::Tod::Night { Color::NONE } else { Color::linear_rgba(1.0, 0.85, 0.65, 0.4) },
                directional_light_exponent: 24.0,
                falloff: FogFalloff::Exponential { density: fk },
            });
        }
    }
    commands.insert_resource(DirectionalLightShadowMap { size: if high { 4096 } else { 2048 } });
    // (the key light's brightness, color and shadows are the scene's: scene.rs)
    for e in &suns {
        commands.entity(e).insert(if ultra {
            CascadeShadowConfigBuilder { num_cascades: 3, first_cascade_far_bound: 7.0, maximum_distance: 50.0, ..default() }.build()
        } else if high {
            CascadeShadowConfigBuilder { num_cascades: 3, first_cascade_far_bound: 6.0, maximum_distance: 40.0, ..default() }.build()
        } else {
            // (each cascade draws every shadow caster again: Medium's two, out to the far wall)
            CascadeShadowConfigBuilder { num_cascades: 2, first_cascade_far_bound: 8.0, maximum_distance: 30.0, ..default() }.build()
        });
    }
}

/// Whether the game started on Vulkan for DLSS (main.rs), and whether the GPU has it (the menu says)
pub static DLSS_LAUNCHED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
pub static DLSS_READY: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// NVIDIA DLSS as the Upscaler setting has it (only in builds with the `dlss` feature, and only
/// where the GPU and driver support it: `DlssSuperResolutionSupported`)
#[derive(bevy::ecs::system::SystemParam)]
pub struct Upscaler<'w> {
    #[cfg(feature = "dlss")]
    supported: Option<Res<'w, bevy::anti_alias::dlss::DlssSuperResolutionSupported>>,
    #[cfg(not(feature = "dlss"))]
    _none: std::marker::PhantomData<&'w ()>,
}

impl Upscaler<'_> {
    /// whether this build and GPU can run it
    pub fn available(&self) -> bool {
        #[cfg(feature = "dlss")]
        return self.supported.is_some();
        #[cfg(not(feature = "dlss"))]
        false
    }

    fn note(&self) { DLSS_READY.store(self.available(), std::sync::atomic::Ordering::Relaxed); }

    fn clear(&self, _e: &mut EntityCommands) {
        #[cfg(feature = "dlss")]
        _e.remove::<bevy::anti_alias::dlss::Dlss<bevy::anti_alias::dlss::DlssSuperResolutionFeature>>();
    }

    /// on the camera, as the setting asks (whether it's on)
    fn apply(&self, _s: &Settings, _e: &mut EntityCommands) -> bool {
        #[cfg(feature = "dlss")]
        if self.available() {
            use bevy::anti_alias::dlss::{Dlss, DlssPerfQualityMode as Q};
            let mode = match _s.upscaler.as_str() { "dlaa" => Q::Dlaa, "quality" => Q::Quality, "balanced" => Q::Balanced, "performance" => Q::Performance, _ => return false };
            _e.insert(Dlss::<bevy::anti_alias::dlss::DlssSuperResolutionFeature> { perf_quality_mode: mode, reset: true, ..default() });
            return true;
        }
        false
    }
}
