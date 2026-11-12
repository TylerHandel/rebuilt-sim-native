//! The scene the match is played in, as the Atmosphere settings choose it (arena.rs `Look`: indoors
//! in the gym or outdoors in a stadium, by day, at sunset or at night, in what weather). arena.rs
//! draws its environment (the sky, the stands, what everything reflects); this lights it and moves
//! what moves in it:
//!   - the key light: the sun (low and warm at sunset, dimmed by clouds, gone in the rain's overcast),
//!     indoors the gym's lights, at night the stage's wash or a stadium light tower's beams;
//!   - outdoors, the stadium's light towers (lit at night) and its turf in place of the gym floor;
//!   - at night, the show: the trusses' spots sweeping in color, flaring to an alliance's color when
//!     it scores; each HUB glowing in its alliance's color while it's active; each robot's LEDs
//!     lighting the carpet round it; camera flashes in the crowd;
//!   - rain: falling streaks round the camera, and the carpet, turf and robots wet (glossier,
//!     darker);
//!   - volumetric fog (Settings haze): a volume of haze over the field the lights' beams show in.
//! Nothing here touches the simulation.

use bevy::asset::RenderAssetUsages;
use bevy::light::{FogVolume, NotShadowCaster, NotShadowReceiver, VolumetricLight};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use sim::constants::{Alliance, HALF_W};

use crate::arena::{Look, Tod, TrussLight, Weather, TOWERS};
use crate::game::Game;
use crate::settings::Settings;

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Show>()
            .add_systems(Startup, (spawn, spawn_grain))
            .add_systems(Update, ((apply, show, flashes, rain, wet).chain(), grain));
    }
}

/// a stadium tower's beams (a spot light), and the towers drawn
#[derive(Component)]
struct Tower;
#[derive(Component)]
struct TowerPart;
/// the light a robot's LEDs throw round it
#[derive(Component)]
struct Pool(usize);
/// a HUB's glow at night
#[derive(Component)]
struct HubGlow(Alliance);
/// a camera flash in the crowd (how long it has left)
#[derive(Component)]
struct Flash(f32);
#[derive(Component)]
struct Rain;
/// the volume of haze over the field
#[derive(Component)]
struct Haze;
/// a truss light's resting pose and color
#[derive(Component)]
struct Base(Transform);

#[derive(Resource, Default)]
struct Show {
    /// what was last applied: the look, the preset, the haze
    applied: Option<(Look, String, String)>,
    /// each alliance's score last frame, and a score's flare (how much is left, its color)
    score: [f64; 2],
    burst: f32,
    burst_color: Vec3,
    /// the turf outdoors (made once the floors are)
    turf: Option<Entity>,
    /// the rain's streaks: where each is in its column (x, z, phase, speed)
    drops: Vec<[f32; 4]>,
    rain_mesh: Option<Handle<Mesh>>,
    /// wet or not, as last applied (and how many materials there were then), and their dry finish
    wet: Option<(bool, usize)>,
    dry: Vec<(AssetId<StandardMaterial>, f32, f32, LinearRgba)>,
    dry_worn: Vec<(AssetId<crate::materials::WornMaterial>, f32, LinearRgba)>,
    dry_brushed: Vec<(AssetId<crate::materials::BrushedMaterial>, f32)>,
    seed: u32,
}

impl Show {
    fn rand(&mut self) -> f32 {
        self.seed = self.seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (self.seed >> 8) as f32 / 16_777_216.0
    }
}

fn alliance_color(a: Alliance) -> Vec3 { if a == Alliance::Blue { Vec3::new(0.12, 0.3, 1.0) } else { Vec3::new(1.0, 0.12, 0.08) } }

/// how far into the night show it is (0 by day; indoors the evening is half way)
fn night_k(look: &Look) -> f32 {
    match (look.outdoor, look.time) { (_, Tod::Night) => 1.0, (false, Tod::Sunset) => 0.5, (true, Tod::Sunset) => 0.35, _ => 0.0 }
}

fn spawn(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>, mut show: ResMut<Show>) {
    show.seed = 0x9e37_79b9;
    // the stadium's light towers: a pole and its bank of lights at each corner, aimed at the field
    let pole = meshes.add(Cylinder::new(0.45, 30.0));
    let bank = meshes.add(Cuboid::new(7.0, 3.2, 0.5));
    let steel = mats.add(StandardMaterial { base_color: Color::srgb(0.28, 0.29, 0.31), perceptual_roughness: 0.6, metallic: 0.7, ..default() });
    let lamps = mats.add(StandardMaterial { base_color: Color::srgb(0.9, 0.9, 0.88), emissive: LinearRgba::rgb(0.0, 0.0, 0.0), ..default() });
    for t in TOWERS {
        let aim = Vec3::new(0.0, 0.0, 0.0);
        commands.spawn((Mesh3d(pole.clone()), MeshMaterial3d(steel.clone()), Transform::from_xyz(t.x * 1.04, 15.0, t.z * 1.04), Visibility::Hidden, TowerPart, NotShadowCaster));
        commands.spawn((Mesh3d(bank.clone()), MeshMaterial3d(lamps.clone()), Transform::from_translation(t).looking_at(Vec3::new(0.0, t.y, 0.0), Vec3::Y), Visibility::Hidden, TowerPart, NotShadowCaster));
        commands.spawn((
            SpotLight { color: Color::srgb(1.0, 0.97, 0.92), intensity: 0.0, range: 90.0, radius: 1.5, shadow_maps_enabled: false, inner_angle: 0.25, outer_angle: 0.42, ..default() },
            Transform::from_translation(t).looking_at(aim, Vec3::Y),
            Visibility::Hidden,
            Tower,
        ));
    }
    // each HUB's glow
    for a in [Alliance::Blue, Alliance::Red] {
        let c = sim::field::hub_center(a);
        commands.spawn((PointLight { intensity: 0.0, range: 7.0, radius: 0.6, shadow_maps_enabled: false, ..default() }, Transform::from_xyz(c.0 as f32, 2.0, c.1 as f32), Visibility::Hidden, HubGlow(a)));
    }
    // the crowd's camera flashes
    let dot = meshes.add(Sphere::new(0.05).mesh().ico(1).unwrap());
    let white = mats.add(StandardMaterial { base_color: Color::WHITE, emissive: LinearRgba::rgb(400.0, 400.0, 390.0), unlit: true, ..default() });
    for _ in 0..32 {
        commands.spawn((Mesh3d(dot.clone()), MeshMaterial3d(white.clone()), Transform::default(), Visibility::Hidden, Flash(0.0), NotShadowCaster, NotShadowReceiver));
    }
    // the rain's streaks (one mesh, remade round the camera each frame)
    let mesh = meshes.add(Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default()));
    let water = mats.add(StandardMaterial { base_color: Color::srgba(0.75, 0.8, 0.88, 0.22), alpha_mode: AlphaMode::Blend, unlit: true, cull_mode: None, ..default() });
    commands.spawn((Mesh3d(mesh.clone()), MeshMaterial3d(water), Transform::default(), Visibility::Hidden, Rain, NotShadowCaster, NotShadowReceiver, bevy::camera::visibility::NoFrustumCulling));
    show.rain_mesh = Some(mesh);
    for _ in 0..2600 {
        let d = [show.rand() * 28.0 - 14.0, show.rand() * 28.0 - 14.0, show.rand() * 9.0, 7.5 + show.rand() * 2.5];
        show.drops.push(d);
    }
    // the haze over the field: from just past its walls up over the trusses
    commands.spawn((
        FogVolume { density_factor: 0.0, absorption: 0.08, scattering: 0.35, scattering_asymmetry: 0.72, fog_color: Color::srgb(0.95, 0.95, 1.0), ..default() },
        Transform::from_xyz(0.0, 6.0, 0.0).with_scale(Vec3::new(44.0, 12.0, 32.0)),
        Visibility::Hidden,
        Haze,
    ));
}

/// The key light: toward it, its illuminance (lux), color, and whether it casts shadows
fn key(look: &Look, g: &str) -> (Vec3, f32, Vec3, bool) {
    let big = g == "high" || g == "ultra";
    let cover = look.cloud();
    let dry = !matches!(look.weather, Weather::Rain | Weather::Fog);
    match (look.outdoor, look.time) {
        (false, Tod::Day) => (look.sun_dir(), 4500.0, Vec3::ONE, true),
        (false, Tod::Sunset) => (look.sun_dir(), 2600.0, Vec3::new(1.0, 0.85, 0.7), true),
        // (the stage's wash from over the field: its shadows only where there's GPU for them)
        (false, Tod::Night) => (Vec3::new(0.2, 1.0, 0.25).normalize(), 1100.0, Vec3::new(0.7, 0.62, 1.0), big),
        (true, Tod::Day) => (look.sun_dir(), match look.weather { Weather::Clear => 11000.0, Weather::Cloudy => 7000.0, Weather::Rain => 1500.0, Weather::Fog => 2200.0 }, Vec3::new(1.0, 0.96, 0.9), dry),
        (true, Tod::Sunset) => (look.sun_dir(), 5500.0 * (1.0 - 0.7 * cover), Vec3::new(1.0, 0.6, 0.36), dry),
        // (a tower's beams from its corner, the others' without shadows)
        (true, Tod::Night) => ((TOWERS[0] - Vec3::Y * 2.0).normalize(), 3000.0, Vec3::new(0.95, 0.96, 1.0), big || g == "medium"),
    }
}

/// what the scene's lights, floors and haze are, for the look the settings ask for
#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn apply(
    settings: Res<Settings>,
    mut show: ResMut<Show>,
    floors: Option<Res<crate::field::Floors>>,
    mut suns: Query<(Entity, &mut DirectionalLight, &mut Transform), (Without<Tower>, Without<TrussLight>)>,
    mut ambient: ResMut<GlobalAmbientLight>,
    mut towers: Query<(Entity, &mut SpotLight, &mut Visibility), (With<Tower>, Without<TrussLight>, Without<TowerPart>)>,
    mut parts: Query<&mut Visibility, (With<TowerPart>, Without<Tower>)>,
    trusses: Query<Entity, With<TrussLight>>,
    mut haze: Query<(&mut FogVolume, &mut Visibility), (With<Haze>, Without<Tower>, Without<TowerPart>)>,
    mut named: Query<(&Name, &mut Visibility), (Without<Haze>, Without<Tower>, Without<TowerPart>, Without<TrussLight>)>,
    mut vis: Query<&mut Visibility, (Without<Name>, Without<Haze>, Without<Tower>, Without<TowerPart>, Without<TrussLight>)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    added: Query<(), Added<Name>>,
    mut commands: Commands,
) {
    let look = Look::of(&settings);
    let g = settings.graphics.clone();
    let key_now = (look, g.clone(), settings.haze.clone());
    // (the turf, once the floors are made: out to the stands)
    if show.turf.is_none() {
        if let Some(f) = &floors {
            // (a mowed disk 30 m across the field's middle; nature.rs's meadow and hills beyond it)
            if let Some(mut m) = mats.get_mut(&f.turf) { m.uv_transform = bevy::math::Affine2::from_scale(Vec2::splat(60.0 / 9.0)); }
            let e = commands.spawn((Mesh3d(meshes.add(Circle::new(30.0).mesh().resolution(96))), MeshMaterial3d(f.turf.clone()), Transform::from_xyz(0.0, -0.006, 0.0).with_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)), Visibility::Hidden)).id();
            show.turf = Some(e);
            show.applied = None;
            // (it's not in the queries until next frame: the look's applied then)
            return;
        }
    }
    // (again when parts come in, the venue's model loading after the look was first applied)
    if show.applied.as_ref() == Some(&key_now) && !settings.is_changed() && added.is_empty() { return }
    show.applied = Some(key_now);
    let arena = g != "low";
    let haze_on = arena && settings.haze != "off";
    let haze_high = settings.haze == "high";
    // the key light
    let (dir, lux, col, shadows) = key(&look, &g);
    for (e, mut s, mut tf) in &mut suns {
        if arena {
            *tf = Transform::from_translation(dir * 20.0).looking_at(Vec3::ZERO, Vec3::Y);
            s.illuminance = lux;
            s.color = Color::linear_rgb(col.x, col.y, col.z);
            s.shadow_maps_enabled = shadows;
            // (the sun's rays in the haze, where it has shadows to cast them)
            if haze_on && haze_high && shadows && look.time != Tod::Night { commands.entity(e).insert(VolumetricLight); } else { commands.entity(e).remove::<VolumetricLight>(); }
        } else {
            *tf = Transform::from_xyz(4.0, 12.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y);
            s.illuminance = 9000.0;
            s.color = Color::WHITE;
            s.shadow_maps_enabled = false;
            commands.entity(e).remove::<VolumetricLight>();
        }
    }
    // the fill: overcast fills in, night empties it
    ambient.brightness = if !arena { 400.0 } else {
        match look.time { Tod::Night => 10.0, Tod::Sunset => 35.0, Tod::Day => if look.outdoor && look.cloud() > 0.5 { 110.0 } else { 60.0 } }
    };
    // the towers: drawn outdoors, lit at night
    let towers_lit = arena && look.outdoor && look.time == Tod::Night;
    for (e, mut s, mut v) in &mut towers {
        *v = if towers_lit { Visibility::Visible } else { Visibility::Hidden };
        s.intensity = if towers_lit { 40_000_000.0 } else { 0.0 };
        if haze_on && towers_lit { commands.entity(e).insert(VolumetricLight); } else { commands.entity(e).remove::<VolumetricLight>(); }
    }
    for mut v in &mut parts { *v = if arena && look.outdoor { Visibility::Visible } else { Visibility::Hidden }; }
    // (the towers' lamps: lit at night)
    // the trusses' lights in the haze
    for e in &trusses { if haze_on { commands.entity(e).insert(VolumetricLight); } else { commands.entity(e).remove::<VolumetricLight>(); } }
    // outdoors: the turf instead of the gym floor
    for (n, mut v) in &mut named {
        // (outdoors, in the countryside: no gym, stands, crowd or its screens; the scoring table,
        // the staff and the drive teams stay)
        if matches!(n.as_str(), "venue-wall" | "venue-accent" | "venue-banner" | "venue-drape" | "venue-stand" | "venue-seat" | "venue-crowd" | "venue-metal" | "venue-screens" | "venue-dark" | "venue-lens") {
            let want = if look.outdoor { Visibility::Hidden } else { Visibility::Inherited };
            if *v != want { *v = want; }
        }
    }
    if let (Some(f), Some(t)) = (&floors, show.turf) {
        if let Ok(mut v) = vis.get_mut(f.ground) { *v = if look.outdoor && arena { Visibility::Hidden } else { Visibility::Inherited }; }
        if let Ok(mut v) = vis.get_mut(t) { *v = if look.outdoor && arena { Visibility::Inherited } else { Visibility::Hidden }; }
    }
    // the haze: thicker at night (it's what the beams are for), thickest in the fog
    for (mut fv, mut v) in &mut haze {
        *v = if haze_on { Visibility::Visible } else { Visibility::Hidden };
        let base = if haze_high { 0.05 } else { 0.032 };
        let t = match look.time { Tod::Night => 1.0, Tod::Sunset => 0.8, Tod::Day => 0.5 };
        let w = if look.outdoor && look.weather == Weather::Fog { 3.0 } else if look.outdoor && look.weather == Weather::Rain { 1.6 } else { 1.0 };
        fv.density_factor = base * t * w;
    }
}

/// The show, each frame: the trusses' spots, the HUBS' glow, the robots' light pools, the score's flare
#[allow(clippy::type_complexity)]
fn show(
    time: Res<Time>,
    settings: Res<Settings>,
    game: Res<Game>,
    glow: Res<crate::lights::LedGlow>,
    mut show: ResMut<Show>,
    mut trusses: Query<(Entity, &mut SpotLight, &mut Transform, Option<&Base>), (With<TrussLight>, Without<Pool>, Without<HubGlow>)>,
    mut hubs: Query<(&HubGlow, &mut PointLight, &mut Visibility), Without<Pool>>,
    mut pools: Query<(&Pool, &mut PointLight, &mut Transform, &mut Visibility), (Without<HubGlow>, Without<TrussLight>)>,
    mut commands: Commands,
) {
    let look = Look::of(&settings);
    let arena = settings.graphics != "low";
    let n = if arena { night_k(&look) } else { 0.0 };
    let t = time.elapsed_secs();
    let dt = time.delta_secs();
    let w = &game.0;
    // a score: the trusses flare to its alliance's color
    let now = [w.match_.total(Alliance::Blue), w.match_.total(Alliance::Red)];
    for (k, a) in [Alliance::Blue, Alliance::Red].into_iter().enumerate() {
        if now[k] > show.score[k] { show.burst = 1.0; show.burst_color = alliance_color(a); }
    }
    show.score = now;
    show.burst = (show.burst - dt * 1.2).max(0.0);
    // the trusses' spots: at night sweeping in color, by day still and white
    for (i, (e, mut s, mut tf, base)) in trusses.iter_mut().enumerate() {
        let Some(base) = base else { commands.entity(e).insert(Base(*tf)); continue };
        if n >= 0.99 {
            let f = i as f32;
            let yaw = (t * 0.45 + f * 1.3).sin() * 0.42;
            let pitch = (t * 0.33 + f * 0.7).sin() * 0.16;
            tf.rotation = base.0.rotation * Quat::from_rotation_y(yaw) * Quat::from_rotation_x(pitch);
            let hue = (f * 0.23 + t * 0.05).fract();
            let palette = [Vec3::new(0.55, 0.25, 1.0), Vec3::new(0.2, 0.55, 1.0), Vec3::new(0.15, 0.95, 0.9), Vec3::new(1.0, 0.3, 0.75)];
            let k = hue * 4.0;
            let (a, b) = (palette[k as usize % 4], palette[(k as usize + 1) % 4]);
            let c = a.lerp(b, k.fract()).lerp(show.burst_color, show.burst);
            s.color = Color::linear_rgb(c.x, c.y, c.z);
            s.intensity = 2_400_000.0 * (1.0 + 0.6 * show.burst);
        } else {
            tf.rotation = base.0.rotation;
            let warm = Vec3::new(1.0, 0.985, 0.96).lerp(Vec3::new(1.0, 0.8, 0.6), n);
            s.color = Color::linear_rgb(warm.x, warm.y, warm.z);
            s.intensity = 1_600_000.0 * (1.0 + 0.3 * n);
        }
    }
    // each HUB glows its alliance's color while it's active
    for (h, mut l, mut v) in &mut hubs {
        let on = n > 0.0 && w.match_.hub_active(h.0);
        *v = if n > 0.0 { Visibility::Visible } else { Visibility::Hidden };
        let c = alliance_color(h.0);
        l.color = Color::linear_rgb(c.x, c.y, c.z);
        l.intensity = if on { 300_000.0 * n } else { 0.0 };
    }
    // the robots' LEDs light the carpet round them
    let have = pools.iter().count();
    for i in have..w.robots.len() {
        commands.spawn((PointLight { intensity: 0.0, range: 2.6, radius: 0.25, shadow_maps_enabled: false, ..default() }, Transform::default(), Visibility::Hidden, Pool(i)));
    }
    for (p, mut l, mut tf, mut v) in &mut pools {
        let Some(r) = w.robots.get(p.0) else { *v = Visibility::Hidden; continue };
        let c = glow.0.get(p.0).copied().unwrap_or(Vec3::ZERO);
        let lum = c.x * 0.2126 + c.y * 0.7152 + c.z * 0.0722;
        *v = if n > 0.0 && lum > 0.002 { Visibility::Visible } else { Visibility::Hidden };
        if let Some(b) = w.physics.bodies.get(r.body) { tf.translation = b.translation() + Vec3::Y * 0.3; }
        let cn = c / c.max_element().max(1e-4);
        l.color = Color::linear_rgb(cn.x, cn.y, cn.z);
        // (a soft pool, not a floodlight: the LEDs light the carpet round the robot)
        l.intensity = 30_000.0 * n * (lum / 0.2).min(1.0);
    }
}

/// the film grain over the picture (Settings film): a tile of noise, moved each frame
#[derive(Component)]
struct Grain;

fn spawn_grain(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    use bevy::image::{ImageSampler, ImageSamplerDescriptor, ImageAddressMode, ImageFilterMode};
    use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
    let n = 256usize;
    let mut px = vec![0u8; n * n * 4];
    let mut s = 0x1234_5678u32;
    for i in 0..n * n {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        // (light and dark specks about mid-gray, as grain lightens and darkens)
        let v = (s >> 24) as u8;
        px[i * 4..i * 4 + 4].copy_from_slice(&[v, v, v, 255]);
    }
    let mut img = Image::new(Extent3d { width: n as u32, height: n as u32, depth_or_array_layers: 1 }, TextureDimension::D2, px, TextureFormat::Rgba8UnormSrgb, RenderAssetUsages::RENDER_WORLD);
    img.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor { address_mode_u: ImageAddressMode::Repeat, address_mode_v: ImageAddressMode::Repeat, mag_filter: ImageFilterMode::Nearest, min_filter: ImageFilterMode::Nearest, ..default() });
    let image = images.add(img);
    commands
        .spawn((Node { position_type: PositionType::Absolute, width: Val::Percent(100.0), height: Val::Percent(100.0), overflow: Overflow::clip(), ..default() }, GlobalZIndex(-100), Pickable::IGNORE, Visibility::Hidden, Grain))
        .with_children(|p| {
            p.spawn((
                Node { position_type: PositionType::Absolute, width: Val::Percent(140.0), height: Val::Percent(140.0), ..default() },
                ImageNode { image, color: Color::srgba(1.0, 1.0, 1.0, 0.012), image_mode: bevy::ui::widget::NodeImageMode::Tiled { tile_x: true, tile_y: true, stretch_value: 1.0 }, ..default() },
                Pickable::IGNORE,
            ));
        });
}

fn grain(settings: Res<Settings>, mut show: ResMut<Show>, mut outer: Query<(&mut Visibility, &Children), With<Grain>>, mut inner: Query<&mut Node, Without<Grain>>) {
    let on = settings.film && settings.graphics != "low";
    for (mut v, kids) in &mut outer {
        let want = if on { Visibility::Inherited } else { Visibility::Hidden };
        if *v != want { *v = want; }
        if !on { continue }
        let (dx, dy) = (show.rand() * 256.0, show.rand() * 256.0);
        for k in kids.iter() { if let Ok(mut nd) = inner.get_mut(k) { nd.left = Val::Px(-dx); nd.top = Val::Px(-dy); } }
    }
}

/// The crowd's camera flashes, at night: one here and there, for an instant
fn flashes(time: Res<Time>, settings: Res<Settings>, mut show: ResMut<Show>, mut q: Query<(&mut Flash, &mut Transform, &mut Visibility)>) {
    let look = Look::of(&settings);
    // (indoors only: outdoors there's no crowd)
    let on = settings.graphics != "low" && look.time == Tod::Night && !look.outdoor;
    let dt = time.delta_secs();
    let mut fire = on && show.rand() < dt * 7.0;
    for (mut f, mut tf, mut v) in &mut q {
        if f.0 > 0.0 { f.0 -= dt; if f.0 <= 0.0 { *v = Visibility::Hidden; } continue; }
        if !fire { continue }
        fire = false;
        // (somewhere in the bleachers: the back stands' rows, or the near ones either side of the
        // scoring table: js venue.js stands)
        let hw = HALF_W as f32;
        let p = if show.rand() < 0.65 {
            let row = (show.rand() * 9.0).floor();
            Vec3::new(show.rand() * 24.0 - 12.0, row * 0.4 + 1.25, -(hw + 2.5) - (row + 0.76) * 0.8)
        } else {
            let row = (show.rand() * 5.0).floor();
            let x = 4.0 + show.rand() * 8.5;
            Vec3::new(if show.rand() < 0.5 { x } else { -x }, row * 0.4 + 1.25, hw + 3.4 + (row + 0.76) * 0.8)
        };
        tf.translation = p;
        f.0 = 0.05;
        *v = Visibility::Visible;
    }
}

/// The rain: streaks falling round the camera, remade each frame
fn rain(time: Res<Time>, settings: Res<Settings>, show: ResMut<Show>, cams: Query<&GlobalTransform, (With<Camera3d>, With<crate::camera::MainCam>)>, mut q: Query<&mut Visibility, With<Rain>>, mut meshes: ResMut<Assets<Mesh>>) {
    let look = Look::of(&settings);
    let on = settings.graphics != "low" && look.outdoor && look.weather == Weather::Rain;
    for mut v in &mut q { let want = if on { Visibility::Visible } else { Visibility::Hidden }; if *v != want { *v = want; } }
    if !on { return }
    let Ok(cam) = cams.single() else { return };
    let c = cam.translation();
    let right = cam.right().as_vec3();
    let t = time.elapsed_secs();
    let (len, w) = (0.42f32, 0.008f32);
    let n = show.drops.len();
    let mut pos = Vec::with_capacity(n * 4);
    let mut idx = Vec::with_capacity(n * 6);
    // (each drop in its own column round the camera, falling and coming round again)
    let side = Vec3::new(right.x, 0.0, right.z).normalize_or(Vec3::X) * w;
    for (k, d) in show.drops.iter().enumerate() {
        let x = c.x + (d[0] - c.x).rem_euclid(28.0) - 14.0;
        let z = c.z + (d[1] - c.z).rem_euclid(28.0) - 14.0;
        let y = (d[2] - t * d[3]).rem_euclid(9.0) + (c.y - 4.0).max(-0.5);
        let p = Vec3::new(x, y, z);
        let top = p + Vec3::new(0.06, len, 0.0);
        let b = (k * 4) as u32;
        pos.extend([(p - side).to_array(), (p + side).to_array(), (top + side).to_array(), (top - side).to_array()]);
        idx.extend([b, b + 1, b + 2, b, b + 2, b + 3]);
    }
    let Some(h) = &show.rain_mesh else { return };
    let Some(mut m) = meshes.get_mut(h) else { return };
    m.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
    m.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0f32, 1.0, 0.0]; n * 4]);
    m.insert_indices(Indices::U32(idx));
}

/// Wet in the rain: the carpet, the turf and the robots' and field's painted, plastic and metal
/// parts glossier and darker; dry again after
#[allow(clippy::too_many_arguments)]
fn wet(
    settings: Res<Settings>,
    floors: Option<Res<crate::field::Floors>>,
    mut show: ResMut<Show>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut worn: ResMut<Assets<crate::materials::WornMaterial>>,
    mut brushed: ResMut<Assets<crate::materials::BrushedMaterial>>,
) {
    let look = Look::of(&settings);
    let want = look.outdoor && look.weather == Weather::Rain && settings.graphics != "low";
    // (again when more materials come in: a robot's model loading)
    let count = worn.len() + brushed.len();
    if show.wet == Some((want, count)) || (show.wet.is_some_and(|w| !w.0) && !want) { return }
    // (the dry finish first, every material as it was)
    for (id, r, refl, c) in show.dry.drain(..) { if let Some(mut m) = mats.get_mut(id) { m.perceptual_roughness = r; m.reflectance = refl; m.base_color = c.into(); } }
    for (id, r, c) in show.dry_worn.drain(..) { if let Some(mut m) = worn.get_mut(id) { m.base.perceptual_roughness = r; m.base.base_color = c.into(); } }
    for (id, r) in show.dry_brushed.drain(..) { if let Some(mut m) = brushed.get_mut(id) { m.base.perceptual_roughness = r; } }
    show.wet = Some((want, count));
    if !want { return }
    if let Some(f) = &floors {
        for h in [&f.carpet, &f.turf] {
            if let Some(mut m) = mats.get_mut(h) {
                show.dry.push((h.id(), m.perceptual_roughness, m.reflectance, m.base_color.to_linear()));
                m.perceptual_roughness *= 0.5;
                m.reflectance = 0.5;
                let c = m.base_color.to_linear();
                m.base_color = Color::linear_rgba(c.red * 0.7, c.green * 0.7, c.blue * 0.7, c.alpha);
            }
        }
    }
    let ids: Vec<_> = worn.ids().collect();
    for id in ids {
        let Some(mut m) = worn.get_mut(id) else { continue };
        let c = m.base.base_color.to_linear();
        show.dry_worn.push((id, m.base.perceptual_roughness, c));
        m.base.perceptual_roughness = (m.base.perceptual_roughness * 0.5).max(0.12);
        m.base.base_color = Color::linear_rgba(c.red * 0.85, c.green * 0.85, c.blue * 0.85, c.alpha);
    }
    let ids: Vec<_> = brushed.ids().collect();
    for id in ids {
        let Some(mut m) = brushed.get_mut(id) else { continue };
        show.dry_brushed.push((id, m.base.perceptual_roughness));
        m.base.perceptual_roughness = (m.base.perceptual_roughness * 0.65).max(0.1);
    }
}
