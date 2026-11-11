//! The arena's light: what the field sits in, for everything to be lit by and to reflect.
//!   - an environment drawn in code (no images): a gym round the field, its high-bay lights in the
//!     ceiling, the trusses' lights, the stands' crowd as a patchwork of shirts, the painted band,
//!     the banners and the big screen. The camera filters it for image-based light (diffuse fill
//!     from every side, reflections in the aluminum and polycarbonate) and shows it as the sky
//!     past the venue's walls;
//!   - the trusses' lights themselves, aimed down and in at the field (venue.js's cans): spot lights,
//!     no shadows (the sun keeps the one shadowed light);
//!   - the grade: a little warmth and contrast, and a soft vignette.
//! How much of it is on follows the graphics preset (camera.rs `graphics`).

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureViewDescriptor, TextureViewDimension};
use sim::constants::{HALF_L, HALF_W};

pub struct ArenaPlugin;

impl Plugin for ArenaPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (make_env, spawn_truss_lights));
    }
}

/// The arena's environment cubemap
#[derive(Resource)]
pub struct ArenaEnv(pub Handle<Image>);

/// One of the trusses' lights
#[derive(Component)]
pub struct TrussLight;
/// The haze in the gym's air (Ultra: the trusses' light shows in it)
#[derive(Component)]
pub struct ArenaHaze;

/// How bright the environment is (cd/m^2 per unit of its radiance)
pub const ENV_INTENSITY: f32 = 550.0;
/// The trusses: height, how far out from the field's sides (js venue.js TRUSS_Y, TRUSS_OUT)
const TRUSS_Y: f32 = 7.2;
const TRUSS_OUT: f32 = 1.2;
/// The gym (as venue.js's full one): its walls' x, the back and near walls' z, its ceiling
const GYM_X: f32 = 21.0;
const GYM_ZB: f32 = -14.1;
const GYM_ZN: f32 = 12.6;
const GYM_H: f32 = 11.0;
/// where the environment is seen from (about a camera's height, the field's middle)
const EYE: Vec3 = Vec3::new(0.0, 1.6, 0.0);

/// the trusses' lights: (where, which way it points)
fn truss_lights() -> Vec<(Vec3, Vec3)> {
    let (hl, hw) = (HALF_L as f32, HALF_W as f32);
    let zt = hw + TRUSS_OUT;
    let mut v = Vec::new();
    for s in [-1.0f32, 1.0] {
        let mut x = -hl + 0.6;
        while x <= hl - 0.5 {
            // (aimed down and in: 0.55 rad off straight down, toward the field)
            v.push((Vec3::new(x, TRUSS_Y - 0.42, s * zt), Vec3::new(0.0, -(0.55f32).cos(), -s * (0.55f32).sin())));
            x += 2.2;
        }
    }
    v
}

fn spawn_truss_lights(mut commands: Commands) {
    // (a light haze over the field and stands, up to the trusses and a little over)
    commands.spawn((
        bevy::light::FogVolume { density_factor: 0.06, scattering: 0.6, absorption: 0.1, fog_color: Color::srgb(0.9, 0.9, 0.92), ..default() },
        Transform::from_xyz(0.0, 4.5, 0.0).with_scale(Vec3::new(36.0, 9.0, 24.0)),
        Visibility::Hidden,
        ArenaHaze,
    ));
    for (p, d) in truss_lights() {
        commands.spawn((
            SpotLight {
                color: Color::srgb(1.0, 0.985, 0.96),
                intensity: 800_000.0,
                range: 22.0,
                radius: 0.13,
                shadow_maps_enabled: false,
                inner_angle: 0.3,
                outer_angle: 0.62,
                ..default()
            },
            Transform::from_translation(p).looking_to(d, Vec3::Y),
            Visibility::Hidden,
            TrussLight,
        ));
    }
}

// ------------------------------------------------------------------ the environment
/// a hash of a cell (0..1)
fn hash(a: i32, b: i32, c: i32) -> f32 {
    let mut h = (a as u32).wrapping_mul(0x8da6_b343) ^ (b as u32).wrapping_mul(0xd816_3841) ^ (c as u32).wrapping_mul(0xcb1a_b31f);
    h ^= h >> 13;
    h = h.wrapping_mul(0x5bd1_e995);
    h ^= h >> 15;
    (h & 0xff_ffff) as f32 / 16_777_216.0
}
/// js venue.js SHIRTS, linear
fn shirt(i: usize) -> Vec3 {
    const S: [u32; 18] = [0x1d4ed8, 0xdc2626, 0x16a34a, 0xf59e0b, 0x7c3aed, 0x0891b2, 0xea580c, 0xdb2777, 0x111827, 0xf3f4f6, 0x65a30d, 0x0f766e, 0xfacc15, 0x9f1239, 0x1e3a8a, 0x6b7280, 0x4338ca, 0xb91c1c];
    let c = Color::srgb_u8((S[i % 18] >> 16) as u8, (S[i % 18] >> 8) as u8, S[i % 18] as u8).to_linear();
    Vec3::new(c.red, c.green, c.blue)
}

/// What the arena's surfaces send back toward EYE along dir (linear, relative units: a lit wall
/// ~0.05, the lights tens)
fn radiance(dir: Vec3, lights: &[(Vec3, Vec3)]) -> Vec3 {
    let d = dir.normalize();
    // the trusses' lights, seen as small bright discs (brighter seen from in front of the can)
    for &(p, aim) in lights {
        let to = (p - EYE).normalize();
        if d.dot(to) > 0.99993 {
            let face = aim.dot(-to).max(0.0);
            return Vec3::new(1.0, 0.95, 0.85) * (20.0 + 40.0 * face);
        }
    }
    // where the ray meets the gym's box
    let mut t = f32::INFINITY;
    let mut surf = 0;
    if d.y > 1e-4 { let tt = (GYM_H - EYE.y) / d.y; if tt < t { t = tt; surf = 1; } }
    if d.y < -1e-4 { let tt = -EYE.y / d.y; if tt < t { t = tt; surf = 2; } }
    if d.x.abs() > 1e-4 { let tt = ((GYM_X * d.x.signum()) - EYE.x) / d.x; if tt < t { t = tt; surf = 3; } }
    if d.z > 1e-4 { let tt = (GYM_ZN - EYE.z) / d.z; if tt < t { t = tt; surf = 4; } }
    if d.z < -1e-4 { let tt = (GYM_ZB - EYE.z) / d.z; if tt < t { t = tt; surf = 5; } }
    let p = EYE + d * t;
    let (hl, hw) = (HALF_L as f32, HALF_W as f32);
    match surf {
        // the ceiling: a dark steel deck and its rows of high-bay lights
        1 => {
            let (gx, gz) = ((p.x / 4.5).round() * 4.5, (p.z / 4.0).round() * 4.0);
            let r = Vec2::new(p.x - gx, p.z - gz).length();
            if r < 0.32 { Vec3::new(1.0, 0.97, 0.9) * 45.0 }
            else if r < 0.42 { Vec3::splat(0.05) }
            // (the roof's joists, lit from below)
            else if (p.x / 1.5).fract().abs() < 0.04 { Vec3::splat(0.03) }
            else { Vec3::new(0.012, 0.013, 0.016) }
        }
        // the floor: the lit carpet, the gym floor round it
        2 => {
            if p.x.abs() < hl && p.z.abs() < hw { Vec3::new(0.13, 0.135, 0.14) }
            else { Vec3::new(0.05, 0.035, 0.022) } // (a wood gym floor, in the trusses' spill)
        }
        // the walls
        _ => {
            let along = if surf == 3 { p.z } else { p.x };
            let y = p.y;
            let back = surf == 5;
            let near = surf == 4;
            // the screen high on the back wall: the live score's glow
            if back && along.abs() < 3.2 && (5.2..8.8).contains(&y) {
                let u = (along + 3.2) / 6.4;
                return if u < 0.3 { Vec3::new(0.05, 0.12, 0.6) } else if u > 0.7 { Vec3::new(0.55, 0.06, 0.05) } else { Vec3::splat(0.25) } * 3.0;
            }
            // the stands and their crowd: a patchwork of shirts, the rows' shadows between
            let stands = (back && along.abs() < 12.5 && y < 4.0) || (near && (4.0..12.5).contains(&along.abs()) && y < 2.8);
            if stands {
                let (cx, cy) = ((along / 0.5).floor() as i32, (y / 0.42).floor() as i32);
                let h = hash(cx, cy, surf);
                let fy = (y / 0.42).fract();
                if fy < 0.18 { return Vec3::splat(0.012); }
                if h > 0.72 { return Vec3::new(0.02, 0.022, 0.026); } // (an empty seat)
                let skin = fy > 0.78;
                let c = if skin { Vec3::new(0.32, 0.18, 0.11) * (0.4 + 0.6 * hash(cx, cy, 7)) } else { shirt((hash(cx, cy, 3) * 18.0) as usize) };
                return c * 0.09;
            }
            // the painted band round the gym
            if (3.35..3.85).contains(&y) { return Vec3::new(0.11, 0.017, 0.03) * 0.5; }
            // banners over the stands
            if (4.9..7.5).contains(&y) && (back || near || surf == 3) {
                let k = ((along + 0.65) / 2.6).floor();
                let off = along + 0.65 - k * 2.6;
                if off < 1.3 && along.abs() < 15.0 {
                    let i = k as i32;
                    let base = shirt((hash(i, surf, 11) * 18.0) as usize);
                    let stripe = ((y - 4.9) / 2.6 * 10.0) as i32 % 5 == 1;
                    return if stripe { shirt((hash(i, surf, 13) * 18.0) as usize) * 0.1 } else { base * 0.1 };
                }
            }
            // the bare wall, lit more toward the floor
            let k = 0.035 - 0.0015 * y;
            Vec3::new(k, k * 1.02, k * 1.08)
        }
    }
}

/// The environment, drawn into a cubemap (RGBA16F, each face size x size: wgpu's face order and
/// orientation)
fn make_env(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let n = 256usize;
    let lights = truss_lights();
    let mut data: Vec<u8> = Vec::with_capacity(n * n * 6 * 8);
    for face in 0..6 {
        for y in 0..n {
            for x in 0..n {
                let u = 2.0 * (x as f32 + 0.5) / n as f32 - 1.0;
                let v = 2.0 * (y as f32 + 0.5) / n as f32 - 1.0;
                let dir = match face {
                    0 => Vec3::new(1.0, -v, -u),
                    1 => Vec3::new(-1.0, -v, u),
                    2 => Vec3::new(u, 1.0, v),
                    3 => Vec3::new(u, -1.0, -v),
                    4 => Vec3::new(u, -v, 1.0),
                    _ => Vec3::new(-u, -v, -1.0),
                };
                let c = radiance(dir, &lights);
                for k in [c.x, c.y, c.z, 1.0] { data.extend_from_slice(&half::f16::from_f32(k).to_le_bytes()); }
            }
        }
    }
    let mut img = Image::new(
        Extent3d { width: n as u32, height: n as u32, depth_or_array_layers: 6 },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba16Float,
        RenderAssetUsages::RENDER_WORLD,
    );
    img.texture_view_descriptor = Some(TextureViewDescriptor { dimension: Some(TextureViewDimension::Cube), ..default() });
    commands.insert_resource(ArenaEnv(images.add(img)));
}
