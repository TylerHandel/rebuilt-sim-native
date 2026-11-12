//! The arena's light: what the field sits in, for everything to be lit by and to reflect.
//!   - an environment drawn in code (no images): a gym round the field, its high-bay lights in the
//!     ceiling, the trusses' lights, the stands' crowd as a patchwork of shirts, the painted band,
//!     the banners and the big screen. The camera filters it for image-based light (diffuse fill
//!     from every side, reflections in the aluminum and polycarbonate) and shows it as the sky
//!     past the venue's walls;
//!   - the trusses' lights themselves, aimed down and in at the field (every other one of venue.js's
//!     cans: 8 spot lights, each twice as bright; no shadows, the sun keeps the one shadowed light);
//!   - the grade: a little warmth and contrast, and a soft vignette.
//! How much of it is on follows the graphics preset (camera.rs `graphics`).

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureViewDescriptor, TextureViewDimension};
use sim::constants::{HALF_L, HALF_W};

pub struct ArenaPlugin;

impl Plugin for ArenaPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<EnvTask>().add_systems(Startup, spawn_truss_lights).add_systems(Update, make_env);
    }
}

/// The arena's environment cubemap
#[derive(Resource)]
pub struct ArenaEnv { pub look: Look, pub sky: Handle<Image>, pub diffuse: Handle<Image>, pub specular: Handle<Image> }

/// One of the trusses' lights
#[derive(Component)]
pub struct TrussLight;


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

/// The stadium (outdoors): its bowl of stands round the field (radius, top), and its light towers
/// (where their banks of lights are)
pub const STAND_R: f32 = 36.0;
const STAND_TOP: f32 = 15.0;
pub const TOWERS: [Vec3; 4] = [Vec3::new(31.0, 30.0, 22.0), Vec3::new(-31.0, 30.0, 22.0), Vec3::new(31.0, 30.0, -22.0), Vec3::new(-31.0, 30.0, -22.0)];

/// The time of day
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Tod { Day, Sunset, Night }
/// The weather (outdoors)
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Weather { Clear, Cloudy, Rain, Fog }
/// What the match is played in (Settings venue, time, weather): what the environment and the
/// lights are made of
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct Look { pub outdoor: bool, pub time: Tod, pub weather: Weather }
impl Look {
    pub fn of(s: &crate::settings::Settings) -> Look {
        let outdoor = s.venue == "stadium";
        let time = match s.time.as_str() { "sunset" => Tod::Sunset, "night" => Tod::Night, _ => Tod::Day };
        let weather = if !outdoor { Weather::Clear } else { match s.weather.as_str() { "cloudy" => Weather::Cloudy, "rain" => Weather::Rain, "fog" => Weather::Fog, _ => Weather::Clear } };
        Look { outdoor, time, weather }
    }
    /// toward the sun (at night outdoors, the moon; the key light's direction either way)
    pub fn sun_dir(&self) -> Vec3 {
        match self.time {
            Tod::Day => Vec3::new(0.29, 0.86, 0.43).normalize(),
            // (low in the west, just over the far stands: long raking shadows toward the scoring table)
            Tod::Sunset => Vec3::new(-0.55, 0.3, -0.83).normalize(),
            Tod::Night => Vec3::new(0.35, 0.62, -0.7).normalize(),
        }
    }
    /// how much of the sky the clouds cover (0..1)
    pub fn cloud(&self) -> f32 { match self.weather { Weather::Clear => 0.0, Weather::Cloudy => 0.55, Weather::Rain => 1.0, Weather::Fog => 0.9 } }
    /// the fog's color, and how thick it is (1/m)
    pub fn haze(&self) -> (Vec3, f32) {
        let c = match self.time { Tod::Day => Vec3::new(0.30, 0.32, 0.35), Tod::Sunset => Vec3::new(0.30, 0.20, 0.16), Tod::Night => Vec3::new(0.012, 0.013, 0.017) };
        let k = if !self.outdoor { 0.0 } else { match self.weather { Weather::Fog => 0.055, Weather::Rain => 0.014, Weather::Cloudy => 0.003, Weather::Clear => 0.0015 } };
        (c, k)
    }
}

/// the trusses' lights: (where, which way it points)
pub fn truss_lights() -> Vec<(Vec3, Vec3)> {
    let (hl, hw) = (HALF_L as f32, HALF_W as f32);
    let zt = hw + TRUSS_OUT;
    let mut v = Vec::new();
    for s in [-1.0f32, 1.0] {
        let mut x = -hl + 0.6;
        while x <= hl - 0.5 {
            // (aimed down and in: 0.55 rad off straight down, toward the field)
            v.push((Vec3::new(x, TRUSS_Y - 0.42, s * zt), Vec3::new(0.0, -(0.55f32).cos(), -s * (0.55f32).sin())));
            x += 4.4;
        }
    }
    v
}

fn spawn_truss_lights(mut commands: Commands) {
    for (p, d) in truss_lights() {
        commands.spawn((
            SpotLight {
                color: Color::srgb(1.0, 0.985, 0.96),
                intensity: 1_600_000.0,
                range: 14.0,
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

/// What the arena's (or the stadium's) surfaces send back toward EYE along dir (linear, relative
/// units: a lit wall ~0.05, the lights tens)
fn radiance(dir: Vec3, lights: &[(Vec3, Vec3)], look: &Look) -> Vec3 {
    if look.outdoor { return outdoor(dir.normalize(), look); }
    // (indoors: in the evening the house lights half down, at night all the way, the stage's up)
    let n = match look.time { Tod::Day => 0.0, Tod::Sunset => 0.5, Tod::Night => 1.0 };
    let d = dir.normalize();
    let (c, kind) = indoor(d, lights);
    match kind {
        Lit::Stage => c * (1.0 + 0.6 * n),
        Lit::Screen => c,
        Lit::House => c * (1.0 - 0.97 * n),
        Lit::Field => c * (1.0 - 0.55 * n),
        Lit::Room(along, y) => {
            // (at night, the stage's colored wash up the walls and over the stands)
            let wash = if n > 0.0 {
                let hue = (along * 0.11).sin() * 0.5 + 0.5;
                let col = Vec3::new(0.35 + 0.4 * hue, 0.08, 0.9 - 0.5 * hue);
                col * 0.05 * (-((y - 4.5) / 3.0).powi(2)).exp()
            } else { Vec3::ZERO };
            c * (1.0 - 0.9 * n) + wash * n
        }
    }
}

/// what kind of light a direction sees indoors: the stage's lights, the screen, the house's lights,
/// the field under them, or the room (lit by the house: along a wall, at a height)
enum Lit { Stage, Screen, House, Field, Room(f32, f32) }

fn indoor(d: Vec3, lights: &[(Vec3, Vec3)]) -> (Vec3, Lit) {
    // the trusses' lights, seen as small bright discs (brighter seen from in front of the can)
    for &(p, aim) in lights {
        let to = (p - EYE).normalize();
        if d.dot(to) > 0.99993 {
            let face = aim.dot(-to).max(0.0);
            return (Vec3::new(1.0, 0.95, 0.85) * (20.0 + 40.0 * face), Lit::Stage);
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
            let c = if r < 0.32 { Vec3::new(1.0, 0.97, 0.9) * 45.0 }
            else if r < 0.42 { Vec3::splat(0.05) }
            // (the roof's joists, lit from below)
            else if (p.x / 1.5).fract().abs() < 0.04 { Vec3::splat(0.03) }
            else { Vec3::new(0.012, 0.013, 0.016) };
            (c, Lit::House)
        }
        // the floor: the lit carpet, the gym floor round it
        2 => {
            if p.x.abs() < hl && p.z.abs() < hw { (Vec3::new(0.13, 0.135, 0.14), Lit::Field) }
            else { (Vec3::new(0.05, 0.035, 0.022), Lit::Room(p.x, 0.0)) } // (a wood gym floor, in the trusses' spill)
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
                return (if u < 0.3 { Vec3::new(0.05, 0.12, 0.6) } else if u > 0.7 { Vec3::new(0.55, 0.06, 0.05) } else { Vec3::splat(0.25) } * 3.0, Lit::Screen);
            }
            // the stands and their crowd: a patchwork of shirts, the rows' shadows between
            let stands = (back && along.abs() < 12.5 && y < 4.0) || (near && (4.0..12.5).contains(&along.abs()) && y < 2.8);
            if stands {
                let (cx, cy) = ((along / 0.5).floor() as i32, (y / 0.42).floor() as i32);
                let h = hash(cx, cy, surf);
                let fy = (y / 0.42).fract();
                if fy < 0.18 { return (Vec3::splat(0.012), Lit::Room(along, y)); }
                if h > 0.72 { return (Vec3::new(0.02, 0.022, 0.026), Lit::Room(along, y)); } // (an empty seat)
                let skin = fy > 0.78;
                let c = if skin { Vec3::new(0.32, 0.18, 0.11) * (0.4 + 0.6 * hash(cx, cy, 7)) } else { shirt((hash(cx, cy, 3) * 18.0) as usize) };
                return (c * 0.09, Lit::Room(along, y));
            }
            // the painted band round the gym
            if (3.35..3.85).contains(&y) { return (Vec3::new(0.11, 0.017, 0.03) * 0.5, Lit::Room(along, y)); }
            // banners over the stands
            if (4.9..7.5).contains(&y) && (back || near || surf == 3) {
                let k = ((along + 0.65) / 2.6).floor();
                let off = along + 0.65 - k * 2.6;
                if off < 1.3 && along.abs() < 15.0 {
                    let i = k as i32;
                    let base = shirt((hash(i, surf, 11) * 18.0) as usize);
                    let stripe = ((y - 4.9) / 2.6 * 10.0) as i32 % 5 == 1;
                    return (if stripe { shirt((hash(i, surf, 13) * 18.0) as usize) * 0.1 } else { base * 0.1 }, Lit::Room(along, y));
                }
            }
            // the bare wall, lit more toward the floor
            let k = 0.035 - 0.0015 * y;
            (Vec3::new(k, k * 1.02, k * 1.08), Lit::Room(along, y))
        }
    }
}

/// 2D value noise (0..1), smooth
fn noise(x: f32, y: f32) -> f32 {
    let (ix, iy) = (x.floor() as i32, y.floor() as i32);
    let (fx, fy) = (x - x.floor(), y - y.floor());
    let (ux, uy) = (fx * fx * (3.0 - 2.0 * fx), fy * fy * (3.0 - 2.0 * fy));
    let a = hash(ix, iy, 91) * (1.0 - ux) + hash(ix + 1, iy, 91) * ux;
    let b = hash(ix, iy + 1, 91) * (1.0 - ux) + hash(ix + 1, iy + 1, 91) * ux;
    a * (1.0 - uy) + b * uy
}
fn fbm(x: f32, y: f32) -> f32 { 0.55 * noise(x, y) + 0.3 * noise(2.1 * x + 5.0, 2.1 * y) + 0.15 * noise(4.3 * x, 4.3 * y + 9.0) }

/// The sky outdoors (above the stands): blue by day, warm low toward the sun at sunset, dark and
/// starry at night; the sun (or moon) in it; the clouds over it
fn sky(d: Vec3, look: &Look) -> Vec3 {
    let sun = look.sun_dir();
    let h = d.y.max(0.0);
    let cs = d.dot(sun);
    let mut c = match look.time {
        Tod::Day => Vec3::new(0.07, 0.16, 0.42).lerp(Vec3::new(0.40, 0.48, 0.58), (1.0 - h).powi(4)) * 0.75,
        Tod::Sunset => {
            let toward = (cs * 0.5 + 0.5).powi(3);
            let horizon = Vec3::new(0.28, 0.14, 0.22).lerp(Vec3::new(1.0, 0.45, 0.14), toward);
            Vec3::new(0.05, 0.07, 0.18).lerp(horizon, (1.0 - h).powi(5)) * 0.55
        }
        Tod::Night => {
            let base = Vec3::new(0.0012, 0.0018, 0.005).lerp(Vec3::new(0.004, 0.005, 0.009), (1.0 - h).powi(3));
            // (the stars: a few cells of the sky, bright points)
            let (a, b) = (((d.x / (d.y + 0.3)) * 160.0).floor() as i32, ((d.z / (d.y + 0.3)) * 160.0).floor() as i32);
            let st = hash(a, b, 5);
            base + if st > 0.9975 && h > 0.15 { Vec3::splat(0.08 * (st - 0.9975) / 0.0025) } else { Vec3::ZERO }
        }
    };
    // the sun's (or moon's) disc and its glow
    let disc = match look.time { Tod::Day => Vec3::new(1.0, 0.96, 0.9) * 3000.0, Tod::Sunset => Vec3::new(1.0, 0.55, 0.25) * 500.0, Tod::Night => Vec3::new(0.75, 0.8, 0.9) * 2.5 };
    let glow = match look.time { Tod::Day => Vec3::new(1.0, 0.9, 0.75) * 0.6, Tod::Sunset => Vec3::new(1.0, 0.5, 0.2) * 1.2, Tod::Night => Vec3::splat(0.004) };
    let cover = look.cloud();
    let mut sunlit = if cs > 0.99996 { disc } else { Vec3::ZERO } + glow * cs.max(0.0).powf(60.0);
    // the clouds: drifting cells over the sky, lit by the sun (warm underneath at sunset)
    if cover > 0.0 {
        let (u, v) = (d.x / (d.y + 0.08) * 0.9, d.z / (d.y + 0.08) * 0.9);
        let n = fbm(u, v);
        let k = if cover >= 1.0 { 1.0 } else { ((n - (1.0 - cover)) / 0.25).clamp(0.0, 1.0) };
        let cloud = match look.time {
            Tod::Day => Vec3::splat(0.55) * (0.75 + 0.35 * n) * if look.weather == Weather::Rain { 0.45 } else { 1.0 },
            Tod::Sunset => Vec3::new(0.6, 0.32, 0.24) * 0.45 * (0.7 + 0.5 * n),
            Tod::Night => Vec3::splat(0.003),
        };
        c = c.lerp(cloud, k * (h * 6.0).min(1.0).max(if cover >= 1.0 { 1.0 } else { 0.0 }));
        sunlit *= 1.0 - k;
    }
    c + sunlit
}

/// What the stadium sends back toward EYE along d: its field and turf, its bowl of stands and the
/// crowd in them, its light towers, the sky; through the haze
fn outdoor(d: Vec3, look: &Look) -> Vec3 {
    let (hl, hw) = (HALF_L as f32, HALF_W as f32);
    // how the stadium is lit: by the sun (and sky), or at night by its towers
    let cover = look.cloud();
    let light = match look.time { Tod::Day => 1.0 - 0.55 * cover, Tod::Sunset => 0.45 - 0.2 * cover, Tod::Night => 0.22 };
    let tint = match look.time { Tod::Day => Vec3::ONE, Tod::Sunset => Vec3::new(1.0, 0.7, 0.5), Tod::Night => Vec3::new(0.9, 0.95, 1.0) };
    let (fog_c, fog_k) = look.haze();
    let through = |c: Vec3, dist: f32| { let f = (-fog_k * dist).exp(); c * f + fog_c * (1.0 - f) };
    // the towers' banks of lights (lit at night, a pale panel by day)
    for t in TOWERS {
        let to = t - EYE;
        let dist = to.length();
        if d.dot(to / dist) > 0.99985 {
            let c = if look.time == Tod::Night { Vec3::new(1.0, 0.97, 0.9) * 60.0 } else { Vec3::splat(0.5) };
            return through(c, dist * 0.5);
        }
    }
    // the stands: the bowl round the field, rising from its edge to its top
    let horiz = Vec2::new(d.x, d.z).length().max(1e-4);
    let t_wall = STAND_R / horiz;
    let y_wall = EYE.y + d.y * t_wall;
    if d.y < 0.0 && -EYE.y / d.y < t_wall {
        // the ground inside the bowl: the carpet, the turf round it (mowed in stripes)
        let t = -EYE.y / d.y;
        let p = EYE + d * t;
        let c = if p.x.abs() < hl && p.z.abs() < hw { Vec3::new(0.13, 0.135, 0.14) }
            else if ((p.x / 4.5).floor() as i32) % 2 == 0 { Vec3::new(0.035, 0.07, 0.025) } else { Vec3::new(0.03, 0.06, 0.02) };
        let wet = if look.weather == Weather::Rain { 0.7 } else { 1.0 };
        return through(c * light * wet * tint, t);
    }
    if y_wall < STAND_TOP {
        // the crowd in rows, the concourse under them, the roof's edge over them
        let along = d.z.atan2(d.x) * STAND_R;
        let c = if y_wall < 1.2 { Vec3::splat(0.03) }
            else if y_wall > STAND_TOP - 1.2 { Vec3::splat(0.015) }
            else {
                let (cx, cy) = ((along / 0.55).floor() as i32, ((y_wall - 1.2) / 0.62).floor() as i32);
                let fy = ((y_wall - 1.2) / 0.62).fract();
                if fy < 0.2 { Vec3::splat(0.012) }
                else if hash(cx, cy, 21) > 0.75 { Vec3::new(0.025, 0.03, 0.04) }
                else { shirt((hash(cx, cy, 23) * 18.0) as usize) * 0.12 }
            };
        let lit = if look.time == Tod::Night { 0.6 } else { light };
        return through(c * lit * tint, t_wall);
    }
    // the sky over it all (the fog swallows it)
    through(sky(d, look), 400.0)
}

/// The environment, drawn into a cubemap (RGBA16F, each face size x size: wgpu's face order and
/// orientation)
/// a cubemap texel's direction (wgpu's face order and orientation)
fn face_dir(face: usize, u: f32, v: f32) -> Vec3 {
    match face {
        0 => Vec3::new(1.0, -v, -u),
        1 => Vec3::new(-1.0, -v, u),
        2 => Vec3::new(u, 1.0, v),
        3 => Vec3::new(u, -1.0, -v),
        4 => Vec3::new(u, -v, 1.0),
        _ => Vec3::new(-u, -v, -1.0),
    }
}
/// every texel of an n x n cube: (face, direction, solid angle)
fn texels(n: usize) -> Vec<(usize, Vec3, f32)> {
    let mut v = Vec::with_capacity(n * n * 6);
    for face in 0..6 { for y in 0..n { for x in 0..n {
        let (u, w) = (2.0 * (x as f32 + 0.5) / n as f32 - 1.0, 2.0 * (y as f32 + 0.5) / n as f32 - 1.0);
        let d = face_dir(face, u, w);
        let l2 = d.length_squared();
        // (a texel's solid angle: its area, 4/n^2, over the distance cubed)
        v.push((face, d / l2.sqrt(), 4.0 / (n * n) as f32 / (l2 * l2.sqrt())));
    } } }
    v
}
fn cube(n: u32, mips: u32, data: Vec<u8>) -> Image {
    let mut img = Image::new(Extent3d { width: n, height: n, depth_or_array_layers: 6 }, TextureDimension::D2, data, TextureFormat::Rgba16Float, RenderAssetUsages::RENDER_WORLD);
    img.texture_descriptor.mip_level_count = mips;
    img.texture_view_descriptor = Some(TextureViewDescriptor { dimension: Some(TextureViewDimension::Cube), ..default() });
    img
}
fn push(data: &mut Vec<u8>, c: Vec3) { for k in [c.x, c.y, c.z, 1.0] { data.extend_from_slice(&half::f16::from_f32(k).to_le_bytes()); } }

/// The environment, drawn into cubemaps once, at the start: the sky (128 a face), and its light
/// filtered here on the CPU, not by the GPU every frame (as a GeneratedEnvironmentMapLight is):
/// the diffuse light from every side (16 a face) and the specular, its mips blurrier for rougher
/// surfaces (64 a face, 7 mips, mip k for perceptual roughness k/6: a Phong lobe of GGX's width)
fn build_env(look: Look) -> EnvBytes {
    let lights = truss_lights();
    let rad = |d: Vec3| radiance(d, &lights, &look);
    // the sky
    let mut sky = Vec::new();
    for (_, d, _) in texels(128) { push(&mut sky, rad(d)); }
    // the source the filters gather from (32 and 16 a face): direction, radiance, solid angle
    let src = |n: usize| -> Vec<(Vec3, Vec3, f32)> { texels(n).into_iter().map(|(_, d, sa)| (d, rad(d), sa)).collect() };
    let (src32, src16) = (src(32), src(16));
    // (a lobe max(0, n.d)^p's average of the radiance, each texel weighed by its solid angle)
    let gather = |src: &[(Vec3, Vec3, f32)], n: Vec3, p: f32| {
        let (mut c, mut w) = (Vec3::ZERO, 0.0f32);
        for (d, l, sa) in src {
            let k = n.dot(*d);
            if k <= 0.0 { continue }
            let wk = k.powf(p) * sa;
            c += *l * wk;
            w += wk;
        }
        c / w.max(1e-9)
    };
    // the diffuse light: the cosine-weighted average (irradiance over pi)
    let mut diffuse = Vec::new();
    for (_, n, _) in texels(16) { push(&mut diffuse, gather(&src16, n, 1.0)); }
    // the specular mips, layer by layer (wgpu's LayerMajor order)
    let mips = 7u32;
    let mut levels: Vec<Vec<Vec3>> = Vec::new();
    for k in 0..mips {
        let n = 64usize >> k;
        let r = k as f32 / (mips - 1) as f32;
        let alpha = (r * r).max(1e-3);
        let p = (2.0 / (alpha * alpha) - 2.0).max(1.0);
        let lv: Vec<Vec3> = texels(n).into_iter().map(|(face, d, _)| {
            if k <= 1 {
                // (sharp: the radiance itself, a few points across the texel)
                let e = 0.5 / n as f32;
                let t = d.any_orthonormal_vector() * e * 2.0;
                let b = d.cross(t);
                let _ = face;
                (rad(d + t) + rad(d - t) + rad(d + b) + rad(d - b)) * 0.25
            } else if k == 2 { gather(&src32, d, p) } else { gather(&src16, d, p) }
        }).collect();
        levels.push(lv);
    }
    let mut spec = Vec::new();
    for face in 0..6 {
        for (k, lv) in levels.iter().enumerate() {
            let n = 64usize >> k;
            for c in &lv[face * n * n..(face + 1) * n * n] { push(&mut spec, *c); }
        }
    }
    EnvBytes { sky, diffuse, spec, mips }
}

/// an environment's cubemaps, made (off the main thread) for a look
struct EnvBytes { sky: Vec<u8>, diffuse: Vec<u8>, spec: Vec<u8>, mips: u32 }

/// the environment being made for a new look
#[derive(Resource, Default)]
struct EnvTask(Option<(Look, bevy::tasks::Task<EnvBytes>)>);

/// The environment for the look the settings ask for: made in the background when that changes
/// (it takes a moment), swapped in when it's done
fn make_env(settings: Res<crate::settings::Settings>, env: Option<Res<ArenaEnv>>, mut task: ResMut<EnvTask>, mut images: ResMut<Assets<Image>>, mut commands: Commands) {
    let want = Look::of(&settings);
    if let Some((look, t)) = &mut task.0 {
        if let Some(b) = bevy::tasks::block_on(bevy::tasks::futures_lite::future::poll_once(t)) {
            let look = *look;
            task.0 = None;
            if look == want {
                commands.insert_resource(ArenaEnv {
                    look,
                    sky: images.add(cube(128, 1, b.sky)),
                    diffuse: images.add(cube(16, 1, b.diffuse)),
                    specular: images.add(cube(64, b.mips, b.spec)),
                });
            }
        }
        return;
    }
    if env.as_ref().is_some_and(|e| e.look == want) { return }
    task.0 = Some((want, bevy::tasks::AsyncComputeTaskPool::get().spawn(async move { build_env(want) })));
}

