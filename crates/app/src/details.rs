//! What a robot that's been through a season has on it that its CAD doesn't: the wiring and the
//! little electrical parts. Worked out from each model once it's in, from its own surfaces (a
//! height map of everything it has, looked down on: where a part can sit on top of it):
//!   - the Robot Signal Light (the rules' orange light, on top of the robot at the back): solid
//!     while the robot's disabled, flashing while it's enabled;
//!   - at each swerve module, the motors' wires: a service loop of power (red, black) and CAN
//!     (yellow, green) up off the motors and over into the robot, zip-tied;
//!   - the motors' status LEDs (a Kraken's, on its back end): orange and off by turns while
//!     disabled, orange while enabled and still, flashing green driving forward and red back;
//!   - stickers on its flat tops: the team's own, a safety stripe, a parts label, a QR inventory tag.
//! Small, so cheap: a few hundred triangles a robot, no shadows from the lights and stickers.

use bevy::asset::RenderAssetUsages;
use bevy::light::NotShadowCaster;
use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use std::collections::HashMap;

use crate::game::Game;
use crate::materials::Look;
use crate::robot_model::RobotModel;

pub struct DetailsPlugin;

impl Plugin for DetailsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, make).add_systems(PostUpdate, light.before(TransformSystems::Propagate));
    }
}

/// a model whose details are on
#[derive(Component)]
struct Detailed;

/// the Robot Signal Light's lens (robot index; None: the robot select's, never enabled)
#[derive(Component)]
struct Rsl(Option<usize>);
/// a module's motor status LED
#[derive(Component)]
struct Led { robot: Option<usize>, module: usize }

/// the lights' looks, shared
#[derive(Resource)]
struct Lamps { rsl_on: Handle<StandardMaterial>, rsl_off: Handle<StandardMaterial>, off: Handle<StandardMaterial>, orange: Handle<StandardMaterial>, green: Handle<StandardMaterial>, red: Handle<StandardMaterial> }

/// grid cell (m)
const CELL: f32 = 0.01;
/// the map's reach either way from the robot's center (m)
const REACH: f32 = 0.7;
const N: usize = (2.0 * REACH / CELL) as usize;

/// The highest surface over each cell, looked down on (robot frame: x forward, z right)
struct HeightMap { h: Vec<f32>, cap: f32 }

impl HeightMap {
    /// (what's under cap high only)
    fn new(cap: f32) -> Self { HeightMap { h: vec![f32::NEG_INFINITY; N * N], cap } }
    fn idx(x: f32, z: f32) -> Option<usize> {
        let (i, j) = (((x + REACH) / CELL).floor(), ((z + REACH) / CELL).floor());
        if i < 0.0 || j < 0.0 || i >= N as f32 || j >= N as f32 { return None }
        Some(i as usize * N + j as usize)
    }
    fn put(&mut self, p: Vec3) { if p.y > self.cap { return } if let Some(k) = Self::idx(p.x, p.z) { if p.y > self.h[k] { self.h[k] = p.y; } } }
    fn at(&self, x: f32, z: f32) -> f32 { Self::idx(x, z).map(|k| self.h[k]).unwrap_or(f32::NEG_INFINITY) }
    /// the highest and lowest top over a box (half sizes hx, hz)
    fn span(&self, x: f32, z: f32, hx: f32, hz: f32) -> (f32, f32) {
        let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
        let mut u = -hx;
        while u <= hx + 1e-4 {
            let mut v = -hz;
            while v <= hz + 1e-4 {
                let h = self.at(x + u, z + v);
                lo = lo.min(h); hi = hi.max(h);
                v += CELL;
            }
            u += CELL;
        }
        (lo, hi)
    }
    /// every triangle of a mesh (to the robot's frame by `to`), filled in a few mm apart
    fn add(&mut self, mesh: &Mesh, to: &bevy::math::Affine3A) {
        let Some(VertexAttributeValues::Float32x3(p)) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else { return };
        let p: Vec<Vec3> = p.iter().map(|v| to.transform_point3(Vec3::from(*v))).collect();
        let tri: Vec<[usize; 3]> = match mesh.indices() {
            Some(Indices::U32(i)) => i.chunks_exact(3).map(|t| [t[0] as usize, t[1] as usize, t[2] as usize]).collect(),
            Some(Indices::U16(i)) => i.chunks_exact(3).map(|t| [t[0] as usize, t[1] as usize, t[2] as usize]).collect(),
            None => (0..p.len() / 3).map(|t| [3 * t, 3 * t + 1, 3 * t + 2]).collect(),
        };
        for [a, b, c] in tri {
            let (Some(&a), Some(&b), Some(&c)) = (p.get(a), p.get(b), p.get(c)) else { continue };
            let n = ((b - a).length().max((c - a).length()).max((c - b).length()) / (0.6 * CELL)).ceil().clamp(1.0, 200.0) as usize;
            for i in 0..=n {
                for j in 0..=(n - i) {
                    let (u, v) = (i as f32 / n as f32, j as f32 / n as f32);
                    self.put(a + (b - a) * u + (c - a) * v);
                }
            }
        }
    }
}

/// a tube along a path (its points), radius r, `sides` round
fn tube(path: &[Vec3], r: f32, sides: usize) -> Mesh {
    let (mut pos, mut nor, mut idx) = (Vec::new(), Vec::new(), Vec::new());
    let mut prev_n = Vec3::Y;
    for (k, &p) in path.iter().enumerate() {
        let t = if k + 1 < path.len() { path[k + 1] - p } else { p - path[k - 1] }.normalize_or(Vec3::X);
        // (a frame carried along so the tube doesn't twist)
        let mut n = prev_n - t * prev_n.dot(t);
        if n.length_squared() < 1e-8 { n = t.any_orthonormal_vector(); }
        let n = n.normalize();
        prev_n = n;
        let b = t.cross(n);
        for s in 0..sides {
            let a = s as f32 / sides as f32 * std::f32::consts::TAU;
            let d = n * a.cos() + b * a.sin();
            pos.push((p + d * r).to_array());
            nor.push(d.to_array());
        }
    }
    for k in 0..path.len() - 1 {
        for s in 0..sides {
            let (a, b) = ((k * sides + s) as u32, (k * sides + (s + 1) % sides) as u32);
            let (c, d) = (a + sides as u32, b + sides as u32);
            idx.extend([a, c, b, b, c, d]);
        }
    }
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, pos)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, nor)
        .with_inserted_indices(Indices::U32(idx))
}

fn bezier(p: [Vec3; 4], t: f32) -> Vec3 {
    let u = 1.0 - t;
    p[0] * u * u * u + p[1] * 3.0 * u * u * t + p[2] * 3.0 * u * t * t + p[3] * t * t * t
}

/// a little hash, for each robot's own choices
fn hash(a: u32, b: u32) -> f32 {
    let mut x = a.wrapping_mul(0x9E37_79B9) ^ b.wrapping_mul(0x85EB_CA6B);
    x ^= x >> 15; x = x.wrapping_mul(0x2C1B_3C6D); x ^= x >> 12;
    (x & 0xffff) as f32 / 65535.0
}

/// the stickers' faces: drawn here (made up: no one's real logo)
fn sticker_images(images: &mut Assets<Image>) -> Vec<(Handle<Image>, Vec2)> {
    let make = |w: u32, h: u32, f: &dyn Fn(f32, f32) -> [u8; 4]| {
        let mut px = Vec::with_capacity((w * h * 4) as usize);
        for y in 0..h { for x in 0..w { px.extend(f((x as f32 + 0.5) / w as f32, (y as f32 + 0.5) / h as f32)); } }
        Image::new(Extent3d { width: w, height: h, depth_or_array_layers: 1 }, TextureDimension::D2, px, TextureFormat::Rgba8UnormSrgb, RenderAssetUsages::default())
    };
    let rr = |u: f32, v: f32, r: f32, a: f32| { let (dx, dy) = ((u - 0.5).abs() - (0.5 - r), (v - 0.5).abs() - (0.5 - r * a)); dx.max(0.0).hypot(dy.max(0.0) / a) <= r };
    vec![
        // a team sticker: white, a colored disk and two bars of "lettering"
        (images.add(make(96, 48, &|u, v| {
            if !rr(u, v, 0.08, 2.0) { return [0, 0, 0, 0] }
            if (u - 0.22).hypot((v - 0.5) * 0.5) < 0.13 { return [235, 120, 30, 255] }
            if (0.42..0.88).contains(&u) && ((0.3..0.42).contains(&v) || (0.55..0.67).contains(&v) && u < 0.75) { return [30, 30, 40, 255] }
            [245, 245, 240, 255]
        })), Vec2::new(0.08, 0.04)),
        // a safety stripe
        (images.add(make(96, 24, &|u, v| {
            if ((u * 96.0 + v * 24.0) / 12.0).floor() as i32 % 2 == 0 { [235, 190, 20, 255] } else { [25, 25, 25, 255] }
        })), Vec2::new(0.1, 0.025)),
        // a parts label: white, a black border, lines of small print
        (images.add(make(64, 40, &|u, v| {
            if !(0.04..0.96).contains(&u) || !(0.06..0.94).contains(&v) { return [20, 20, 20, 255] }
            if ((v * 7.0).fract() < 0.35) && u > 0.12 && u < 0.88 - 0.3 * ((v * 7.0).floor() % 2.0) { return [60, 60, 60, 255] }
            [250, 250, 250, 255]
        })), Vec2::new(0.05, 0.03)),
        // an inventory tag: a QR-ish square
        (images.add(make(32, 32, &|u, v| {
            let (i, j) = ((u * 11.0) as u32, (v * 11.0) as u32);
            let finder = |a: u32, b: u32| a < 3 && b < 3;
            let on = finder(i, j) || finder(10 - i.min(10), j) || finder(i, 10 - j.min(10)) || hash(i * 31 + 7, j) > 0.5;
            if i == 0 || j == 0 || i == 10 || j == 10 { [255, 255, 255, 255] } else if on { [10, 10, 10, 255] } else { [255, 255, 255, 255] }
        })), Vec2::new(0.03, 0.03)),
    ]
}

#[allow(clippy::too_many_arguments)]
fn make(
    models: Query<(Entity, &RobotModel, &GlobalTransform, Has<crate::showcase::Showcase>), Without<Detailed>>,
    children: Query<&Children>,
    parts: Query<(&Mesh3d, &GlobalTransform)>,
    mut wait: Local<HashMap<Entity, u32>>,
    lamps: Option<Res<Lamps>>,
    mut stickers: Local<Vec<(Handle<StandardMaterial>, Vec2)>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut commands: Commands,
) {
    let lamps = match lamps {
        Some(l) => Lamps { rsl_on: l.rsl_on.clone(), rsl_off: l.rsl_off.clone(), off: l.off.clone(), orange: l.orange.clone(), green: l.green.clone(), red: l.red.clone() },
        None => {
            let lamp = |mats: &mut Assets<StandardMaterial>, c: Color, e: LinearRgba| mats.add(StandardMaterial { base_color: c, emissive: e, perceptual_roughness: 0.35, ..default() });
            let l = Lamps {
                rsl_on: lamp(&mut mats, Color::srgba(1.0, 0.55, 0.1, 1.0), LinearRgba::rgb(16.0, 4.0, 0.3)),
                rsl_off: mats.add(StandardMaterial { base_color: Color::srgba(0.75, 0.32, 0.05, 1.0), perceptual_roughness: 0.25, reflectance: 0.4, ..default() }),
                off: mats.add(StandardMaterial { base_color: Color::srgb(0.03, 0.03, 0.03), perceptual_roughness: 0.4, ..default() }),
                orange: lamp(&mut mats, Color::srgb(1.0, 0.5, 0.1), LinearRgba::rgb(18.0, 6.0, 0.4)),
                green: lamp(&mut mats, Color::srgb(0.2, 1.0, 0.3), LinearRgba::rgb(1.0, 20.0, 2.0)),
                red: lamp(&mut mats, Color::srgb(1.0, 0.15, 0.1), LinearRgba::rgb(22.0, 1.0, 0.6)),
            };
            commands.insert_resource(Lamps { rsl_on: l.rsl_on.clone(), rsl_off: l.rsl_off.clone(), off: l.off.clone(), orange: l.orange.clone(), green: l.green.clone(), red: l.red.clone() });
            l
        }
    };
    if stickers.is_empty() {
        *stickers = sticker_images(&mut images).into_iter().map(|(img, size)| (mats.add(StandardMaterial {
            base_color_texture: Some(img), alpha_mode: AlphaMode::Mask(0.5), perceptual_roughness: 0.45, ..default()
        }), size)).collect();
    }
    for (root, m, root_gt, showcase) in &models {
        if !m.ready { continue }
        // (two frames after it's in: its parts' transforms propagated)
        let w = wait.entry(root).or_insert(0);
        *w += 1;
        if *w < 3 { continue }
        wait.remove(&root);
        commands.entity(root).insert(Detailed);
        let key = m.key().to_string();
        let robot = if showcase { None } else { Some(m.robot) };
        let to_robot = root_gt.affine().inverse();
        // what moves (its parts, and everything under them): left out of the map the stickers and
        // the light sit on
        let mut moving = std::collections::HashSet::new();
        for p in m.moving_parts() { moving.insert(p); for d in children.iter_descendants(p) { moving.insert(d); } }
        // (low: what's up to the swerve modules' motors, under whatever's built over them)
        let (mut low, mut still) = (HeightMap::new(0.24), HeightMap::new(f32::INFINITY));
        for e in children.iter_descendants(root) {
            let Ok((mh, gt)) = parts.get(e) else { continue };
            let Some(mesh) = meshes.get(&mh.0) else { continue };
            let to = to_robot * gt.affine();
            low.add(mesh, &to);
            if !moving.contains(&e) { still.add(mesh, &to); }
        }
        let frame = &sim::auto::data()["robots"][&key]["frame"];
        let hl = frame["length"].as_f64().unwrap_or(0.7) as f32 / 2.0;
        let hw = frame["width"].as_f64().unwrap_or(0.7) as f32 / 2.0;
        let seed = key.bytes().fold(7u32, |a, b| a.wrapping_mul(31).wrapping_add(b as u32));
        let mut spawn_part = |commands: &mut Commands, mesh: Mesh, mat: Handle<StandardMaterial>, t: Transform, shadow: bool| {
            let mut e = commands.spawn((Mesh3d(meshes.add(mesh)), MeshMaterial3d(mat), t, Look));
            if !shadow { e.insert(NotShadowCaster); }
            let id = e.id();
            commands.entity(root).add_child(id);
            id
        };

        // ---- the Robot Signal Light: on top of what's at the back, middle (a flat spot, its
        // footprint 9.5 x 4.5 cm, as near the back middle as one is)
        let mut best: Option<(f32, Vec3)> = None;
        let mut x = -hl + 0.06;
        while x < -hl + 0.3 {
            let mut z = -0.16;
            while z <= 0.16 {
                let (lo, hi) = still.span(x, z, 0.025, 0.05);
                if lo > 0.05 && hi < 1.3 {
                    let score = (hi - lo) * 20.0 + z.abs() + (x + hl - 0.06) * 0.6;
                    if best.is_none_or(|b| score < b.0) { best = Some((score, Vec3::new(x, hi, z))); }
                }
                z += CELL;
            }
            x += CELL;
        }
        if let Some((_, at)) = best {
            let base = mats.add(StandardMaterial { base_color: Color::srgb(0.05, 0.05, 0.05), perceptual_roughness: 0.6, ..default() });
            spawn_part(&mut commands, Cuboid::new(0.045, 0.018, 0.095).into(), base, Transform::from_translation(at + Vec3::Y * 0.009), true);
            let lens = spawn_part(&mut commands, Cuboid::new(0.038, 0.032, 0.085).into(), lamps.rsl_off.clone(), Transform::from_translation(at + Vec3::Y * 0.034), false);
            commands.entity(lens).insert(Rsl(robot));
        }

        // ---- each module's wires and its motor's LED
        let wire = |c: Color| StandardMaterial { base_color: c, perceptual_roughness: 0.45, reflectance: 0.35, ..default() };
        let wmats = [mats.add(wire(Color::srgb(0.65, 0.05, 0.04))), mats.add(wire(Color::srgb(0.03, 0.03, 0.03))), mats.add(wire(Color::srgb(0.85, 0.7, 0.05))), mats.add(wire(Color::srgb(0.05, 0.45, 0.12)))];
        let tie = mats.add(StandardMaterial { base_color: Color::srgb(0.02, 0.02, 0.02), perceptual_roughness: 0.55, ..default() });
        for (i, c) in m.module_points().iter().enumerate() {
            // (the motor's top: the highest thing within 3.5 cm of the module's center, up to 24 cm)
            let (_, top) = low.span(c.x, c.y, 0.035, 0.035);
            if !top.is_finite() || top < 0.06 { continue }
            let inward = (-*c).normalize_or(Vec2::X);
            let side = Vec2::new(-inward.y, inward.x);
            let p0 = Vec3::new(c.x + inward.x * 0.012, top + 0.003, c.y + inward.y * 0.012);
            let p3 = Vec3::new(c.x + inward.x * 0.13, top - 0.03, c.y + inward.y * 0.13);
            let lift = 0.045 + 0.02 * hash(seed, i as u32);
            let ctrl = [p0, p0 + Vec3::Y * lift + Vec3::new(inward.x, 0.0, inward.y) * 0.01, p3 + Vec3::Y * (lift + 0.015), p3];
            let path: Vec<Vec3> = (0..=16).map(|k| bezier(ctrl, k as f32 / 16.0)).collect();
            // (four wires twisted loosely round the bundle's middle)
            for (w, mat) in wmats.iter().enumerate() {
                let r = if w < 2 { 0.0017 } else { 0.0011 };
                let ph = w as f32 * std::f32::consts::FRAC_PI_2;
                let p: Vec<Vec3> = path.iter().enumerate().map(|(k, q)| {
                    let a = ph + k as f32 * 0.35;
                    *q + Vec3::new(side.x, 0.0, side.y) * 0.0022 * a.cos() + Vec3::Y * 0.0022 * a.sin()
                }).collect();
                spawn_part(&mut commands, tube(&p, r, 6), mat.clone(), Transform::IDENTITY, true);
            }
            // (zip ties round it, a third and two thirds along)
            for t in [0.33f32, 0.68] {
                let q = bezier(ctrl, t);
                let d = (bezier(ctrl, t + 0.02) - bezier(ctrl, t - 0.02)).normalize_or(Vec3::Y);
                let ring = Transform::from_translation(q).with_rotation(Quat::from_rotation_arc(Vec3::Y, d));
                spawn_part(&mut commands, Cylinder::new(0.0052, 0.0028).into(), tie.clone(), ring, true);
                // (its cut tail)
                spawn_part(&mut commands, Cuboid::new(0.0025, 0.0012, 0.006).into(), tie.clone(), ring * Transform::from_xyz(0.0055, 0.0, 0.0), true);
            }
            // (the status LED: a short strip on the motor's back end, beside the wires)
            let at = Vec3::new(c.x - inward.x * 0.012 + side.x * 0.01, top + 0.0015, c.y - inward.y * 0.012 + side.y * 0.01);
            let led = spawn_part(&mut commands, Cuboid::new(0.004, 0.002, 0.012).into(), lamps.off.clone(), Transform::from_translation(at).with_rotation(Quat::from_rotation_y(-inward.y.atan2(inward.x))), false);
            commands.entity(led).insert(Led { robot, module: i });
        }

        // ---- stickers: on flat tops, inside the frame, away from the light
        let mut placed: Vec<Vec2> = best.map(|b| vec![Vec2::new(b.1.x, b.1.z)]).unwrap_or_default();
        let n = 2 + (hash(seed, 99) * 2.0) as usize;
        let mut tries = 0u32;
        while placed.len() < n + usize::from(best.is_some()) && tries < 400 {
            tries += 1;
            let k = (hash(seed, 1000 + tries) * stickers.len() as f32) as usize % stickers.len();
            let (mat, size) = stickers[k].clone();
            let x = (hash(seed, 2000 + tries) * 2.0 - 1.0) * (hl - 0.06);
            let z = (hash(seed, 3000 + tries) * 2.0 - 1.0) * (hw - 0.06);
            if placed.iter().any(|p| p.distance(Vec2::new(x, z)) < 0.12) { continue }
            let r = size.max_element() / 2.0 + 0.004;
            let (lo, hi) = still.span(x, z, r, r);
            if !(lo > 0.03 && hi - lo < 0.002 && hi < 1.2) { continue }
            placed.push(Vec2::new(x, z));
            let turn = Quat::from_rotation_y(if hash(seed, 4000 + tries) > 0.5 { 0.0 } else { std::f32::consts::FRAC_PI_2 } + (hash(seed, 5000 + tries) - 0.5) * 0.3);
            let quad = Plane3d::new(Vec3::Y, size / 2.0);
            spawn_part(&mut commands, quad.into(), mat, Transform::from_translation(Vec3::new(x, hi + 0.0008, z)).with_rotation(turn), false);
        }
        info!("robot {key}: details on (light {}, {} module loops, {} stickers)", best.is_some(), m.module_points().len(), placed.len() - usize::from(best.is_some()));
    }
}

/// Each frame: the signal light and the motors' LEDs as the robot's state has them
fn light(
    game: Res<Game>,
    time: Res<Time>,
    lamps: Option<Res<Lamps>>,
    mut rsl: Query<(&Rsl, &mut MeshMaterial3d<StandardMaterial>), Without<Led>>,
    mut leds: Query<(&Led, &mut MeshMaterial3d<StandardMaterial>), Without<Rsl>>,
) {
    let Some(l) = lamps else { return };
    let t = time.elapsed_secs();
    let enabled = |i: Option<usize>| i.and_then(|i| game.0.robots.get(i)).is_some_and(|r| r.enabled);
    let set = |m: &mut MeshMaterial3d<StandardMaterial>, h: &Handle<StandardMaterial>| if m.0 != *h { m.0 = h.clone(); };
    // (the RSL: solid disabled, flashing about 1.7 times a second enabled)
    for (r, mut m) in &mut rsl {
        let on = !enabled(r.0) || (t * 1.7).fract() < 0.5;
        set(&mut m, if on { &l.rsl_on } else { &l.rsl_off });
    }
    for (led, mut m) in &mut leds {
        let r = led.robot.and_then(|i| game.0.robots.get(i));
        let h = match r {
            Some(r) if r.enabled => {
                let w = r.swerve.modules.get(led.module).map(|md| md.w).unwrap_or(0.0);
                let blink = (t * 5.0).fract() < 0.5;
                if w.abs() < 0.5 { &l.orange } else if !blink { &l.off } else if w > 0.0 { &l.green } else { &l.red }
            }
            // (disabled: orange and off by turns)
            _ => if (t * 1.2).fract() < 0.5 { &l.orange } else { &l.off },
        };
        set(&mut m, h);
    }
}
