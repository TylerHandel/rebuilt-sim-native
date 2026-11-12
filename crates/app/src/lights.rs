//! What glows: the HUBs' light bars (js field.js setHubLights) and the robots' LED strips (js
//! leds.js), drawn from the match's and the robots' state. Visual only: nothing here touches the
//! physics. With bloom on (High and up, camera.rs) they glow.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use sim::constants::{Alliance, HUB, TIMING};
use sim::game_match::{HubLight, Phase};

use crate::game::Game;
use crate::robot_model::RobotModel;
use crate::settings::Settings;

pub struct LightsPlugin;

impl Plugin for LightsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LedGlow>().add_systems(Startup, spawn_hub_lights)
            .add_systems(PostUpdate, (add_leds, hub_lights, leds));
    }
}

/// each robot's LEDs, their average color (linear, by robot index): what they light round them
/// (scene.rs's light pools at night)
#[derive(Resource, Default)]
pub struct LedGlow(pub Vec<Vec3>);

/// How bright what glows is drawn (unlit: not dimmed by the camera's exposure): bright enough to
/// read in daylight, low enough that the tone mapping keeps its color (much more washes it to
/// white, and at night the eye's adaptation brightens it further)
pub fn glow(settings: &Settings) -> f32 {
    match settings.graphics.as_str() { "ultra" | "high" => 2.0, _ => 1.6 }
}

// ------------------------------------------------------------------ the HUBs

/// js field.js ALLIANCE_COLOR
fn alliance_color(a: Alliance) -> Vec3 {
    let c = if a == Alliance::Blue { 0x1f5fe0u32 } else { 0xd92b2b };
    let l = Color::srgb_u8((c >> 16) as u8, (c >> 8) as u8, c as u8).to_linear();
    Vec3::new(l.red, l.green, l.blue)
}

#[derive(Component)]
struct HubLights(Alliance);

/// A box from its center, size and turn
fn box_mesh(meshes: &mut Assets<Mesh>, size: Vec3) -> Handle<Mesh> { meshes.add(Cuboid::new(size.x, size.y, size.z)) }

/// js field.js: the LED diffusers wrapping the HUB below its roof (with the field CAD shown) and
/// the bars along the funnel's hexagonal rim, one material per HUB
fn spawn_hub_lights(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let hs = (HUB::size / 2.0) as f32;
    let (y0, y1) = (HUB::lightY0 as f32, HUB::lightY1 as f32);
    let (dy, dh, o) = ((y0 + y1) / 2.0, y1 - y0, hs + 0.012);
    let rim_y = HUB::rimFront as f32 + 0.01;
    let r = HUB::hexR as f32;
    for a in [Alliance::Blue, Alliance::Red] {
        let (hx, _) = sim::field::hub_center(a);
        let mat = mats.add(StandardMaterial { base_color: Color::srgb(0.13, 0.13, 0.13), perceptual_roughness: 0.4, ..default() });
        let root = commands.spawn((Transform::from_xyz(hx as f32, 0.0, 0.0), Visibility::default(), HubLights(a), MeshMaterial3d(mat.clone()))).id();
        let mut kids = Vec::new();
        for (w, d, px, pz) in [(HUB::size as f32 - 0.02, 0.012, 0.0, o), (HUB::size as f32 - 0.02, 0.012, 0.0, -o), (0.012, HUB::size as f32 - 0.02, o, 0.0), (0.012, HUB::size as f32 - 0.02, -o, 0.0)] {
            kids.push(commands.spawn((Mesh3d(box_mesh(&mut meshes, Vec3::new(w, dh, d))), MeshMaterial3d(mat.clone()), Transform::from_xyz(px, dy, pz), bevy::light::NotShadowCaster)).id());
        }
        for k in 0..6 {
            let v = |k: usize| { let t = k as f32 * std::f32::consts::PI / 3.0; Vec3::new(r * t.cos(), rim_y, r * t.sin()) };
            let (p0, p1) = (v(k), v((k + 1) % 6));
            let len = p0.distance(p1);
            let rot = Quat::from_rotation_arc(Vec3::X, (p1 - p0).normalize());
            kids.push(commands.spawn((Mesh3d(box_mesh(&mut meshes, Vec3::new(len, 0.025, 0.025))), MeshMaterial3d(mat.clone()), Transform::from_translation((p0 + p1) / 2.0).with_rotation(rot), bevy::light::NotShadowCaster)).id());
        }
        commands.entity(root).add_children(&kids);
    }
}

/// js field.js setHubLights, each frame from the match (Match.hubLightMode)
fn hub_lights(game: Res<Game>, time: Res<Time>, settings: Res<Settings>, hubs: Query<(&HubLights, &MeshMaterial3d<StandardMaterial>)>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let t = time.elapsed_secs();
    let g = glow(&settings) / 2.2;
    for (h, mat) in &hubs {
        let col = alliance_color(h.0);
        let (color, k) = match game.0.match_.hub_light_mode(h.0) {
            HubLight::Active => (col, 2.2),
            HubLight::Warning => (col, 0.4 + 1.8 * (0.5 + 0.5 * (t * 14.0).sin())),
            HubLight::Chase => (col.lerp(Vec3::ONE, 0.5 + 0.5 * (t * 9.0).sin()), 2.0),
            HubLight::White => (Vec3::ONE, 1.8),
            HubLight::Green => (Vec3::new(0.016, 1.0, 0.133), 1.5),
            HubLight::Off => (col, 0.0),
        };
        let Some(mut m) = mats.get_mut(&mat.0) else { continue };
        let e = color * k * g;
        m.emissive = LinearRgba::rgb(e.x, e.y, e.z);
        m.base_color = if k > 0.0 { Color::linear_rgb(color.x, color.y, color.z) } else { Color::srgb(0.13, 0.13, 0.13) };
    }
}

// ------------------------------------------------------------------ the robots' LEDs

/// A run of LEDs (js leds.js LED_LAYOUT): along a path, or round an arc ({c: [x, z], r, y, a0,
/// a1}, degrees from +x toward -z); `n` from index `i0`; `dot` an LED's length over its spacing
enum Along { Path(&'static [[f32; 3]]), Arc { c: [f32; 2], r: f32, y: f32, a0: f32, a1: f32 } }
struct Run { along: Along, n: usize, i0: usize, dot: f32, pitch: Option<f32> }

const fn path(p: &'static [[f32; 3]], n: usize, i0: usize) -> Run { Run { along: Along::Path(p), n, i0, dot: 0.6, pitch: None } }

/// js leds.js LED_LAYOUT: where each team's strips are (the sources are with the JS)
fn layout(key: &str) -> Option<(usize, Vec<Run>)> {
    const IN: f32 = 0.0254;
    Some(match key {
        "1114" => (60, vec![Run { dot: 0.45, ..path(&[[-0.296, 0.40, -0.285], [-0.296, 0.69, -0.285], [-0.296, 0.69, 0.285], [-0.296, 0.40, 0.285]], 60, 0) }]),
        "1678" => {
            const A: f32 = 0.15 + 12.0 / 60.0;
            const B: f32 = 0.15 + 17.0 / 60.0;
            (17, vec![
                path(&[[-0.02, 0.15, -0.330], [-0.02, A, -0.330]], 12, 0),
                path(&[[-0.02, A, -0.330], [-0.02, B, -0.330]], 5, 12),
                path(&[[-0.02, 0.15, 0.330], [-0.02, A, 0.330]], 12, 0),
                path(&[[-0.02, A, 0.330], [-0.02, B, 0.330]], 5, 12),
            ])
        }
        "1690" => (40, vec![Run { along: Along::Arc { c: [-0.134, -0.1872], r: 0.165, y: 0.39, a0: -105.0, a1: 105.0 }, n: 40, i0: 0, dot: 1.0, pitch: None }]),
        "2056" => (36, vec![Run { along: Along::Arc { c: [-0.1334, 0.1841], r: 0.125, y: 0.55, a0: 0.0, a1: 350.0 }, n: 36, i0: 0, dot: 1.0, pitch: None }]),
        "7769" => (8, vec![Run { pitch: Some(0.01), ..path(&[[-0.31, 0.18, 0.35], [-0.31, 0.26, 0.35]], 8, 0) }]),
        "8793" => (68, vec![
            Run { pitch: Some(0.008), ..path(&[[0.27, 0.215, -0.24], [0.27, 0.215, -0.17]], 8, 0) },
            Run { along: Along::Arc { c: [-0.127, 0.0], r: 8.0 * IN + 0.008, y: 0.331, a0: -150.0, a1: 150.0 }, n: 60, i0: 8, dot: 0.8, pitch: None },
        ]),
        _ => return None,
    })
}

/// One LED's place: its center, along the strip, facing out, and its spacing
struct Frame { p: Vec3, along: Vec3, out: Vec3, len: f32 }

/// js leds.js frames(run)
fn frames(run: &Run) -> Vec<Frame> {
    match &run.along {
        Along::Arc { c, r, y, a0, a1 } => {
            let da = (a1 - a0) / run.n as f32;
            (0..run.n).map(|i| {
                let a = (a0 + (i as f32 + 0.5) * da).to_radians();
                Frame {
                    p: Vec3::new(c[0] + r * a.cos(), *y, c[1] - r * a.sin()),
                    along: Vec3::new(-a.sin(), 0.0, -a.cos()),
                    out: Vec3::new(a.cos(), 0.0, -a.sin()),
                    len: (da.to_radians() * r).abs(),
                }
            }).collect()
        }
        Along::Path(pts) => {
            let pts: Vec<Vec3> = pts.iter().map(|q| Vec3::from_array(*q)).collect();
            let segs: Vec<f32> = pts.windows(2).map(|w| w[1].distance(w[0])).collect();
            let total: f32 = segs.iter().sum();
            let step = run.pitch.unwrap_or(total / run.n as f32);
            let start = if run.pitch.is_some() { (total - step * run.n as f32) / 2.0 } else { 0.0 };
            (0..run.n).map(|i| {
                let (mut s, mut k) = (start + (i as f32 + 0.5) * step, 0);
                while k < segs.len() - 1 && s > segs[k] { s -= segs[k]; k += 1; }
                let along = (pts[k + 1] - pts[k]).normalize();
                let p = pts[k] + along * s;
                // outward: across the strip, away from the robot's middle
                let radial = Vec3::new(p.x, 0.0, p.z);
                let mut out = radial - along * radial.dot(along);
                if p.x.abs() > p.z.abs() * 0.9 || out.length_squared() < 1e-6 { out = Vec3::new(if p.x < 0.0 { -1.0 } else { 1.0 }, 0.0, 0.0) - along * along.x; }
                if out.length_squared() < 1e-6 { out = Vec3::Y; }
                Frame { p, along, out: out.normalize(), len: step }
            }).collect()
        }
    }
}

/// js leds.js pushBox: one box at frame f (len along, w across, t out, standing `depth` out),
/// all its corners LED `led`'s
#[allow(clippy::too_many_arguments)]
fn push_box(pos: &mut Vec<[f32; 3]>, nrm: &mut Vec<[f32; 3]>, of: &mut Vec<usize>, idx: &mut Vec<u32>, f: &Frame, len: f32, w: f32, t: f32, depth: f32, led: usize) {
    let side = f.along.cross(f.out).normalize();
    let c = f.p + f.out * (depth + t / 2.0);
    let ax = [f.along * (len / 2.0), side * (w / 2.0), f.out * (t / 2.0)];
    let dirs = [f.along, side, f.out];
    for k in 0..3 {
        for s in [-1.0f32, 1.0] {
            let n = dirs[k] * s;
            let (a, b) = (ax[(k + 1) % 3], ax[(k + 2) % 3]);
            let base = pos.len() as u32;
            let ctr = c + ax[k] * s;
            for (i, j) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                pos.push((ctr + a * i + b * j).to_array());
                nrm.push(n.to_array());
                of.push(led);
            }
            if a.cross(b).dot(n) < 0.0 { idx.extend([base, base + 2, base + 1, base, base + 3, base + 2]); }
            else { idx.extend([base, base + 1, base + 2, base, base + 2, base + 3]); }
        }
    }
}

/// A robot's strips: each LED's color on its corners' vertex colors (`of`: which LED a vertex is;
/// `n`, the backing's: always dark). (js leds.js puts them in a texture; Bevy draws vertex colors
/// as they change without rebuilding the material.) Unlit, so a color over 1 is a light: it blooms.
#[derive(Component)]
struct Leds { robot: usize, key: String, n: usize, mesh: Handle<Mesh>, of: Vec<usize>, rgb: Vec<[u8; 3]>, glow: f32, shown: Vec<[f32; 3]> }

#[derive(Component)]
struct LedsAdded;

/// js leds.js addLeds: each robot model with LEDs gets one mesh of them, on its root
fn add_leds(
    game: Res<Game>,
    models: Query<(Entity, &RobotModel), Without<LedsAdded>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut commands: Commands,
) {
    const W: f32 = 0.011;
    const T: f32 = 0.004;
    for (e, m) in &models {
        commands.entity(e).insert(LedsAdded);
        let Some(r) = game.0.robots.get(m.robot) else { continue };
        let Some((n, runs)) = layout(&r.key) else { continue };
        let (mut pos, mut nrm, mut of, mut idx) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for run in &runs {
            for (i, f) in frames(run).iter().enumerate() {
                push_box(&mut pos, &mut nrm, &mut of, &mut idx, f, f.len * 1.001, W + 0.003, 0.002, 0.0, n); // backing
                push_box(&mut pos, &mut nrm, &mut of, &mut idx, f, f.len * run.dot, W, T - 0.002, 0.002, run.i0 + i);
            }
        }
        let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
        let dark = [0.006f32, 0.006, 0.007, 1.0];
        mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, vec![dark; pos.len()]);
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, nrm);
        mesh.insert_indices(Indices::U32(idx));
        let mesh = meshes.add(mesh);
        let mat = mats.add(StandardMaterial { base_color: Color::WHITE, unlit: true, ..default() });
        let child = commands.spawn((
            Mesh3d(mesh.clone()), MeshMaterial3d(mat), Transform::default(),
            bevy::light::NotShadowCaster, bevy::light::NotShadowReceiver,
            Leds { robot: m.robot, key: r.key.clone(), n, mesh, of, rgb: vec![[0; 3]; n], glow: 0.0, shown: vec![[0.0; 3]; n] },
        )).id();
        commands.entity(e).add_child(child);
    }
}

/// The robot's state as the LEDs see it (js leds.js stateOf)
struct LedState { enabled: bool, auto: bool, blue: bool, climbing: bool, shooting: bool, want_shoot: bool, passing: bool, ready: bool, intaking: bool, full: bool, has: bool, hub_active: bool, hub_next: Option<f64>, left: Option<f64> }

type Rgb = [f32; 3];
const OFF: Rgb = [0.0; 3];
const RED: Rgb = [255.0, 0.0, 0.0];
const GREEN: Rgb = [0.0, 255.0, 0.0];
const BLUE: Rgb = [0.0, 0.0, 255.0];
const WHITE: Rgb = [255.0; 3];
const YELLOW: Rgb = [255.0, 255.0, 0.0];
const ORANGE: Rgb = [255.0, 165.0, 0.0];
const PURPLE: Rgb = [128.0, 0.0, 128.0];
const AQUA: Rgb = [0.0, 255.0, 255.0];
fn scale(c: Rgb, k: f32) -> Rgb { [c[0] * k, c[1] * k, c[2] * k] }
fn hsv(h: f32) -> Rgb {
    let f = |n: f32| { let k = (n + h * 6.0) % 6.0; 255.0 * (1.0 - k.min(4.0 - k).clamp(0.0, 1.0)) };
    [f(5.0), f(3.0), f(1.0)]
}
fn blink(t: f32, period: f32) -> bool { t % period < period / 2.0 }

/// js leds.js SCHEMES and defaultScheme: each team's own where their code is public
fn scheme(key: &str, s: &LedState, set: &mut dyn FnMut(usize, Rgb), n: usize, t: f32) {
    let all = |set: &mut dyn FnMut(usize, Rgb), c: Rgb| for i in 0..n { set(i, c) };
    use std::f32::consts::TAU;
    match key {
        "8793" => {
            let b = 0.05 + (0.5 + 0.5 * (TAU * t).sin()) * 0.95;
            if s.enabled && s.auto { return all(set, scale([128.0, 0.0, 128.0], b)) }
            if !s.enabled { return all(set, scale([255.0, 20.0, 0.0], b)) }
            let prim = if s.hub_active { GREEN } else { RED };
            let mut hz = 0.0;
            if let Some(l) = s.left.filter(|l| *l <= 10.0) { hz = 1.0 + (1.0 - l as f32 / 10.0) * 7.0; }
            else if let Some(h) = s.hub_next.filter(|h| *h <= 5.0) { hz = 1.0 + (1.0 - h as f32 / 5.0) * 7.0; }
            if hz == 0.0 { return all(set, prim) }
            let ph = (t * 2.0 * hz).floor() as usize % 2;
            for i in 0..n { set(i, if (i / 10 + ph) % 2 == 0 { prim } else { [255.0, 200.0, 0.0] }) }
        }
        "1678" => {
            if !s.enabled { return all(set, AQUA) }
            if s.want_shoot {
                let on = blink(t, 0.18);
                return all(set, if s.ready { if on { YELLOW } else { BLUE } } else if on { OFF } else { RED });
            }
            if s.intaking { return all(set, if blink(t, 0.16) { ORANGE } else { YELLOW }) }
            for i in 0..n { set(i, if i >= 12 { GREEN } else { OFF }) }
        }
        "1114" => {
            if !s.enabled { return all(set, RED) }
            if s.climbing { for i in 0..n { set(i, hsv((((i as f32 - t * 39.37) / 100.0) % 1.0 + 1.0) % 1.0)) } return }
            if s.want_shoot && s.passing { for i in 0..n { set(i, if i < n / 2 { GREEN } else { PURPLE }) } return }
            if s.want_shoot { return all(set, if s.ready { GREEN } else { BLUE }) }
            all(set, YELLOW)
        }
        "1690" => {
            let cyan = [0.0, 200.0, 255.0];
            if !s.enabled { return all(set, scale(cyan, 0.25 + 0.2 * (TAU * t / 3.0).sin())) }
            all(set, if s.shooting { [255.0, 90.0, 0.0] } else { cyan })
        }
        _ => {
            let a = if s.blue { BLUE } else { RED };
            if !s.enabled { return all(set, scale(a, 0.2 + 0.25 * (0.5 + 0.5 * (TAU * t / 2.5).sin()))) }
            if s.climbing { for i in 0..n { set(i, hsv(((i as f32 / n as f32 - t * 0.5) % 1.0 + 1.0) % 1.0)) } return }
            if s.shooting { return all(set, if blink(t, 0.1) { WHITE } else { scale(WHITE, 0.15) }) }
            if s.want_shoot { return all(set, if s.ready { GREEN } else if blink(t, 0.25) { YELLOW } else { OFF }) }
            if s.full { return all(set, PURPLE) }
            if s.intaking { let k = (t * 20.0).floor() as usize; for i in 0..n { set(i, if (i + k) % 4 < 2 { ORANGE } else { scale(ORANGE, 0.15) }) } return }
            all(set, if s.has { a } else { scale(a, 0.35) })
        }
    }
}

/// js leds.js updateLeds, once a frame per robot: its colors rewritten only when one changes
fn leds(game: Res<Game>, time: Res<Time>, settings: Res<Settings>, mut strips: Query<&mut Leds>, mut meshes: ResMut<Assets<Mesh>>, mut glow_of: ResMut<LedGlow>) {
    let w = &game.0;
    let m = &w.match_;
    let t = time.elapsed_secs();
    let dt = time.delta_secs();
    let k = glow(&settings);
    for mut l in &mut strips {
        let l = &mut *l;
        let Some(r) = w.robots.get(l.robot) else { continue };
        let s = LedState {
            enabled: r.enabled && m.robot_enabled(),
            auto: m.phase == Phase::Auto,
            blue: r.alliance == Alliance::Blue,
            climbing: r.climb_state != sim::robot::ClimbState::None,
            shooting: r.out.feeding > 0.0,
            want_shoot: r.want_shoot || r.cmd.shoot || r.cmd.pass,
            passing: r.cmd.pass,
            ready: r.ready,
            intaking: r.out.intake_speed > 0.0,
            full: r.full,
            has: !r.stored.is_empty(),
            hub_active: m.hub_active(r.alliance),
            hub_next: m.hub_next_change(r.alliance),
            left: (m.phase == Phase::Teleop).then(|| (TIMING::teleop - m.phase_time).max(0.0)),
        };
        let mut changed = l.glow != k;
        let (n, rgb) = (l.n, &mut l.rgb);
        scheme(&l.key, &s, &mut |i, c| {
            if i >= n { return }
            let v = c.map(|x| x.round().clamp(0.0, 255.0) as u8);
            if rgb[i] != v { rgb[i] = v; changed = true; }
        }, n, t);
        // (their average, for the light they cast: eased over a third of a second, so a flashing
        // pattern doesn't strobe the floor)
        if glow_of.0.len() <= l.robot { glow_of.0.resize(l.robot + 1, Vec3::ZERO); }
        let mut avg = Vec3::ZERO;
        for c in l.rgb.iter() { let q = Color::srgb_u8(c[0], c[1], c[2]).to_linear(); avg += Vec3::new(q.red, q.green, q.blue); }
        let avg = avg / l.rgb.len().max(1) as f32;
        glow_of.0[l.robot] = glow_of.0[l.robot].lerp(avg, 1.0 - (-dt / 0.35).exp());
        // (each LED eases to its color over a few frames: the patterns read, without the hard strobe)
        let ease = 1.0 - (-dt / 0.06).exp();
        for (i, c) in l.rgb.iter().enumerate() {
            let q = Color::srgb_u8(c[0], c[1], c[2]).to_linear();
            let to = [q.red, q.green, q.blue];
            for j in 0..3 {
                let d = to[j] - l.shown[i][j];
                if d.abs() > 1e-3 { l.shown[i][j] += d * ease; changed = true; } else if d != 0.0 { l.shown[i][j] = to[j]; changed = true; }
            }
        }
        if !changed { continue }
        l.glow = k;
        let Some(mut mesh) = meshes.get_mut(&l.mesh) else { continue };
        // (linear, times how bright lights are drawn)
        let lin: Vec<[f32; 4]> = l.shown.iter().map(|q| [q[0] * k, q[1] * k, q[2] * k, 1.0]).collect();
        let dark = [0.006f32, 0.006, 0.007, 1.0];
        let cols: Vec<[f32; 4]> = l.of.iter().map(|&i| if i < n { lin[i] } else { dark }).collect();
        mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, cols);
    }
}
