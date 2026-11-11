//! What the match does to the light and air: a burst over a HUB as FUEL goes in (sparks in the
//! alliance's color and a flash of light: the more FUEL in at once, the bigger), and, as the Motion
//! effects setting asks, a fading streak behind each FUEL in flight.

use std::collections::{HashMap, VecDeque};

use bevy::prelude::*;
use sim::constants::{Alliance, HUB};

use crate::game::{Game, WorldGen};
use crate::settings::Settings;

pub struct EffectsPlugin;

impl Plugin for EffectsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup).add_systems(Update, (scores, sparks, flashes, trails));
    }
}

#[derive(Resource)]
struct Fx { spark: Handle<Mesh>, mats: [Handle<StandardMaterial>; 2] }

#[derive(Component)]
struct Spark { v: Vec3, life: f32, max: f32, size: f32 }
#[derive(Component)]
struct Flash { life: f32, max: f32, peak: f32 }

fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    // (emissive past bloom's threshold: they glow on High and Ultra)
    let m = |c: LinearRgba, mats: &mut Assets<StandardMaterial>| mats.add(StandardMaterial { base_color: Color::BLACK, emissive: c, unlit: false, ..default() });
    commands.insert_resource(Fx {
        spark: meshes.add(Sphere::new(1.0).mesh().ico(1).unwrap()),
        mats: [m(LinearRgba::rgb(4.0, 12.0, 60.0), &mut mats), m(LinearRgba::rgb(60.0, 8.0, 5.0), &mut mats)],
    });
}

/// a small random (an LCG: the same bursts every run, nothing to do with the sim)
fn rnd(s: &mut u32) -> f32 { *s = s.wrapping_mul(1664525).wrapping_add(1013904223); (*s >> 8) as f32 / 16_777_216.0 }

/// FUEL in a HUB (its alliance's FUEL points going up): a burst over it
fn scores(game: Res<Game>, wg: Res<WorldGen>, settings: Res<Settings>, fx: Res<Fx>, mut last: Local<Option<(u32, [f64; 2])>>, mut seed: Local<u32>, mut commands: Commands) {
    let m = &game.0.match_;
    let now = [Alliance::Blue, Alliance::Red].map(|a| { let s = &m.score[if a == Alliance::Blue { 0 } else { 1 }]; s.auto_fuel + s.tele_fuel + s.inactive_fuel });
    let prev = match *last { Some((g, p)) if g == wg.0 => p, _ => now };
    *last = Some((wg.0, now));
    if settings.graphics == "low" { return }
    for (k, a) in [Alliance::Blue, Alliance::Red].into_iter().enumerate() {
        let d = now[k] - prev[k];
        if d <= 0.0 { continue }
        let (x, z) = sim::field::hub_center(a);
        let top = Vec3::new(x as f32, HUB::rimFront as f32 + 0.15, z as f32);
        let n = (6.0 + 4.0 * d).min(28.0) as usize;
        for _ in 0..n {
            let ang = rnd(&mut seed) * std::f32::consts::TAU;
            let out = 0.8 + 1.8 * rnd(&mut seed);
            let v = Vec3::new(ang.cos() * out, 2.5 + 2.5 * rnd(&mut seed), ang.sin() * out);
            let max = 0.5 + 0.5 * rnd(&mut seed);
            let size = 0.012 + 0.014 * rnd(&mut seed);
            commands.spawn((
                Mesh3d(fx.spark.clone()), MeshMaterial3d(fx.mats[k].clone()),
                Transform::from_translation(top).with_scale(Vec3::splat(size)),
                bevy::light::NotShadowCaster, Spark { v, life: max, max, size },
            ));
        }
        let c = if a == Alliance::Blue { Color::srgb(0.55, 0.7, 1.0) } else { Color::srgb(1.0, 0.6, 0.5) };
        let peak = 150_000.0 * d.min(4.0) as f32;
        commands.spawn((
            PointLight { color: c, intensity: peak, range: 7.0, shadow_maps_enabled: false, ..default() },
            Transform::from_translation(top + Vec3::Y * 0.4),
            Flash { life: 0.35, max: 0.35, peak },
        ));
    }
}

fn sparks(time: Res<Time>, mut q: Query<(Entity, &mut Spark, &mut Transform)>, mut commands: Commands) {
    let dt = time.delta_secs();
    for (e, mut s, mut t) in &mut q {
        s.life -= dt;
        if s.life <= 0.0 { commands.entity(e).despawn(); continue }
        s.v.y -= 9.81 * dt;
        t.translation += s.v * dt;
        t.scale = Vec3::splat(s.size * (s.life / s.max).sqrt());
    }
}

fn flashes(time: Res<Time>, mut q: Query<(Entity, &mut Flash, &mut PointLight)>, mut commands: Commands) {
    let dt = time.delta_secs();
    for (e, mut f, mut l) in &mut q {
        f.life -= dt;
        if f.life <= 0.0 { commands.entity(e).despawn(); continue }
        let k = f.life / f.max;
        l.intensity = f.peak * k * k;
    }
}

/// the streaks: each FUEL moving fast off the carpet, where it's been the last TRAIL s
const TRAIL: f32 = 0.3;
fn trails(game: Res<Game>, time: Res<Time>, settings: Res<Settings>, mut hist: Local<HashMap<usize, VecDeque<(Vec3, f32)>>>, mut gizmos: Gizmos) {
    if !matches!(settings.motion.as_str(), "trails" | "both") { hist.clear(); return }
    let now = time.elapsed_secs();
    let w = &game.0;
    for (i, b) in w.fuel.balls.iter().enumerate() {
        let Some(body) = w.physics.bodies.get(b.body) else { continue };
        let (p, v) = (body.translation(), body.linvel());
        let fast = body.is_enabled() && v.length() > 4.0 && p.y > 0.25;
        let h = hist.entry(i).or_default();
        if fast { h.push_back((p, now)); }
        while h.front().is_some_and(|(_, t)| now - t > TRAIL) { h.pop_front(); }
        if h.len() < 2 { continue }
        gizmos.linestrip_gradient(h.iter().map(|(q, t)| {
            let a = (1.0 - (now - t) / TRAIL).clamp(0.0, 1.0);
            (*q, Color::srgba(1.0, 0.85, 0.2, 0.55 * a))
        }));
    }
    hist.retain(|_, h| !h.is_empty());
}
