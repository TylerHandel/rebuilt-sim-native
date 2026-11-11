//! As the Motion effects setting asks: a fading streak behind each FUEL in flight. (Nothing the real
//! game doesn't have: a HUB shows FUEL going in with its own lights, lights.rs.)

use std::collections::{HashMap, VecDeque};

use bevy::prelude::*;

use crate::game::Game;
use crate::settings::Settings;

pub struct EffectsPlugin;

impl Plugin for EffectsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, trails);
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
