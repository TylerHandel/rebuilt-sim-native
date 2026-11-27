//! js wallCutaway.js: the alliance wall fades out when it's about to hide your robot. As the line
//! from the camera to the robot comes down toward the top of an ALLIANCE WALL's solid part (a
//! Driver Station's base under its glass, the TOWER WALL, the OUTPOST), that whole end of the field
//! (the wall and everything behind it, from just in front of its face) fades away, over a short
//! ramp: starting when the line passes FADE_START over the solid part's top, gone by FADE_END, so
//! it's all the way out by the time it would block anything. A screen-door fade (fragments dropped
//! in a fine ordered pattern, assets/shaders/worn.wesl and worn_prepass.wesl) on the field's
//! materials (materials.rs FieldPart; never the robots or the FUEL), so the CAD's big merged opaque
//! meshes don't need sorting as transparent ones would.

use bevy::prelude::*;
use sim::constants::{ALLIANCE_WALL_H, DS_BASE_H, DS_SPLIT_FY, HALF_L, HALF_W, OUTPOST, TOWER};

use crate::camera::CameraRig;
use crate::game::Game;
use crate::materials::{FieldMats, WornMaterial};

/// m the line to the robot passes over the solid part's top: starts to fade
const FADE_START: f32 = 0.35;
/// and gone
const FADE_END: f32 = 0.1;
/// what fades: from this far in front of the wall's face back
const BAND: f32 = 0.3;
/// how fast it follows (1/s)
const RATE: f32 = 6.0;

pub struct CutawayPlugin;

impl Plugin for CutawayPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Cutaway>()
            .add_systems(PostUpdate, update.after(crate::camera::place).before(TransformSystems::Propagate));
    }
}

/// a stretch of an ALLIANCE WALL that can't be seen through (world x, z0..z1, top h)
struct Span { x: f32, z0: f32, z1: f32, h: f32 }

#[derive(Resource)]
pub struct Cutaway { spans: Vec<Span>, fade: f32, side: f32, sent: (f32, f32) }

impl Default for Cutaway {
    /// js field.js _buildSide: the Driver Stations' bases under their glass, the TOWER WALL and the
    /// OUTPOST, each end
    fn default() -> Self {
        let z_of = |fy: f64| (HALF_W - fy) as f32;
        let out_edge = z_of(OUTPOST::fy + OUTPOST::structW / 2.0);
        let tz = z_of(TOWER::fy);
        let (tz0, tz1) = (tz - TOWER::width as f32 / 2.0, tz + TOWER::width as f32 / 2.0);
        let (ds, wall) = (DS_BASE_H as f32, ALLIANCE_WALL_H as f32);
        let hw = HALF_W as f32;
        let segs = [(-hw, z_of(DS_SPLIT_FY), ds), (z_of(DS_SPLIT_FY), tz0, ds), (tz0, tz1, wall), (tz1, out_edge, ds), (out_edge, hw, wall)];
        let mut spans = Vec::new();
        for s in [1.0f32, -1.0] {
            let x = -s * HALF_L as f32;
            for (z0, z1, h) in segs { spans.push(Span { x, z0: (s * z0).min(s * z1), z1: (s * z0).max(s * z1), h }); }
        }
        Cutaway { spans, fade: 0.0, side: 1.0, sent: (-1.0, 0.0) }
    }
}

impl Cutaway {
    /// js `_block`: how far below the top of a wall's solid part the line from the camera to the
    /// robot crosses it (m; negative: over it), and which end
    fn block(&self, c: Vec3, r: Vec3) -> (f32, f32) {
        let (mut best, mut side) = (f32::NEG_INFINITY, 0.0);
        for y in [0.12f32, 0.3, 0.5] {
            for sp in &self.spans {
                let dx = r.x - c.x;
                if dx.abs() < 1e-6 { continue }
                let t = (sp.x - c.x) / dx;
                if t <= 0.0 || t >= 1.0 { continue }
                let z = c.z + (r.z - c.z) * t;
                let yy = c.y + (y - c.y) * t;
                if z < sp.z0 || z > sp.z1 { continue }
                if sp.h - yy > best { best = sp.h - yy; side = sp.x.signum(); }
            }
        }
        (best, side)
    }
}

fn update(
    mut cut: ResMut<Cutaway>,
    rig: Res<CameraRig>,
    game: Res<Game>,
    time: Res<Time>,
    field: Res<FieldMats>,
    mut worn: ResMut<Assets<WornMaterial>>,
) {
    let robot = game.0.mine().and_then(|r| game.0.physics.bodies.get(r.body)).map(|b| b.translation());
    let mut want = 0.0;
    if let Some(r) = robot {
        let (b, side) = cut.block(rig.pos, r);
        if side != 0.0 {
            // (b = -FADE_START: just starting; b = -FADE_END: gone)
            want = ((b + FADE_START) / (FADE_START - FADE_END)).clamp(0.0, 1.0);
            if want > 0.0 { cut.side = side; }
        }
    }
    // (screenshots: REBUILT_SHOT_CUT=1 cuts the end nearest the camera away regardless)
    if std::env::var("REBUILT_SHOT_CUT").is_ok_and(|v| v == "1") { want = 1.0; cut.side = if rig.pos.x < 0.0 { -1.0 } else { 1.0 }; }
    let dt = time.delta_secs();
    cut.fade += (want - cut.fade) * (RATE * dt).min(1.0);
    if cut.fade < 0.01 && want == 0.0 { cut.fade = 0.0; }
    // (the field's materials told only while it's fading; their eye, which pass is the camera's,
    // each frame while it's out)
    let (f, s) = (cut.fade, cut.side);
    if f == 0.0 && cut.sent.0 == 0.0 { return }
    cut.sent = (f, s);
    let v = Vec4::new(f, s, HALF_L as f32 - BAND, 0.0);
    let eye = rig.pos.extend(1.0);
    for h in &field.mats {
        // (w: its scratches, kept)
        if let Some(mut m) = worn.get_mut(h) { m.extension.cut = v.with_w(m.extension.cut.w); m.extension.eye = eye; }
    }
}
