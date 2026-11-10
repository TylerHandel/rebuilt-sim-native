//! The nets over a robot's load (js robotModels.js hopperNet): a grid pinned at its edges, held up
//! by the FUEL under it, each free point at its neighbors' average (a taut sheet; a slack one hangs a
//! little below them). Its shape comes from the rig (tools/export-models.mjs: userData.netSpec), and
//! how it's draped (how far its front is out, its edges' heights, whether it's shown) from the
//! robot's state the way the rig's poses are: the args at rest, and what each input changes.

use bevy::asset::RenderAssetUsages;
use bevy::image::{ImageAddressMode, ImageSampler, ImageSamplerDescriptor};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use serde::Deserialize;
use std::collections::HashMap;

const R: f64 = sim::constants::FUEL::radius;

#[derive(Deserialize, Clone)]
pub struct Slack { pub sag: f64, #[serde(rename = "sagOut")] pub sag_out: Option<f64> }
#[derive(Deserialize, Clone)]
pub struct Hole { pub x: f64, pub z: f64, pub r: f64, pub y: Option<f64> }
#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct NetSpec {
    pub x0: f64, pub x1: f64, pub x1_out: f64, pub x_fixed: f64, pub hw: f64, pub y: f64, pub y_front: f64,
    #[serde(rename = "NX")] pub nx: usize,
    #[serde(rename = "NZ")] pub nz: usize,
    pub slack: Option<Slack>, pub hole: Option<Hole>, pub color: Option<u32>,
}
/// js drapeArgs per input: [ex, the back edge's height, the front's, xBack, shown]
#[derive(Deserialize, Clone)]
pub struct NetBy { pub samples: Vec<f64>, pub rows: Vec<[Option<f64>; 5]> }
#[derive(Deserialize, Clone)]
pub struct RigNet { #[allow(dead_code)] pub name: String, pub parent: String, pub spec: NetSpec, pub rest: [Option<f64>; 5], pub by: HashMap<String, NetBy> }

impl RigNet {
    /// its drape's args for the robot's state: rest, and what each input it has moves (the inputs
    /// in `skip` left out: others that take over from them have their say)
    pub fn args(&self, inputs: &[(String, f32)], taken: &dyn Fn(&str) -> bool) -> [Option<f64>; 5] {
        let mut a = self.rest;
        for (name, v) in inputs {
            let Some(by) = self.by.get(name.as_str()) else { continue };
            if taken(name) { continue; }
            let row = lerp(&by.samples, &by.rows, *v as f64);
            for j in 0..5 { if let (Some(x), Some(r0), Some(ai)) = (row[j], self.rest[j], a[j]) { a[j] = Some(ai + x - r0); } else if self.rest[j].is_none() && row[j].is_some() { a[j] = row[j]; } }
        }
        a
    }
}
fn lerp(s: &[f64], rows: &[[Option<f64>; 5]], v: f64) -> [Option<f64>; 5] {
    let n = s.len();
    if v <= s[0] { return rows[0]; }
    if v >= s[n - 1] { return rows[n - 1]; }
    let k = s.partition_point(|x| *x <= v).clamp(1, n - 1);
    let t = (v - s[k - 1]) / (s[k] - s[k - 1]);
    let mut out = [None; 5];
    for j in 0..5 { out[j] = match (rows[k - 1][j], rows[k][j]) { (Some(a), Some(b)) => Some(a + (b - a) * t), (a, _) => a }; }
    out
}

/// A net as it's draped now
#[derive(Component)]
pub struct Net {
    spec: NetSpec,
    base: Vec<f64>, pin: Vec<bool>, h_now: Vec<f64>, h_min: Vec<f64>, h_pin: Vec<f64>, hang_mid: f64,
    pos: Vec<[f32; 3]>,
    mesh: Handle<Mesh>,
    last: Option<(Vec<[f64; 3]>, [Option<f64>; 4])>,
}

impl Net {
    fn in_hole(&self, x: f64, z: f64, pad: f64) -> bool { self.spec.hole.as_ref().is_some_and(|h| (x - h.x).hypot(z - h.z) < h.r + pad) }

    /// js hopperNet(parent, spec): the grid, its pins, its mesh
    pub fn new(spec: NetSpec, meshes: &mut Assets<Mesh>) -> (Self, Handle<Mesh>) {
        let (nx, nz) = (spec.nx, spec.nz);
        let nv = (nx + 1) * (nz + 1);
        let mut me = Net { spec: spec.clone(), base: vec![0.0; nv], pin: vec![false; nv], h_now: vec![spec.y; nv], h_min: vec![0.0; nv], h_pin: vec![0.0; nv], hang_mid: 1.0, pos: vec![[0.0; 3]; nv], mesh: Handle::default(), last: None };
        let mut uv = vec![[0.0f32; 2]; nv];
        for i in 0..=nx {
            for j in 0..=nz {
                let k = i * (nz + 1) + j;
                let x = spec.x0 + ((spec.x1_out - spec.x0) * i as f64) / nx as f64;
                let z = -spec.hw + (2.0 * spec.hw * j as f64) / nz as f64;
                me.base[k] = x;
                me.pin[k] = i == 0 || i == nx || j == 0 || j == nz || me.in_hole(x, z, 0.01);
                me.pos[k] = [x as f32, spec.y as f32, z as f32];
                // (the texture repeats 10 times a meter, as the game's)
                uv[k] = [(x * 10.0) as f32, (z * 10.0) as f32];
            }
        }
        let mut idx: Vec<u32> = Vec::new();
        for i in 0..nx {
            for j in 0..nz {
                let k = (i * (nz + 1) + j) as u32;
                let cx = spec.x0 + ((spec.x1_out - spec.x0) * (i as f64 + 0.5)) / nx as f64;
                let cz = -spec.hw + (2.0 * spec.hw * (j as f64 + 0.5)) / nz as f64;
                if me.in_hole(cx, cz, 0.0) { continue; }
                let n1 = (nz + 1) as u32;
                idx.extend([k, k + 1, k + n1 + 1, k, k + n1 + 1, k + n1]);
            }
        }
        // (a slack net: how it hangs under its own weight, for a drop of 1 at its middle)
        if let Some(sl) = &spec.slack {
            let mut hang = vec![0.0; nv];
            for _ in 0..400 {
                for i in 1..nx {
                    for j in 1..nz {
                        let k = i * (nz + 1) + j;
                        if me.pin[k] { continue; }
                        hang[k] += 1.9 * ((hang[k - 1] + hang[k + 1] + hang[k - nz - 1] + hang[k + nz + 1]) / 4.0 + 1.0 - hang[k]);
                    }
                }
            }
            me.hang_mid = hang.iter().copied().fold(f64::NEG_INFINITY, f64::max).max(1e-9);
            for k in 0..nv { me.h_now[k] = spec.y - (sl.sag * hang[k]) / me.hang_mid; }
        }
        let mut m = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
        m.insert_attribute(Mesh::ATTRIBUTE_POSITION, me.pos.clone());
        m.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0f32, 1.0, 0.0]; nv]);
        m.insert_attribute(Mesh::ATTRIBUTE_UV_0, uv);
        m.insert_indices(Indices::U32(idx));
        let h = meshes.add(m);
        me.mesh = h.clone();
        (me, h)
    }

    /// js `drape(load, ex, at)`: load, the FUEL under it (the robot's frame)
    pub fn drape(&mut self, load: &[[f64; 3]], ex: f64, at_y: Option<f64>, at_yf: Option<f64>, at_xb: Option<f64>, meshes: &mut Assets<Mesh>) {
        // (nothing that's moved a millimeter: nothing to do)
        if let Some((l, a)) = &self.last {
            if a == &[Some(ex), at_y, at_yf, at_xb] && l.len() == load.len() && l.iter().zip(load).all(|(p, q)| (0..3).all(|j| (p[j] - q[j]).abs() <= 0.001)) { return; }
        }
        self.last = Some((load.to_vec(), [Some(ex), at_y, at_yf, at_xb]));
        let s = self.spec.clone();
        let (nx, nz) = (s.nx, s.nz);
        let nv = self.pos.len();
        let kx = if s.x1_out > s.x_fixed { (s.x1 + ex - s.x_fixed) / (s.x1_out - s.x_fixed) } else { 1.0 };
        let (yb, yf, xf) = (at_y.unwrap_or(s.y), at_yf.unwrap_or(s.y_front), s.x1 + ex);
        let xb = match at_xb { Some(v) if v > s.x0 => v.min(xf - 0.02), _ => s.x0 };
        for k in 0..nv {
            let mut x = if self.base[k] <= s.x_fixed { self.base[k] } else { s.x_fixed + (self.base[k] - s.x_fixed) * kx };
            if xb > s.x0 { x = xb + ((x - s.x0) * (xf - xb)) / (xf - s.x0); }
            self.pos[k][0] = x as f32;
            self.h_pin[k] = yb + (yf - yb) * ((x - s.x0) / (xf - s.x0).max(0.01)).clamp(0.0, 1.0);
            if let Some(hy) = s.hole.as_ref().and_then(|h| h.y) { if self.in_hole(x, self.pos[k][2] as f64, 0.01) { self.h_pin[k] = hy; } }
            self.h_min[k] = if s.slack.is_some() { f64::NEG_INFINITY } else { self.h_pin[k] };
        }
        let sag = s.slack.as_ref().map_or(0.0, |sl| sl.sag + (sl.sag_out.unwrap_or(sl.sag) - sl.sag) * (if s.x1_out > s.x1 { ex / (s.x1_out - s.x1) } else { 0.0 }).clamp(0.0, 1.0));
        let drop = sag / self.hang_mid;
        for p in load {
            if p[1] + R < yb.min(yf) - sag { continue; }
            for i in 0..=nx {
                let rx = self.pos[i * (nz + 1)][0] as f64 - p[0];
                if rx >= R || rx <= -R { continue; }
                for k in i * (nz + 1)..(i + 1) * (nz + 1) {
                    if self.pin[k] { continue; }
                    let (dx, dz) = (self.pos[k][0] as f64 - p[0], self.pos[k][2] as f64 - p[2]);
                    let d2 = dx * dx + dz * dz;
                    if d2 < R * R { self.h_min[k] = self.h_min[k].max(p[1] + (R * R - d2).sqrt() + 0.004); }
                }
            }
        }
        for _ in 0..10 {
            for i in 1..nx {
                for j in 1..nz {
                    let k = i * (nz + 1) + j;
                    if self.pin[k] { continue; }
                    self.h_now[k] = self.h_min[k].max((self.h_now[k - 1] + self.h_now[k + 1] + self.h_now[k - nz - 1] + self.h_now[k + nz + 1]) / 4.0 - drop);
                }
            }
        }
        for k in 0..nv { self.pos[k][1] = (if self.pin[k] { self.h_pin[k] } else { self.h_now[k] }) as f32; }
        // (its normals across the grid, by its neighbors either side)
        let mut nrm = vec![[0.0f32; 3]; nv];
        for i in 0..=nx {
            let (ia, ib) = (i.saturating_sub(1) * (nz + 1), (i + 1).min(nx) * (nz + 1));
            for j in 0..=nz {
                let k = i * (nz + 1) + j;
                let (ja, jb) = (i * (nz + 1) + j.saturating_sub(1), i * (nz + 1) + (j + 1).min(nz));
                let u = Vec3::from(self.pos[ib + j]) - Vec3::from(self.pos[ia + j]);
                let v = Vec3::from(self.pos[jb]) - Vec3::from(self.pos[ja]);
                nrm[k] = v.cross(u).normalize_or(Vec3::Y).into();
            }
        }
        if let Some(mut m) = meshes.get_mut(&self.mesh) {
            m.insert_attribute(Mesh::ATTRIBUTE_POSITION, self.pos.clone());
            m.insert_attribute(Mesh::ATTRIBUTE_NORMAL, nrm);
        }
    }
}

/// js fineNetTexture(): a fine mesh, its cords and knots white (tinted by the material's color)
pub fn net_texture(images: &mut Assets<Image>) -> Handle<Image> {
    let n = 128usize;
    let mut px = vec![0u8; n * n * 4];
    let mut put = |x: usize, y: usize, a: u8| { let o = (y * n + x) * 4; px[o] = 255; px[o + 1] = 255; px[o + 2] = 255; px[o + 3] = px[o + 3].max(a); };
    for y in 0..n {
        for x in 0..n {
            let cord = |c: usize| [1.0f32, 65.0].iter().any(|p| ((c as f32 + 0.5) - p).abs() < 1.1);
            if cord(x) || cord(y) { put(x, y, 242); }
            let knot = |c: usize| [1.0f32, 65.0].iter().any(|p| ((c as f32 + 0.5) - p).abs() < 2.5);
            if knot(x) && knot(y) { put(x, y, 255); }
        }
    }
    let mut img = Image::new(Extent3d { width: n as u32, height: n as u32, depth_or_array_layers: 1 }, TextureDimension::D2, px, TextureFormat::Rgba8UnormSrgb, RenderAssetUsages::RENDER_WORLD);
    img.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor { address_mode_u: ImageAddressMode::Repeat, address_mode_v: ImageAddressMode::Repeat, anisotropy_clamp: 8, ..ImageSamplerDescriptor::linear() });
    images.add(img)
}
