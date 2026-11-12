//! Outdoors, the field's in the countryside instead of a stadium: mowed turf round it (scene.rs),
//! then a meadow of grass blades bending in the wind, trees (grown here: trunk, branches, leaf
//! cards), a treeline of cut-out trees beyond them, and hills rolling off into the haze. All of it
//! made here at the start (no models to load), and as much as the Graphics preset says: none on
//! Low; on Medium sparse grass close by and trees without shadows; denser and further on High and
//! Ultra, and the trees' shadows.
//!
//! The wind's in the foliage's vertex shader (assets/shaders/foliage.wesl): grass blades and leaves
//! bend by how far up them they are (uv_b.x) and how stiff they are (uv_b.y); the trees sway as a
//! whole (their transforms). Foliage has no depth prepass (it'd be drawn there unbent), and its
//! shadows don't sway.

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::VisibilityRange;
use bevy::light::NotShadowCaster;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::pbr::{ExtendedMaterial, MaterialExtension};
use bevy::prelude::*;
use bevy::render::render_resource::{AsBindGroup, Extent3d, TextureDimension, TextureFormat};
use bevy::shader::ShaderRef;

use crate::arena::{fbm, hash, noise, Look, Tod, Weather, TOWERS};
use crate::settings::Settings;

/// The foliage's material: standard PBR, bent by the wind
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone)]
pub struct Foliage {
    /// how hard the wind blows (foliage.wesl)
    #[uniform(100)]
    pub wind: f32,
}

impl MaterialExtension for Foliage {
    fn vertex_shader() -> ShaderRef { "shaders/foliage.wesl".into() }
    fn enable_prepass() -> bool { false }
}

pub type FoliageMaterial = ExtendedMaterial<StandardMaterial, Foliage>;

pub struct NaturePlugin;

impl Plugin for NaturePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<FoliageMaterial>::default())
            .init_resource::<Built>()
            .add_systems(Update, (build, weather, sway).chain());
    }
}

/// The countryside's root (shown outdoors, from Medium up)
#[derive(Component)]
struct Nature;

/// A tree: where it stands, how it sways
#[derive(Component)]
struct Tree { base: Transform, phase: f32, give: f32 }

/// What's been made, for which preset; its materials (the weather wets them)
#[derive(Resource, Default)]
struct Built {
    preset: Option<String>,
    root: Option<Entity>,
    foliage: Vec<(Handle<FoliageMaterial>, LinearRgba)>,
    plain: Vec<(Handle<StandardMaterial>, LinearRgba, f32)>,
    weather: Option<(Weather, Tod)>,
}

/// How much there is of it on each preset: grass blades in a 2 m patch, how far the grass reaches
/// from the camera (m), how many trees
struct Amount { blades: usize, reach: f32, trees: usize, shadows: bool }

fn amount(g: &str) -> Amount {
    match g {
        "ultra" => Amount { blades: 520, reach: 46.0, trees: 110, shadows: true },
        "high" => Amount { blades: 340, reach: 36.0, trees: 80, shadows: true },
        _ => Amount { blades: 140, reach: 26.0, trees: 45, shadows: false },
    }
}

/// The ground's height (m): flat round the field, rising into hills further out
pub fn ground_h(x: f32, z: f32) -> f32 {
    let r = x.hypot(z);
    let near = smooth(48.0, 150.0, r);
    let far = smooth(180.0, 700.0, r);
    near * (2.0 + 16.0 * fbm(x / 95.0 + 3.0, z / 95.0 - 7.0)) + far * 55.0 * fbm(x / 260.0 - 11.0, z / 260.0 + 4.0)
}

fn smooth(a: f32, b: f32, x: f32) -> f32 { let t = ((x - a) / (b - a)).clamp(0.0, 1.0); t * t * (3.0 - 2.0 * t) }

/// Where the field's people and tables are: no grass there (x, z half-sizes, m)
const CLEAR: (f32, f32) = (12.5, 10.0);

/// A seeded random sequence (the same countryside every time)
struct Rng(u32);
impl Rng {
    fn f(&mut self) -> f32 { self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223); (self.0 >> 8) as f32 / 16_777_216.0 }
    fn range(&mut self, a: f32, b: f32) -> f32 { a + (b - a) * self.f() }
}

/// Mesh being built: positions, normals, uv, uv_b (bend, stiffness), colors, indices
#[derive(Default)]
struct Geo { p: Vec<[f32; 3]>, n: Vec<[f32; 3]>, uv: Vec<[f32; 2]>, uvb: Vec<[f32; 2]>, c: Vec<[f32; 4]>, i: Vec<u32> }

impl Geo {
    fn v(&mut self, p: Vec3, n: Vec3, uv: Vec2, uvb: Vec2, c: [f32; 4]) -> u32 {
        self.p.push(p.into()); self.n.push(n.normalize_or(Vec3::Y).into()); self.uv.push(uv.into()); self.uvb.push(uvb.into()); self.c.push(c);
        (self.p.len() - 1) as u32
    }
    fn tri(&mut self, a: u32, b: u32, c: u32) { self.i.extend([a, b, c]); }
    fn mesh(self, colors: bool, bend: bool) -> Mesh {
        // (kept in the main world too: other systems look meshes over after they're in)
        let mut m = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
        m.insert_attribute(Mesh::ATTRIBUTE_POSITION, self.p);
        m.insert_attribute(Mesh::ATTRIBUTE_NORMAL, self.n);
        m.insert_attribute(Mesh::ATTRIBUTE_UV_0, self.uv);
        if bend { m.insert_attribute(Mesh::ATTRIBUTE_UV_1, self.uvb); }
        if colors { m.insert_attribute(Mesh::ATTRIBUTE_COLOR, self.c); }
        m.insert_indices(Indices::U32(self.i));
        m
    }
}

fn lin(r: f32, g: f32, b: f32) -> [f32; 4] { [r, g, b, 1.0] }

/// A 2 m patch of meadow grass: `n` blades, each a bent strip of two segments and a tip, darker at
/// the root; a few dry ones
fn grass_patch(n: usize, seed: u32) -> Mesh {
    let mut g = Geo::default();
    let mut r = Rng(seed);
    for _ in 0..n {
        let base = Vec3::new(r.range(-1.0, 1.0), 0.0, r.range(-1.0, 1.0));
        let h = r.range(0.16, 0.42) * if r.f() < 0.1 { 1.5 } else { 1.0 };
        // (half-widths of 1.5-3 cm: a meadow's broad blades, and enough to read from the stands)
        let w = r.range(0.015, 0.03);
        let yaw = r.range(0.0, std::f32::consts::TAU);
        let side = Vec3::new(yaw.cos(), 0.0, yaw.sin());
        let lean_dir = Vec3::new(-yaw.sin(), 0.0, yaw.cos());
        let lean = r.range(0.05, 0.35);
        let stiff = r.range(0.7, 1.25);
        let dry = r.f() < 0.07;
        let tone = r.range(0.75, 1.2);
        let (root, tip) = if dry { (lin(0.04, 0.035, 0.012), lin(0.2, 0.16, 0.06)) } else { (lin(0.008, 0.025, 0.004), lin(0.06 * tone, 0.16 * tone, 0.025 * tone)) };
        let col = |t: f32| -> [f32; 4] { let mut c = [0.0; 4]; for k in 0..4 { c[k] = root[k] + (tip[k] - root[k]) * t; } c };
        // (the blade's normal leans up, so the meadow's lit as a surface, not as edge-on strips)
        let nrm = (lean_dir * 0.4 + Vec3::Y).normalize();
        let at = |t: f32| base + Vec3::Y * h * t + lean_dir * lean * h * t * t;
        let a0 = g.v(at(0.0) - side * w, nrm, Vec2::ZERO, Vec2::new(0.0, stiff), col(0.0));
        let a1 = g.v(at(0.0) + side * w, nrm, Vec2::ZERO, Vec2::new(0.0, stiff), col(0.0));
        let b0 = g.v(at(0.55) - side * w * 0.75, nrm, Vec2::ZERO, Vec2::new(0.55, stiff), col(0.55));
        let b1 = g.v(at(0.55) + side * w * 0.75, nrm, Vec2::ZERO, Vec2::new(0.55, stiff), col(0.55));
        let tp = g.v(at(1.0), nrm, Vec2::ZERO, Vec2::new(1.0, stiff), col(1.0));
        g.tri(a0, a1, b1); g.tri(a0, b1, b0); g.tri(b0, b1, tp);
    }
    g.mesh(true, true)
}

/// The ground beyond the turf: rings out to 900 m, the hills' heights, a meadow's greens
fn terrain() -> Mesh {
    let mut g = Geo::default();
    let (rings, segs) = (56usize, 160usize);
    let r_at = |i: usize| 27.0 * (900.0f32 / 27.0).powf(i as f32 / (rings - 1) as f32);
    let h = |x: f32, z: f32| ground_h(x, z) - if x.hypot(z) < 31.0 { 0.03 } else { 0.0 };
    for i in 0..rings {
        let rr = r_at(i);
        for s in 0..segs {
            let a = s as f32 / segs as f32 * std::f32::consts::TAU;
            let (x, z) = (rr * a.cos(), rr * a.sin());
            let e = 0.5;
            let n = Vec3::new(h(x - e, z) - h(x + e, z), 2.0 * e, h(x, z - e) - h(x, z + e));
            let v = fbm(x / 23.0, z / 23.0);
            let m = noise(x / 7.0 + 40.0, z / 7.0);
            let far = smooth(150.0, 800.0, rr);
            // (a meadow's greens, browner where it's patchy; the far hills a little bluer)
            let c = [0.018 + 0.02 * m, 0.07 + 0.04 * v - 0.01 * far, 0.012 + 0.008 * m + 0.012 * far, 1.0];
            g.v(Vec3::new(x, h(x, z), z), n, Vec2::new(x, z) / 9.0, Vec2::ZERO, c);
        }
    }
    for i in 0..rings - 1 {
        for s in 0..segs {
            let (a, b) = ((i * segs + s) as u32, (i * segs + (s + 1) % segs) as u32);
            let (c, d) = (a + segs as u32, b + segs as u32);
            // (wound counter-clockwise seen from above)
            g.tri(a, d, c); g.tri(a, b, d);
        }
    }
    g.mesh(true, false)
}

/// An alpha-cut texture with mips that keep their coverage: each smaller mip's alpha scaled so as
/// many of its texels pass the cut (0.5) as the full-size one's (plain averaging thins a leaf card
/// to nothing far off); the clear texels take the cut-out's average color (no dark fringes)
fn cutout(w: usize, h: usize, mut px: Vec<u8>) -> Image {
    use bevy::image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
    let cover = |p: &[u8]| p.chunks(4).filter(|c| c[3] >= 128).count() as f32 / (p.len() / 4) as f32;
    let (mut sum, mut n) = ([0u64; 3], 0u64);
    for c in px.chunks(4) { if c[3] >= 128 { for k in 0..3 { sum[k] += c[k] as u64; } n += 1; } }
    let avg: Vec<u8> = (0..3).map(|k| (sum[k] / n.max(1)) as u8).collect();
    for c in px.chunks_mut(4) { if c[3] < 128 { c[0] = avg[0]; c[1] = avg[1]; c[2] = avg[2]; } }
    let c0 = cover(&px);
    let mut data = px.clone();
    let (mut cw, mut ch, mut cur) = (w, h, px);
    let mut levels = 1;
    while cw > 1 || ch > 1 {
        let (nw, nh) = ((cw / 2).max(1), (ch / 2).max(1));
        let mut next = vec![0u8; nw * nh * 4];
        for y in 0..nh { for x in 0..nw { for k in 0..4 {
            let at = |xx: usize, yy: usize| cur[((yy.min(ch - 1)) * cw + xx.min(cw - 1)) * 4 + k] as u32;
            next[(y * nw + x) * 4 + k] = ((at(2 * x, 2 * y) + at(2 * x + 1, 2 * y) + at(2 * x, 2 * y + 1) + at(2 * x + 1, 2 * y + 1) + 2) / 4) as u8;
        } } }
        // (the alpha scale that gives back the full-size coverage: by bisection)
        let (mut lo, mut hi) = (1.0f32, 8.0f32);
        for _ in 0..16 {
            let mid = 0.5 * (lo + hi);
            let c = next.chunks(4).filter(|c| c[3] as f32 * mid >= 128.0).count() as f32 / (nw * nh) as f32;
            if c < c0 { lo = mid } else { hi = mid }
        }
        let mut out = next.clone();
        for c in out.chunks_mut(4) { c[3] = (c[3] as f32 * hi).min(255.0) as u8; }
        data.extend_from_slice(&out);
        (cw, ch, cur) = (nw, nh, next);
        levels += 1;
    }
    let mut img = Image::new(Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 }, TextureDimension::D2, data, TextureFormat::Rgba8UnormSrgb, RenderAssetUsages::RENDER_WORLD);
    img.texture_descriptor.mip_level_count = levels;
    img.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::ClampToEdge, address_mode_v: ImageAddressMode::ClampToEdge,
        mag_filter: ImageFilterMode::Linear, min_filter: ImageFilterMode::Linear, mipmap_filter: ImageFilterMode::Linear,
        anisotropy_clamp: 8,
        ..default()
    });
    img
}

/// Leaf clusters on a card (256 square): a few dozen leaves, each a pointed oval with a midrib,
/// in a spread of greens; clear between them (alpha cut)
fn leaf_texture(needles: bool) -> Image {
    let n = 256usize;
    let mut px = vec![0u8; n * n * 4];
    let mut r = Rng(if needles { 77 } else { 31 });
    // (dense: about half the card's leaf, as a twig's cluster is)
    let count = if needles { 420 } else { 150 };
    for _ in 0..count {
        let (cx, cy) = (r.range(0.12, 0.88) * n as f32, r.range(0.12, 0.88) * n as f32);
        let ang = r.range(0.0, std::f32::consts::TAU);
        let (len, wid) = if needles { (r.range(14.0, 26.0), 1.6) } else { (r.range(18.0, 30.0), r.range(7.0, 11.0)) };
        let tone = r.range(0.7, 1.15);
        let base = if needles { [28.0, 62.0, 34.0] } else { [58.0, 104.0, 30.0] };
        let (ca, sa) = (ang.cos(), ang.sin());
        let reach = len as i32 + 2;
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                let (x, y) = (cx as i32 + dx, cy as i32 + dy);
                if x < 0 || y < 0 || x >= n as i32 || y >= n as i32 { continue }
                let (u, v) = ((dx as f32 * ca + dy as f32 * sa) / len, (-dx as f32 * sa + dy as f32 * ca) / wid);
                // (a leaf: widest a third of the way along, pointed at the tip)
                let w = (1.0 - u * u).max(0.0) * (1.0 - 0.35 * u);
                if u.abs() > 1.0 || v.abs() > w { continue }
                let rib = if !needles && v.abs() < 0.12 { 1.18 } else { 1.0 };
                let shade = (0.8 + 0.25 * u) * tone * rib * (1.0 - 0.25 * v.abs());
                let i = (y as usize * n + x as usize) * 4;
                for k in 0..3 { px[i + k] = (base[k] * shade).clamp(0.0, 255.0) as u8; }
                px[i + 3] = 255;
            }
        }
    }
    cutout(n, n, px)
}

/// The treeline's trees, cut out (4 in a row, each 128 x 256): two broad crowns, two spires
fn treeline_texture() -> Image {
    let (w, h) = (512usize, 256usize);
    let mut px = vec![0u8; w * h * 4];
    for cell in 0..4 {
        let spire = cell >= 2;
        let mut r = Rng(900 + cell as u32);
        for y in 0..h {
            for x in 0..128usize {
                let (u, v) = (x as f32 / 128.0 - 0.5, 1.0 - y as f32 / h as f32);
                let trunk = u.abs() < 0.035 && v < 0.25;
                let crown = if spire {
                    // (a spire: tiers that narrow to the tip)
                    let tier = (v * 7.0).fract();
                    v > 0.12 && u.abs() < (1.0 - v) * 0.42 * (0.75 + 0.25 * (1.0 - tier))
                } else {
                    // (a crown: lumpy, wider than tall)
                    let (dx, dy) = (u / 0.46, (v - 0.58) / 0.4);
                    let lump = 0.18 * noise(u * 9.0 + cell as f32 * 7.0, v * 9.0);
                    v > 0.2 && dx * dx + dy * dy < 1.0 - lump
                };
                if !(trunk || crown) { continue }
                let i = (y * w + cell * 128 + x) * 4;
                let n = 0.75 + 0.5 * noise(x as f32 / 6.0 + cell as f32 * 30.0, y as f32 / 6.0) + 0.1 * r.f();
                let c = if crown && !trunk { if spire { [20.0, 40.0, 26.0] } else { [36.0, 64.0, 24.0] } } else { [40.0, 32.0, 24.0] };
                // (lit from above: the top lighter)
                let lit = n * (0.7 + 0.5 * v);
                for k in 0..3 { px[i + k] = (c[k] * lit).clamp(0.0, 255.0) as u8; }
                px[i + 3] = 255;
            }
        }
    }
    cutout(w, h, px)
}

/// A tapered limb from a to b (radii ra, rb): 6 sides
fn limb(g: &mut Geo, a: Vec3, b: Vec3, ra: f32, rb: f32, bark: [f32; 4]) {
    let d = (b - a).normalize_or(Vec3::Y);
    let s = d.any_orthonormal_vector();
    let t = d.cross(s);
    let base = g.p.len() as u32;
    for (p, rad, vv) in [(a, ra, 0.0), (b, rb, 1.0)] {
        for k in 0..6 {
            let ang = k as f32 / 6.0 * std::f32::consts::TAU;
            let o = s * ang.cos() + t * ang.sin();
            g.v(p + o * rad, o, Vec2::new(k as f32 / 6.0, vv), Vec2::ZERO, bark);
        }
    }
    for k in 0..6u32 {
        let k1 = (k + 1) % 6;
        // (wound counter-clockwise seen from outside)
        g.tri(base + k, base + 6 + k1, base + 6 + k);
        g.tri(base + k, base + k1, base + 6 + k1);
    }
}

/// A leaf card at p, facing about n, `size` across: bends with the wind (uv_b)
fn card(g: &mut Geo, p: Vec3, n: Vec3, up: Vec3, size: f32, bend: f32) {
    let n = n.normalize_or(Vec3::Y);
    let s = up.cross(n).normalize_or(Vec3::X) * size * 0.5;
    let u = n.cross(s).normalize_or(Vec3::Y) * size * 0.5;
    // (its normal half toward up and out, so the crown's lit as a volume)
    let nn = (n + Vec3::Y * 0.6).normalize();
    let c = [1.0, 1.0, 1.0, 1.0];
    let a = g.v(p - s - u, nn, Vec2::new(0.0, 1.0), Vec2::new(bend, 0.35), c);
    let b = g.v(p + s - u, nn, Vec2::new(1.0, 1.0), Vec2::new(bend, 0.35), c);
    let cc = g.v(p + s + u, nn, Vec2::new(1.0, 0.0), Vec2::new(bend, 0.35), c);
    let d = g.v(p - s + u, nn, Vec2::new(0.0, 0.0), Vec2::new(bend, 0.35), c);
    g.tri(a, b, cc); g.tri(a, cc, d);
}

/// A broadleaf tree: a trunk that forks into branches three times, leaf cards round the twigs
fn broadleaf(seed: u32) -> (Mesh, Mesh, f32) {
    let mut r = Rng(seed);
    let (mut bark, mut leaves) = (Geo::default(), Geo::default());
    let bark_c = lin(0.06, 0.045, 0.03);
    let height = r.range(6.5, 10.5);
    struct Br { a: Vec3, d: Vec3, len: f32, rad: f32, lvl: u32 }
    let mut stack = vec![Br { a: Vec3::ZERO, d: Vec3::new(r.range(-0.08, 0.08), 1.0, r.range(-0.08, 0.08)).normalize(), len: height * 0.45, rad: r.range(0.16, 0.26), lvl: 0 }];
    while let Some(b) = stack.pop() {
        // (each limb in two pieces, bending a little toward the light)
        let mid = b.a + b.d * b.len * 0.5 + Vec3::new(r.range(-0.1, 0.1), 0.0, r.range(-0.1, 0.1)) * b.len;
        let d2 = (b.d + Vec3::Y * 0.15).normalize();
        let end = mid + d2 * b.len * 0.5;
        let r_end = b.rad * 0.62;
        limb(&mut bark, b.a, mid, b.rad, b.rad * 0.8, bark_c);
        limb(&mut bark, mid, end, b.rad * 0.8, r_end, bark_c);
        if b.lvl >= 3 {
            // twigs: leaf cards round the end, more toward the outside of the crown
            for _ in 0..5 {
                let p = end + Vec3::new(r.range(-0.5, 0.5), r.range(-0.3, 0.5), r.range(-0.5, 0.5));
                let n = Vec3::new(r.range(-1.0, 1.0), r.range(-0.3, 1.0), r.range(-1.0, 1.0));
                let bend = (p.y / height).clamp(0.3, 1.0);
                card(&mut leaves, p, n, Vec3::Y, r.range(0.9, 1.4), bend);
            }
            continue;
        }
        let kids = if b.lvl == 0 { 4 } else { 3 };
        let az0 = r.range(0.0, std::f32::consts::TAU);
        for k in 0..kids {
            let az = az0 + k as f32 / kids as f32 * std::f32::consts::TAU + r.range(-0.4, 0.4);
            let tilt = r.range(0.45, 0.95);
            let out = Vec3::new(az.cos(), 0.0, az.sin());
            let d = (b.d * tilt.cos() + out * tilt.sin()).normalize();
            let from = b.a + (end - b.a) * r.range(0.65, 1.0);
            stack.push(Br { a: from, d, len: b.len * r.range(0.55, 0.75), rad: r_end * 0.8, lvl: b.lvl + 1 });
        }
    }
    (bark.mesh(true, false), leaves.mesh(false, true), height)
}

/// A conifer: a straight trunk, whorls of drooping branches shorter toward the top, needle cards
/// along them
fn conifer(seed: u32) -> (Mesh, Mesh, f32) {
    let mut r = Rng(seed);
    let (mut bark, mut leaves) = (Geo::default(), Geo::default());
    let bark_c = lin(0.05, 0.035, 0.025);
    let height = r.range(9.0, 15.0);
    limb(&mut bark, Vec3::ZERO, Vec3::Y * height, r.range(0.18, 0.26), 0.03, bark_c);
    let mut y = r.range(1.4, 2.2);
    while y < height - 0.4 {
        let left = 1.0 - y / height;
        let reach = (0.35 + 3.0 * left) * r.range(0.85, 1.1);
        let n = 7;
        let az0 = r.range(0.0, std::f32::consts::TAU);
        for k in 0..n {
            let az = az0 + k as f32 / n as f32 * std::f32::consts::TAU;
            let out = Vec3::new(az.cos(), 0.0, az.sin());
            let tip = Vec3::Y * y + out * reach - Vec3::Y * reach * 0.3;
            limb(&mut bark, Vec3::Y * y, tip, 0.04 * left + 0.01, 0.008, bark_c);
            // (needle cards along it, lying along the branch)
            let cards = (reach / 0.5).ceil() as usize;
            for c in 0..cards {
                let t = (c as f32 + 0.6) / cards as f32;
                let p = Vec3::Y * y + (tip - Vec3::Y * y) * t + Vec3::Y * 0.08;
                let nrm = Vec3::Y + out * 0.3 + Vec3::new(r.range(-0.3, 0.3), 0.0, r.range(-0.3, 0.3));
                card(&mut leaves, p, nrm, out, r.range(0.7, 1.0) * (0.5 + 0.5 * left).max(0.55), (y / height).clamp(0.3, 1.0));
            }
        }
        y += r.range(0.42, 0.6);
    }
    (bark.mesh(true, false), leaves.mesh(false, true), height)
}

/// The treeline: crossed cut-out cards on the hills, in clumps, 115-300 m out
fn treeline() -> Mesh {
    let mut g = Geo::default();
    let mut r = Rng(4242);
    let mut placed = 0;
    while placed < 900 {
        let ang = r.range(0.0, std::f32::consts::TAU);
        let dist = r.range(115.0, 300.0);
        let (x, z) = (dist * ang.cos(), dist * ang.sin());
        // (clumps: fewer where the noise is low)
        if noise(x / 40.0, z / 40.0) < 0.38 { continue }
        placed += 1;
        let cell = (hash(placed, 3, 9) * 4.0) as usize % 4;
        let h = r.range(9.0, 19.0) * if cell >= 2 { 1.2 } else { 1.0 };
        let w = h * 0.5;
        let base = Vec3::new(x, ground_h(x, z) - 0.3, z);
        let (u0, u1) = (cell as f32 * 0.25, cell as f32 * 0.25 + 0.25);
        let c = [1.0; 4];
        let rot = r.range(0.0, 1.57);
        for k in 0..2 {
            let a = rot + k as f32 * std::f32::consts::FRAC_PI_2;
            let s = Vec3::new(a.cos(), 0.0, a.sin()) * w * 0.5;
            let n = Vec3::new(-a.sin(), 0.6, a.cos());
            let i0 = g.v(base - s, n, Vec2::new(u0, 1.0), Vec2::ZERO, c);
            let i1 = g.v(base + s, n, Vec2::new(u1, 1.0), Vec2::ZERO, c);
            let i2 = g.v(base + s + Vec3::Y * h, n, Vec2::new(u1, 0.0), Vec2::ZERO, c);
            let i3 = g.v(base - s + Vec3::Y * h, n, Vec2::new(u0, 0.0), Vec2::ZERO, c);
            g.tri(i0, i1, i2); g.tri(i0, i2, i3);
        }
    }
    g.mesh(false, false)
}

/// how hard the wind blows, by the weather
fn wind_of(look: &Look) -> f32 {
    match look.weather { Weather::Clear => 0.8, Weather::Cloudy => 1.15, Weather::Rain => 1.8, Weather::Fog => 0.35 }
}

/// Made the first time it's needed (outdoors, Medium up), again when the preset changes
#[allow(clippy::too_many_arguments)]
fn build(
    settings: Res<Settings>,
    mut built: ResMut<Built>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut foliage: ResMut<Assets<FoliageMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut roots: Query<&mut Visibility, With<Nature>>,
) {
    let look = Look::of(&settings);
    let g = settings.graphics.as_str();
    // (REBUILT_OFF=nature leaves it out: shot.rs, to see what it costs)
    let want = look.outdoor && g != "low" && !std::env::var("REBUILT_OFF").is_ok_and(|v| v.split(',').any(|x| x.trim() == "nature"));
    if let Some(root) = built.root {
        if let Ok(mut v) = roots.get_mut(root) {
            let vis = if want { Visibility::Inherited } else { Visibility::Hidden };
            if *v != vis { *v = vis; }
        }
    }
    if !want || built.preset.as_deref() == Some(g) { return }
    if let Some(root) = built.root.take() { commands.entity(root).despawn(); }
    built.preset = Some(g.to_string());
    built.foliage.clear();
    built.plain.clear();
    built.weather = None;
    let amt = amount(g);
    let wind = wind_of(&look);
    let mut fol = |base: StandardMaterial, built: &mut Built| {
        let c = base.base_color.to_linear();
        let h = foliage.add(ExtendedMaterial { base, extension: Foliage { wind } });
        built.foliage.push((h.clone(), c));
        h
    };
    let grass_mat = fol(StandardMaterial { base_color: Color::WHITE, perceptual_roughness: 0.75, reflectance: 0.3, double_sided: true, cull_mode: None, ..default() }, &mut built);
    let leaf_mat = fol(StandardMaterial {
        base_color: Color::WHITE, base_color_texture: Some(images.add(leaf_texture(false))), alpha_mode: AlphaMode::Mask(0.5),
        perceptual_roughness: 0.7, reflectance: 0.35, double_sided: true, cull_mode: None, diffuse_transmission: 0.25, ..default()
    }, &mut built);
    let needle_mat = fol(StandardMaterial {
        base_color: Color::WHITE, base_color_texture: Some(images.add(leaf_texture(true))), alpha_mode: AlphaMode::Mask(0.5),
        perceptual_roughness: 0.75, reflectance: 0.3, double_sided: true, cull_mode: None, diffuse_transmission: 0.15, ..default()
    }, &mut built);
    let mut plain = |m: StandardMaterial, built: &mut Built| {
        let (c, r) = (m.base_color.to_linear(), m.perceptual_roughness);
        let h = mats.add(m);
        built.plain.push((h.clone(), c, r));
        h
    };
    let ground_mat = plain(StandardMaterial { base_color: Color::WHITE, perceptual_roughness: 0.92, reflectance: 0.25, ..default() }, &mut built);
    let bark_mat = plain(StandardMaterial { base_color: Color::WHITE, perceptual_roughness: 0.9, reflectance: 0.2, ..default() }, &mut built);
    let line_mat = plain(StandardMaterial {
        base_color: Color::WHITE, base_color_texture: Some(images.add(treeline_texture())), alpha_mode: AlphaMode::Mask(0.5),
        perceptual_roughness: 0.9, double_sided: true, cull_mode: None, ..default()
    }, &mut built);

    let root = commands.spawn((Nature, Transform::default(), Visibility::Inherited)).id();
    built.root = Some(root);
    commands.entity(root).with_children(|p| {
        p.spawn((Mesh3d(meshes.add(terrain())), MeshMaterial3d(ground_mat.clone()), Transform::default(), NotShadowCaster));
        p.spawn((Mesh3d(meshes.add(treeline())), MeshMaterial3d(line_mat.clone()), Transform::default(), NotShadowCaster));
        // the meadow: 2 m patches out to 70 m (round the people and tables), each drawn only within
        // the preset's reach of the camera, fading out at its edge
        let patches = [meshes.add(grass_patch(amt.blades, 11)), meshes.add(grass_patch(amt.blades, 23)), meshes.add(grass_patch(amt.blades, 37))];
        let range = VisibilityRange { start_margin: 0.0..0.0, end_margin: (amt.reach - 6.0)..amt.reach, use_aabb: false };
        for i in -35..35 {
            for j in -35..35 {
                let (x, z) = (i as f32 * 2.0 + 1.0, j as f32 * 2.0 + 1.0);
                if x.hypot(z) > 70.0 || (x.abs() < CLEAR.0 && z.abs() < CLEAR.1) { continue }
                let k = (hash(i, j, 5) * 3.0) as usize % 3;
                let yaw = hash(i, j, 6) * std::f32::consts::TAU;
                p.spawn((
                    Mesh3d(patches[k].clone()), MeshMaterial3d(grass_mat.clone()),
                    Transform::from_xyz(x, ground_h(x, z), z).with_rotation(Quat::from_rotation_y(yaw)),
                    range.clone(), NotShadowCaster,
                ));
            }
        }
        // the trees: four broadleaves and three conifers, grown once and planted round about
        let kinds: Vec<(Handle<Mesh>, Handle<Mesh>, bool)> = (0..7).map(|k| {
            let (b, l, _) = if k < 4 { broadleaf(100 + k) } else { conifer(200 + k) };
            (meshes.add(b), meshes.add(l), k >= 4)
        }).collect();
        let mut rng = Rng(777);
        let mut planted = 0;
        let mut tries = 0;
        while planted < amt.trees && tries < 5000 {
            tries += 1;
            let ang = rng.range(0.0, std::f32::consts::TAU);
            let dist = rng.range(42.0, 112.0);
            let (x, z) = (dist * ang.cos(), dist * ang.sin());
            // (in groves, clear of the light towers)
            if noise(x / 22.0 + 8.0, z / 22.0) < 0.42 || TOWERS.iter().any(|t| Vec2::new(t.x - x, t.z - z).length() < 6.0) { continue }
            planted += 1;
            let k = (rng.f() * 7.0) as usize % 7;
            let s = rng.range(0.8, 1.25);
            let base = Transform::from_xyz(x, ground_h(x, z) - 0.05, z).with_rotation(Quat::from_rotation_y(rng.range(0.0, 6.28))).with_scale(Vec3::splat(s));
            let (bark_m, leaf_m, needles) = &kinds[k];
            let mut t = p.spawn((Tree { base, phase: rng.range(0.0, 6.28), give: rng.range(0.6, 1.2) }, base, Visibility::Inherited));
            t.with_children(|c| {
                let mut b = c.spawn((Mesh3d(bark_m.clone()), MeshMaterial3d(bark_mat.clone()), Transform::default()));
                if !amt.shadows { b.insert(NotShadowCaster); }
                let mut l = c.spawn((Mesh3d(leaf_m.clone()), MeshMaterial3d(if *needles { needle_mat.clone() } else { leaf_mat.clone() }), Transform::default()));
                if !amt.shadows { l.insert(NotShadowCaster); }
            });
        }
    });
}

/// The weather on it: how hard the wind blows; wet and darker in the rain; the leaves' colors
fn weather(settings: Res<Settings>, mut built: ResMut<Built>, mut foliage: ResMut<Assets<FoliageMaterial>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let look = Look::of(&settings);
    let now = (look.weather, look.time);
    if built.weather == Some(now) || built.root.is_none() { return }
    built.weather = Some(now);
    let wet = look.weather == Weather::Rain;
    let wind = wind_of(&look);
    let k = if wet { 0.72 } else { 1.0 };
    for (h, c) in &built.foliage {
        if let Some(mut m) = foliage.get_mut(h) {
            m.extension.wind = wind;
            m.base.base_color = Color::linear_rgba(c.red * k, c.green * k, c.blue * k, c.alpha);
            m.base.perceptual_roughness = if wet { 0.35 } else { 0.72 };
        }
    }
    for (h, c, r) in &built.plain {
        if let Some(mut m) = mats.get_mut(h) {
            m.base_color = Color::linear_rgba(c.red * k, c.green * k, c.blue * k, c.alpha);
            m.perceptual_roughness = if wet { r * 0.55 } else { *r };
        }
    }
}

/// The trees sway as a whole in the gusts (the leaves flutter in the shader)
fn sway(time: Res<Time>, settings: Res<Settings>, mut trees: Query<(&Tree, &mut Transform)>) {
    let look = Look::of(&settings);
    if !look.outdoor { return }
    let t = time.elapsed_secs();
    let w = wind_of(&look);
    let dir = Vec3::new(0.85, 0.0, 0.53);
    let axis = Vec3::Y.cross(dir).normalize();
    for (tree, mut tf) in &mut trees {
        let p = tree.base.translation;
        let gust = 0.55 + 0.45 * (p.x * 0.85 * 0.18 + p.z * 0.53 * 0.18 - t * 1.7).sin();
        let a = w * tree.give * (0.012 * gust + 0.004 * (t * 1.9 + tree.phase).sin());
        tf.rotation = Quat::from_axis_angle(axis, a) * tree.base.rotation;
    }
}
