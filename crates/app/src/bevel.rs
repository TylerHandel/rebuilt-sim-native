//! The CAD's sharp edges, rounded as they're drawn (assets/shaders/brushed.wesl and worn.wesl
//! `bevel`). CAD meshes are flat faces meeting at knife edges, where real parts have a fillet, or at
//! least a deburr, that catches the light. Rather than more triangles, each triangle carries at its
//! corners its distance to each of its own three edges (interpolated, the distance from the point
//! drawn to each edge: a distance to a line is linear over a triangle), and how sharply each edge
//! turns: the dihedral's tan(angle / 2) in quarters, convex positive, concave negative, 0 where the
//! surface goes on smoothly. The shader bends the normal near a sharp edge as a fillet would.
//!
//! Worked out here, as each model's meshes come in, rather than shipped (it would triple the
//! models' size): packed in the uvs, which these meshes (their materials untextured) don't use:
//! UV_0 = (d0, d1), UV_1 = (d2, code), code = s0 + 16 s1 + 256 s2, s = 8 + quarters (-7..7).

use std::collections::{HashMap, HashSet};

use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::prelude::*;

pub struct BevelPlugin;

impl Plugin for BevelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<BevelQueue>().add_systems(Last, bake_queued);
    }
}

/// meshes to work out (materials.rs queues the brushed and worn parts'), and those done
#[derive(Resource, Default)]
pub struct BevelQueue { pub todo: Vec<Handle<Mesh>>, done: HashSet<AssetId<Mesh>> }

fn bake_queued(mut q: ResMut<BevelQueue>, mut meshes: ResMut<Assets<Mesh>>) {
    if q.todo.is_empty() { return }
    let todo = std::mem::take(&mut q.todo);
    for h in todo {
        if !q.done.insert(h.id()) { continue }
        // (read first, so a mesh with no sharp edge isn't marked changed)
        let Some(m) = meshes.get(&h) else { q.done.remove(&h.id()); continue };
        let Some(data) = edges(m) else { continue };
        if let Some(mut m) = meshes.get_mut(&h) { apply(&mut *m, data); }
    }
}

/// a sharp edge: the faces' normals this far apart (degrees)
const SHARP: f32 = 25.0;

struct Baked { tris: Vec<[u32; 3]>, code: Vec<u32>, alt: Vec<[f32; 3]> }

/// the triangles, each one's edges' code and its corners' altitudes; None if nothing's sharp
fn edges(m: &Mesh) -> Option<Baked> {
    if m.primitive_topology() != PrimitiveTopology::TriangleList { return None }
    let Some(VertexAttributeValues::Float32x3(p)) = m.attribute(Mesh::ATTRIBUTE_POSITION) else { return None };
    let tris: Vec<[u32; 3]> = match m.indices() {
        Some(i) => { let v: Vec<u32> = i.iter().map(|k| k as u32).collect(); v.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect() }
        None => (0..p.len() as u32 / 3).map(|t| [3 * t, 3 * t + 1, 3 * t + 2]).collect(),
    };
    if tris.is_empty() { return None }
    let v = |i: u32| Vec3::from(p[i as usize]);
    // (corners welded by position: the CAD splits its vertices at every hard edge)
    let (mut lo, mut hi) = (Vec3::INFINITY, Vec3::NEG_INFINITY);
    for q in p { lo = lo.min(Vec3::from(*q)); hi = hi.max(Vec3::from(*q)); }
    let step = ((hi - lo).max_element() * 1e-5).max(1e-12);
    let mut ids: HashMap<[i64; 3], u32> = HashMap::with_capacity(p.len());
    let weld: Vec<u32> = p.iter().map(|q| {
        let k = [(q[0] / step).round() as i64, (q[1] / step).round() as i64, (q[2] / step).round() as i64];
        let n = ids.len() as u32;
        *ids.entry(k).or_insert(n)
    }).collect();
    // each face's normal (unit), centroid and doubled area
    let nt = tris.len();
    let (mut fnrm, mut fc, mut fa) = (Vec::with_capacity(nt), Vec::with_capacity(nt), Vec::with_capacity(nt));
    for t in &tris {
        let (a, b, c) = (v(t[0]), v(t[1]), v(t[2]));
        let n = (b - a).cross(c - a);
        fa.push(n.length());
        fnrm.push(n.normalize_or_zero());
        fc.push((a + b + c) / 3.0);
    }
    // the faces on each edge (a triangle's edge i: opposite its corner i)
    let key = |t: &[u32; 3], i: usize| {
        let (a, b) = (weld[t[(i + 1) % 3] as usize] as u64, weld[t[(i + 2) % 3] as usize] as u64);
        if a < b { a << 32 | b } else { b << 32 | a }
    };
    let mut on: HashMap<u64, (u32, u32, u8)> = HashMap::with_capacity(nt * 2);
    for (ti, t) in tris.iter().enumerate() {
        for i in 0..3 {
            let e = on.entry(key(t, i)).or_insert((ti as u32, u32::MAX, 0));
            if e.2 == 1 { e.1 = ti as u32; }
            e.2 = e.2.saturating_add(1);
        }
    }
    let (mut code, mut alt, mut any) = (vec![0u32; nt], vec![[0.0f32; 3]; nt], false);
    let sharp = SHARP.to_radians().cos();
    for (ti, t) in tris.iter().enumerate() {
        if fa[ti] <= 0.0 { continue }
        let (mut c, mut has) = (0u32, false);
        for i in 0..3 {
            let mut s: i32 = 0;
            let &(a, b, n) = &on[&key(t, i)];
            if n == 2 {
                let o = if a as usize == ti { b } else { a } as usize;
                let cos = fnrm[ti].dot(fnrm[o]).clamp(-1.0, 1.0);
                if fa[o] > 0.0 && cos < sharp {
                    // (convex: the other face falls away behind this one's plane)
                    let convex = fnrm[ti].dot(fc[o] - fc[ti]) < 0.0;
                    let q = ((cos.acos().min(2.2) / 2.0).tan() * 4.0).round().clamp(1.0, 7.0) as i32;
                    s = if convex { q } else { -q };
                    has = true;
                }
            }
            c += ((s + 8) as u32) << (4 * i);
        }
        if has {
            code[ti] = c;
            any = true;
            // (each corner's distance to its opposite edge: the doubled area over that edge)
            for (j, a) in alt[ti].iter_mut().enumerate() {
                let el = v(t[(j + 2) % 3]).distance(v(t[(j + 1) % 3]));
                *a = if el > 0.0 { fa[ti] / el } else { 0.0 };
            }
        }
    }
    any.then_some(Baked { tris, code, alt })
}

/// each triangle its own corners, the uvs carrying the edges
fn apply(m: &mut Mesh, b: Baked) {
    m.insert_indices(Indices::U32(b.tris.iter().flatten().copied().collect()));
    if m.try_duplicate_vertices().is_err() { return }
    let n = b.tris.len() * 3;
    let (mut uv0, mut uv1) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for t in 0..b.tris.len() {
        for j in 0..3 {
            let mut d = [0.0f32; 3];
            if b.code[t] != 0 { d[j] = b.alt[t][j]; }
            uv0.push([d[0], d[1]]);
            uv1.push([d[2], b.code[t] as f32]);
        }
    }
    m.insert_attribute(Mesh::ATTRIBUTE_UV_0, uv0);
    m.insert_attribute(Mesh::ATTRIBUTE_UV_1, uv1);
}
