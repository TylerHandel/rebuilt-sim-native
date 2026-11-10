//! The field: its CAD model (drawn), and its colliders exactly as the sim has them (F1 shows them).

use bevy::prelude::*;
use bevy::light::CascadeShadowConfigBuilder;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::asset::RenderAssetUsages;
use rapier3d::prelude::TypedShape;

use crate::game::Game;

pub struct FieldPlugin;

impl Plugin for FieldPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(GlobalAmbientLight { brightness: 400.0, ..default() })
            .add_systems(Startup, (spawn_field, spawn_floor, spawn_colliders))
            .add_systems(Update, toggle_colliders);
    }
}

/// The field's CAD scene.
#[derive(Resource)]
pub struct FieldScene(pub Handle<WorldAsset>);

fn spawn_field(mut commands: Commands, assets: Res<AssetServer>) {
    // cad/manifest.json: the field's CAD is in meters, turned [-90, 0, 180] degrees (XYZ)
    let d = std::f32::consts::PI / 180.0;
    let scene = assets.load(GltfAssetLabel::Scene(0).from_asset("cad/field/field.glb"));
    commands.insert_resource(FieldScene(scene.clone()));
    commands.spawn((
        WorldAssetRoot(scene),
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -90.0 * d, 0.0, 180.0 * d)),
    ));
    commands.spawn((
        DirectionalLight { illuminance: 9000.0, shadow_maps_enabled: true, ..default() },
        Transform::from_xyz(4.0, 12.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
        CascadeShadowConfigBuilder { maximum_distance: 40.0, ..default() }.build(),
    ));
}

#[derive(Component)]
pub struct ColliderView;

/// A triangle mesh of one collider's shape, in its own frame.
pub fn collider_mesh(shape: TypedShape) -> Option<Mesh> {
    let (v, i) = match shape {
        TypedShape::Cuboid(s) => s.to_trimesh(),
        TypedShape::ConvexPolyhedron(s) => s.to_trimesh(),
        TypedShape::Capsule(s) => s.to_trimesh(12, 6),
        TypedShape::Cylinder(s) => s.to_trimesh(16),
        TypedShape::Ball(s) => s.to_trimesh(12, 6),
        TypedShape::TriMesh(s) => (s.vertices().to_vec(), s.indices().to_vec()),
        _ => return None,
    };
    let pos: Vec<[f32; 3]> = v.iter().map(|p| [p.x, p.y, p.z]).collect();
    let mut m = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    m.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
    m.insert_indices(Indices::U32(i.into_iter().flatten().collect()));
    let m = m.with_duplicated_vertices().with_computed_flat_normals();
    Some(m)
}

fn spawn_colliders(mut commands: Commands, game: Res<Game>, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let mat = mats.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.1, 0.8, 0.45),
        depth_bias: 10.0,
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        cull_mode: None,
        ..default()
    });
    let fixed = &game.0.physics.bodies[game.0.fixed];
    for h in fixed.colliders() {
        let c = &game.0.physics.colliders[*h];
        let Some(mesh) = collider_mesh(c.shape().as_typed_shape()) else { continue };
        let (t, r) = (c.translation(), c.rotation());
        commands.spawn((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(mat.clone()),
            Transform::from_translation(t).with_rotation(r),
            Visibility::Hidden,
            ColliderView,
        ));
    }
}

fn toggle_colliders(keys: Res<ButtonInput<KeyCode>>, mut q: Query<&mut Visibility, With<ColliderView>>) {
    if keys.just_pressed(KeyCode::F1) {
        for mut v in &mut q {
            *v = if *v == Visibility::Hidden { Visibility::Visible } else { Visibility::Hidden };
        }
    }
}

/// js field.js _buildFloor: the carpet (its texture as _carpetTexture paints it) and the dark floor
/// around the field
fn spawn_floor(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>, mut images: ResMut<Assets<Image>>) {
    use sim::constants::{FIELD_L, FIELD_W};
    let (hl, hw) = ((FIELD_L / 2.0) as f32, (FIELD_W / 2.0) as f32);
    // (as three.js's PlaneGeometry turned flat: the image's top row at -z, its left column at -x)
    let mut m = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    m.insert_attribute(Mesh::ATTRIBUTE_POSITION, vec![[-hl, 0.0, -hw], [hl, 0.0, -hw], [hl, 0.0, hw], [-hl, 0.0, hw]]);
    m.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 1.0, 0.0]; 4]);
    m.insert_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    m.insert_indices(Indices::U32(vec![0, 2, 1, 0, 3, 2]));
    let carpet = mats.add(StandardMaterial { base_color_texture: Some(images.add(carpet_texture())), perceptual_roughness: 0.95, metallic: 0.0, ..default() });
    commands.spawn((Mesh3d(meshes.add(m)), MeshMaterial3d(carpet), Transform::default()));
    let outer = mats.add(StandardMaterial { base_color: Color::srgb_u8(0x23, 0x25, 0x2b), perceptual_roughness: 1.0, ..default() });
    commands.spawn((Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(30.0, 20.0)))), MeshMaterial3d(outer), Transform::from_xyz(0.0, -0.005, 0.0)));
}

/// js field.js _carpetTexture: gray carpet with a little noise, the ALLIANCE ZONES' faint tints, the
/// seam's gaffer tape, the ROBOT STARTING LINES, the CENTER LINE and the DEPOT outlines
fn carpet_texture() -> Image {
    use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
    use sim::constants::{ALLIANCE_ZONE_DEPTH, DEPOT, FIELD_L, FIELD_W, IN};
    let w = 4096usize;
    let h = (4096.0 * FIELD_W / FIELD_L).round() as usize;
    let mut px = vec![0u8; w * h * 4];
    // (the noise: Math.random there, a small LCG here)
    let mut seed = 0x2026_u32;
    for p in px.chunks_exact_mut(4) {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let n = ((seed >> 8) as f32 / 16_777_216.0 - 0.5) * 18.0;
        for (i, c) in [0x62u8, 0x66, 0x6d].iter().enumerate() { p[i] = (*c as f32 + n).round().clamp(0.0, 255.0) as u8; }
        p[3] = 255;
    }
    let (sx, sy) = (w as f64 / FIELD_L, h as f64 / FIELD_W);
    // a canvas fillRect in pixels, mixed in sRGB as the canvas mixes
    let mut fill = |x0: f64, y0: f64, x1: f64, y1: f64, c: [u8; 3], a: f32| {
        let (x0, x1) = (x0.round().max(0.0) as usize, (x1.round() as usize).min(w));
        let (y0, y1) = (y0.round().max(0.0) as usize, (y1.round() as usize).min(h));
        for y in y0..y1 { for x in x0..x1 {
            let i = (y * w + x) * 4;
            for k in 0..3 { px[i + k] = (c[k] as f32 * a + px[i + k] as f32 * (1.0 - a)).round() as u8; }
        } }
    };
    // canvas x = fx, canvas y = fy mirrored
    let rect = |fill: &mut dyn FnMut(f64, f64, f64, f64, [u8; 3], f32), fx0: f64, fy0: f64, fx1: f64, fy1: f64, c: [u8; 3], a: f32| {
        fill(fx0 * sx, (FIELD_W - fy1) * sy, fx1 * sx, (FIELD_W - fy0) * sy, c, a);
    };
    let (blue, red) = ([0x2f, 0x6f, 0xf0], [0xe0, 0x34, 0x34]);
    rect(&mut fill, 0.0, 0.0, ALLIANCE_ZONE_DEPTH, FIELD_W, blue, 0.06);
    rect(&mut fill, FIELD_L - ALLIANCE_ZONE_DEPTH, 0.0, FIELD_L, FIELD_W, red, 0.04);
    let tape = 2.0 * IN;
    rect(&mut fill, 0.0, FIELD_W / 2.0 - 1.5 * IN, FIELD_L, FIELD_W / 2.0 + 1.5 * IN, [0x1e, 0x1f, 0x23], 1.0);
    rect(&mut fill, ALLIANCE_ZONE_DEPTH - tape, 0.0, ALLIANCE_ZONE_DEPTH, FIELD_W, blue, 1.0);
    rect(&mut fill, FIELD_L - ALLIANCE_ZONE_DEPTH, 0.0, FIELD_L - ALLIANCE_ZONE_DEPTH + tape, FIELD_W, red, 1.0);
    rect(&mut fill, FIELD_L / 2.0 - tape / 2.0, 0.0, FIELD_L / 2.0 + tape / 2.0, FIELD_W, [0xf2, 0xf2, 0xf2], 1.0);
    // the DEPOT outlines (a 6 px stroke, centered on the rectangle's edges)
    for (fx0, fx1, fyc, c) in [(0.0, DEPOT::depth, DEPOT::fy, blue), (FIELD_L - DEPOT::depth, FIELD_L, FIELD_W - DEPOT::fy, red)] {
        let (x0, x1) = (fx0 * sx, fx1 * sx);
        let (y0, y1) = ((FIELD_W - (fyc + DEPOT::width / 2.0)) * sy, (FIELD_W - (fyc - DEPOT::width / 2.0)) * sy);
        fill(x0 - 3.0, y0 - 3.0, x1 + 3.0, y0 + 3.0, c, 1.0);
        fill(x0 - 3.0, y1 - 3.0, x1 + 3.0, y1 + 3.0, c, 1.0);
        fill(x0 - 3.0, y0 - 3.0, x0 + 3.0, y1 + 3.0, c, 1.0);
        fill(x1 - 3.0, y0 - 3.0, x1 + 3.0, y1 + 3.0, c, 1.0);
    }
    Image::new(Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 }, TextureDimension::D2, px, TextureFormat::Rgba8UnormSrgb, RenderAssetUsages::RENDER_WORLD)
}
