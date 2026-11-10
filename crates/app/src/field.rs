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
            .add_systems(Startup, (spawn_field, spawn_colliders))
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
