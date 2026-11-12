//! The robots' and field's materials, as the arena's light shows them. The GLBs carry their CAD's
//! materials (js robotModels.js's aluminum and kinds, the bumpers' fabric, the polycarbonate);
//! here, by what each material is:
//!   - polycarbonate (clear blended sheets): a weak reflection, so the arena's light doesn't ring
//!     each panel's edges (Bevy's grazing-angle reflectance follows the head-on one); not drawn as
//!     refracting glass (specular transmission tinted what's behind it gray and darkened hoppers);
//!   - aluminum (metallic): brushed (assets/shaders/brushed.wesl: js robotModels.js's grain, in
//!     each part's own frame), its finish showing the arena in it; anodized (darker) more muted;
//!   - the bumpers' fabric (the "alliance" material, and its red): matte cloth;
//!   - black parts (rubber, belts, plastics): duller;
//!   - painted and plastic parts, and those black ones: worn (assets/shaders/worn.wesl: color and
//!     gloss uneven by a few percent, scuffed near the carpet). Not the venue's.
//! The brushed and worn parts' sharp edges are rounded as they're drawn (bevel.rs).
//! Each material's tuned copies are made once and shared. Only the CAD's meshes: not the app's own
//! (the HUB light bars and LEDs, lights.rs, update their materials in place; they, the venue's and
//! the nets are the ones that cast no shadow).

use std::collections::HashMap;

use bevy::gltf::GltfMaterialName;
use bevy::pbr::{ExtendedMaterial, MaterialExtension};
use bevy::prelude::*;
use bevy::render::render_resource::AsBindGroup;
use bevy::shader::ShaderRef;

/// Brushed aluminum: the standard material, its grain drawn over it (grain: how much shows)
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub struct Brushed {
    #[uniform(100)]
    pub grain: f32,
}
impl MaterialExtension for Brushed {
    fn fragment_shader() -> ShaderRef { "shaders/brushed.wesl".into() }
}
pub type BrushedMaterial = ExtendedMaterial<StandardMaterial, Brushed>;

/// Worn paint and plastic: the standard material, unevenness and scuffs over it (wear: how much)
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub struct Worn {
    #[uniform(100)]
    pub wear: f32,
}
impl MaterialExtension for Worn {
    fn fragment_shader() -> ShaderRef { "shaders/worn.wesl".into() }
}
pub type WornMaterial = ExtendedMaterial<StandardMaterial, Worn>;

pub struct MaterialsPlugin;

impl Plugin for MaterialsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((MaterialPlugin::<BrushedMaterial>::default(), MaterialPlugin::<WornMaterial>::default()))
            .init_resource::<Tuned>()
            .add_systems(PostUpdate, tune);
    }
}

/// what a material is, for its look
#[derive(Clone, Copy, PartialEq)]
enum Kind { Clear, Metal, Fabric, Dark, Painted, Other }

/// a tuned copy: a standard material, or a brushed one
#[derive(Clone)]
enum Copy { Std(Handle<StandardMaterial>), Brushed(Handle<BrushedMaterial>), Worn(Handle<WornMaterial>) }
/// each original material's tuned copy, by its id
#[derive(Resource, Default)]
struct Tuned { map: HashMap<AssetId<StandardMaterial>, Copy> }

/// a mesh whose material is tuned (or set as it should be elsewhere, field.rs's glass: left alone)
#[derive(Component)]
pub struct Look;

fn kind(m: &StandardMaterial, name: Option<&str>) -> Kind {
    if m.unlit || m.emissive != LinearRgba::BLACK { return Kind::Other; }
    if name == Some("alliance") { return Kind::Fabric; }
    let c = m.base_color.to_linear();
    if !matches!(m.alpha_mode, AlphaMode::Opaque) {
        return if m.metallic <= 0.1 && m.perceptual_roughness <= 0.3 && m.base_color_texture.is_none() { Kind::Clear } else { Kind::Other };
    }
    if m.metallic >= 0.6 { return Kind::Metal; }
    if m.base_color_texture.is_none() && c.red.max(c.green).max(c.blue) < 0.05 { return Kind::Dark; }
    if m.base_color_texture.is_none() { return Kind::Painted; }
    Kind::Other
}

/// newly in meshes: their materials swapped for their tuned copies
#[allow(clippy::type_complexity)]
fn tune(
    new: Query<(Entity, &MeshMaterial3d<StandardMaterial>, &Mesh3d, Option<&GltfMaterialName>, Option<&Name>), (Added<MeshMaterial3d<StandardMaterial>>, With<Mesh3d>, Without<Look>, Without<bevy::light::NotShadowCaster>)>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut brushed: ResMut<Assets<BrushedMaterial>>,
    mut worn: ResMut<Assets<WornMaterial>>,
    mut tuned: ResMut<Tuned>,
    mut bevel: ResMut<crate::bevel::BevelQueue>,
    mut commands: Commands,
) {
    let put = |commands: &mut Commands, e: Entity, c: &Copy| match c {
        Copy::Std(h) => { commands.entity(e).insert((MeshMaterial3d(h.clone()), Look)); }
        Copy::Brushed(h) => { commands.entity(e).remove::<MeshMaterial3d<StandardMaterial>>().insert((MeshMaterial3d(h.clone()), Look)); }
        Copy::Worn(h) => { commands.entity(e).remove::<MeshMaterial3d<StandardMaterial>>().insert((MeshMaterial3d(h.clone()), Look)); }
    };
    for (e, h, mesh, name, ename) in &new {
        // (not the venue's: it's drawn plain)
        if ename.is_some_and(|n| n.as_str().starts_with("venue")) { continue; }
        let id = h.0.id();
        if let Some(c) = tuned.map.get(&id).cloned() {
            if !matches!(c, Copy::Std(_)) { bevel.todo.push(mesh.0.clone()); }
            put(&mut commands, e, &c);
            continue;
        }
        let Some(m) = mats.get(&h.0).cloned() else { continue };
        let k = kind(&m, name.map(|n| n.0.as_str()));
        if k == Kind::Other { continue; }
        let mut p = m.clone();
        let c = match k {
            // (reflectance 0.2: F0 0.0064, the grazing reflectance 0.32 where 0.5 reached 1)
            Kind::Clear => { p.reflectance = 0.2; Copy::Std(mats.add(p)) }
            // (raw aluminum, light: all the grain; anodized, dark: its dyed oxide mutes it)
            Kind::Metal => {
                p.perceptual_roughness = p.perceptual_roughness.min(0.36);
                let c = p.base_color.to_linear();
                let grain = if c.red.max(c.green).max(c.blue) > 0.2 { 1.0 } else { 0.45 };
                Copy::Brushed(brushed.add(ExtendedMaterial { base: p, extension: Brushed { grain } }))
            }
            Kind::Fabric => { p.perceptual_roughness = 0.93; p.reflectance = 0.2; Copy::Std(mats.add(p)) }
            Kind::Dark => {
                p.perceptual_roughness = p.perceptual_roughness.max(0.78);
                p.reflectance = 0.35;
                Copy::Worn(worn.add(ExtendedMaterial { base: p, extension: Worn { wear: 0.7 } }))
            }
            Kind::Painted => Copy::Worn(worn.add(ExtendedMaterial { base: p, extension: Worn { wear: 1.0 } })),
            Kind::Other => continue,
        };
        tuned.map.insert(id, c.clone());
        if !matches!(c, Copy::Std(_)) { bevel.todo.push(mesh.0.clone()); }
        put(&mut commands, e, &c);
    }
}
