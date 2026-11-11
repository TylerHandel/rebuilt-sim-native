//! The robots' and field's materials, as the arena's light shows them. The GLBs carry their CAD's
//! materials (js robotModels.js's aluminum and kinds, the bumpers' fabric, the polycarbonate);
//! here, by what each material is:
//!   - polycarbonate (clear, glossy, blended): its reflection as a polycarbonate sheet's; and from
//!     High up, real glass: what's behind it seen through it, refracted and blurred by its finish
//!     (specular transmission), not just faded;
//!   - aluminum (metallic): a finish that shows the arena in it;
//!   - the bumpers' fabric (the "alliance" material, and its red): matte cloth;
//!   - black parts (rubber, belts, plastics): duller.
//! Each material's tuned copies are made once and shared.

use std::collections::HashMap;

use bevy::gltf::GltfMaterialName;
use bevy::prelude::*;

use crate::settings::Settings;

pub struct MaterialsPlugin;

impl Plugin for MaterialsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Tuned>().add_systems(PostUpdate, (tune, glass));
    }
}

/// what a material is, for its look
#[derive(Clone, Copy, PartialEq)]
enum Kind { Glass, Metal, Fabric, Dark, Other }

/// each original material's tuned copy (and a glass one, for the glass), by its id
#[derive(Resource, Default)]
struct Tuned { map: HashMap<AssetId<StandardMaterial>, (Handle<StandardMaterial>, Option<Handle<StandardMaterial>>)> }

/// a mesh whose material is tuned: its plain and its glass versions
#[derive(Component)]
struct Look { plain: Handle<StandardMaterial>, glass: Option<Handle<StandardMaterial>> }

fn kind(m: &StandardMaterial, name: Option<&str>) -> Kind {
    if m.unlit || m.emissive != LinearRgba::BLACK { return Kind::Other; }
    if name == Some("alliance") { return Kind::Fabric; }
    let c = m.base_color.to_linear();
    if matches!(m.alpha_mode, AlphaMode::Blend) && m.perceptual_roughness <= 0.15 && m.metallic <= 0.1 && m.base_color_texture.is_none() { return Kind::Glass; }
    if m.metallic >= 0.6 { return Kind::Metal; }
    if m.base_color_texture.is_none() && c.red.max(c.green).max(c.blue) < 0.05 { return Kind::Dark; }
    Kind::Other
}

/// newly in meshes: their materials swapped for their tuned copies
#[allow(clippy::type_complexity)]
fn tune(
    new: Query<(Entity, &MeshMaterial3d<StandardMaterial>, Option<&GltfMaterialName>), (Added<MeshMaterial3d<StandardMaterial>>, Without<Look>)>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut tuned: ResMut<Tuned>,
    settings: Res<Settings>,
    mut commands: Commands,
) {
    let glassy = matches!(settings.graphics.as_str(), "high" | "ultra");
    for (e, h, name) in &new {
        let id = h.0.id();
        if let Some((plain, glass)) = tuned.map.get(&id).cloned() {
            commands.entity(e).insert((MeshMaterial3d(if glassy { glass.clone().unwrap_or(plain.clone()) } else { plain.clone() }), Look { plain, glass }));
            continue;
        }
        let Some(m) = mats.get(&h.0).cloned() else { continue };
        let k = kind(&m, name.map(|n| n.0.as_str()));
        if k == Kind::Other { continue; }
        let mut p = m.clone();
        let mut glass = None;
        match k {
            // (polycarbonate's index 1.58: about 5% reflected straight on)
            Kind::Glass => {
                p.reflectance = 0.56;
                let mut g = p.clone();
                g.alpha_mode = AlphaMode::Opaque;
                let mut c = g.base_color.to_linear();
                c.alpha = 1.0;
                g.base_color = Color::LinearRgba(c);
                g.specular_transmission = 0.92;
                g.diffuse_transmission = 0.0;
                g.thickness = 0.003;
                g.ior = 1.58;
                g.perceptual_roughness = m.perceptual_roughness.max(0.06);
                glass = Some(mats.add(g));
            }
            Kind::Metal => { p.perceptual_roughness = p.perceptual_roughness.min(0.34); }
            Kind::Fabric => { p.perceptual_roughness = 0.93; p.reflectance = 0.2; }
            Kind::Dark => { p.perceptual_roughness = p.perceptual_roughness.max(0.78); p.reflectance = 0.35; }
            Kind::Other => {}
        }
        let plain = mats.add(p);
        tuned.map.insert(id, (plain.clone(), glass.clone()));
        commands.entity(e).insert((MeshMaterial3d(if glassy { glass.clone().unwrap_or(plain.clone()) } else { plain.clone() }), Look { plain, glass }));
    }
}

/// the preset changed: glass from High up, blended sheets below
fn glass(settings: Res<Settings>, mut q: Query<(&Look, &mut MeshMaterial3d<StandardMaterial>)>) {
    if !settings.is_changed() { return }
    let glassy = matches!(settings.graphics.as_str(), "high" | "ultra");
    for (l, mut m) in &mut q {
        if l.glass.is_none() { continue }
        let want = if glassy { l.glass.clone().unwrap_or(l.plain.clone()) } else { l.plain.clone() };
        if m.0 != want { m.0 = want; }
    }
}
