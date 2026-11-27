//! The robots' and field's materials, as the arena's light shows them. The GLBs carry their CAD's
//! materials (js robotModels.js's aluminum and kinds, the bumpers' fabric, the polycarbonate);
//! here, by what each material is:
//!   - polycarbonate (clear blended sheets, 40% of the CAD's opacity, glossy: clear, not hazy): a
//!     modest reflection, so the arena's light doesn't ring
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

/// Worn paint and plastic: the standard material, unevenness and scuffs over it (wear: how much).
/// The field's are cut away where the alliance wall hides your robot (cutaway.rs: cut, eye).
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub struct Worn {
    #[uniform(100)]
    pub wear: f32,
    #[uniform(101)]
    pub cut: Vec4,
    #[uniform(102)]
    pub eye: Vec4,
}
impl MaterialExtension for Worn {
    fn fragment_shader() -> ShaderRef { "shaders/worn.wesl".into() }
    fn prepass_fragment_shader() -> ShaderRef { "shaders/worn_prepass.wesl".into() }
}
pub type WornMaterial = ExtendedMaterial<StandardMaterial, Worn>;

/// The bumpers' cloth: its weave, seams and rubs (assets/shaders/fabric.wesl). fab: how worn, and
/// the bumpers' bottom and top (world y)
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub struct Fabric {
    #[uniform(100)]
    pub fab: Vec4,
}
impl MaterialExtension for Fabric {
    fn fragment_shader() -> ShaderRef { "shaders/fabric.wesl".into() }
}
pub type FabricMaterial = ExtendedMaterial<StandardMaterial, Fabric>;

/// Polycarbonate's fine scratches and smudges (assets/shaders/scratched.wesl): how scratched
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub struct Scratched {
    #[uniform(100)]
    pub scratch: f32,
}
impl MaterialExtension for Scratched {
    fn fragment_shader() -> ShaderRef { "shaders/scratched.wesl".into() }
}
pub type ScratchedMaterial = ExtendedMaterial<StandardMaterial, Scratched>;

pub struct MaterialsPlugin;

impl Plugin for MaterialsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((MaterialPlugin::<BrushedMaterial>::default(), MaterialPlugin::<WornMaterial>::default(), MaterialPlugin::<FabricMaterial>::default(), MaterialPlugin::<ScratchedMaterial>::default()))
            .init_resource::<Tuned>().init_resource::<FieldMats>()
            .add_systems(PostUpdate, tune);
    }
}

/// what a material is, for its look
/// how much of its CAD opacity a robot's polycarbonate keeps (js robotModels.js applyPoly's 0.4)
const CLEAR_OPACITY: f32 = 0.4;

#[derive(Clone, Copy, PartialEq)]
enum Kind { Clear, Metal, Fabric, Dark, Painted, Other }

/// a tuned copy: brushed, worn, cloth or scratched
#[derive(Clone)]
enum Copy { Brushed(Handle<BrushedMaterial>), Worn(Handle<WornMaterial>), Fabric(Handle<FabricMaterial>), Scratched(Handle<ScratchedMaterial>) }
/// each original material's tuned copy, by its id
#[derive(Resource, Default)]
struct Tuned { map: HashMap<AssetId<StandardMaterial>, Copy> }

/// a mesh whose material is tuned (or set as it should be elsewhere, field.rs's glass: left alone)
#[derive(Component)]
pub struct Look;

/// a mesh of the field's CAD: its materials are all worn ones (so the alliance wall's cutaway,
/// cutaway.rs, can drop their fragments), the opaque ones masked (so they can be dropped from the
/// prepass too). Their own copies, apart from the robots'.
#[derive(Component)]
pub struct FieldPart;

/// the field's worn materials, for the cutaway
#[derive(Resource, Default)]
pub struct FieldMats { pub mats: Vec<Handle<WornMaterial>>, map: HashMap<AssetId<StandardMaterial>, (Handle<WornMaterial>, bool)> }

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
    new: Query<(Entity, &MeshMaterial3d<StandardMaterial>, &Mesh3d, Option<&GltfMaterialName>, Option<&Name>, Has<FieldPart>), (Added<MeshMaterial3d<StandardMaterial>>, With<Mesh3d>, Without<Look>, Without<bevy::light::NotShadowCaster>)>,
    mats: ResMut<Assets<StandardMaterial>>,
    mut brushed: ResMut<Assets<BrushedMaterial>>,
    mut worn: ResMut<Assets<WornMaterial>>,
    mut fabric: ResMut<Assets<FabricMaterial>>,
    mut scratched: ResMut<Assets<ScratchedMaterial>>,
    mut tuned: ResMut<Tuned>,
    mut field: ResMut<FieldMats>,
    mut bevel: ResMut<crate::bevel::BevelQueue>,
    mut commands: Commands,
) {
    let put = |commands: &mut Commands, e: Entity, c: &Copy| match c {
        Copy::Brushed(h) => { commands.entity(e).remove::<MeshMaterial3d<StandardMaterial>>().insert((MeshMaterial3d(h.clone()), Look)); }
        Copy::Worn(h) => { commands.entity(e).remove::<MeshMaterial3d<StandardMaterial>>().insert((MeshMaterial3d(h.clone()), Look)); }
        Copy::Fabric(h) => { commands.entity(e).remove::<MeshMaterial3d<StandardMaterial>>().insert((MeshMaterial3d(h.clone()), Look)); }
        Copy::Scratched(h) => { commands.entity(e).remove::<MeshMaterial3d<StandardMaterial>>().insert((MeshMaterial3d(h.clone()), Look)); }
    };
    // (only the brushed and worn parts' edges are rounded)
    let bevelled = |c: &Copy| matches!(c, Copy::Brushed(_) | Copy::Worn(_));
    for (e, h, mesh, name, ename, is_field) in &new {
        // (not the venue's: it's drawn plain)
        if ename.is_some_and(|n| n.as_str().starts_with("venue")) { continue; }
        let id = h.0.id();
        if is_field {
            let (w, bev) = match field.map.get(&id) {
                Some(w) => w.clone(),
                None => {
                    let Some(m) = mats.get(&h.0).cloned() else { continue };
                    let k = kind(&m, name.map(|n| n.0.as_str()));
                    let mut p = m.clone();
                    let wear = match k {
                        Kind::Dark => { p.perceptual_roughness = p.perceptual_roughness.max(0.78); p.reflectance = 0.35; 0.7 }
                        Kind::Painted => 1.0,
                        Kind::Metal => { p.perceptual_roughness = p.perceptual_roughness.min(0.36); 0.5 }
                        Kind::Fabric => { p.perceptual_roughness = 0.93; p.reflectance = 0.2; 0.0 }
                        Kind::Clear | Kind::Other => 0.0,
                    };
                    if matches!(p.alpha_mode, AlphaMode::Opaque) { p.alpha_mode = AlphaMode::Mask(0.5); }
                    // (the CAD's edges rounded where the robots' are: not on textured parts, whose
                    // uvs the rounding would take)
                    let bev = matches!(k, Kind::Dark | Kind::Painted | Kind::Metal);
                    let w = worn.add(ExtendedMaterial { base: p, extension: Worn { wear, ..default() } });
                    field.map.insert(id, (w.clone(), bev));
                    field.mats.push(w.clone());
                    (w, bev)
                }
            };
            if bev { bevel.todo.push(mesh.0.clone()); }
            commands.entity(e).remove::<MeshMaterial3d<StandardMaterial>>().insert((MeshMaterial3d(w), Look));
            continue;
        }
        if let Some(c) = tuned.map.get(&id).cloned() {
            if bevelled(&c) { bevel.todo.push(mesh.0.clone()); }
            put(&mut commands, e, &c);
            continue;
        }
        let Some(m) = mats.get(&h.0).cloned() else { continue };
        let k = kind(&m, name.map(|n| n.0.as_str()));
        if k == Kind::Other { continue; }
        let mut p = m.clone();
        let c = match k {
            // (reflectance 0.2: F0 0.0064, the grazing reflectance 0.32 where 0.5 reached 1)
            // (clear sheet, as the browser game's from Low up, js robotModels.js applyPoly: 40% of
            // the CAD's opacity, glossy; the user: clear, not hazy)
            Kind::Clear => {
                let mut c = p.base_color.to_linear();
                c.alpha *= CLEAR_OPACITY;
                p.base_color = Color::LinearRgba(c);
                p.perceptual_roughness = p.perceptual_roughness.min(0.08);
                p.reflectance = 0.3;
                // (a season's fine scratches and smudges, where the light catches them)
                Copy::Scratched(scratched.add(ExtendedMaterial { base: p, extension: Scratched { scratch: 0.8 } }))
            }
            // (raw aluminum, light: all the grain; anodized, dark: its dyed oxide mutes it)
            Kind::Metal => {
                p.perceptual_roughness = p.perceptual_roughness.min(0.36);
                let c = p.base_color.to_linear();
                let grain = if c.red.max(c.green).max(c.blue) > 0.2 { 1.0 } else { 0.45 };
                Copy::Brushed(brushed.add(ExtendedMaterial { base: p, extension: Brushed { grain } }))
            }
            // (its weave, seams and a season's rubs)
            Kind::Fabric => {
                p.perceptual_roughness = 0.93;
                p.reflectance = 0.2;
                let (y0, y1) = (sim::constants::BUMPER_Y0 as f32, sim::constants::BUMPER_Y1 as f32);
                Copy::Fabric(fabric.add(ExtendedMaterial { base: p, extension: Fabric { fab: Vec4::new(0.8, y0, y1, 0.0) } }))
            }
            Kind::Dark => {
                p.perceptual_roughness = p.perceptual_roughness.max(0.78);
                p.reflectance = 0.35;
                Copy::Worn(worn.add(ExtendedMaterial { base: p, extension: Worn { wear: 0.7, ..default() } }))
            }
            Kind::Painted => Copy::Worn(worn.add(ExtendedMaterial { base: p, extension: Worn { wear: 1.0, ..default() } })),
            Kind::Other => continue,
        };
        tuned.map.insert(id, c.clone());
        if bevelled(&c) { bevel.todo.push(mesh.0.clone()); }
        put(&mut commands, e, &c);
    }
}
