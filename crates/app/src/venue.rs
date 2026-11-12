//! The venue round the field (js venue.js, exported by tools/export-models.mjs as
//! assets/models/venue.glb): the bleachers and their crowd, the scoring table and its volunteers,
//! the field-reset crew, the referees, the drive teams behind their driver stations, the lighting
//! trusses, the pipe and drape with its team banners, the gym's walls and the screens. Drawn only:
//! nothing here touches the simulation, and nothing casts a shadow. The screens show the live score
//! and match time (a UI drawn into a texture, as venue.js draws its canvas); the drive teams aren't
//! drawn at the Driver Station view, where they'd stand in the way; and at Low there's no venue.

use bevy::camera::RenderTarget;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use bevy::world_serialization::{WorldAssetRoot, WorldInstanceReady};
use sim::constants::Alliance;

use crate::camera::{CamMode, CameraRig};
use crate::game::Game;
use crate::settings::Settings;
use crate::ui::{Face, Fonts};

pub struct VenuePlugin;

impl Plugin for VenuePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn)
            .add_systems(Update, (show, scoreboard_ui, scoreboard));
    }
}

#[derive(Component)]
struct Venue;
/// the drive teams' people (hidden at the Driver Station view)
#[derive(Component)]
struct DriveTeams;
/// the screens' picture
#[derive(Resource)]
struct Screens { image: Handle<Image>, cam: Entity, ui: bool, t: f32 }
#[derive(Component, Clone, Copy)]
enum ScreenText { Blue, Red, BlueTeams, RedTeams, Clock, Phase }

fn spawn(mut commands: Commands, assets: Res<AssetServer>, mut images: ResMut<Assets<Image>>) {
    let scene = assets.load(GltfAssetLabel::Scene(0).from_asset("models/venue.glb"));
    commands
        .spawn((Venue, Transform::default(), Visibility::Hidden))
        .with_children(|p| { p.spawn((WorldAssetRoot(scene), Transform::default())).observe(on_ready); });
    // (the screens' picture: 512 x 288, as venue.js's canvas; its camera draws only when asked)
    let image = images.add(Image::new_target_texture(512, 288, TextureFormat::Rgba8UnormSrgb, None));
    let cam = commands
        .spawn((
            Camera2d,
            Camera { order: -10, clear_color: ClearColorConfig::Custom(Color::srgb_u8(0x05, 0x07, 0x0c)), ..default() },
            RenderTarget::Image(image.clone().into()),
        ))
        .id();
    commands.insert_resource(Screens { image, cam, ui: false, t: 0.0 });
}

/// The GLB is in: the drive teams marked, the screens showing their picture, the trusses' lenses
/// glowing, and nothing casting a shadow
fn on_ready(
    ev: On<WorldInstanceReady>,
    children: Query<&Children>,
    names: Query<&Name>,
    meshes: Query<(), With<Mesh3d>>,
    screens: Res<Screens>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mesh3d: Query<&Mesh3d>,
    mut mesh_assets: ResMut<Assets<Mesh>>,
    mut commands: Commands,
) {
    // (the render target's rows run the other way from the canvas the GLB's uvs were made for)
    let flip = bevy::math::Affine2::from_cols_array(&[1.0, 0.0, 0.0, -1.0, 0.0, 1.0]);
    let screen = mats.add(StandardMaterial { base_color_texture: Some(screens.image.clone()), unlit: true, uv_transform: flip, ..default() });
    // (venue.js's lenses: an emissive 1.6 in three's units; bright enough here to bloom)
    let lens = mats.add(StandardMaterial { base_color: Color::BLACK, emissive: LinearRgba::rgb(60.0, 57.0, 50.0), ..default() });
    for e in children.iter_descendants(ev.entity) {
        // (drawn plain: materials.rs leaves it be)
        // (nor does it take any: far off, shadows would hardly show on it, and the crowd's many
        // pixels needn't look them up)
        if meshes.contains(e) { commands.entity(e).insert((bevy::light::NotShadowCaster, bevy::light::NotShadowReceiver, crate::materials::Look)); }
        let Ok(n) = names.get(e) else { continue };
        let set = |mat: &Handle<StandardMaterial>, commands: &mut Commands| {
            if meshes.contains(e) { commands.entity(e).insert(MeshMaterial3d(mat.clone())); }
            for k in children.iter_descendants(e) { if meshes.contains(k) { commands.entity(k).insert(MeshMaterial3d(mat.clone())); } }
        };
        match n.as_str() {
            "venue-driveteams" => { commands.entity(e).insert(DriveTeams); }
            "venue-crowd" => {
                for k in children.iter_descendants(e).chain([e]) {
                    if let Ok(h) = mesh3d.get(k) { if let Some(mut m) = mesh_assets.get_mut(&h.0) { slim_crowd(&mut m); } }
                }
            }
            "venue-screens" => set(&screen, &mut commands),
            "venue-lens" => set(&lens, &mut commands),
            _ => {}
        }
    }
}

/// The crowd in the stands (venue.js personGeometry: ten boxes a person, ~430 of them), as little
/// as shows from the field: no necks or hands (a few cm, under a pixel from the field), and no faces
/// that never face it (the boxes' bottoms, and backs: everyone faces the field). From ~52k
/// triangles to under half: each a speck at that distance, and specks are what GPUs draw worst.
fn slim_crowd(m: &mut Mesh) {
    use bevy::mesh::VertexAttributeValues as V;
    if m.indices().is_some() { return }
    let Some(V::Float32x3(p)) = m.attribute(Mesh::ATTRIBUTE_POSITION) else { return };
    let n = p.len();
    if n % 36 != 0 { return }
    let mut keep = Vec::with_capacity(n);
    for b in (0..n).step_by(36) {
        let (mut lo, mut hi) = (Vec3::INFINITY, Vec3::NEG_INFINITY);
        for q in &p[b..b + 36] { lo = lo.min(Vec3::from(*q)); hi = hi.max(Vec3::from(*q)); }
        if (hi - lo).max_element() < 0.11 { continue }
        let c = (lo + hi) / 2.0;
        // (toward the field: its middle, level)
        let to = Vec3::new(-c.x, 0.0, -c.z).normalize_or_zero();
        for t in (b..b + 36).step_by(3) {
            let (a, bb, cc) = (Vec3::from(p[t]), Vec3::from(p[t + 1]), Vec3::from(p[t + 2]));
            let nrm = (bb - a).cross(cc - a).normalize_or_zero();
            if nrm.y < -0.5 || nrm.dot(to) < -0.5 { continue }
            keep.extend([t, t + 1, t + 2]);
        }
    }
    let ids: Vec<u32> = keep.iter().map(|&i| i as u32).collect();
    m.insert_indices(bevy::mesh::Indices::U32(ids));
    let _ = m.try_duplicate_vertices();
}

/// Shown from Medium up; the drive teams not at the Driver Station view
fn show(
    settings: Res<Settings>,
    rig: Option<Res<CameraRig>>,
    mut venue: Query<&mut Visibility, (With<Venue>, Without<DriveTeams>)>,
    mut teams: Query<&mut Visibility, (With<DriveTeams>, Without<Venue>)>,
) {
    let on = settings.graphics != "low";
    for mut v in &mut venue {
        let want = if on { Visibility::Inherited } else { Visibility::Hidden };
        if *v != want { *v = want; }
    }
    let at_ds = rig.is_some_and(|r| r.mode == CamMode::Driver);
    for mut v in &mut teams {
        let want = if at_ds { Visibility::Hidden } else { Visibility::Inherited };
        if *v != want { *v = want; }
    }
}

/// The screens' layout (venue.js _drawScreen): each alliance's score and teams either side, the
/// match time and phase between
fn scoreboard_ui(mut screens: ResMut<Screens>, fonts: Option<Res<Fonts>>, mut commands: Commands) {
    if screens.ui { return }
    let Some(fonts) = fonts else { return };
    screens.ui = true;
    let white = Color::WHITE;
    let panel = |c: Color, score: ScreenText, teams: ScreenText, commands: &mut ChildSpawnerCommands| {
        commands
            .spawn((Node { width: Val::Px(180.0), height: Val::Px(190.0), flex_direction: FlexDirection::Column, align_items: AlignItems::Center, justify_content: JustifyContent::Center, row_gap: Val::Px(6.0), ..default() }, BackgroundColor(c)))
            .with_children(|p| {
                p.spawn((fonts.text("0", Face::Barlow(800), 96.0, white), score));
                p.spawn((fonts.text("", Face::Barlow(700), 22.0, white), teams));
            });
    };
    commands
        .spawn((
            Node { width: Val::Percent(100.0), height: Val::Percent(100.0), padding: UiRect::axes(Val::Px(12.0), Val::Px(0.0)), align_items: AlignItems::Center, justify_content: JustifyContent::SpaceBetween, ..default() },
            UiTargetCamera(screens.cam),
        ))
        .with_children(|p| {
            panel(Color::srgb_u8(0x1d, 0x4e, 0xd8), ScreenText::Blue, ScreenText::BlueTeams, p);
            p.spawn(Node { flex_direction: FlexDirection::Column, align_items: AlignItems::Center, row_gap: Val::Px(10.0), ..default() }).with_children(|c| {
                c.spawn(fonts.text("MATCH", Face::Barlow(700), 30.0, Color::srgb_u8(0xe5, 0xe7, 0xeb)));
                c.spawn((fonts.text("", Face::Barlow(800), 64.0, white), ScreenText::Clock));
                c.spawn((fonts.text("", Face::Barlow(700), 22.0, Color::srgb_u8(0xfb, 0xbf, 0x24)), ScreenText::Phase));
            });
            panel(Color::srgb_u8(0xc8, 0x1e, 0x1e), ScreenText::Red, ScreenText::RedTeams, p);
        });
}

/// The screens, a few times a second
fn scoreboard(time: Res<Time>, mut screens: ResMut<Screens>, game: Res<Game>, mut texts: Query<(&ScreenText, &mut Text)>) {
    screens.t -= time.delta_secs();
    if screens.t > 0.0 { return }
    screens.t = 0.25;
    let w = &game.0;
    let m = &w.match_;
    let teams = |a: Alliance| w.robots.iter().filter(|r| r.alliance == a).map(|r| r.key.clone()).collect::<Vec<_>>().join("  ");
    for (k, mut t) in &mut texts {
        let s = match k {
            ScreenText::Blue => format!("{}", m.total(Alliance::Blue)),
            ScreenText::Red => format!("{}", m.total(Alliance::Red)),
            ScreenText::BlueTeams => teams(Alliance::Blue),
            ScreenText::RedTeams => teams(Alliance::Red),
            ScreenText::Clock => m.clock_text(),
            ScreenText::Phase => m.phase_label().chars().take(13).collect(),
        };
        if t.0 != s { t.0 = s; }
    }
}
