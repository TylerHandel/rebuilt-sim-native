//! Draws the game: every FUEL where the sim has it, and each robot as its colliders (its bumpers in
//! its alliance's color) until the robots' CAD models come over.

use bevy::prelude::*;
use rapier3d::prelude::ColliderHandle;
use sim::constants::Alliance;

use crate::field::collider_mesh;
use crate::game::{Game, WorldGen};
use crate::robot_model::{self, AssetDir, RobotModel};

pub struct DrawPlugin;

impl Plugin for DrawPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup).add_systems(PostUpdate, (respawn, sync, hide_boxes).chain());
    }
}

#[derive(Component)]
struct FuelView(usize);

/// One of robot `.1`'s colliders, drawn (until its model is in)
#[derive(Component)]
struct PartView(ColliderHandle, usize);

#[derive(Resource)]
struct Looks {
    fuel: (Handle<Mesh>, Handle<StandardMaterial>),
    blue: Handle<StandardMaterial>,
    red: Handle<StandardMaterial>,
    frame: Handle<StandardMaterial>,
    wheel: Handle<StandardMaterial>,
    part: Handle<StandardMaterial>,
}

fn setup(mut commands: Commands, game: Res<Game>, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>, mut images: ResMut<Assets<Image>>, fonts: Res<crate::ui::Fonts>) {
    let r = game.0.data.fuel.body.radius;
    // (js fuelSkin.js fuelMaterial: the molded skin, vivid and only a little glossy)
    let skin = fuel_skin(&mut commands, &mut images, &fonts);
    let fuel_mat = mats.add(StandardMaterial {
        base_color_texture: Some(skin), perceptual_roughness: 0.55, reflectance: 0.4, clearcoat: 0.25, clearcoat_perceptual_roughness: 0.4, ..default()
    });
    let mut m = |c: Color, rough: f32| mats.add(StandardMaterial { base_color: c, perceptual_roughness: rough, ..default() });
    commands.insert_resource(Looks {
        // (24 x 12: round at any distance it's seen from, and its ~500 drawn again into each
        // shadow cascade: the triangles count)
        fuel: (meshes.add(Sphere::new(r).mesh().uv(24, 12)), fuel_mat),
        blue: m(Color::srgb(0.10, 0.25, 0.85), 0.8),
        red: m(Color::srgb(0.85, 0.12, 0.10), 0.8),
        frame: m(Color::srgb(0.55, 0.57, 0.60), 0.4),
        wheel: m(Color::srgb(0.05, 0.05, 0.05), 0.9),
        part: m(Color::srgb(0.30, 0.32, 0.35), 0.6),
    });
}

/// js fuelSkin.js fuelTexture: FUEL's skin, wrapped round it (longitude across, latitude down): the
/// lemon yellow of the official ball, its molding's mid seam, a black print on one side and a date
/// code on the other, so its spin shows from any side. Drawn once, by a UI into a texture.
fn fuel_skin(commands: &mut Commands, images: &mut Assets<Image>, fonts: &crate::ui::Fonts) -> Handle<Image> {
    use crate::ui::Face;
    use bevy::camera::RenderTarget;
    let (w, h) = (512.0, 256.0);
    let image = images.add(Image::new_target_texture(w as u32, h as u32, bevy::render::render_resource::TextureFormat::Rgba8UnormSrgb, None));
    let cam = commands.spawn((
        Camera2d,
        Camera { order: -20, clear_color: ClearColorConfig::Custom(Color::srgb_u8(0xe9, 0xce, 0x02)), ..default() },
        RenderTarget::Image(image.clone().into()),
    )).id();
    // (its camera keeps drawing: stopped after its first frames, a slow first load could leave the
    // skin undrawn, and a 512 x 256 UI is next to nothing)
    let ink = Color::srgb_u8(0x16, 0x16, 0x16);
    // (a line of text centered on (cx, cy))
    let line = |s: &'static str, size: f32, weight: u16, cx: f32, cy: f32| {
        (Node { position_type: PositionType::Absolute, left: Val::Px(cx - 128.0), top: Val::Px(cy - size * 0.62), width: Val::Px(256.0), justify_content: JustifyContent::Center, ..default() },
         children![fonts.text(s, Face::Barlow(weight), size, ink)])
    };
    commands.spawn((Node { width: Val::Percent(100.0), height: Val::Percent(100.0), ..default() }, UiTargetCamera(cam))).with_children(|p| {
        p.spawn((Node { position_type: PositionType::Absolute, left: Val::Px(0.0), top: Val::Px(h / 2.0 - 1.5), width: Val::Percent(100.0), height: Val::Px(3.0), ..default() }, BackgroundColor(Color::srgb_u8(0xc4, 0xa9, 0x00))));
        p.spawn(line("FUEL", 38.0, 800, w * 0.25, h * 0.39));
        p.spawn(line("REBUILT 2026", 21.0, 700, w * 0.25, h * 0.6));
        p.spawn(line("026", 17.0, 700, w * 0.75, h * 0.44));
    });
    image
}


/// At the start and after a restart (a new World), make the views of its bodies.
fn respawn(
    mut commands: Commands,
    game: Res<Game>,
    world_gen: Res<WorldGen>,
    mut made: Local<Option<u32>>,
    looks: Res<Looks>,
    mut meshes: ResMut<Assets<Mesh>>,
    fuel: Query<Entity, With<FuelView>>,
    parts: Query<Entity, With<PartView>>,
    models: Query<Entity, (With<RobotModel>, Without<crate::showcase::Showcase>)>,
    assets: Res<AssetServer>,
    dir: Res<AssetDir>,
) {
    if *made == Some(world_gen.0) { return }
    *made = Some(world_gen.0);
    let w = &game.0;
    for e in fuel.iter().chain(parts.iter()).chain(models.iter()) { commands.entity(e).despawn(); }
    for i in 0..w.fuel.balls.len() {
        commands.spawn((Mesh3d(looks.fuel.0.clone()), MeshMaterial3d(looks.fuel.1.clone()), Transform::default(), FuelView(i)));
    }
    for (i, r) in w.robots.iter().enumerate() {
        robot_model::spawn(&mut commands, &assets, &dir, i, &r.key);
        let body = &w.physics.bodies[r.body];
        for &h in body.colliders() {
            let c = &w.physics.colliders[h];
            let Some(mesh) = collider_mesh(c.shape().as_typed_shape()) else { continue };
            let mat = if h == r.bumper {
                if r.alliance == Alliance::Blue { looks.blue.clone() } else { looks.red.clone() }
            } else if h == r.frame {
                looks.frame.clone()
            } else if c.shape().as_ball().is_some() {
                looks.wheel.clone()
            } else {
                looks.part.clone()
            };
            commands.spawn((Mesh3d(meshes.add(mesh)), MeshMaterial3d(mat), Transform::default(), PartView(h, i)));
        }
    }
}

fn sync(game: Res<Game>, mut fuel: Query<(&FuelView, &mut Transform, &mut Visibility), Without<PartView>>, mut parts: Query<(&PartView, &mut Transform), Without<FuelView>>) {
    let w = &game.0;
    // (a generic robot's FUEL are in its hopper, in its frame, not bodies: drawn where it holds them)
    let mut held: std::collections::HashMap<usize, (Vec3, Quat)> = std::collections::HashMap::new();
    for r in &w.robots {
        let (Some(g), Some(b)) = (&r.generic, w.physics.bodies.get(r.body)) else { continue };
        let (bt, bq) = (b.translation(), *b.rotation());
        for e in &g.hopper.list { held.insert(e.b, (bt + bq * Vec3::new(e.p[0] as f32, e.p[1] as f32, e.p[2] as f32), bq)); }
    }
    for (f, mut t, mut v) in &mut fuel {
        if let Some(&(p, q)) = held.get(&f.0) {
            if *v != Visibility::Inherited { *v = Visibility::Inherited; }
            t.translation = p;
            t.rotation = q;
            continue;
        }
        let Some(b) = w.fuel.balls.get(f.0).and_then(|b| w.physics.bodies.get(b.body)) else { continue };
        let show = b.is_enabled() && b.translation().y > -1.0;
        let want = if show { Visibility::Inherited } else { Visibility::Hidden };
        if *v != want { *v = want; }
        t.translation = b.translation();
        t.rotation = *b.rotation();
    }
    for (p, mut t) in &mut parts {
        let Some(c) = w.physics.colliders.get(p.0) else { continue };
        let pose = c.position();
        t.translation = pose.translation;
        t.rotation = pose.rotation;
    }
}

/// A robot whose model is in isn't drawn as its colliders.
fn hide_boxes(models: Query<&RobotModel>, mut parts: Query<(&PartView, &mut Visibility)>) {
    for (p, mut v) in &mut parts {
        let hide = models.iter().any(|m| m.robot == p.1 && m.ready);
        let want = if hide { Visibility::Hidden } else { Visibility::Inherited };
        if *v != want { *v = want; }
    }
}
