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

fn setup(mut commands: Commands, game: Res<Game>, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let r = game.0.data.fuel.body.radius;
    let mut m = |c: Color, rough: f32| mats.add(StandardMaterial { base_color: c, perceptual_roughness: rough, ..default() });
    commands.insert_resource(Looks {
        fuel: (meshes.add(Sphere::new(r).mesh().ico(3).unwrap()), m(Color::srgb(0.98, 0.80, 0.05), 0.85)),
        blue: m(Color::srgb(0.10, 0.25, 0.85), 0.8),
        red: m(Color::srgb(0.85, 0.12, 0.10), 0.8),
        frame: m(Color::srgb(0.55, 0.57, 0.60), 0.4),
        wheel: m(Color::srgb(0.05, 0.05, 0.05), 0.9),
        part: m(Color::srgb(0.30, 0.32, 0.35), 0.6),
    });
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
    models: Query<Entity, With<RobotModel>>,
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
    for (f, mut t, mut v) in &mut fuel {
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
