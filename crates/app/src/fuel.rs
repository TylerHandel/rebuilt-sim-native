//! The FUEL: the sim steps at its fixed rate; each FUEL is drawn where the sim has it.
//! Space drops a heap of FUEL on the field; R resets the field.

use bevy::prelude::*;
use std::time::Instant;

#[derive(Resource)]
pub struct SimState(pub sim::Sim);

/// How long the last physics steps took (ms per step, smoothed).
#[derive(Resource, Default)]
pub struct StepTime(pub f32);

#[derive(Component)]
pub struct Fuel(pub usize);

#[derive(Resource)]
struct FuelLook(Handle<Mesh>, Handle<StandardMaterial>);

pub struct FuelPlugin;

impl Plugin for FuelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<StepTime>()
            .add_systems(Startup, setup)
            .add_systems(FixedUpdate, step)
            .add_systems(Update, (keys, spawn_new, sync).chain());
    }
}

fn setup(mut commands: Commands, sim: Res<SimState>, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let r = sim.0.data.fuel.body.radius;
    let mesh = meshes.add(Sphere::new(r).mesh().ico(3).unwrap());
    // the FUEL's yellow foam
    let mat = mats.add(StandardMaterial { base_color: Color::srgb(0.98, 0.80, 0.05), perceptual_roughness: 0.85, ..default() });
    commands.insert_resource(FuelLook(mesh, mat));
}

fn step(mut sim: ResMut<SimState>, mut st: ResMut<StepTime>) {
    let t = Instant::now();
    sim.0.step();
    let ms = t.elapsed().as_secs_f32() * 1000.0;
    st.0 = if st.0 == 0.0 { ms } else { st.0 * 0.95 + ms * 0.05 };
}

fn keys(keys: Res<ButtonInput<KeyCode>>, mut sim: ResMut<SimState>, mut commands: Commands, q: Query<Entity, With<Fuel>>) {
    if keys.just_pressed(KeyCode::Space) {
        for i in 0..60 {
            let (a, b, c) = ((i % 5) as f32, ((i / 5) % 4) as f32, (i / 20) as f32);
            sim.0.add_fuel(sim::Vector::new(-0.4 + a * 0.16, 2.0 + c * 0.16, 1.5 + b * 0.16));
        }
    }
    if keys.just_pressed(KeyCode::KeyR) {
        let data = sim.0.data.clone();
        sim.0 = sim::Sim::new(data);
        for e in &q { commands.entity(e).despawn(); }
    }
}

fn spawn_new(mut commands: Commands, sim: Res<SimState>, look: Res<FuelLook>, q: Query<&Fuel>) {
    let have = q.iter().count();
    for i in have..sim.0.fuel.len() {
        commands.spawn((Mesh3d(look.0.clone()), MeshMaterial3d(look.1.clone()), Transform::default(), Fuel(i)));
    }
}

fn sync(sim: Res<SimState>, mut q: Query<(&Fuel, &mut Transform)>) {
    for (f, mut t) in &mut q {
        let Some(h) = sim.0.fuel.get(f.0) else { continue };
        let b = &sim.0.physics.bodies[*h];
        t.translation = b.translation();
        t.rotation = *b.rotation();
    }
}
