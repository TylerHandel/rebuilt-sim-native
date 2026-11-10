//! The simulation, without graphics: the field, the FUEL and (later) the robots, their mechanisms and
//! the match rules, stepped at a fixed rate. The app renders it; tests and benchmarks run it headless.

pub mod ballistics;
pub mod constants;
pub mod field;
pub mod fuel;
pub mod game;
pub mod game_match;
pub mod human_player;
pub mod parity;
pub mod rng;
pub mod world;

use rapier3d::prelude::*;
pub use rapier3d::math::{Pose, Rotation, Vector};
pub use world::WorldData;

/// Rapier packs a collider's groups as memberships << 16 | filter (the JS API does, and the export
/// keeps that packing).
pub fn groups(packed: u32) -> InteractionGroups {
    InteractionGroups::new(
        Group::from_bits_truncate(packed >> 16),
        Group::from_bits_truncate(packed & 0xffff),
        InteractionTestMode::And,
    )
}

fn combine(rule: Option<u8>) -> CoefficientCombineRule {
    match rule {
        Some(1) => CoefficientCombineRule::Min,
        Some(2) => CoefficientCombineRule::Multiply,
        Some(3) => CoefficientCombineRule::Max,
        _ => CoefficientCombineRule::Average,
    }
}

fn shape(s: &world::ShapeData) -> ColliderBuilder {
    use world::ShapeData::*;
    let v3 = |f: &[f32]| f.chunks_exact(3).map(|p| Vector::new(p[0], p[1], p[2])).collect::<Vec<_>>();
    match s {
        Cuboid { half } => ColliderBuilder::cuboid(half[0], half[1], half[2]),
        Ball { radius } => ColliderBuilder::ball(*radius),
        Capsule { half_height, radius } => ColliderBuilder::capsule_y(*half_height, *radius),
        Cylinder { half_height, radius } => ColliderBuilder::cylinder(*half_height, *radius),
        Convex { points } => ColliderBuilder::convex_hull(&v3(points)).expect("convex hull"),
        Trimesh { vertices, indices } => ColliderBuilder::trimesh(
            v3(vertices),
            indices.chunks_exact(3).map(|t| [t[0], t[1], t[2]]).collect(),
        )
        .expect("trimesh"),
    }
}

/// The physics world with the field in it: the settings, the fixed body and its colliders, made in
/// the JS game's order (js/physics.js, js/field.js; as exported)
pub fn field_physics(data: &WorldData) -> (PhysicsWorld, RigidBodyHandle) {
    let mut physics = PhysicsWorld::default();
    physics.gravity = Vector::new(0.0, -data.physics.gravity, 0.0);
    let ip = &mut physics.integration_parameters;
    ip.dt = data.physics.dt;
    ip.num_solver_iterations = data.physics.solver_iterations;
    if let Some(p) = data.physics.prediction_distance {
        ip.normalized_prediction_distance = p;
    }
    let fixed = physics.insert_body(RigidBodyBuilder::fixed());
    for c in &data.field {
        let [x, y, z] = c.translation;
        let [qx, qy, qz, qw] = c.rotation;
        let pose = Pose::from_parts(Vector::new(x, y, z), Rotation::from_xyzw(qx, qy, qz, qw).normalize());
        let col = shape(&c.shape)
            .position(pose)
            .friction(c.friction)
            .restitution(c.restitution)
            .friction_combine_rule(combine(c.friction_combine))
            .restitution_combine_rule(combine(c.restitution_combine))
            .collision_groups(groups(c.collision_groups))
            .solver_groups(groups(c.solver_groups))
            .sensor(c.sensor);
        physics.insert_collider(col, Some(fixed));
    }
    (physics, fixed)
}

/// The field and the FUEL a match starts with, as bodies, and nothing else (the engine parity
/// scenario and the benchmark; the game is game::World)
pub struct Sim {
    pub physics: PhysicsWorld,
    pub data: WorldData,
    pub fixed: RigidBodyHandle,
    pub fuel: Vec<RigidBodyHandle>,
    /// Simulated time, s.
    pub time: f64,
}

impl Sim {
    pub fn new(data: WorldData) -> Self {
        let (physics, fixed) = field_physics(&data);
        let mut sim = Sim { physics, data, fixed, fuel: Vec::new(), time: 0.0 };
        let starts = sim.data.fuel.start.clone();
        for p in starts {
            sim.add_fuel(Vector::new(p[0], p[1], p[2]));
        }
        sim
    }

    pub fn add_fuel(&mut self, at: Vector) -> RigidBodyHandle {
        let f = &self.data.fuel;
        let body = RigidBodyBuilder::dynamic()
            .translation(at)
            .linear_damping(f.body.linear_damping)
            .angular_damping(f.body.angular_damping)
            .ccd_enabled(f.body.ccd)
            .can_sleep(true);
        let col = ColliderBuilder::ball(f.body.radius)
            .mass(f.body.mass)
            .friction(f.collider.friction)
            .restitution(f.collider.restitution)
            .collision_groups(groups(f.collider.collision_groups));
        let (h, _) = self.physics.insert(body, col);
        self.fuel.push(h);
        h
    }

    pub fn dt(&self) -> f32 {
        self.physics.integration_parameters.dt
    }

    pub fn step(&mut self) {
        self.physics.step();
        self.time += self.dt() as f64;
    }

    /// Each FUEL's position and orientation.
    pub fn fuel_poses(&self) -> impl Iterator<Item = (Vector, Rotation)> + '_ {
        self.fuel.iter().map(|h| {
            let b = &self.physics.bodies[*h];
            (b.translation(), *b.rotation())
        })
    }
}
