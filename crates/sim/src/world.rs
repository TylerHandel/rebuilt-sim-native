//! The field and its FUEL as rebuilt-sim-2026 exports them (tools/export-native.mjs): every static
//! collider exactly as the browser game builds it, the physics settings, and where a match starts
//! the FUEL. Loading this instead of rebuilding the field by hand keeps both games' fields identical.

use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct WorldData {
    pub source: String,
    #[serde(default)]
    pub outposts: Option<Outposts>,
    #[serde(default, rename = "trenchArms")]
    pub trench_arms: Option<TrenchArms>,
    pub physics: PhysicsData,
    pub groups: Groups,
    pub field: Vec<ColliderData>,
    pub fuel: FuelData,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PhysicsData {
    pub gravity: f32,
    pub dt: f32,
    pub solver_iterations: usize,
    pub prediction_distance: Option<f32>,
    /// Every integration parameter the JS API has, as the JS game's engine holds them (checked
    /// against this sim's in tests/basics.rs)
    #[serde(default)]
    pub integration: serde_json::Map<String, serde_json::Value>,
}

/// The collision group bits (js/constants.js GROUP).
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub struct Groups {
    pub r#static: u32,
    pub ball: u32,
    pub robot: u32,
    pub wheel: u32,
    pub robot_barrier: u32,
    pub intake: u32,
    pub terrain: u32,
    pub held: u32,
    pub trench_arm: u32,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum ShapeData {
    Cuboid { half: [f32; 3] },
    Ball { radius: f32 },
    #[serde(rename_all = "camelCase")]
    Capsule { half_height: f32, radius: f32 },
    #[serde(rename_all = "camelCase")]
    Cylinder { half_height: f32, radius: f32 },
    Convex { points: Vec<f32> },
    Trimesh { vertices: Vec<f32>, indices: Vec<u32> },
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ColliderData {
    pub shape: ShapeData,
    pub translation: [f32; 3],
    /// x, y, z, w
    pub rotation: [f32; 4],
    pub friction: f32,
    pub restitution: f32,
    pub friction_combine: Option<u8>,
    pub restitution_combine: Option<u8>,
    /// Rapier's packing: memberships in the high 16 bits, filter in the low 16.
    pub collision_groups: u32,
    pub solver_groups: u32,
    pub sensor: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FuelBody {
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub ccd: bool,
    pub mass: f32,
    pub radius: f32,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FuelCollider {
    pub friction: f32,
    pub restitution: f32,
    pub collision_groups: u32,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FuelData {
    pub total: usize,
    pub body: FuelBody,
    pub collider: FuelCollider,
    /// The FUEL on the field when a match starts.
    pub start: Vec<[f32; 3]>,
    /// The FUEL the robots hold as preload (placed with the robots).
    pub preloaded: usize,
}

impl WorldData {
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }

    /// The copy built into this crate (assets/world.json).
    pub fn bundled() -> Self {
        Self::from_json(include_str!("../../../assets/world.json")).expect("assets/world.json")
    }
}

/// field.js outposts[a]: its world-space info
#[derive(Debug, Clone, Deserialize)]
pub struct Outposts { pub blue: Outpost, pub red: Outpost }

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Rect { pub x0: f64, pub x1: f64, pub z0: f64, pub z1: f64 }

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Opening { pub x: f64, pub y: f64, pub z: f64, pub half_w: f64 }

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Point { pub x: f64, pub y: f64, pub z: f64 }

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Outpost { pub corral: Rect, pub upper_opening: Opening, pub hp_throw_pos: Point, pub into_field: f64 }

/// physics.js: the TRENCH arms' boxes and the solver groups of the rule that lets a frame under them
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TrenchArms { pub arms: Vec<ArmBox>, pub arms_bottom: f64, pub arm_solver_groups: u32, pub under_solver_groups: u32 }

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct ArmBox { pub x: f64, pub z: f64, pub hx: f64, pub hz: f64 }
