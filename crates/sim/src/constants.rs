//! js/constants.js. Its values are generated from assets/constants.json (build.rs), with the JS
//! names: `FUEL.radius` is `FUEL::radius`, `GROUP.BALL` is `GROUP::BALL` (u32). Its functions are
//! ported below.
#![allow(non_snake_case, non_upper_case_globals, non_camel_case_types, dead_code)]
include!(concat!(env!("OUT_DIR"), "/constants.rs"));

/// An alliance (js: the strings 'blue' and 'red').
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Alliance { Blue, Red }
pub use Alliance::{Blue as BLUE_A, Red as RED_A};

/// js `other(a)`
pub fn other(a: Alliance) -> Alliance { if a == Alliance::Blue { Alliance::Red } else { Alliance::Blue } }
/// js `sideSign(a)`: blue half is -X, red half +X
pub fn side_sign(a: Alliance) -> f64 { if a == Alliance::Blue { 1.0 } else { -1.0 } }

/// js `fw(fx, fy, alliance)`: blue-side field coords to world (x, z) for that alliance's half
pub fn fw(fx: f64, fy: f64, alliance: Alliance) -> (f64, f64) {
    let x = fx - HALF_L;
    let z = HALF_W - fy;
    if alliance == Alliance::Blue { (x, z) } else { (-x, -z) }
}

/// js `groups(member, filter)`: Rapier's packing, a filter that takes STATIC taking TRENCH_ARM too
pub fn groups(member: u32, filter: u32) -> u32 {
    ((member & 0xffff) << 16) | ((if filter & GROUP::STATIC != 0 { filter | GROUP::TRENCH_ARM } else { filter }) & 0xffff)
}

/// One part of a TRENCH arm (js `trenchArmParts`): across the arm u0..u1, along it g0..g1
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmPart { pub u0: f64, pub u1: f64, pub g0: f64, pub g1: f64 }
/// js `trenchArmParts(fixed)`
pub fn trench_arm_parts(fixed: bool) -> Vec<ArmPart> {
    let skirt_in = TRENCH::width - 0.19 * IN; // (the skirt's inner side plate)
    if fixed { return vec![ArmPart { u0: -3.0 * IN, u1: 3.0 * IN, g0: -8.66 * IN, g1: skirt_in }]; }
    vec![
        ArmPart { u0: -2.0 * IN, u1: 2.0 * IN, g0: -3.25 * IN, g1: 43.07 * IN },  // main tube
        ArmPart { u0: -4.0 * IN, u1: 4.0 * IN, g0: 43.07 * IN, g1: 47.07 * IN },  // cross tube
        ArmPart { u0: -7.0 * IN, u1: -4.0 * IN, g0: 43.07 * IN, g1: 61.37 * IN }, // the fork, back to the pivot
        ArmPart { u0: 4.0 * IN, u1: 7.0 * IN, g0: 43.07 * IN, g1: 61.37 * IN },
    ]
}
