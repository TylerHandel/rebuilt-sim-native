//! js/field.js: what the game asks of the field besides its colliders (those are exported:
//! assets/world.json).
use crate::constants::*;

/// js `Field.hubCenter(alliance)`: (x, z)
pub fn hub_center(a: Alliance) -> (f64, f64) {
    let x = HUB::fx - HALF_L;
    if a == Alliance::Blue { (x, 0.0) } else { (-x, 0.0) }
}

/// js `Field.allianceLineX(alliance)`
pub fn alliance_line_x(a: Alliance) -> f64 {
    let x = -HALF_L + ALLIANCE_ZONE_DEPTH;
    if a == Alliance::Blue { x } else { -x }
}

#[derive(Debug, Clone, Copy)]
pub struct Slot { pub x: f64, pub y: f64, pub z: f64, edge: f64 }

/// js `field.neutralSlots(count)`: the NEUTRAL ZONE's FUEL, fewest from the outer corners in
pub fn neutral_slots(count: usize) -> Vec<Slot> {
    let r = FUEL::radius;
    let (cols, rows) = (34usize, 12usize);
    let (dz, dx) = (0.1535, 2.0 * r + 0.004);
    let mut slots = Vec::with_capacity(cols * rows);
    for row in 0..rows {
        let side = if row < 6 { -1.0 } else { 1.0 };
        let k = (row % 6) as f64;
        let x = side * (0.0254 + r + 0.002 + k * dx);
        for c in 0..cols {
            let z = (c as f64 - (cols as f64 - 1.0) / 2.0) * dz;
            let edge = jsmath::min(c as f64, (cols - 1 - c) as f64) + jsmath::min(k, 5.0 - k) * 0.5;
            slots.push(Slot { x, y: r + 0.002, z, edge });
        }
    }
    // (JS's sort is stable, as sort_by is)
    slots.sort_by(|a, b| a.edge.partial_cmp(&b.edge).unwrap());
    slots.split_off(slots.len() - count)
}

/// js `field.depotSlots(alliance)`
pub fn depot_slots(a: Alliance) -> Vec<Slot> {
    let r = FUEL::radius;
    let zc = HALF_W - DEPOT::fy;
    let mut slots = Vec::with_capacity(24);
    for i in 0..4 {
        for j in 0..6 {
            let x = -HALF_L + 0.004 + r + i as f64 * (2.0 * r + 0.001);
            let z = zc + (j as f64 - 2.5) * (2.0 * r + 0.002);
            slots.push(if a == Alliance::Blue { Slot { x, y: r + 0.002, z, edge: 0.0 } } else { Slot { x: -x, y: r + 0.002, z: -z, edge: 0.0 } });
        }
    }
    slots
}
