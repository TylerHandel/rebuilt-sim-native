//! 2910's physical robot (js/mech/phys2910.js, js/mech/r2910.js), so far: its intake pivot, the
//! part of it that moves the robot's colliders (the hopper slides out as the arm comes down).
//! Its rollers, floor, drum and hood arrive with the mechanism core (M3b).
use crate::mech::drives::{Drives, Motor, Shaft};
use jsmath as m;

const DEG: f64 = m::PI / 180.0;
/// js r2910 INTAKE (the parts used here)
pub const DEPLOY: f64 = 114.3;
pub const RETRACT: f64 = 10.0;
const CAD_DEG: f64 = -1.5;
const MASS: f64 = 8.1;
const COM: f64 = 0.24;

/// js PIVOT_MM: their Motion Magic (degrees of arm), Slot0 kP and kV
struct Mm { v: f64, a: f64, kp: f64, kv: f64 }
fn pivot_mm() -> Mm {
    Mm { v: (5500.0 / 60.0 / 40.0) * 360.0, a: 4000.0, kp: (5.0 * 40.0) / 360.0, kv: 12.0 / ((5800.0 / 60.0) / (40.0 / 360.0)) }
}

/// js `profile(p, v, goal, vmax, amax, dt)`: a trapezoidal profile's next setpoint
pub fn profile(p: f64, v: f64, goal: f64, vmax: f64, amax: f64, dt: f64) -> [f64; 2] {
    let d = goal - p;
    let dir = m::sign(d);
    if m::abs(d) < 1e-6 && m::abs(v) < 1e-6 { return [goal, 0.0]; }
    let v_stop = m::sqrt(2.0 * amax * m::abs(d));
    let want = dir * m::min(vmax, v_stop);
    let nv = v + m::max(-amax * dt, m::min(amax * dt, want - v));
    let np = p + nv * dt;
    if (goal - np) * dir < 0.0 { return [goal, 0.0]; }
    [np, nv]
}

fn intake_rot(deg: f64) -> f64 { ((DEPLOY - deg) * m::PI) / 180.0 }

pub struct Phys2910 {
    pub drives: Drives,
    pub pivot: usize,
    pub arm_deg: f64,
    pub arm_set: [f64; 2],
    pub arm_goal: f64,
}

impl Phys2910 {
    pub fn new() -> Self {
        let mut drives = Drives::default();
        let pivot = drives.shaft(Shaft::new("intake pivot", Some(Motor::new("krakenX60", 1.0, 40.0, 0.9, 75.0, 30.0)), 2.0 * MASS * COM * COM, 0.5, 0.0));
        let mut p = Phys2910 { drives, pivot, arm_deg: 0.0, arm_set: [0.0, 0.0], arm_goal: DEPLOY };
        p.reset();
        p
    }
    pub fn reset(&mut self) {
        self.arm_deg = 0.0; self.arm_set = [0.0, 0.0];
        self.arm_goal = DEPLOY;
        self.drives.shafts[self.pivot].omega_ = 0.0;
        self.drives.shafts[self.pivot].cmd = Some(0.0);
    }
    fn arm_angle(&self) -> f64 { CAD_DEG * DEG + intake_rot(self.arm_deg) }

    /// js `preSteps` up to its yield (the parts that move the robot)
    pub fn pre_mech(&mut self, dt: f64, on: bool) {
        let mm = pivot_mm();
        self.arm_set = profile(self.arm_set[0], self.arm_set[1], self.arm_goal, mm.v, mm.a, dt);
        let volts = if on { mm.kv * self.arm_set[1] + mm.kp * (self.arm_set[0] - self.arm_deg) } else { 0.0 };
        self.drives.shafts[self.pivot].cmd = Some(m::max(-1.0, m::min(1.0, -volts / 12.0)));
        let l = -MASS * 9.81 * COM * m::cos(self.arm_angle()) * dt;
        self.drives.impulse(self.pivot, l);
    }
    /// the mechanisms' step, for the pivot: its shaft's motor (Mechanism.step's drives; V 12)
    pub fn mech_run(&mut self, dt: f64) { self.drives.integrate_shaft(self.pivot, dt, 12.0, None); }
    /// js `preSteps` after its yield: the arm turns as its shaft did; returns the robot's intakeDeploy
    pub fn post_mech(&mut self, dt: f64) -> f64 {
        let om = self.drives.omega(self.pivot);
        self.arm_deg -= (om * dt) / DEG;
        if self.arm_deg > DEPLOY { self.arm_deg = DEPLOY; if self.drives.shafts[self.pivot].omega_ < 0.0 { self.drives.shafts[self.pivot].omega_ = 0.0; } }
        if self.arm_deg < -2.0 { self.arm_deg = -2.0; if self.drives.shafts[self.pivot].omega_ > 0.0 { self.drives.shafts[self.pivot].omega_ = 0.0; } }
        m::max(0.0, m::min(1.0, self.arm_deg / DEPLOY))
    }
    /// js `postStep`, its intake's goal (no FUEL held, not feeding: DEPLOY, or as it was when off)
    pub fn post_step(&mut self, on: bool, intaking: bool, feed: bool) {
        if intaking { self.arm_goal = DEPLOY; }
        else if feed { self.arm_goal = RETRACT; }
        else { self.arm_goal = if on { DEPLOY } else { self.arm_goal }; }
    }
}
