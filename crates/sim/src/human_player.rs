//! js/humanPlayer.js: the HUMAN PLAYER at each OUTPOST: throwing FUEL at their HUB (from the CORRAL
//! first, then the CHUTE), the CHUTE DOOR, and reloading the CHUTE from the CORRAL.
use crate::constants::*;
use crate::fuel::{rand, Fuel, LaunchOpts};
use crate::game_match::{ai, LaunchBy, LaunchInfo, Match, Phase};
use crate::rng::Mulberry32;
use rapier3d::prelude::PhysicsWorld;

/// What the driver tells their HUMAN PLAYER (js controls)
#[derive(Debug, Clone, Copy, Default)]
pub struct HpControls { pub toggle_door: bool, pub throw_held: bool }

/// js util `gauss()`: a standard normal by Box-Muller, from the match's random stream
pub fn gauss(rng: &mut Mulberry32) -> f64 {
    let (mut u, mut v) = (0.0, 0.0);
    while u == 0.0 { u = rng.random(); }
    while v == 0.0 { v = rng.random(); }
    jsmath::sqrt(-2.0 * jsmath::log(u)) * jsmath::cos(2.0 * jsmath::PI * v)
}

/// js HumanPlayer's table: the HUMAN PLAYER's throws at the HUB (the same for both: the throw
/// position's height is the OUTPOSTS')
fn hp_table(h0: f64) -> crate::ballistics::ShotTable {
    use crate::ballistics::{ShotTable, TableOpts, H0};
    static T: std::sync::OnceLock<std::sync::Mutex<Vec<(u64, ShotTable)>>> = std::sync::OnceLock::new();
    let t = T.get_or_init(Default::default);
    if let Some((_, x)) = t.lock().unwrap().iter().find(|(k, _)| *k == h0.to_bits()) { return x.clone(); }
    let mut o = TableOpts::new(H0::At(h0), HUB::targetHeight, 18.0, 62.0, 14.0);
    o.clear_dist = HUB::size / 2.0 + FUEL::radius + 0.02;
    o.clear_height = HUB::rimFront + FUEL::radius + 0.04;
    let table = ShotTable::new(&o);
    t.lock().unwrap().push((h0.to_bits(), table.clone()));
    table
}

#[derive(Debug, Clone)]
pub struct HumanPlayer {
    pub alliance: Alliance,
    pub auto: bool,
    pub door_open: bool,
    pub release_timer: f64,
    pub throw_cooldown: f64,
    pub reload_timer: f64,
    pub arm_t: f64,
    pub thrown: u32,
    pub released: u32,
}

impl HumanPlayer {
    pub fn new(alliance: Alliance) -> Self {
        HumanPlayer { alliance, auto: false, door_open: false, release_timer: 0.0, throw_cooldown: 0.0, reload_timer: 0.0, arm_t: 0.0, thrown: 0, released: 0 }
    }

    /// js `throwOne(t)`: one FUEL at the HUB (from the CORRAL first, otherwise from the CHUTE)
    pub fn throw_one(&mut self, t: f64, fuel: &mut Fuel, phys: &mut PhysicsWorld, rng: &mut Mulberry32) -> bool {
        if self.throw_cooldown > 0.0 { return false; }
        let b = match fuel.corral_balls(self.alliance).first().copied() {
            Some(b) => { fuel.to_robot(phys, b); Some(b) } // (picked up by the HUMAN PLAYER)
            None => fuel.take_from_chute(self.alliance),
        };
        let Some(b) = b else { return false };
        let o = if self.alliance == Alliance::Blue { fuel.outposts.blue } else { fuel.outposts.red };
        let p = [o.hp_throw_pos.x, o.hp_throw_pos.y, o.hp_throw_pos.z];
        let c = crate::field::hub_center(self.alliance);
        let Some(sol) = crate::ballistics::solve_moving_shot(&hp_table(p[1]), p, [0.0, 0.0], [c.0, c.1]) else {
            fuel.add_to_chute(phys, self.alliance, b);
            return false;
        };
        // (a good HUMAN PLAYER lands roughly half of these ~6m throws)
        let deg = jsmath::PI / 180.0;
        let psi = sol.psi + gauss(rng) * 2.8 * deg;
        let th = sol.theta + gauss(rng) * 3.0 * deg;
        let v = sol.v * (1.0 + gauss(rng) * 0.045);
        let vel = [jsmath::cos(psi) * jsmath::cos(th) * v, jsmath::sin(th) * v, -jsmath::sin(psi) * jsmath::cos(th) * v];
        fuel.launch(phys, b, p, vel, LaunchOpts { info: Some(LaunchInfo { by: LaunchBy::Hp, alliance: Some(self.alliance), legal: true, t: Some(t) }), from_outside: true, t: Some(t), ..Default::default() });
        self.throw_cooldown = 1.05;
        self.arm_t = 0.35;
        self.thrown += 1;
        true
    }

    /// js `update(dt, t, controls)`
    pub fn update(&mut self, dt: f64, t: f64, c: HpControls, fuel: &mut Fuel, phys: &mut PhysicsWorld, m: &Match, rng: &mut Mulberry32) {
        self.throw_cooldown = jsmath::max(0.0, self.throw_cooldown - dt);
        let live = m.phase == Phase::Auto || m.phase == Phase::Teleop;
        if live {
            if c.toggle_door { self.door_open = !self.door_open; }
            let want_throw = c.throw_held || (self.auto && m.hub_active(self.alliance));
            if want_throw { self.throw_one(t, fuel, phys, rng); }
            // CHUTE DOOR open: FUEL rolls down the 15deg CHUTE and out the upper opening
            if self.door_open {
                self.release_timer += dt;
                while self.release_timer >= 0.15 {
                    self.release_timer -= 0.15;
                    let Some(b) = fuel.take_from_chute(self.alliance) else { break };
                    let o = if self.alliance == Alliance::Blue { fuel.outposts.blue } else { fuel.outposts.red };
                    let (op, s) = (o.upper_opening, o.into_field);
                    let z = op.z + rand(rng, -op.half_w, op.half_w);
                    let vx = s * rand(rng, 1.3, 2.0);
                    let vy = rand(rng, -0.3, 0.1);
                    let vz = rand(rng, -0.25, 0.25);
                    fuel.launch(phys, b, [op.x, op.y, z], [vx, vy, vz], LaunchOpts { info: Some(LaunchInfo { by: LaunchBy::Hp, alliance: Some(self.alliance), legal: true, t: Some(t) }), t: Some(t), ..Default::default() });
                    self.released += 1;
                }
            } else { self.release_timer = 0.0; }
            // reload the CHUTE from the CORRAL when the HUMAN PLAYER isn't throwing
            self.reload_timer += dt;
            if !want_throw && self.reload_timer > 0.7 && (fuel.chute_count(self.alliance) as f64) < OUTPOST::chuteCapacity {
                if let Some(&b) = fuel.corral_balls(self.alliance).first() {
                    fuel.add_to_chute(phys, self.alliance, b);
                    self.reload_timer = 0.0;
                    self.arm_t = 0.3;
                }
            }
        }
        self.arm_t = jsmath::max(0.0, self.arm_t - dt);
        let _ = ai;
    }
}
