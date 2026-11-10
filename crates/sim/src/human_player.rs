//! js/humanPlayer.js: the HUMAN PLAYER at each OUTPOST. (Throwing at the HUB needs the shot
//! solver, ported with ballistics in M3: until then a HUMAN PLAYER set to throw panics.)
use crate::constants::*;
use crate::fuel::{rand, Fuel, LaunchOpts};
use crate::game_match::{ai, LaunchBy, LaunchInfo, Match, Phase};
use crate::rng::Mulberry32;
use rapier3d::prelude::PhysicsWorld;

/// What the driver tells their HUMAN PLAYER (js controls)
#[derive(Debug, Clone, Copy, Default)]
pub struct HpControls { pub toggle_door: bool, pub throw_held: bool }

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

    /// js `update(dt, t, controls)`
    pub fn update(&mut self, dt: f64, t: f64, c: HpControls, fuel: &mut Fuel, phys: &mut PhysicsWorld, m: &Match, rng: &mut Mulberry32) {
        self.throw_cooldown = jsmath::max(0.0, self.throw_cooldown - dt);
        let live = m.phase == Phase::Auto || m.phase == Phase::Teleop;
        if live {
            if c.toggle_door { self.door_open = !self.door_open; }
            let want_throw = c.throw_held || (self.auto && m.hub_active(self.alliance));
            if want_throw { todo!("HUMAN PLAYER throws need ballistics' ShotTable (M3)"); }
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
