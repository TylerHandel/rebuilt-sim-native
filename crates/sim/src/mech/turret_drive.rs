//! js/mech/turretDrive.js: a turret on its motor (the motor model through the turret's gearing,
//! under the team's position loop, against its inertia, its bearings' and gears' friction and its
//! energy chain), so it gets where the aim asks as fast as its motor takes it; and js/echain.js's
//! chain load for 4414's spool (its arm table, made by the browser game's chain model: assets/mech).
use crate::mech::drives::Motor;
use crate::mech::r4414::GenChain;
use jsmath as m;

const DEG: f64 = m::PI / 180.0;
const TURN: f64 = 2.0 * m::PI;
fn sgn(x: f64) -> f64 { if x > 0.0 { 1.0 } else if x < 0.0 { -1.0 } else { 0.0 } }

/// js echain.js chainLoad*'s answer
#[derive(Clone, Copy, Debug)]
pub struct Load { pub torque: f64, pub friction: f64, pub viscous: f64 }

/// js CHAIN_LOSS
const ETA: f64 = 0.25;
const MU_PIN: f64 = 0.3;
const R_PIN: f64 = 0.0035;
const MU_SLIDE: f64 = 0.3;
const G: f64 = 9.81;

/// js `chainLoad4414(chain, psi)` (EI 3e-3, bushings 4 N)
pub fn chain_load4414(c: &GenChain, psi: f64) -> Load {
    let (ei, bushing) = (3e-3, 4.0);
    let p = c.pitch;
    let arm = m::max(0.02, arm_at(c, psi));
    let t = c.spring / 2.0;
    let jt = 2.0 * m::asin(p / (2.0 * c.rho_t));
    let jc = 2.0 * m::asin(p / (2.0 * c.rho_c));
    let (per_t, per_c) = (0.5 / p, 1.0 / p);
    let turn_per_m = 2.0 * m::PI / c.perim;
    let hyst = ETA * ei * (per_t * 2.0 * jt / c.rho_t + per_c * 2.0 * jc / c.rho_c + turn_per_m / c.rc);
    let pin = MU_PIN * R_PIN * t * (per_t * 2.0 * jt + per_c * 2.0 * jc + turn_per_m);
    Load { torque: -t * arm, friction: (hyst + pin + bushing / 2.0) * arm, viscous: 0.0 }
}
/// js `armAt(chain, psi)`: the arm table, once a degree
fn arm_at(c: &GenChain, psi: f64) -> f64 {
    let n = c.n as f64;
    let f = m::min(n, m::max(0.0, (psi - c.y0) / DEG));
    let i = m::min(n - 1.0, m::floor(f));
    let iu = i as usize;
    c.arm[iu] + (c.arm[iu + 1] - c.arm[iu]) * (f - i)
}

/// js `TrackChain` (8793's), as its load reads it (assets/mech/gen8793.json)
#[derive(Clone, Copy, Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TrackChain { pub ro: f64, pub ri: f64, pub rho: f64, pub a_f: f64, pub n: f64, pub mount_a: f64, pub pitch: f64, pub stop: f64, pub mass: f64 }
#[derive(serde::Deserialize)]
struct Gen8793 { chain: TrackChain }
/// 8793's chain
pub fn track8793() -> TrackChain {
    static G: std::sync::OnceLock<TrackChain> = std::sync::OnceLock::new();
    *G.get_or_init(|| serde_json::from_str::<Gen8793>(include_str!("../../../../assets/mech/gen8793.json")).expect("gen8793.json").chain)
}
impl TrackChain {
    /// js `_aU(L, aM)`
    fn a_u(&self, l: f64, a_m: f64) -> f64 { (l - m::PI * self.rho + self.ro * self.a_f + self.ri * a_m) / (self.ro + self.ri) }
    /// js `insideLinks(psi)`
    pub fn inside_links(&self, psi: f64) -> f64 {
        let a_m = self.mount_a + psi;
        let a_u = self.a_u(self.n * self.pitch, a_m);
        m::max(0.0, (self.ri * (a_u - a_m)) / self.pitch)
    }
}
/// js `chainLoad8793(chain, psi)` (EI 1.5e-3, wireMass 0.0012)
pub fn chain_load8793(c: &TrackChain, psi: f64) -> Load {
    let (ei, wire_mass) = (1.5e-3, 0.0012);
    let p = c.pitch;
    let per_rad = (c.ro * c.ri) / ((c.ro + c.ri) * p);
    let j_u = m::min(c.stop, 2.0 * m::asin(p / (2.0 * c.rho)));
    let (j_o, j_i) = (2.0 * m::asin(p / (2.0 * c.ro)), 2.0 * m::asin(p / (2.0 * c.ri)));
    let swing = (j_u - j_o) + (j_u + j_i);
    let mu = ei / c.rho;
    let hyst = ETA * mu * swing * per_rad;
    let fwall = mu / c.rho;
    let pin = MU_PIN * R_PIN * fwall * swing * per_rad;
    let slide_wall = MU_SLIDE * fwall * c.ri;
    let slide_floor = MU_SLIDE * c.inside_links(psi) * (c.mass + wire_mass) * G * c.ri;
    let k = ei / p;
    let spring = -per_rad * 0.5 * k * (j_i * j_i - j_o * j_o);
    Load { torque: spring, friction: hyst + pin + slide_wall + slide_floor, viscous: 0.0 }
}

/// js `SwivelChain` (1690's), as its load reads it (assets/mech/gen1690.json: its arm table once a degree)
#[derive(Clone, Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SwivelChain { pub y0: f64, pub n: f64, pub arm: Vec<f64>, pub pitch: f64, pub rho: f64, pub hub: f64, pub spring: f64, pub guide_a1: f64, pub guide_r: f64, pub a0: f64 }
/// js `chainLoad1690(chain, psi)` (EI 3e-3, slide 2 N, muGuide 0.15)
pub fn chain_load1690(c: &SwivelChain, psi: f64) -> Load {
    let (ei, slide, mu_guide) = (3e-3, 2.0, 0.15);
    let p = c.pitch;
    let f = m::min(c.n, m::max(0.0, (psi - c.y0) / DEG));
    let i = m::min(c.n - 1.0, m::floor(f));
    let iu = i as usize;
    let arm = c.arm[iu] + (c.arm[iu + 1] - c.arm[iu]) * (f - i);
    let t = c.spring / 2.0;
    let jw = 2.0 * m::asin(p / (2.0 * c.rho));
    let jh = 2.0 * m::asin(p / (2.0 * c.hub));
    let guide_turn = c.guide_a1 - c.a0;
    let hyst = ETA * ei * ((0.5 / p) * 2.0 * jw / c.rho + (1.0 / p) * jh / c.hub + guide_turn / (2.0 * m::PI) / c.guide_r);
    let pin = MU_PIN * R_PIN * t * ((0.5 / p) * 2.0 * jw + (1.0 / p) * jh);
    let capstan = t * (m::exp(mu_guide * guide_turn) - 1.0);
    let a = m::abs(arm);
    Load { torque: t * arm, friction: (hyst + pin + capstan + slide / 2.0) * a, viscous: 0.0 }
}
/// 1690's chain
pub fn swivel1690() -> &'static SwivelChain {
    #[derive(serde::Deserialize)]
    struct G { chain: SwivelChain }
    static C: std::sync::OnceLock<SwivelChain> = std::sync::OnceLock::new();
    C.get_or_init(|| serde_json::from_str::<G>(include_str!("../../../../assets/mech/gen1690.json")).expect("gen1690.json").chain)
}

/// What a turret's chain is
#[derive(Clone, Copy, Debug)]
pub enum Chain { Spool4414, Track8793, Swivel1690 }

/// js `TurretDrive`
#[derive(Clone, Debug)]
pub struct TurretDrive {
    pub motor: Motor,
    j: f64,
    friction: f64,
    viscous: f64,
    kp: f64, kd: f64, kv: f64, ks: f64,
    /// the feedforward against the chain (V per N·m), when it has one
    ff_v_per_nm: Option<f64>,
    max_rate: f64,
    limits: [f64; 2],
    chain: Option<Chain>,
    sub: usize,
    pub yaw: Option<f64>,
    pub omega: f64,
    goal: Option<f64>,
    t: Option<f64>,
    pub current: f64,
    pub load: f64,
}

impl TurretDrive {
    fn chain_load(&self, yaw: f64) -> Option<Load> {
        match self.chain { Some(Chain::Spool4414) => Some(chain_load4414(&crate::mech::r4414::gen4414().chain, yaw)), Some(Chain::Track8793) => Some(chain_load8793(&track8793(), yaw)), Some(Chain::Swivel1690) => Some(chain_load1690(swivel1690(), yaw)), None => None }
    }

    /// js `step(cur, goal, dt, t)`: from cur toward goal; where the turret is after
    pub fn step(&mut self, cur: f64, goal: f64, dt: f64, t: Option<f64>) -> f64 {
        let lapse = match (t, self.t) { (Some(t), Some(pt)) => t - pt > 1.5 * dt, _ => false };
        if self.yaw.is_none() || m::abs(cur - self.yaw.unwrap()) > 1e-9 || lapse { self.yaw = Some(cur); self.omega = 0.0; self.goal = None; }
        self.t = t;
        let mut rate = 0.0;
        if let Some(g0) = self.goal { let r = (goal - g0) / dt; if m::abs(r) <= self.max_rate * 1.5 { rate = m::max(-self.max_rate, m::min(self.max_rate, r)); } }
        self.goal = Some(goal);
        let k = self.motor.ratio / TURN;
        let h = dt / self.sub as f64;
        let [lo, hi] = self.limits;
        let mut yaw = self.yaw.unwrap();
        for _ in 0..self.sub {
            let e = (goal - yaw) * k;
            let v_ref = rate * k;
            let v = self.omega * k;
            let pid = self.kp * e + self.kd * (v_ref - v);
            let ff = match self.ff_v_per_nm { Some(vpn) => -self.chain_load(yaw).unwrap().torque * vpn, None => 0.0 };
            let volts = pid + self.kv * v_ref + (if m::abs(e) > 5e-4 { self.ks * sgn(pid) } else { 0.0 }) + ff;
            let mut duty = m::max(-1.0, m::min(1.0, volts / 12.0));
            if (yaw >= hi && duty > 0.0) || (yaw <= lo && duty < 0.0) { duty = 0.0; }
            let (tm, _) = self.motor.torque(self.omega, duty, 12.0);
            let l = self.chain_load(yaw);
            let fc = self.friction + l.map_or(0.0, |l| l.friction);
            let ta = tm + l.map_or(0.0, |l| l.torque) - (self.viscous + l.map_or(0.0, |l| l.viscous)) * self.omega;
            if self.omega == 0.0 {
                if m::abs(ta) > fc { self.omega = (h * (ta - sgn(ta) * fc)) / self.j; }
            } else {
                let w = self.omega + (h * (ta - sgn(self.omega) * fc)) / self.j;
                self.omega = if sgn(w) != sgn(self.omega) { 0.0 } else { w };
            }
            yaw += self.omega * h;
            if yaw > hi + 5.0 * DEG { yaw = hi + 5.0 * DEG; self.omega = m::min(0.0, self.omega); }
            if yaw < lo - 5.0 * DEG { yaw = lo - 5.0 * DEG; self.omega = m::max(0.0, self.omega); }
            self.current = self.motor.current;
            self.load = l.map_or(0.0, |l| l.torque - sgn(self.omega) * l.friction);
        }
        self.yaw = Some(yaw);
        yaw
    }
}

/// js `drive4414()`: an X44 through 55.7:1, their loop (estimates), a feedforward against the chain
pub fn drive4414() -> TurretDrive {
    let motor = Motor::new("krakenX44", 1.0, 55.7, 0.9, 40.0, 30.0);
    let v_per_nm = motor.rm / (motor.ratio * motor.eff * motor.kt);
    let j = 0.07 + motor.j;
    TurretDrive {
        motor, j, friction: 1.5, viscous: 0.02, kp: 1.5, kd: 0.05, kv: 0.095, ks: 0.09, ff_v_per_nm: Some(v_per_nm),
        max_rate: 360.0 * DEG, limits: [-200.0 * DEG, 200.0 * DEG], chain: Some(Chain::Spool4414), sub: 8,
        yaw: None, omega: 0.0, goal: None, t: None, current: 0.0, load: 0.0,
    }
}

/// js `drive8793()`: an X60 through 10:1, their loop, against the chain (no feedforward)
pub fn drive8793() -> TurretDrive {
    let motor = Motor::new("krakenX60", 1.0, 10.0, 0.9, 40.0, 30.0);
    let j = 0.06 + motor.j;
    TurretDrive {
        motor, j, friction: 4.0, viscous: 0.02, kp: 4.0, kd: 0.25, kv: 0.25, ks: 0.65, ff_v_per_nm: None,
        max_rate: 360.0 * DEG, limits: [-200.0 * DEG, 270.0 * DEG], chain: Some(Chain::Track8793), sub: 8,
        yaw: None, omega: 0.0, goal: None, t: None, current: 0.0, load: 0.0,
    }
}

/// js `drive1690()`: an X60 through 50:1, its loop (estimates), against the swivel chain
pub fn drive1690() -> TurretDrive {
    let motor = Motor::new("krakenX60", 1.0, 50.0, 0.9, 40.0, 30.0);
    let j = 0.07 + motor.j;
    TurretDrive {
        motor, j, friction: 1.0, viscous: 0.02, kp: 3.0, kd: 0.1, kv: 0.124, ks: 0.15, ff_v_per_nm: None,
        max_rate: 360.0 * DEG, limits: [-200.0 * DEG, 200.0 * DEG], chain: Some(Chain::Swivel1690), sub: 8,
        yaw: None, omega: 0.0, goal: None, t: None, current: 0.0, load: 0.0,
    }
}
