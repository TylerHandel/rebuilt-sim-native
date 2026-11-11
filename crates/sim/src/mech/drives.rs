//! js/mech/mechanism.js: motors, shafts and gear trains (Motor, Shaft, Train, velLoop). Shafts and
//! trains point at each other in JS; here they live in one arena (Drives) and refer by index.
use crate::drivetrain::{motor_spec, MotorSpec};
use jsmath as m;

const TAU: f64 = 2.0 * m::PI;

#[derive(Debug, Clone)]
pub struct Motor {
    pub kind: String, pub count: f64, pub ratio: f64, pub eff: f64, pub stator_limit: f64, pub supply_limit: f64,
    pub m: MotorSpec, pub rm: f64, pub kt: f64, pub kv: f64, pub j: f64, pub current: f64, pub limited: bool,
}

impl Motor {
    /// js `new Motor(type, { count, ratio, eff, statorLimit, supplyLimit })`
    pub fn new(kind: &str, count: f64, ratio: f64, eff: f64, stator_limit: f64, supply_limit: f64) -> Self {
        let mm = motor_spec(kind);
        let rm = mm.v / mm.stall_current;
        let kt = mm.stall_torque / mm.stall_current;
        let kv = mm.free / (mm.v - mm.free_current * rm);
        let j = count * mm.rotor_inertia * ratio * ratio;
        Motor { kind: kind.into(), count, ratio, eff, stator_limit, supply_limit, m: mm, rm, kt, kv, j, current: 0.0, limited: false }
    }
    /// (js defaults: count 1, ratio 1, eff 0.9, statorLimit 60, supplyLimit Infinity)
    pub fn x(kind: &str) -> Self { Motor::new(kind, 1.0, 1.0, 0.9, 60.0, f64::INFINITY) }

    /// js `torque(w, cmd, V)`: (tau, limited)
    pub fn torque(&mut self, w: f64, cmd: f64, v: f64) -> (f64, bool) {
        let eb = (w * self.ratio) / self.kv;
        let i = (cmd * v - eb) / self.rm;
        let e = m::sign(i) * eb;
        let isup = self.supply_limit * v;
        let cap = m::min(self.stator_limit, if isup < f64::INFINITY { (-e + m::sqrt(e * e + 4.0 * self.rm * isup)) / (2.0 * self.rm) } else { f64::INFINITY });
        let lim = m::abs(i) > cap;
        let ic = if lim { m::sign(i) * cap } else { i };
        self.current = ic * self.count;
        self.limited = lim;
        (self.count * self.kt * ic * self.ratio * self.eff, lim)
    }
    /// js `get slope()`
    pub fn slope(&self) -> f64 { (self.count * self.kt * self.ratio * self.ratio * self.eff) / (self.kv * self.rm) }
    /// js `get freeSpeed()`
    pub fn free_speed(&self) -> f64 { self.m.free / self.ratio }
}

/// A controller's velocity loop (js shaft.vel / train.vel)
#[derive(Debug, Clone, Copy)]
pub struct VelLoop { pub target: f64, pub kv: f64, pub kp: f64, pub ks: f64, pub lo: f64, pub hi: f64, pub per: Option<f64>, pub rate: f64 }

#[derive(Debug, Clone)]
pub struct Shaft {
    pub name: String,
    pub motor: Option<Motor>,
    pub inertia: f64, pub drag: f64, pub friction: f64,
    p0: f64, acc: f64, b: f64,
    pub omega_: f64,
    /// duty (-1..1); None: coasting (js cmd === null)
    pub cmd: Option<f64>,
    pub train: Option<usize>,
    pub k: f64,
    pub lo: f64, pub hi: f64, free: f64,
    pub load: f64,
    pub vel: Option<VelLoop>,
    /// js r7769.js Racks (and 1114's): its racks' friction holds it still against small pushes,
    /// its stops (lo, hi) as well: integrated and pushed as if free, then held to them
    pub racks: bool,
    /// js r604.js's cam: not a shaft of its own but one geared off `cam.0` by `cam.1` (its speed that
    /// one's times k; immovable to what pushes on it, its impulses passed on times k)
    pub cam: Option<(usize, f64)>,
}

impl Shaft {
    /// js `new Shaft({ name, motor, inertia = 5e-5, drag = 2e-4, friction = 0 })`
    pub fn new(name: &str, motor: Option<Motor>, inertia: f64, drag: f64, friction: f64) -> Self {
        Shaft { name: name.into(), motor, inertia, drag, friction, p0: 0.0, acc: 0.0, b: 0.0, omega_: 0.0, cmd: Some(0.0), train: None, k: 1.0,
            lo: f64::NEG_INFINITY, hi: f64::INFINITY, free: 0.0, load: 0.0, vel: None, racks: false, cam: None }
    }
}

#[derive(Debug, Clone)]
pub struct Coupled { pub motor: Motor, pub k: f64, pub cmd: Option<f64> }

#[derive(Debug, Clone)]
pub struct Train {
    pub name: String,
    pub motor: Motor,
    pub omega: f64,
    pub cmd: Option<f64>,
    pub shafts: Vec<usize>,
    pub coupled: Vec<Coupled>,
    pub vel: Option<VelLoop>,
}

/// Every shaft and train of a robot's mechanisms
#[derive(Debug, Clone, Default)]
pub struct Drives { pub shafts: Vec<Shaft>, pub trains: Vec<Train> }

impl Drives {
    pub fn shaft(&mut self, s: Shaft) -> usize { self.shafts.push(s); self.shafts.len() - 1 }
    pub fn train(&mut self, name: &str, motor: Motor) -> usize {
        self.trains.push(Train { name: name.into(), motor, omega: 0.0, cmd: Some(0.0), shafts: Vec::new(), coupled: Vec::new(), vel: None });
        self.trains.len() - 1
    }
    /// js `train.add(s, k)`
    pub fn add(&mut self, t: usize, s: Shaft, k: f64) -> usize {
        let i = self.shaft(s);
        self.shafts[i].train = Some(t);
        self.shafts[i].k = k;
        self.trains[t].shafts.push(i);
        i
    }

    /// js `shaft.omega`
    pub fn omega(&self, s: usize) -> f64 {
        let sh = &self.shafts[s];
        if let Some((o, k)) = sh.cam { return self.omega(o) * k; }
        match sh.train { Some(t) => self.trains[t].omega * sh.k, None => sh.omega_ }
    }
    /// js `shaft.omega = v`
    pub fn set_omega(&mut self, s: usize, v: f64) {
        let (t, k) = (self.shafts[s].train, self.shafts[s].k);
        match t { Some(t) => self.trains[t].omega = v / k, None => self.shafts[s].omega_ = v }
    }
    /// js `train.J`
    pub fn train_j(&self, t: usize) -> f64 {
        let tr = &self.trains[t];
        tr.motor.j + tr.coupled.iter().fold(0.0, |a, c| a + c.motor.j * c.k * c.k)
            + tr.shafts.iter().fold(0.0, |a, &s| a + self.shafts[s].inertia * self.shafts[s].k * self.shafts[s].k)
    }
    /// js `train.drag`
    pub fn train_drag(&self, t: usize) -> f64 {
        self.trains[t].shafts.iter().fold(0.0, |a, &s| a + self.shafts[s].drag * self.shafts[s].k * self.shafts[s].k)
    }
    /// js `shaft.J`
    pub fn j(&self, s: usize) -> f64 {
        let sh = &self.shafts[s];
        if sh.cam.is_some() { return f64::INFINITY; }
        match sh.train { Some(t) => self.train_j(t) / (sh.k * sh.k), None => sh.inertia + sh.motor.as_ref().map_or(0.0, |mo| mo.j) }
    }
    /// js `shaft.stop(lo, hi)`
    pub fn stop(&mut self, s: usize, lo: f64, hi: f64) {
        let sh = &mut self.shafts[s];
        sh.lo = lo; sh.hi = hi; sh.free = sh.omega_;
        sh.omega_ = m::min(hi, m::max(lo, sh.omega_));
    }
    /// js `shaft.impulse(L)`
    pub fn impulse(&mut self, s: usize, l: f64) {
        if let Some((o, k)) = self.shafts[s].cam { return self.impulse(o, l * k); }
        if self.shafts[s].racks && self.shafts[s].train.is_none() {
            let (lo, hi) = (self.shafts[s].lo, self.shafts[s].hi);
            self.shafts[s].lo = f64::NEG_INFINITY; self.shafts[s].hi = f64::INFINITY;
            self.shafts[s].racks = false;
            self.impulse(s, l);
            let sh = &mut self.shafts[s];
            sh.racks = true; sh.lo = lo; sh.hi = hi;
            sh.omega_ = m::min(hi, m::max(lo, sh.omega_));
            return;
        }
        if let Some(t) = self.shafts[s].train {
            let tj = self.train_j(t);
            self.trains[t].omega += (l * self.shafts[s].k) / tj;
        } else if self.shafts[s].b > 0.0 {
            self.shafts[s].acc += l;
            let acc = self.shafts[s].acc;
            self.fric(s, acc);
        } else if self.shafts[s].lo == f64::NEG_INFINITY && self.shafts[s].hi == f64::INFINITY {
            let j = self.j(s);
            self.shafts[s].omega_ += l / j;
        } else {
            let j = self.j(s);
            let sh = &mut self.shafts[s];
            sh.free += l / j;
            sh.omega_ = m::min(sh.hi, m::max(sh.lo, sh.free));
        }
    }
    fn fric(&mut self, s: usize, acc: f64) {
        let j = self.j(s);
        let sh = &mut self.shafts[s];
        let p = sh.p0 + acc;
        sh.omega_ = (m::sign(p) * m::max(0.0, m::abs(p) - sh.b)) / j;
    }

    /// js `shaft.integrate(dt, V, slice)`
    pub fn integrate_shaft(&mut self, s: usize, dt: f64, v: f64, slice: Option<(f64, f64)>) {
        if self.shafts[s].train.is_some() { return; }
        if self.shafts[s].racks {
            let (lo, hi) = (self.shafts[s].lo, self.shafts[s].hi);
            self.shafts[s].lo = f64::NEG_INFINITY; self.shafts[s].hi = f64::INFINITY;
            self.shafts[s].racks = false;
            self.integrate_shaft(s, dt, v, slice);
            let sh = &mut self.shafts[s];
            sh.racks = true; sh.lo = lo; sh.hi = hi;
            sh.omega_ = m::min(hi, m::max(lo, sh.omega_));
            return;
        }
        let j = self.j(s);
        if self.shafts[s].vel.is_some() && self.shafts[s].motor.is_some() {
            let vl = self.shafts[s].vel.unwrap();
            let per = vl.per.unwrap_or(self.shafts[s].motor.as_ref().unwrap().ratio / TAU);
            vel_loop(vl, per, dt, slice, &mut |me: &mut Self, set: bool, h: f64| {
                if set { let om = me.shafts[s].omega_; me.shafts[s].cmd = Some(vel_cmd(vl, per, om)); }
                if h > 0.0 { me.shaft_drive(s, h, v, j); }
            }, self);
        } else { self.shaft_drive(s, dt, v, j); }
        let sh = &mut self.shafts[s];
        if sh.lo != f64::NEG_INFINITY || sh.hi != f64::INFINITY { sh.free = sh.omega_; sh.omega_ = m::min(sh.hi, m::max(sh.lo, sh.omega_)); }
        if sh.friction > 0.0 && sh.lo == f64::NEG_INFINITY && sh.hi == f64::INFINITY {
            sh.p0 = sh.omega_ * j; sh.acc = 0.0; sh.b = sh.friction * dt;
            self.fric(s, 0.0);
        } else { self.shafts[s].b = 0.0; }
    }
    fn shaft_drive(&mut self, s: usize, dt: f64, v: f64, j: f64) {
        let sh = &mut self.shafts[s];
        match (&mut sh.motor, sh.cmd) {
            (None, _) | (Some(_), None) => {
                sh.omega_ = (sh.omega_ + (dt * sh.load) / j) / (1.0 + (dt * sh.drag) / j);
                if let Some(mo) = &mut sh.motor { mo.current = 0.0; }
            }
            (Some(mo), Some(cmd)) => {
                let (tau, limited) = mo.torque(sh.omega_, cmd, v);
                let kd = if limited { 0.0 } else { mo.slope() };
                sh.omega_ = (j * sh.omega_ + dt * (tau + sh.load + kd * sh.omega_)) / (j + dt * (kd + sh.drag));
            }
        }
    }

    /// js `train.integrate(dt, V, slice)`
    pub fn integrate_train(&mut self, t: usize, dt: f64, v: f64, slice: Option<(f64, f64)>) {
        let j = self.train_j(t);
        if let Some(vl) = self.trains[t].vel {
            let per = vl.per.unwrap_or(self.trains[t].motor.ratio / TAU);
            vel_loop(vl, per, dt, slice, &mut |me: &mut Self, set: bool, h: f64| {
                if set { let om = me.trains[t].omega; me.trains[t].cmd = Some(vel_cmd(vl, per, om)); }
                if h > 0.0 { me.train_drive(t, h, v, j); }
            }, self);
        } else { self.train_drive(t, dt, v, j); }
    }
    fn train_drive(&mut self, t: usize, dt: f64, v: f64, j: f64) {
        let drag = self.train_drag(t);
        let tr = &mut self.trains[t];
        let (mut tau, mut kd) = (0.0, 0.0);
        let om = tr.omega;
        // (the main motor, then the coupled ones, in order)
        match tr.cmd {
            None => tr.motor.current = 0.0,
            Some(cmd) => { let (ta, lim) = tr.motor.torque(om * 1.0, cmd, v); tau += 1.0 * ta; if !lim { kd += 1.0 * 1.0 * tr.motor.slope(); } }
        }
        for c in &mut tr.coupled {
            match c.cmd {
                None => c.motor.current = 0.0,
                Some(cmd) => { let (ta, lim) = c.motor.torque(om * c.k, cmd, v); tau += c.k * ta; if !lim { kd += c.k * c.k * c.motor.slope(); } }
            }
        }
        tr.omega = (j * tr.omega + dt * (tau + kd * tr.omega)) / (j + dt * (kd + drag));
    }
}

/// the duty a velocity loop sets from the speed now (js velLoop's set)
fn vel_cmd(vl: VelLoop, per: f64, omega: f64) -> f64 {
    let tgt = vl.target * per;
    let volts = vl.kv * tgt + vl.kp * (tgt - omega * per) + vl.ks * m::sign(tgt);
    m::min(1.0, m::max(-1.0, m::min(vl.hi, m::max(vl.lo, volts)) / 12.0))
}

/// js `velLoop(a, dt, drive, slice)`: f(me, set, h) sets the duty first when `set`, then runs the motor h long
fn vel_loop(vl: VelLoop, _per: f64, dt: f64, slice: Option<(f64, f64)>, f: &mut dyn FnMut(&mut Drives, bool, f64), me: &mut Drives) {
    let rate = vl.rate;
    match slice {
        None => {
            let n = m::max(1.0, m::ceil(dt * rate)) as usize;
            for _ in 0..n { f(me, true, dt / n as f64); }
        }
        Some((t0, big_t)) => {
            let n = m::max(1.0, m::ceil(big_t * rate));
            let p = big_t / n;
            let t1 = t0 + dt;
            let mut t = t0;
            while t < t1 - 1e-12 {
                let i = m::floor(t / p + 1e-9);
                let end = m::min(t1, (i + 1.0) * p);
                let set = m::abs(t - i * p) < 1e-9;
                f(me, set, end - t);
                t = end;
            }
        }
    }
}
