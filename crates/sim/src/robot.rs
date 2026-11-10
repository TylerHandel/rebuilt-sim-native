//! js/robot.js, so far for a physical robot driving with nothing in it: its rigid body as built
//! (_createBody), spawn, the drive (_preStepDrive, _alongWalls, _wheelsDown, tip damping), the
//! hopper sliding out (_hopper) and the intake's collider (_stepTopColliders). Shooting, the
//! intake's FUEL and the pose estimate arrive with the mechanisms (M3).
use crate::constants::*;
use crate::drivetrain::{DriveCfg, Drivetrain, SwerveDrive};
use crate::mech::r2910::Phys2910;
use crate::parity::Cmd;
use jsmath as m;
use rapier3d::prelude::*;
use serde_json::Value;

const WHEEL_R: f64 = 0.05;
const TIP_RATE: f64 = 2.5;
const TIP_DAMP: f64 = 4.0;

fn num(v: &Value, k: &str) -> f64 { v[k].as_f64().unwrap_or_else(|| panic!("config: no number {k}")) }
fn opt(v: &Value, k: &str) -> Option<f64> { v.get(k).and_then(|x| x.as_f64()) }
fn vf(x: f64, y: f64, z: f64) -> Vector { Vector::new(x as f32, y as f32, z as f32) }
fn grp(g: u32) -> InteractionGroups { crate::groups(g) }

/// three.js Vector3.applyQuaternion (r169), on f64s
pub fn apply_quat(v: [f64; 3], q: [f64; 4]) -> [f64; 3] {
    let [vx, vy, vz] = v;
    let [qx, qy, qz, qw] = q;
    let tx = 2.0 * (qy * vz - qz * vy);
    let ty = 2.0 * (qz * vx - qx * vz);
    let tz = 2.0 * (qx * vy - qy * vx);
    [vx + qw * tx + qy * tz - qz * ty, vy + qw * ty + qz * tx - qx * tz, vz + qw * tz + qx * ty - qy * tx]
}

/// js robotConfigs `frameShape(cfg)`
pub fn frame_shape(cfg: &Value) -> Vec<[f64; 2]> {
    let f = &cfg["frame"];
    let (l, w) = (num(f, "length"), num(f, "width"));
    if let Some(round) = f.get("round").filter(|x| !x.is_null()) {
        let (r, cx, front) = (num(round, "r"), num(round, "cx"), num(round, "front"));
        let mut pts = vec![[l / 2.0, front / 2.0]];
        for i in 0..=24 { let a = m::PI / 2.0 + (i as f64 / 24.0) * m::PI; pts.push([cx + r * m::cos(a), r * m::sin(a)]); }
        pts.push([l / 2.0, -front / 2.0]);
        return pts;
    }
    let c = opt(f, "chamfer").unwrap_or(0.0);
    let mut pts = vec![[l / 2.0, w / 2.0], [l / 2.0, -w / 2.0]];
    if c > 0.0 { pts.extend([[-l / 2.0 + c, -w / 2.0], [-l / 2.0, -w / 2.0 + c], [-l / 2.0, w / 2.0 - c], [-l / 2.0 + c, w / 2.0]]); }
    else { pts.extend([[-l / 2.0, -w / 2.0], [-l / 2.0, w / 2.0]]); }
    pts
}

/// js robotConfigs `offsetShape(pts, d)`
pub fn offset_shape(pts: &[[f64; 2]], d: f64) -> Vec<[f64; 2]> {
    let n = pts.len();
    let mut area = 0.0;
    for i in 0..n { let (a, b) = (pts[i], pts[(i + 1) % n]); area += a[0] * b[1] - b[0] * a[1]; }
    let s = if area > 0.0 { 1.0 } else { -1.0 };
    let nrm = |a: [f64; 2], b: [f64; 2]| { let (dx, dz) = (b[0] - a[0], b[1] - a[1]); let l0 = m::hypot2(dx, dz); let l = if l0 == 0.0 { 1.0 } else { l0 }; [(s * dz) / l, (-s * dx) / l] };
    (0..n).map(|i| {
        let p = pts[i];
        let n0 = nrm(pts[(i + n - 1) % n], p);
        let n1 = nrm(p, pts[(i + 1) % n]);
        let (mx, mz) = (n0[0] + n1[0], n0[1] + n1[1]);
        let k = d / m::max(0.2, mx * n0[0] + mz * n0[1]);
        [p[0] + mx * k, p[1] + mz * k]
    }).collect()
}

/// js robotConfigs `modulePoints(cfg)`
pub fn module_points(cfg: &Value) -> Vec<[f64; 2]> {
    let f = &cfg["frame"];
    let inset = 0.075;
    if let Some(at) = cfg["drive"].get("moduleAt").and_then(|x| x.as_array()) {
        let (x, z) = (at[0].as_f64().unwrap(), at[1].as_f64().unwrap());
        return vec![[x, z], [x, -z], [-x, z], [-x, -z]];
    }
    let rw = opt(&cfg["drive"], "wheel").unwrap_or(4.0 * IN) / 2.0;
    let chamfer = opt(f, "chamfer").unwrap_or(0.0);
    let cut = if chamfer != 0.0 { m::max(inset, chamfer / 2.0 + (rw + 0.006) * m::SQRT1_2) } else { inset };
    if let Some(round) = f.get("round").filter(|x| !x.is_null()) {
        let (r, cx) = (num(round, "r"), num(round, "cx"));
        let d = (r - 0.1) * m::SQRT1_2;
        return vec![[cx + d, d], [cx + d, -d], [cx - d, d], [cx - d, -d]];
    }
    let (l, w) = (num(f, "length"), num(f, "width"));
    [[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]].iter().map(|&[sx, sz]| { let d = if sx < 0.0 { cut } else { inset }; [sx * (l / 2.0 - d), sz * (w / 2.0 - d)] }).collect()
}

/// What decides whether a collider reaches a TRENCH arm by the end of the step (physics.js
/// passesUnder's top())
#[derive(Debug, Clone)]
pub enum Top { Frame { body: RigidBodyHandle, pts: Vec<f64> }, Lid { body: RigidBodyHandle, lid: ColliderHandle, hl: f64, hw: f64 } }

/// js physics.underArm entry
#[derive(Debug, Clone)]
pub struct UnderArm { pub collider: ColliderHandle, pub top: Top, pub under: bool }

pub enum Phys { R2910(Phys2910) }

pub struct Robot {
    pub key: String,
    pub cfg: Value,
    pub alliance: Alliance,
    pub body: RigidBodyHandle,
    pub bumper: ColliderHandle,
    pub frame: ColliderHandle,
    pub hopper: Option<(ColliderHandle, ColliderHandle)>,
    pub hopper_y: f64,
    pub arm_collider: ColliderHandle,
    pub arm_wall: Option<ColliderHandle>,
    pub intake_block: Option<(f64, f64, f64)>,
    pub lid: Option<(ColliderHandle, f64)>,
    pub wheel_pts: Vec<[f64; 2]>,
    pub half_l: f64,
    pub half_w: f64,
    pub height: f64,
    pub swerve: SwerveDrive,
    pub max_speed: f64,
    pub max_omega: f64,
    pub phys: Option<Phys>,
    pub cmd: Cmd,
    pub enabled: bool,
    pub intake_deploy: f64,
    pub hopper_deploy: f64,
    pub last_t: f64,
    pub pos: [f64; 3],
    pub quat: [f64; 4],
    pub yaw: f64,
    pub vel: [f64; 3],
    pub omega: f64,
    pub traction: f64,
}

impl Robot {
    /// js `new Robot(...)`: its body made in the world now (after the FUEL and any robot before it);
    /// under: the world's physics.underArm, which its frame (and any lid) join
    pub fn new(key: &str, cfg: Value, alliance: Alliance, phys: &mut PhysicsWorld, under: &mut Vec<UnderArm>) -> Self {
        let t = BUMPER_T;
        let half_l = num(&cfg["frame"], "length") / 2.0 + t;
        let half_w = num(&cfg["frame"], "width") / 2.0 + t;
        let height = num(&cfg, "height");
        let mass = num(&cfg, "mass");
        let pts = module_points(&cfg);
        let dcfg: DriveCfg = serde_json::from_value(cfg["drive"].clone()).unwrap();
        let drivetrain = Drivetrain::new(&dcfg, mass, m::hypot2(pts[0][0], pts[0][1]));
        let swerve = SwerveDrive::new(drivetrain, pts.clone());
        let mut r = Robot {
            key: key.into(), cfg, alliance, body: RigidBodyHandle::invalid(), bumper: ColliderHandle::invalid(), frame: ColliderHandle::invalid(),
            hopper: None, hopper_y: 0.0, arm_collider: ColliderHandle::invalid(), arm_wall: None, intake_block: None, lid: None,
            wheel_pts: Vec::new(), half_l, half_w, height, swerve,
            max_speed: 0.0, max_omega: 0.0, phys: None, cmd: Cmd::default(), enabled: false, intake_deploy: 0.0, hopper_deploy: 0.0,
            last_t: 0.0, pos: [0.0; 3], quat: [0.0, 0.0, 0.0, 1.0], yaw: 0.0, vel: [0.0; 3], omega: 0.0, traction: 0.0,
        };
        r.max_speed = num(&r.cfg["drive"], "maxSpeed");
        r.max_omega = num(&r.cfg["drive"], "maxOmega");
        r.create_body(phys, under);
        r.phys = match r.cfg.get("physical").and_then(|x| x.as_str()) {
            Some("2910") => Some(Phys::R2910(Phys2910::new())),
            Some(p) => panic!("robot {p}'s mechanisms aren't ported yet"),
            None => panic!("robot {key} isn't physical: not in the native game (PLAN.md D11)"),
        };
        r.reset();
        r
    }

    fn create_body(&mut self, phys: &mut PhysicsWorld, under: &mut Vec<UnderArm>) {
        let cfg = self.cfg.clone();
        let mass = num(&cfg, "mass");
        let dt = phys.integration_parameters.dt as f64;
        self.body = phys.insert_body(RigidBodyBuilder::dynamic().ccd_enabled(true).can_sleep(false).linear_damping(0.0).angular_damping(0.0));
        let body = self.body;
        let robot_groups = groups(GROUP::ROBOT, GROUP::STATIC | GROUP::TERRAIN | GROUP::BALL | GROUP::ROBOT_BARRIER | GROUP::ROBOT | GROUP::INTAKE);
        let (y0, y1) = (BUMPER_Y0, BUMPER_Y1);
        let (bc, cr) = (0.03, 0.04);
        let shape = frame_shape(&cfg);
        let mut pts: Vec<Vector> = Vec::new();
        for (y, inset) in [(y0, bc), (y0 + bc, 0.0), (y1 - 0.025, 0.0), (y1, 0.025)] {
            for [x, z] in offset_shape(&shape, BUMPER_T - inset - cr) {
                for k in 0..8 { let a = (k as f64 / 8.0) * m::PI * 2.0; pts.push(vf(x + cr * m::cos(a), y, z + cr * m::sin(a))); }
            }
        }
        self.bumper = phys.insert_collider(ColliderBuilder::convex_hull(&pts).unwrap().mass((mass * 0.15) as f32)
            .friction(0.05).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.1).collision_groups(grp(robot_groups)), Some(body));
        let (sl, sw) = (num(&cfg["frame"], "length") / 2.0, num(&cfg["frame"], "width") / 2.0);
        let mut up: Vec<f64> = Vec::new();
        for [x, z] in offset_shape(&shape, -0.05) { up.extend([x, y1, z]); }
        for [x, z] in &shape { up.extend([*x, y1 + 0.05, *z, *x, self.height, *z]); }
        let upv: Vec<Vector> = up.chunks_exact(3).map(|p| vf(p[0], p[1], p[2])).collect();
        self.frame = phys.insert_collider(ColliderBuilder::convex_hull(&upv).unwrap().mass((mass * 0.3) as f32)
            .friction(0.05).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.1)
            .collision_groups(grp(groups(GROUP::ROBOT, GROUP::STATIC | GROUP::TERRAIN | GROUP::BALL | GROUP::ROBOT_BARRIER | GROUP::ROBOT | GROUP::INTAKE))), Some(body));
        passes_under(phys, under, self.frame, Top::Frame { body, pts: up });
        let _ = dt;
        // most of a robot's weight is low: mass only, it touches nothing
        phys.insert_collider(ColliderBuilder::cuboid(sl as f32, 0.03, sw as f32).translation(vf(0.0, 0.05, 0.0)).mass((mass * 0.5) as f32).collision_groups(grp(0)), Some(body));
        for [mx, mz] in module_points(&cfg) {
            self.wheel_pts.push([mx, mz]);
            phys.insert_collider(ColliderBuilder::ball(WHEEL_R as f32).translation(vf(mx, WHEEL_R, mz)).mass((mass * 0.0125) as f32)
                .friction(0.0).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.0)
                .collision_groups(grp(groups(GROUP::WHEEL, GROUP::STATIC | GROUP::TERRAIN | GROUP::ROBOT_BARRIER | GROUP::BALL))), Some(body));
        }
        // the extending hopper section
        let st = &cfg["storage"];
        let ext = opt(st, "extLen").unwrap_or(0.0);
        let bx = st.get("box");
        let box_false = matches!(bx, Some(Value::Bool(false)));
        let box_obj = bx.filter(|b| b.is_object());
        let ic = &cfg["intake"];
        let fold = ic.get("fold").filter(|x| !x.is_null());
        if ext != 0.0 && !box_false && (fold.is_none() || box_obj.is_some()) {
            let (by0, by1) = match box_obj { Some(b) => (num(b, "y0"), num(b, "y1")), None => (0.17, self.height) };
            let half_h = (by1 - by0) / 2.0;
            self.hopper_y = by0 + half_h;
            let (hx, hy, hz) = (ext / 2.0, half_h, match box_obj { Some(b) => num(b, "hw"), None => num(&cfg["frame"], "width") / 2.0 - 0.02 });
            let shp = || -> ColliderBuilder {
                if let Some(lip) = st.get("lip").and_then(|x| x.as_array()) {
                    let (lx, ly) = (lip[0].as_f64().unwrap(), lip[1].as_f64().unwrap());
                    let cx = num(&cfg["frame"], "length") / 2.0 + hx;
                    let mut p = Vec::new();
                    for z in [-hz, hz] {
                        for [x, y] in [[-hx, -hy], [-hx, hy], [hx, hy], [hx, ly - self.hopper_y], [lx - cx, -hy]] { p.push(vf(x, y, z)); }
                    }
                    ColliderBuilder::convex_hull(&p).unwrap()
                } else { ColliderBuilder::cuboid(hx as f32, hy as f32, hz as f32) }
            };
            let tr = vf(num(&cfg["frame"], "length") / 2.0 - ext / 2.0, self.hopper_y, 0.0);
            let col = phys.insert_collider(shp().translation(tr).mass(if box_obj.is_some() { 0.0 } else { 0.5 }).friction(0.05).restitution(0.1).collision_groups(grp(0)), Some(body));
            let wall = phys.insert_collider(shp().translation(tr).density(0.0).friction(0.1).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.1).collision_groups(grp(0)), Some(body));
            self.hopper = Some((col, wall));
        }
        let bay = &cfg["bay"];
        let solid = groups(GROUP::ROBOT, GROUP::STATIC | GROUP::ROBOT | GROUP::INTAKE);
        if let Some(lift) = bay.get("lift").filter(|x| !x.is_null()) {
            let _ = lift;
            let x1 = bay.get("slope").filter(|x| !x.is_null()).map_or(num(bay, "x1"), |s| num(s, "x"));
            let x0 = num(bay, "x0");
            let lid_x = (x0 + x1) / 2.0;
            let (hl, hw) = ((x1 - x0) / 2.0, num(bay, "hw"));
            let lid = phys.insert_collider(ColliderBuilder::cuboid(hl as f32, 0.006, hw as f32).translation(vf(lid_x, num(bay, "top"), 0.0))
                .density(0.0).friction(0.05).restitution(0.1).collision_groups(grp(solid)), Some(body));
            passes_under(phys, under, lid, Top::Lid { body, lid, hl, hw });
            self.lid = Some((lid, lid_x));
        }
        let intake_groups = groups(GROUP::INTAKE, GROUP::STATIC | GROUP::ROBOT | GROUP::INTAKE | GROUP::ROBOT_BARRIER);
        let _ = intake_groups;
        if fold.is_some() || ic.get("slide").filter(|x| !x.is_null()).is_some() {
            panic!("folding and sliding intakes arrive with their robots (M3c)");
        }
        let (bhx, by0, by1) = (num(ic, "reach") / 2.0 + 0.02, 0.03, 0.19);
        self.intake_block = Some((bhx, by0, by1));
        self.arm_collider = phys.insert_collider(ColliderBuilder::cuboid(bhx as f32, ((by1 - by0) / 2.0) as f32, (num(ic, "width") / 2.0) as f32)
            .translation(vf(self.half_l - bhx, (by0 + by1) / 2.0, 0.0))
            .density(0.0).friction(0.05).friction_combine_rule(CoefficientCombineRule::Min).restitution(0.05).collision_groups(grp(0)), Some(body));
        // deployed intake roller, and its side plates
        let (reach, iw) = (num(ic, "reach"), num(ic, "width"));
        phys.insert_collider(ColliderBuilder::cuboid(0.03, 0.05, (iw / 2.0) as f32).translation(vf(self.half_l + reach - 0.05, 0.1, 0.0))
            .mass(0.1).friction(0.1).restitution(0.05).collision_groups(grp(0)), Some(body));
        for sz in [-1.0, 1.0] {
            phys.insert_collider(ColliderBuilder::cuboid((reach / 2.0) as f32, 0.08, 0.006).translation(vf(self.half_l + reach / 2.0, 0.11, sz * (iw / 2.0 + 0.006)))
                .density(0.0).friction(0.1).restitution(0.05).collision_groups(grp(0)), Some(body));
        }
    }

    /// js `reset()` (what play reads so far)
    pub fn reset(&mut self) {
        if let Some(Phys::R2910(p)) = &mut self.phys { p.reset(); }
        self.last_t = 0.0;
        self.intake_deploy = 0.0;
        self.hopper_deploy = 0.0;
        self.enabled = false;
        self.cmd = Cmd::default();
    }

    /// js `spawn(x, z, yaw)`
    pub fn spawn(&mut self, phys: &mut PhysicsWorld, x: f64, z: f64, yaw: f64) {
        let b = &mut phys.bodies[self.body];
        b.set_body_type(RigidBodyType::Dynamic, true);
        b.set_translation(vf(x, 0.003, z), true);
        let q = [0.0, m::sin(yaw / 2.0), 0.0, m::cos(yaw / 2.0)];
        b.set_rotation(Rotation::from_xyzw(q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32), true);
        b.set_linvel(vf(0.0, 0.0, 0.0), true);
        b.set_angvel(vf(0.0, 0.0, 0.0), true);
        self.yaw = yaw;
        let h = yaw / 2.0;
        self.quat = [0.0, m::sin(h), 0.0, m::cos(h)];
        self.pos = [x, 0.0, z];
        self.vel = [0.0; 3];
        self.omega = 0.0;
        self.swerve.reset();
    }

    /// js `preStep(dt)` for a robot alone (its mechanisms between its two halves)
    pub fn pre_step(&mut self, phys: &mut PhysicsWorld, dt: f64, teleop: bool) {
        self.pre_step_drive(phys, dt, teleop);
        let on = self.enabled;
        match &mut self.phys {
            Some(Phys::R2910(p)) => { p.pre_mech(dt, on); p.mech_run(dt); self.intake_deploy = p.post_mech(dt); }
            None => {}
        }
    }

    fn pre_step_drive(&mut self, phys: &mut PhysicsWorld, dt: f64, teleop: bool) {
        {
            let b = &mut phys.bodies[self.body];
            b.reset_forces(true);
            b.reset_torques(true);
        }
        let (mut tvx, mut tvz, mut tw) = (0.0, 0.0, 0.0);
        if self.enabled {
            tvx = self.cmd.vx; tvz = self.cmd.vz; tw = self.cmd.omega;
            let w = self.along_walls(phys, tvx, tvz);
            tvx = w[0]; tvz = w[1];
        }
        let sp = m::hypot2(tvx, tvz);
        if sp > self.max_speed { tvx *= self.max_speed / sp; tvz *= self.max_speed / sp; }
        tw = crate::fuel::clamp(tw, -self.max_omega, self.max_omega);
        let wheel = m::hypot2(tvx, tvz) + m::abs(tw) * self.swerve.d.r;
        if wheel > self.max_speed { let k = self.max_speed / wheel; tvx *= k; tvz *= k; tw *= k; }
        let down = self.wheels_down(phys);
        self.traction = down.iter().filter(|x| **x).count() as f64 / self.wheel_pts.len() as f64;
        {
            let b = &mut phys.bodies[self.body];
            let av = b.angvel();
            let k = 1.0 / (1.0 + TIP_DAMP * dt);
            let (mut ax, mut az) = (av.x as f64 * k, av.z as f64 * k);
            let h = m::hypot2(ax, az);
            if h > TIP_RATE { ax *= TIP_RATE / h; az *= TIP_RATE / h; }
            b.set_angvel(vf(ax, av.y as f64, az), true);
        }
        let b = &mut phys.bodies[self.body];
        let q = b.rotation();
        let (qx, qy, qz, qw) = (q.x as f64, q.y as f64, q.z as f64, q.w as f64);
        let yaw = m::atan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz));
        self.swerve.d.limits_for(teleop);
        self.swerve.step(b, yaw, tvx, tvz, tw, down, dt);
    }

    /// js `_alongWalls(vx, vz)`
    fn along_walls(&self, phys: &PhysicsWorld, mut vx: f64, mut vz: f64) -> [f64; 2] {
        let sp = m::hypot2(vx, vz);
        if sp < 0.05 { return [vx, vz]; }
        let wall = GROUP::STATIC | GROUP::ROBOT_BARRIER;
        let mut walls: Vec<[f64; 2]> = Vec::new();
        let mut cs = vec![self.bumper, self.frame, self.arm_wall.unwrap_or(self.arm_collider)];
        if let Some((_, w)) = self.hopper { cs.push(w); }
        let fixed_parent = |h: ColliderHandle| phys.colliders[h].parent().map(|p| phys.bodies[p].is_fixed()).unwrap_or(false);
        for c in cs {
            let cg = phys.colliders[c].collision_groups();
            if cg.memberships.bits() == 0 && cg.filter.bits() == 0 { continue; }
            for pair in phys.narrow_phase.contact_pairs_with(c) {
                let o = if pair.collider1 == c { pair.collider2 } else { pair.collider1 };
                let og = phys.colliders[o].collision_groups().memberships.bits();
                if !fixed_parent(o) || (og & wall) == 0 { continue; }
                let flipped = pair.collider1 != c;
                for mf in pair.manifolds() {
                    if mf.points.is_empty() || mf.points[0].dist > 0.02 { continue; }
                    let n = mf.data.normal;
                    let k = if flipped { -1.0 } else { 1.0 };
                    let (nx, nz) = (n.x as f64, n.z as f64);
                    let h = m::hypot2(nx, nz);
                    if h < 0.7 { continue; }
                    walls.push([(k * nx) / h, (k * nz) / h]);
                }
            }
        }
        for [nx, nz] in walls {
            let into = vx * nx + vz * nz;
            if into <= 0.05 || into > 0.966 * m::hypot2(vx, vz) { continue; }
            vx -= (into - 0.05) * nx; vz -= (into - 0.05) * nz;
        }
        [vx, vz]
    }

    /// js `_wheelsDown()`: a short ray down from each wheel's center
    fn wheels_down(&self, phys: &PhysicsWorld) -> Vec<bool> {
        let b = &phys.bodies[self.body];
        let q = b.rotation();
        let qv = [q.x as f64, q.y as f64, q.z as f64, q.w as f64];
        let p = b.translation();
        let floor = groups(GROUP::WHEEL, GROUP::STATIC | GROUP::TERRAIN | GROUP::BALL);
        self.wheel_pts.iter().map(|w| {
            let v = apply_quat([w[0], WHEEL_R, w[1]], qv);
            let ray = Ray::new(vf(p.x as f64 + v[0], p.y as f64 + v[1], p.z as f64 + v[2]), Vector::new(0.0, -1.0, 0.0));
            phys.cast_ray(&ray, (WHEEL_R + 0.02) as f32, true, QueryFilter::default().groups(grp(floor)).exclude_rigid_body(self.body)).is_some()
        }).collect()
    }

    /// js `postStep(dt, t)` (what moves the robot: its pose read back, the hopper, the intake's collider)
    pub fn post_step(&mut self, phys: &mut PhysicsWorld, dt: f64, t: f64) {
        let b = &phys.bodies[self.body];
        let (tr, q) = (b.translation(), *b.rotation());
        self.pos = [tr.x as f64, tr.y as f64, tr.z as f64];
        self.quat = [q.x as f64, q.y as f64, q.z as f64, q.w as f64];
        let [qx, qy, qz, qw] = self.quat;
        let fx = 1.0 - 2.0 * (qy * qy + qz * qz);
        let fz = 2.0 * (qx * qz - qw * qy);
        self.yaw = m::atan2(-fz, fx);
        let lv = b.linvel();
        self.vel = [lv.x as f64, 0.0, lv.z as f64];
        self.omega = b.angvel().y as f64;
        let on = self.enabled;
        self.last_t = t;
        // (2910's intake pivot is its mechanisms': its hopper still slides out with the first deploy)
        self.hopper_step(phys, dt, on);
        self.step_top_colliders(phys);
        if let Some(Phys::R2910(p)) = &mut self.phys { p.post_step(on, on && self.cmd.intake, false); }
    }

    /// js `_hopper(dt, on)` (nothing held: what the load needs of the extension is nothing)
    fn hopper_step(&mut self, phys: &mut PhysicsWorld, dt: f64, on: bool) {
        let st = &self.cfg["storage"];
        let ext = opt(st, "extLen").unwrap_or(0.0);
        if ext == 0.0 { return; }
        let latched = st["extend"].as_str() == Some("latched");
        let mut want = if latched { if self.hopper_deploy > 0.02 || self.intake_deploy > 0.3 { 1.0 } else { 0.0 } } else { self.intake_deploy };
        want = m::max(want, 0.0);
        if on || want < self.hopper_deploy { self.hopper_deploy = approach(self.hopper_deploy, want, dt / 0.45); }
        let Some((col, wall)) = self.hopper else { return };
        let out = self.hopper_deploy * ext;
        let fx = num(&self.cfg["frame"], "length") / 2.0 - ext / 2.0 + out;
        phys.colliders[col].set_position_wrt_parent(Pose::from_translation(vf(fx, self.hopper_y, 0.0)));
        phys.colliders[wall].set_position_wrt_parent(Pose::from_translation(vf(fx, self.hopper_y, 0.0)));
        let ball = if st.get("box").filter(|b| b.is_object()).is_some() { 0 } else { GROUP::BALL };
        let is_out = self.hopper_deploy > 0.05;
        phys.colliders[col].set_collision_groups(grp(if is_out { groups(GROUP::ROBOT, ball | GROUP::ROBOT | GROUP::INTAKE) } else { 0 }));
        phys.colliders[wall].set_collision_groups(grp(if is_out { groups(GROUP::ROBOT, GROUP::ROBOT_BARRIER | GROUP::STATIC) } else { 0 }));
    }

    /// js `_stepTopColliders()` (an intake block; lids and folding or sliding intakes later)
    fn step_top_colliders(&mut self, phys: &mut PhysicsWorld) {
        let ic = &self.cfg["intake"];
        if let Some((bhx, by0, by1)) = self.intake_block {
            let out = self.intake_deploy * (num(ic, "reach") + 0.04);
            let x = match opt(ic, "front") { Some(front) => self.half_l + self.intake_deploy * (front - self.half_l) - bhx, None => self.half_l - bhx + out - 0.02 };
            phys.colliders[self.arm_collider].set_position_wrt_parent(Pose::from_translation(vf(x, (by0 + by1) / 2.0, 0.0)));
            phys.colliders[self.arm_collider].set_collision_groups(grp(if self.intake_deploy > 0.05 { groups(GROUP::INTAKE, GROUP::STATIC | GROUP::ROBOT | GROUP::INTAKE | GROUP::ROBOT_BARRIER) } else { 0 }));
        }
    }
}

/// js util `approach(cur, target, maxStep)`
pub fn approach(cur: f64, target: f64, max_step: f64) -> f64 {
    let d = target - cur;
    if m::abs(d) <= max_step { return target; }
    cur + m::sign(d) * max_step
}

/// js physics `passesUnder(collider, top)`
fn passes_under(phys: &mut PhysicsWorld, under: &mut Vec<UnderArm>, c: ColliderHandle, top: Top) {
    let ug = (0xffff0000u32 | (0xffff & !GROUP::TRENCH_ARM)) as u32;
    phys.colliders[c].set_solver_groups(grp(ug));
    under.push(UnderArm { collider: c, top, under: true });
}

/// js physics.step()'s TRENCH arm rule, before the world's step
pub fn under_arms(phys: &mut PhysicsWorld, under: &mut [UnderArm], arms: &crate::world::TrenchArms) {
    let dt = phys.integration_parameters.dt as f64;
    for u in under.iter_mut() {
        let p = phys.colliders[u.collider].translation();
        let (px, pz) = (p.x as f64, p.z as f64);
        if !arms.arms.iter().any(|a| m::abs(px - a.x) < a.hx + 1.0 && m::abs(pz - a.z) < a.hz + 1.0) { continue; }
        let is_under = top(phys, &u.top, dt) < arms.arms_bottom;
        if is_under != u.under {
            u.under = is_under;
            phys.colliders[u.collider].set_solver_groups(grp(if is_under { arms.under_solver_groups } else { 0xffffffff }));
        }
    }
}

fn top(phys: &PhysicsWorld, t: &Top, dt: f64) -> f64 {
    match t {
        Top::Frame { body, pts } => {
            let b = &phys.bodies[*body];
            let (p, q, v, w, c) = (b.translation(), *b.rotation(), b.linvel(), b.angvel(), b.center_of_mass());
            let (qx, qy, qz, qw) = (q.x as f64, q.y as f64, q.z as f64, q.w as f64);
            let rx = [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)];
            let ry = [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx)];
            let rz = [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy)];
            let mut tp = f64::NEG_INFINITY;
            for k in 0..pts.len() / 3 {
                let (x, y, z) = (pts[3 * k], pts[3 * k + 1], pts[3 * k + 2]);
                let wx = p.x as f64 + rx[0] * x + rx[1] * y + rx[2] * z;
                let wy = p.y as f64 + ry[0] * x + ry[1] * y + ry[2] * z;
                let wz = p.z as f64 + rz[0] * x + rz[1] * y + rz[2] * z;
                let vy = v.y as f64 + w.z as f64 * (wx - c.x as f64) - w.x as f64 * (wz - c.z as f64);
                tp = m::max(tp, wy + m::max(0.0, vy) * dt);
            }
            tp
        }
        Top::Lid { body, lid, hl, hw } => {
            let b = &phys.bodies[*body];
            let t = phys.colliders[*lid].translation();
            let (q, v, w) = (*b.rotation(), b.linvel(), b.angvel());
            let (qx, qy, qz, qw) = (q.x as f64, q.y as f64, q.z as f64, q.w as f64);
            let tip = m::abs(2.0 * (qw * qz + qx * qy)) * hl + m::abs(2.0 * (qw * qx - qy * qz)) * hw;
            t.y as f64 + 0.006 + tip + m::max(0.0, v.y as f64 + m::abs(w.z as f64) * hl + m::abs(w.x as f64) * hw) * dt
        }
    }
}
