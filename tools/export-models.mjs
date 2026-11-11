// Exports the browser game's robot models, as it draws them, for the native app:
//   assets/models/<key>.glb       the model buildRobotModel(cfg, 'blue') makes once its CAD parts
//                                 have loaded (what's hidden left out); the bumpers' fabric is the
//                                 material named "alliance", recolored here for red
//   assets/models/<key>.rig.json  how its moving parts follow the robot's state: found by running
//                                 the model's own anim(state, dt) (js/robotModels.js), one input at
//                                 a time, and seeing which parts it moves and how:
//     nodes   each moving part's rest pose: position, Euler rotation (three.js's, with its order),
//             scale; the GLB's node of the same name
//     pose    inputs anim sets a pose from (intakeDeploy, hopperDeploy, hoodDeg...): per input,
//             its samples and each part's pose at each (in between, linear: anim's are lerps)
//     rate    inputs anim turns parts by, per second (intakeSpeed, flywheel...): per input, each
//             part's Euler rates at 1
//     nets    the nets over the load: each one's shape, the node it hangs on, how anim drapes it
//             (its args at rest and per input, as pose); the app makes and drapes its own
//     modules the swerve modules: each one's steering pivot and wheel, which the game turns itself
//             (js/robot.js), and where its steering axis is
// It loads the game in Chromium (Playwright) from a local server over the checkout, since the
// models need a browser's loaders.
//   node tools/export-models.mjs ../rebuilt-sim-2026 [key...]
import { createServer } from 'node:http';
import { mkdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { extname, join, normalize, resolve } from 'node:path';
import { chromium } from 'playwright';
import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { dequantize } from '@gltf-transform/functions';

const src = resolve(process.argv[2] ?? '../rebuilt-sim-2026');
const here = resolve(import.meta.dirname, '..');
const out = join(here, 'assets', 'models');
const only = process.argv.slice(3);

const PAGE = `<!doctype html><meta charset="utf-8">
<script type="importmap">{"imports": {
  "three": "/node_modules/three/build/three.module.js",
  "three/addons/": "/node_modules/three/examples/jsm/",
  "rapier": "/node_modules/@dimforge/rapier3d-simd-compat/dist/rapier.mjs"
}}</script>
<script type="module">
import * as THREE from 'three';
import { GLTFExporter } from 'three/addons/exporters/GLTFExporter.js';
import { buildRobotModel } from '/js/robotModels.js';
import { ROBOTS, ROBOT_ORDER } from '/js/robotConfigs.js';
import { ALLIANCE_COLOR_CSS } from '/js/field.js';
import { Venue } from '/js/venue.js';

// every fetch the loaders make, to know when the CAD is in
let pending = 0, last = performance.now();
const f0 = window.fetch.bind(window);
window.fetch = (...a) => { pending++; last = performance.now(); return f0(...a).finally(() => { pending--; last = performance.now(); }); };
const settle = async () => {
  for (;;) {
    await new Promise((r) => setTimeout(r, 100));
    if (pending === 0 && performance.now() - last > 800) return;
  }
};
const shown = (o) => { for (let p = o; p; p = p.parent) if (!p.visible) return false; return true; };
const pose = (o) => [o.position.x, o.position.y, o.position.z, o.rotation.x, o.rotation.y, o.rotation.z, o.scale.x, o.scale.y, o.scale.z];
const same = (a, b) => a.every((v, i) => Math.abs(v - b[i]) < 1e-7);

window.keys = () => ROBOT_ORDER.concat(Object.keys(ROBOTS).filter((k) => !ROBOT_ORDER.includes(k)));
window.physical = (k) => !!ROBOTS[k].physical;

window.exportRobot = async (key) => {
  const cfg = ROBOTS[key];
  const m = buildRobotModel(cfg, 'blue');
  await settle();
  const root = m.root;
  root.updateMatrixWorld(true);
  // the hopper's FUEL (drawn from the game's state: not part of the model), and anything instanced
  root.traverse((o) => { if (o.isInstancedMesh) o.visible = false; });
  // the bumpers' fabric
  const blue = new THREE.Color(ALLIANCE_COLOR_CSS.blue);
  root.traverse((o) => {
    if (!o.isMesh) return;
    for (const mt of [].concat(o.material)) if (mt?.color && mt.color.equals(blue) && mt.sheen !== undefined && mt.sheen > 0) mt.name = 'alliance';
  });
  // names: rig<i> for every node, mod<i>.pivot / .wheel for the modules
  const all = [];
  root.traverse((o) => { o.name = 'rig' + all.length; all.push(o); });
  const modules = (m.modules || []).map((md, i) => {
    md.pivot.name = 'mod' + i + '.pivot';
    md.wheel.name = 'mod' + i + '.wheel';
    const p = md.pivot.parent.position;
    return { pivot: md.pivot.name, wheel: md.wheel.name, x: p.x, z: p.z };
  });
  const base = all.map(pose);
  const restore = () => all.forEach((o, i) => { const b = base[i]; o.position.set(b[0], b[1], b[2]); o.rotation.set(b[3], b[4], b[5]); o.scale.set(b[6], b[7], b[8]); });
  const sh = cfg.shooter || {};
  const hoodLo = sh.hood ? Math.min(sh.hood.stowDeg, sh.hoodMin ?? sh.hood.stowDeg) : sh.hoodMin ?? 0;
  const hoodHi = sh.hood ? Math.max(sh.hood.stowDeg, sh.hoodMax ?? sh.hood.stowDeg) : sh.hoodMax ?? 0;
  const st0 = {
    intakeDeploy: 0, hopperDeploy: 0, intakeSpeed: 0, flywheel: 0, feeding: 0,
    hoodDeg: sh.hood ? sh.hood.stowDeg : sh.hoodMin ?? 0, turretYaw: 0, climbLift: 0, climbState: 'none',
    rotorAngle: 0, load: [], lift: 0,
  };
  const run = (st, dt) => { try { m.anim(st, dt); return true; } catch (e) { return String(e); } };
  // a state with input name (a dotted path into st.mech or st.turretYaws for the ones that take
  // those: mech's other fields at their rest, MECH_REST) set to v
  const MECH_REST = {
    // (js/mech/phys1690.js mechPose, at rest: in, its kicker bar folded, the hood stowed)
    '1690': () => ({ ext: st0.intakeDeploy, kick: 0, hood: ((st0.hoodDeg + 90) * Math.PI) / 180, wheel: 0, feeder: 0, floor: 0, top: 0, belts: [0, 0], vkick: 0, feed: 0, intakeRollers: 0, bow: 0 }),
    // (js/mech/phys7769.js mechPose, at rest: the intake in, the kick bar hanging, the hood down)
    '7769': () => ({ ext: 0, kick: 0, hood: 0, roller: 0, belts: 0, feed: 0, wheels: 0 }),
    // (js/mech/phys1706.js mechPose, at rest: the spindexers unturned, the hoods steep, the wheels still)
    '1706': () => ({ spin: [0, 0], hood: [0, 0], accel: [0, 0] }),
    // (js/mech/phys2056.js mechPose, at rest: the intake in, its kicker bar folded, the hood stowed)
    '2056': () => ({ ext: st0.intakeDeploy, kick: 0, hood: ((st0.hoodDeg + 90) * Math.PI) / 180, wheel: 0, kicker: 0, floor: 0, belts: [0, 0], vkick: 0, feed: 0, intakeRollers: 0, bow: 0 }),
  };
  const withInput = (name, v) => {
    const st = { ...st0 };
    const [root, ...rest] = name.split('.');
    if (!rest.length) { st[name] = v; return st; }
    if (root === 'mech') { if (!MECH_REST[key]) return null; st.mech = MECH_REST[key](); }
    else if (root === 'turretYaws') st.turretYaws = [st0.turretYaw, st0.turretYaw];
    let o = root === 'mech' ? st.mech : st;
    if (root !== 'mech') o = st;
    const path = root === 'mech' ? rest : [root, ...rest];
    // (an input that isn't one of this robot's: a path through what isn't an object, or a number
    // in place of one, as mech.belts.0 is 1690's and mech.belts 7769's)
    for (let i = 0; i < path.length - 1; i++) { o = o[path[i]]; if (o === null || typeof o !== 'object') return null; }
    if (typeof o[path[path.length - 1]] === 'object' && o[path[path.length - 1]] !== null) return null;
    o[path[path.length - 1]] = v;
    return st;
  };
  const errors = [];
  // the rest pose: what anim makes of the state at rest (the model as built may differ from it)
  restore();
  const ok0 = run(st0, 0);
  if (ok0 !== true) errors.push('rest: ' + ok0);
  const rest = all.map(pose);
  // inputs anim sets a pose from: [name, lo, hi]
  // (and what a physical robot says of its parts itself, which take over from those where they
  // move the same part: an arm's own turn, a hood's, a rotor's, a floor's, each turret of two; its
  // mechanisms' pose, mech: 1690's carriage, kicker bar, hood and top plate)
  const PI = Math.PI;
  const POSE = [['intakeDeploy', 0, 1], ['hopperDeploy', 0, 1], ['hoodDeg', hoodLo, hoodHi], ['turretYaw', -PI, PI], ['lift', 0, 1],
    ['armTurn', -PI, PI], ['hoodTurn', -PI, PI], ['rotorTurn', 0, 2 * PI], ['floorTurn', -PI, PI], ['rotorAngle', 0, 2 * PI],
    ['turretYaws.0', -PI, PI], ['turretYaws.1', -PI, PI],
    ['mech.ext', 0, 1], ['mech.spin.0', 0, 2 * PI], ['mech.spin.1', 0, 2 * PI], ['mech.kick', -PI, PI / 2], ['mech.hood', PI / 2, PI], ['mech.bow', 0, 0.1]];
  const N = 16;
  const poseOut = {}, moving = new Set();
  for (const [name, lo, hi] of POSE) {
    if (!(hi > lo)) continue;
    const samples = [], rows = {};
    for (let s = 0; s <= N; s++) {
      const v = lo + (hi - lo) * s / N;
      samples.push(v);
      restore();
      const st = withInput(name, v);
      if (!st) break;
      const ok = run(st, 0);
      if (ok !== true) { errors.push(name + ': ' + ok); break; }
      all.forEach((o, i) => { const p = pose(o); if (!same(p, rest[i])) rows[i] = true; });
    }
    const idx = Object.keys(rows).map(Number).filter((i) => shown(all[i]));
    if (!idx.length) continue;
    const nodes = {};
    for (const i of idx) nodes[all[i].name] = [];
    for (const v of samples) {
      restore();
      run(withInput(name, v), 0);
      for (const i of idx) nodes[all[i].name].push(pose(all[i]));
    }
    idx.forEach((i) => moving.add(i));
    poseOut[name] = { samples, nodes };
  }
  // inputs anim turns parts by: their Euler rates at 1 (one second's turn)
  const rateOut = {};
  for (const name of ['intakeSpeed', 'flywheel', 'feeding', 'floorSpin', 'mech.intakeRollers', 'mech.floor', 'mech.belts.0', 'mech.belts.1', 'mech.vkick', 'mech.feed', 'mech.wheel', 'mech.feeder', 'mech.kicker', 'mech.roller', 'mech.belts', 'mech.wheels']) {
    const st1 = withInput(name, 1);
    if (!st1) continue;
    // (at rest first: a mech input's rest is with mech there, at its rest)
    restore();
    run(name.startsWith('mech.') ? withInput(name, 0) : st0, 0);
    const a = all.map(pose);
    const ok = run(st1, 1);
    if (ok !== true) { errors.push(name + ': ' + ok); continue; }
    const nodes = {};
    all.forEach((o, i) => {
      const b = pose(o), d = [3, 4, 5].map((k) => b[k] - a[i][k]);
      if (d.some((x) => Math.abs(x) > 1e-9) && shown(o)) { nodes[o.name] = d; moving.add(i); }
    });
    if (Object.keys(nodes).length) rateOut[name] = nodes;
  }
  // the nets over the load (hopperNet): the app drapes its own over the FUEL each frame. Each one's
  // shape (userData.netSpec), the node it hangs on, and how anim drapes it (userData.drapeArgs: ex,
  // its back and front edges' heights, xBack; and whether it's shown) at rest and per input, as the
  // poses are
  const nets = [];
  root.traverse((o) => { if (o.userData?.netSpec) nets.push(o); });
  const argsOf = () => nets.map((o) => { const a = o.userData.drapeArgs ?? { ex: 0, y: null, yFront: null, xBack: null }; return [a.ex, a.y, a.yFront, a.xBack, o.visible ? 1 : 0]; });
  const sameArg = (x, y) => x === y || (x !== null && y !== null && Math.abs(x - y) < 1e-9);
  restore();
  run(st0, 0);
  const netRest = argsOf();
  const netOut = nets.map((o, i) => ({ name: o.name, parent: o.parent.name, spec: o.userData.netSpec, rest: netRest[i], by: {} }));
  for (const [name, lo, hi] of nets.length ? POSE : []) {
    if (!(hi > lo)) continue;
    const samples = [], rows = nets.map(() => []), moved = nets.map(() => false);
    for (let s = 0; s <= N; s++) {
      const v = lo + (hi - lo) * s / N, st = withInput(name, v);
      if (!st) break;
      restore();
      if (run(st, 0) !== true) break;
      samples.push(v);
      argsOf().forEach((a, i) => { rows[i].push(a); if (a.some((x, j) => !sameArg(x, netRest[i][j]))) moved[i] = true; });
    }
    netOut.forEach((n, i) => { if (moved[i] && samples.length === N + 1) n.by[name] = { samples, rows: rows[i] }; });
  }
  restore();
  run(st0, 0);
  const nodes = {};
  for (const i of moving) { const o = all[i]; nodes[o.name] = { rest: rest[i], order: o.rotation.order }; }
  for (const md of m.modules || []) for (const o of [md.pivot, md.wheel]) nodes[o.name] = { rest: pose(o), order: o.rotation.order };
  for (const o of nets) o.visible = false; // (not in the GLB: the app makes its own)
  root.updateMatrixWorld(true);
  const glb = await new GLTFExporter().parseAsync(root, { binary: true, onlyVisible: true });
  let bin = '';
  const u8 = new Uint8Array(glb);
  for (let i = 0; i < u8.length; i += 0x8000) bin += String.fromCharCode.apply(null, u8.subarray(i, i + 0x8000));
  return { glb: btoa(bin), rig: { key, nodes, pose: poseOut, rate: rateOut, modules, nets: netOut, errors } };
};
// The venue (js/venue.js, its fullest level): everything but its people as it builds it, and its
// people baked at rest into one mesh per pose (their colors from their seeds as its shader makes
// them; the referees' stripes left gray), the drive teams apart from the rest (the app hides them
// at the Driver Station view). The screens' mesh keeps its name, venue-screens: the app draws them.
window.exportVenue = async () => {
  const scene = new THREE.Scene();
  const v = new Venue(scene);
  v.setLevel(3);
  await settle();
  const root = v.root;
  root.updateMatrixWorld(true);
  const fract = (x) => x - Math.floor(x);
  const mixv = (a, b, t) => a.map((x, i) => x + (b[i] - x) * t);
  const bake = (mesh, pick) => {
    const g = mesh.geometry, pos = g.attributes.position, nor = g.attributes.normal, col = g.attributes.color;
    const info = g.attributes.aInfo, sh = g.attributes.aShirt;
    const P = [], N = [], C = [];
    const m = new THREE.Matrix4(), nm = new THREE.Matrix3(), p = new THREE.Vector3(), n = new THREE.Vector3();
    for (let k = 0; k < mesh.count; k++) {
      mesh.getMatrixAt(k, m);
      const at = new THREE.Vector3().setFromMatrixPosition(m);
      if (!pick(at)) continue;
      nm.getNormalMatrix(m);
      const seed = info.getX(k), stripe = info.getW(k);
      const shirt = [sh.getX(k), sh.getY(k), sh.getZ(k)];
      const skin = mixv([0.62, 0.40, 0.29], [0.13, 0.07, 0.045], fract(seed * 7.13));
      const pants = fract(seed * 3.71) > 0.72 ? [0.33, 0.27, 0.17] : mixv([0.02, 0.025, 0.04], [0.04, 0.07, 0.14], fract(seed * 5.3));
      const hair = mixv([0.01, 0.008, 0.006], [0.20, 0.11, 0.05], Math.pow(fract(seed * 11.3), 2));
      for (let i = 0; i < pos.count; i++) {
        const part = col.getX(i), armA = col.getY(i), shY = col.getZ(i);
        p.fromBufferAttribute(pos, i); n.fromBufferAttribute(nor, i);
        if (part > 1.25 && part < 3.5 && part !== 2) {
          const ca = Math.cos(armA), sa = Math.sin(armA), qy = p.y - shY, qz = p.z;
          p.set(p.x, qy * ca - qz * sa + shY, qy * sa + qz * ca);
          n.set(n.x, n.y * ca - n.z * sa, n.y * sa + n.z * ca);
        }
        p.applyMatrix4(m); n.applyMatrix3(nm).normalize();
        P.push(p.x, p.y, p.z); N.push(n.x, n.y, n.z);
        let c = part < 0.5 ? shirt : part < 1.75 ? skin : part < 2.5 ? pants : part < 3.5 ? shirt : hair;
        if (stripe > 0.5 && (part < 0.5 || (part > 2.5 && part < 3.5))) c = [0.3, 0.3, 0.3];
        C.push(c[0], c[1], c[2]);
      }
    }
    const out = new THREE.BufferGeometry();
    out.setAttribute('position', new THREE.Float32BufferAttribute(P, 3));
    out.setAttribute('normal', new THREE.Float32BufferAttribute(N, 3));
    out.setAttribute('color', new THREE.Float32BufferAttribute(C, 3));
    return out;
  };
  const people = [];
  root.traverse((o) => { if (o.isInstancedMesh) people.push(o); });
  const mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.9, metalness: 0 });
  // (the drive teams: the standing people just behind the driver stations)
  const isTeam = (p) => Math.abs(p.x) > 8.5 && Math.abs(p.x) < 10 && Math.abs(p.z) < 4.2;
  for (const o of people) {
    o.parent.remove(o);
    const standing = o.name === 'venue-people';
    const add = (name, pick) => {
      const g = bake(o, pick);
      if (!g.attributes.position.count) return;
      const mm = new THREE.Mesh(g, mat);
      mm.name = name;
      root.add(mm);
    };
    if (standing) { add('venue-people', (p) => !isTeam(p)); add('venue-driveteams', isTeam); }
    else add('venue-crowd', () => true);
  }
  // (the drape and its banners behind the scoring table: the Broadcast camera looks over the table
  // from behind them, where they'd fill the bottom of its view)
  const behindTable = (x, z) => z > 6.6 && Math.abs(x) < 4.6;
  root.traverse((o) => {
    if (!o.isMesh || !['venue-drape', 'venue-banner'].includes(o.name)) return;
    const g = o.geometry.index ? o.geometry.toNonIndexed() : o.geometry, p = g.attributes.position;
    const keep = [];
    for (let t = 0; t < p.count; t += 3) {
      const cx = (p.getX(t) + p.getX(t + 1) + p.getX(t + 2)) / 3, cz = (p.getZ(t) + p.getZ(t + 1) + p.getZ(t + 2)) / 3;
      if (!behindTable(cx, cz)) for (let k = 0; k < 3; k++) keep.push(t + k);
    }
    const out = new THREE.BufferGeometry();
    for (const [name, a] of Object.entries(g.attributes)) {
      const arr = new a.array.constructor(keep.length * a.itemSize);
      keep.forEach((i, j) => { for (let c = 0; c < a.itemSize; c++) arr[j * a.itemSize + c] = a.array[i * a.itemSize + c]; });
      out.setAttribute(name, new THREE.BufferAttribute(arr, a.itemSize, a.normalized));
    }
    o.geometry = out;
  });
  // (Lambert's materials as the glTF's standard: rough, not metal)
  root.traverse((o) => {
    if (!o.isMesh) return;
    const m0 = o.material;
    if (m0.isMeshLambertMaterial) {
      o.material = new THREE.MeshStandardMaterial({ color: m0.color, map: m0.map, side: m0.side, roughness: 0.92, metalness: o.name === 'venue-metal' ? 0.6 : 0, vertexColors: m0.vertexColors });
    }
  });
  root.updateMatrixWorld(true);
  const glb = await new GLTFExporter().parseAsync(root, { binary: true, onlyVisible: true });
  let bin = '';
  const u8 = new Uint8Array(glb);
  for (let i = 0; i < u8.length; i += 0x8000) bin += String.fromCharCode.apply(null, u8.subarray(i, i + 0x8000));
  return { glb: btoa(bin), count: v.count };
};
window.ready = true;
</script>`;

const TYPES = { '.js': 'text/javascript', '.mjs': 'text/javascript', '.json': 'application/json', '.glb': 'model/gltf-binary', '.png': 'image/png', '.jpg': 'image/jpeg', '.html': 'text/html' };
const server = createServer((req, res) => {
  const url = decodeURIComponent(req.url.split('?')[0]);
  if (url === '/__export.html') { res.writeHead(200, { 'content-type': 'text/html' }); return res.end(PAGE); }
  const p = normalize(join(src, url));
  if (!p.startsWith(src)) { res.writeHead(403); return res.end(); }
  try {
    if (!statSync(p).isFile()) throw new Error();
    res.writeHead(200, { 'content-type': TYPES[extname(p)] ?? 'application/octet-stream' });
    res.end(readFileSync(p));
  } catch { res.writeHead(404); res.end(); }
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const port = server.address().port;

const browser = await chromium.launch();
const page = await browser.newPage();
page.on('console', (m) => { if ((m.type() === 'warning' || m.type() === 'error') && !/normalized normal/.test(m.text())) console.log('  page:', m.text()); });
page.on('pageerror', (e) => console.log('  page error:', e.message));
await page.goto(`http://127.0.0.1:${port}/__export.html`);
await page.waitForFunction(() => window.ready === true, null, { timeout: 60000 });
const keys = only.length ? only : (await page.evaluate(() => window.keys())).filter((k) => true);
mkdirSync(out, { recursive: true });
for (const key of keys) {
  if (key === 'venue') continue;
  if (!only.length && !(await page.evaluate((k) => window.physical(k), key))) continue;
  const t = Date.now();
  const { glb, rig } = await page.evaluate((k) => window.exportRobot(k), key);
  // (the CAD parts' meshes come over quantized, as the game loads them: Bevy doesn't read
  // KHR_mesh_quantization, so they're written back as plain floats)
  const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
  const doc = await io.readBinary(new Uint8Array(Buffer.from(glb, 'base64')));
  await doc.transform(dequantize());
  for (const e of doc.getRoot().listExtensionsUsed()) if (e.extensionName === 'KHR_mesh_quantization') e.dispose();
  const buf = Buffer.from(await io.writeBinary(doc));
  writeFileSync(join(out, key + '.glb'), buf);
  writeFileSync(join(out, key + '.rig.json'), JSON.stringify(rig));
  const np = Object.values(rig.pose).reduce((n, p) => n + Object.keys(p.nodes).length, 0);
  const nr = Object.values(rig.rate).reduce((n, p) => n + Object.keys(p).length, 0);
  console.log(`${key}: ${(buf.length / 1e6).toFixed(1)} MB, ${Object.keys(rig.pose).join('/') || 'no poses'} (${np} parts), ${Object.keys(rig.rate).join('/') || 'no rates'} (${nr} parts), ${rig.modules.length} modules, ${((Date.now() - t) / 1000).toFixed(1)} s${rig.errors.length ? '; anim errors: ' + rig.errors.join('; ') : ''}`);
}
if (!only.length || only.includes('venue')) {
  const t = Date.now();
  const { glb, count } = await page.evaluate(() => window.exportVenue());
  const buf = Buffer.from(glb, 'base64');
  writeFileSync(join(out, 'venue.glb'), buf);
  console.log(`venue: ${(buf.length / 1e6).toFixed(1)} MB, ${count.seated} seated and ${count.standing} standing, ${((Date.now() - t) / 1000).toFixed(1)} s`);
}
await browser.close();
server.close();
