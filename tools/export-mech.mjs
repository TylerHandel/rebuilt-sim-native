// The physical robots' generated mechanism data, from the browser game's own code, for the native
// sim to build its mechanisms from (rather than re-deriving each robot's geometry):
//   - each surface its js/mech/r<robot>.js builds in code (no CAD: 4414's), as the JS Surface keeps
//     it: its triangles (`tri`, after the zero-area ones are dropped: built from them again, the
//     same tree and the same order), name, mu, tread, cell, pose; a Block's section too;
//   - the wheels' places that come from curves (4414's hook and ramp rollers);
//   - a turret's energy chain as its load uses it (js/echain.js: the arm table, once a degree);
//   - a robot's shot fits (js/mech/shot4414.js).
//   node tools/export-mech.mjs ../rebuilt-sim-2026 assets/mech
import { writeFileSync, mkdirSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

const src = resolve(process.argv[2] ?? '../rebuilt-sim-2026');
const out = resolve(process.argv[3] ?? 'assets/mech');
mkdirSync(out, { recursive: true });
const load = (p) => import(pathToFileURL(join(src, p)).href);

const mech = await load('js/mech/mechanism.js');
const surface = (s) => ({
  name: s.name, tri: Array.from(s.tri), mu: s.mu, tread: s.tread === Infinity ? null : s.tread, cell: s.cell,
  pos: Array.from(s.pos), axes: s.axes.map((a) => Array.from(a)),
  block: s instanceof mech.Block ? { c: s.c, h: s.h, r: s.r } : null,
});

// ---- 4414
{
  const r = await load('js/mech/r4414.js'), shot = await load('js/mech/shot4414.js'), ec = await load('js/echain.js');
  const t = r.build4414(), s = r.pathSurfaces4414(t);
  // (the chain's arm table, as echain.js armAt makes it)
  const DEG = Math.PI / 180, c = new ec.SpoolChain(ec.SPOOL4414), [y0, y1] = c.o.yaw, n = Math.round((y1 - y0) / DEG);
  const arm = Array.from({ length: n + 1 }, (_, i) => { c.solve(y0 + i * DEG); return c.arm; });
  const data = {
    source: 'rebuilt-sim-2026 js/mech/r4414.js, shot4414.js, echain.js (tools/export-mech.mjs)',
    // (in the order the robot's object lists them: the order a Mechanism meets them)
    surfaces: Object.entries(s).map(([k, v]) => ({ key: k, ...surface(v) })),
    hookRollers: r.hookRollerAt(), rampRollers: r.rampRollerAt(),
    chain: { y0, n, arm, pitch: c.link.pitch, spring: c.o.spring, rhoT: c.rhoT, rhoC: c.rhoC, perim: c.perim, rc: c.rc },
    fit: shot.FIT, launch: shot.LAUNCH,
  };
  writeFileSync(join(out, 'gen4414.json'), JSON.stringify(data));
  console.log(`4414: ${Object.keys(s).length} surfaces, ${data.hookRollers.length} + ${data.rampRollers.length} rollers, chain ${n + 1} -> ${join(out, 'gen4414.json')}`);
}

// ---- 8793
{
  const ec = await load('js/echain.js');
  // (its turret's chain, as chainLoad8793 reads it: the runs' pin lines, the U, its links, and
  // where the U is for a yaw, insideLinks)
  const c = new ec.TrackChain();
  const data = {
    source: 'rebuilt-sim-2026 js/echain.js (tools/export-mech.mjs)',
    chain: { ro: c.ro, ri: c.ri, rho: c.rho, aF: c.aF, n: c.n, mountA: c.o.mountA, pitch: c.link.pitch, stop: c.link.stop, mass: c.link.mass },
  };
  writeFileSync(join(out, 'gen8793.json'), JSON.stringify(data));
  console.log(`8793: chain of ${c.n} links -> ${join(out, 'gen8793.json')}`);
}
