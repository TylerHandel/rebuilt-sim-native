// Brings the game's world over from the browser version (github.com/TylerHandel/rebuilt-sim-2026):
//   assets/world.json  the field's colliders and the FUEL, exported by its tools/export-native.mjs
//   assets/constants.json, assets/robots.json  its constants and robot configs, by the same tool
//   assets/parity/<script>.json  its traces of each parity/scripts/<script>.json (native-trace.mjs)
//   assets/parity-engine.json  digests of that world stepped by its Rapier (tools/native-parity.mjs)
//   assets/parity-mech.json    digests of its mechanisms stepping a scene on it (tools/native-mech.mjs)
//   assets/mech/path<team>.json  each physical robot's ball path from its CAD (js/mech/path*.js)
//   assets/cad/        its field's CAD model, made loadable here: Bevy doesn't read
//                      KHR_mesh_quantization, so quantized meshes are written back as plain floats
//   assets/models/     its robots as it draws them, and how their parts move (export-models.mjs)
//   assets/SOURCE      the commit they came from
// It needs that repo checked out with its npm dependencies installed:
//   node tools/sync-assets.mjs ../rebuilt-sim-2026
import { execFileSync } from 'node:child_process';
import { cpSync, mkdirSync, readdirSync, rmSync, statSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { dequantize } from '@gltf-transform/functions';

const src = resolve(process.argv[2] ?? '../rebuilt-sim-2026');
const here = resolve(import.meta.dirname, '..');
const assets = join(here, 'assets');

const sha = execFileSync('git', ['-C', src, 'rev-parse', 'HEAD'], { encoding: 'utf8' }).trim();
execFileSync('node', ['--import', './tools/node-env.mjs', 'tools/export-native.mjs', join(assets, 'world.json')], { cwd: src, stdio: 'inherit' });
// (what its Rapier makes of that world, for the sim's parity test: PLAN.md)
execFileSync('node', ['tools/native-parity.mjs', join(assets, 'world.json'), join(assets, 'parity-engine.json')], { cwd: src, stdio: 'inherit' });
// (the physical robots' ball paths from their CAD, as meshes: js/mech/path*.js, data the
// mechanisms are built from)
mkdirSync(join(assets, 'mech'), { recursive: true });
for (const f of readdirSync(join(src, 'js', 'mech')).filter((f) => /^path\d+\.js$/.test(f))) {
  const mesh = (await import(pathToFileURL(join(src, 'js', 'mech', f)).href)).default;
  writeFileSync(join(assets, 'mech', f.replace(/\.js$/, '.json')), JSON.stringify(mesh));
}
// (and what its mechanisms make of a scene of wheels, surfaces and FUEL: tools/native-mech.mjs)
execFileSync('node', ['--import', './tools/node-env.mjs', 'tools/native-mech.mjs', join(assets, 'world.json'), join(assets, 'parity-mech.json')], { cwd: src, stdio: 'inherit' });
// (and what the game does with each script in parity/scripts: its traces, which the sim's parity
// tests match)
const scripts = join(here, 'parity', 'scripts'), traces = join(assets, 'parity');
rmSync(traces, { recursive: true, force: true });
mkdirSync(traces, { recursive: true });
for (const f of readdirSync(scripts).filter((f) => f.endsWith('.json')).sort()) {
  execFileSync('node', ['--import', './tools/node-env.mjs', 'tools/native-trace.mjs', join(scripts, f), join(traces, f)], { cwd: src, stdio: 'inherit' });
}

const cad = join(assets, 'cad');
rmSync(cad, { recursive: true, force: true });
// (the field's; the robots' CAD parts come inside their models, below)
mkdirSync(cad, { recursive: true });
cpSync(join(src, 'cad', 'field'), join(cad, 'field'), { recursive: true, filter: (p) => !/\.(md)$/i.test(p) });

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
const glbs = [];
(function walk(d) { for (const f of readdirSync(d)) { const p = join(d, f); if (statSync(p).isDirectory()) walk(p); else if (f.endsWith('.glb')) glbs.push(p); } })(cad);
let n = 0;
for (const p of glbs) {
  const doc = await io.read(p);
  if (!doc.getRoot().listExtensionsUsed().some((e) => e.extensionName === 'KHR_mesh_quantization')) continue;
  await doc.transform(dequantize());
  for (const e of doc.getRoot().listExtensionsUsed()) if (e.extensionName === 'KHR_mesh_quantization') e.dispose();
  await io.write(p, doc);
  n++;
}
// the robots, as the game draws them
rmSync(join(assets, 'models'), { recursive: true, force: true });
execFileSync('node', [join(here, 'tools', 'export-models.mjs'), src], { cwd: join(here, 'tools'), stdio: 'inherit' });
writeFileSync(join(assets, 'SOURCE'), sha + '\n');
console.log(`assets from rebuilt-sim-2026 ${sha.slice(0, 7)}: world.json, ${glbs.length} field models (${n} dequantized), the robots' models`);
