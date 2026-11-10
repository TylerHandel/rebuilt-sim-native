// Brings the game's world over from the browser version (github.com/TylerHandel/rebuilt-sim-2026):
//   assets/world.json  the field's colliders and the FUEL, exported by its tools/export-native.mjs
//   assets/parity-engine.json  digests of that world stepped by its Rapier (tools/native-parity.mjs)
//   assets/cad/        its CAD models (field and robots), made loadable here: Bevy doesn't read
//                      KHR_mesh_quantization, so quantized meshes are written back as plain floats
//   assets/SOURCE      the commit they came from
// It needs that repo checked out with its npm dependencies installed:
//   node tools/sync-assets.mjs ../rebuilt-sim-2026
import { execFileSync } from 'node:child_process';
import { cpSync, mkdirSync, readdirSync, rmSync, statSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
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

const cad = join(assets, 'cad');
rmSync(cad, { recursive: true, force: true });
cpSync(join(src, 'cad'), cad, { recursive: true, filter: (p) => !/\.(md)$/i.test(p) });

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
writeFileSync(join(assets, 'SOURCE'), sha + '\n');
console.log(`assets from rebuilt-sim-2026 ${sha.slice(0, 7)}: world.json, ${glbs.length} models (${n} dequantized)`);
