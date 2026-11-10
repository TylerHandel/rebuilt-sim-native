// The JS files in PORT.md's ledger that changed in the JS game since the commit they were ported
// at: what's left to carry over.
//   node tools/port-status.mjs [../rebuilt-sim-2026]
import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const src = resolve(process.argv[2] ?? '../rebuilt-sim-2026');
const ledger = readFileSync(new URL('../PORT.md', import.meta.url), 'utf8');
let behind = 0;
for (const line of ledger.split('\n')) {
  const cells = line.split('|').map((c) => c.trim());
  if (cells.length < 6 || !/^[0-9a-f]{7,40}$/.test(cells[3])) continue;
  const files = [...cells[1].matchAll(/`((?:js|tools)\/[^` ]+)/g)].map((m) => m[1].split(' ')[0]);
  for (const f of new Set(files)) {
    const log = execFileSync('git', ['-C', src, 'log', '--oneline', `${cells[3]}..HEAD`, '--', f], { encoding: 'utf8' }).trim();
    if (!log) continue;
    behind++;
    console.log(`${f} (ported at ${cells[3]}):\n  ${log.split('\n').join('\n  ')}`);
  }
}
console.log(behind ? `${behind} file(s) changed since they were ported` : 'everything ported is up to date');
