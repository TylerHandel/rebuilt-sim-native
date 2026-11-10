// What JavaScript's Math gives, for crates/jsmath's test: a million inputs per function, made by
// the same generator the test has, and a digest of the results in chunks of 50,000.
//   node tools/jsmath-vectors.mjs [assets/parity-jsmath.json] [--full <dir>]
// --full writes every input and result too (<dir>/<fn>.bin: f64 little-endian, inputs then the
// result, per case), for JSMATH_VECTORS=<dir> cargo test -p jsmath, which then names the cases
// that differ. NaN results count as one NaN (JS can't tell NaNs apart).
import { mkdirSync, writeFileSync } from 'node:fs';

export const N = 1_000_000, CHUNK = 50_000;
// [name, arity, f]
export const FNS = [
  ['acos', 1, Math.acos], ['acosh', 1, Math.acosh], ['asin', 1, Math.asin], ['asinh', 1, Math.asinh],
  ['atan', 1, Math.atan], ['atanh', 1, Math.atanh], ['cbrt', 1, Math.cbrt], ['cos', 1, Math.cos],
  ['cosh', 1, Math.cosh], ['exp', 1, Math.exp], ['expm1', 1, Math.expm1], ['log', 1, Math.log],
  ['log1p', 1, Math.log1p], ['log2', 1, Math.log2], ['log10', 1, Math.log10], ['sin', 1, Math.sin],
  ['sinh', 1, Math.sinh], ['tan', 1, Math.tan], ['tanh', 1, Math.tanh], ['round', 1, Math.round],
  ['sign', 1, Math.sign], ['fround', 1, Math.fround], ['sqrt', 1, Math.sqrt],
  ['atan2', 2, Math.atan2], ['pow', 2, Math.pow], ['powop', 2, (a, b) => a ** b], ['max', 2, Math.max],
  ['min', 2, Math.min], ['hypot2', 2, Math.hypot], ['imul', 2, Math.imul], ['hypot3', 3, Math.hypot],
];

// the inputs: mulberry32 (u32 out), seeded by the function's name
function fnv32(s) { let h = 0x811c9dc5; for (const c of s) h = Math.imul(h ^ c.charCodeAt(0), 0x01000193) >>> 0; return h; }
function rng(seed) {
  let a = seed | 0;
  return () => { a = (a + 0x6d2b79f5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return (t ^ (t >>> 14)) >>> 0; };
}
const dv = new DataView(new ArrayBuffer(8));
const fromBits = (hi, lo) => { dv.setUint32(4, hi >>> 0, true); dv.setUint32(0, lo >>> 0, true); return dv.getFloat64(0, true); };
const SPECIAL = [0, -0, Infinity, -Infinity, NaN, 1, -1, 0.5, -0.5, 2, -2, Math.PI, Math.PI / 2, -Math.PI / 2, 5e-324, -5e-324, Number.MAX_VALUE, 2.2250738585072014e-308, 1e-300, 1e300, 3, 0.1, 10, 1.5, -1.5, 2.5, -2.5, 0.49999999999999994, -0.49999999999999994, 4294967296, -2147483648, 2147483647.5];
// one input: a special value, a double of any sign with exponent in [-60, 60], one near a multiple
// of pi/2 (to a few ulps), a small integer or half-integer (the exponents ** takes), or any bits
export function input(next) {
  const m = next() % 16;
  if (m === 0) return SPECIAL[next() % SPECIAL.length];
  if (m <= 10) { const e = (next() % 121) - 60, s = next() & 1, hiM = next() & 0xfffff, lo = next(); return fromBits(((s << 31) | ((1023 + e) << 20) | hiM) >>> 0, lo); }
  if (m <= 12) { const k = (next() % 2001) - 1000, x = k * (Math.PI / 2); dv.setFloat64(0, x, true); const b = dv.getBigUint64(0, true) + BigInt((next() % 9) - 4); dv.setBigUint64(0, BigInt.asUintN(64, b), true); return dv.getFloat64(0, true); }
  if (m <= 14) return ((next() % 41) - 20) / (1 + (next() & 1));
  return fromBits(next(), next());
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const args = process.argv.slice(2), full = args.includes('--full') ? args[args.indexOf('--full') + 1] : null;
  const out = args.find((a, i) => !a.startsWith('--') && args[i - 1] !== '--full') ?? 'assets/parity-jsmath.json';
  if (full) mkdirSync(full, { recursive: true });
  const fns = {};
  const bits = new DataView(new ArrayBuffer(8));
  for (const [name, arity, f] of FNS) {
    const next = rng(fnv32(name)), chunks = [], buf = full ? new Float64Array(N * (arity + 1)) : null;
    let h = 0xcbf29ce484222325n;
    for (let i = 0; i < N; i++) {
      const xs = [];
      for (let k = 0; k < arity; k++) xs.push(input(next));
      let r = f(...xs);
      if (Number.isNaN(r)) r = NaN;
      bits.setFloat64(0, r, true);
      for (let k = 0; k < 8; k++) h = ((h ^ BigInt(bits.getUint8(k))) * 0x100000001b3n) & 0xffffffffffffffffn;
      if (buf) { buf.set(xs, i * (arity + 1)); buf[i * (arity + 1) + arity] = r; }
      if ((i + 1) % CHUNK === 0) { chunks.push(h.toString(16).padStart(16, '0')); h = 0xcbf29ce484222325n; }
    }
    fns[name] = chunks;
    if (buf) writeFileSync(`${full}/${name}.bin`, Buffer.from(buf.buffer));
    process.stdout.write(name + ' ');
  }
  writeFileSync(out, JSON.stringify({ source: 'rebuilt-sim-native tools/jsmath-vectors.mjs', node: process.version, v8: process.versions.v8, n: N, chunk: CHUNK, fns }, null, 1));
  console.log(`\n${FNS.length} functions to ${out}`);
}
