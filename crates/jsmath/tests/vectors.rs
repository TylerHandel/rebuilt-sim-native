//! Every function against what Node's JavaScript gives, a million inputs each, to the bit
//! (assets/parity-jsmath.json, written by tools/jsmath-vectors.mjs; the inputs are made here the
//! same way). JSMATH_VECTORS=<dir> (the tool's --full output) names the cases that differ.
use std::collections::BTreeMap;

const SPECIAL: [f64; 32] = [0.0, -0.0, f64::INFINITY, f64::NEG_INFINITY, f64::NAN, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0,
    std::f64::consts::PI, std::f64::consts::PI / 2.0, -std::f64::consts::PI / 2.0, 5e-324, -5e-324, f64::MAX,
    2.2250738585072014e-308, 1e-300, 1e300, 3.0, 0.1, 10.0, 1.5, -1.5, 2.5, -2.5, 0.49999999999999994,
    -0.49999999999999994, 4294967296.0, -2147483648.0, 2147483647.5];

fn fnv32(s: &str) -> u32 { s.bytes().fold(0x811c9dc5u32, |h, c| (h ^ c as u32).wrapping_mul(0x01000193)) }

struct Rng(u32);
impl Rng {
    fn next(&mut self) -> u32 {
        self.0 = self.0.wrapping_add(0x6d2b79f5);
        let a = self.0;
        let mut t = (a ^ (a >> 15)).wrapping_mul(1 | a);
        t = t.wrapping_add((t ^ (t >> 7)).wrapping_mul(61 | t)) ^ t;
        t ^ (t >> 14)
    }
}

fn input(r: &mut Rng) -> f64 {
    let m = r.next() % 16;
    if m == 0 { return SPECIAL[(r.next() % SPECIAL.len() as u32) as usize]; }
    if m <= 10 {
        let e = (r.next() % 121) as i64 - 60;
        let s = (r.next() & 1) as u64;
        let him = (r.next() & 0xfffff) as u64;
        let lo = r.next() as u64;
        return f64::from_bits((s << 63) | (((1023 + e) as u64) << 52) | (him << 32) | lo);
    }
    if m <= 12 {
        let k = (r.next() % 2001) as i64 - 1000;
        let x = k as f64 * (std::f64::consts::PI / 2.0);
        let d = (r.next() % 9) as i64 - 4;
        return f64::from_bits((x.to_bits() as i64).wrapping_add(d) as u64);
    }
    if m <= 14 { return ((r.next() % 41) as i64 - 20) as f64 / (1 + (r.next() & 1)) as f64; }
    let (hi, lo) = (r.next() as u64, r.next() as u64);
    f64::from_bits((hi << 32) | lo)
}

fn call(name: &str, x: &[f64]) -> f64 {
    use jsmath::*;
    match name {
        "acos" => acos(x[0]), "acosh" => acosh(x[0]), "asin" => asin(x[0]), "asinh" => asinh(x[0]),
        "atan" => atan(x[0]), "atanh" => atanh(x[0]), "cbrt" => cbrt(x[0]), "cos" => cos(x[0]),
        "cosh" => cosh(x[0]), "exp" => exp(x[0]), "expm1" => expm1(x[0]), "log" => log(x[0]),
        "log1p" => log1p(x[0]), "log2" => log2(x[0]), "log10" => log10(x[0]), "sin" => sin(x[0]),
        "sinh" => sinh(x[0]), "tan" => tan(x[0]), "tanh" => tanh(x[0]), "round" => round(x[0]),
        "sign" => sign(x[0]), "fround" => fround(x[0]), "sqrt" => sqrt(x[0]),
        "atan2" => atan2(x[0], x[1]), "pow" | "powop" => pow(x[0], x[1]), "max" => max(x[0], x[1]),
        "min" => min(x[0], x[1]), "hypot2" => hypot2(x[0], x[1]), "imul" => imul(x[0], x[1]),
        "hypot3" => hypot3(x[0], x[1], x[2]),
        _ => panic!("no function {name}"),
    }
}
fn arity(name: &str) -> usize {
    match name { "atan2" | "pow" | "powop" | "max" | "min" | "hypot2" | "imul" => 2, "hypot3" => 3, _ => 1 }
}
fn canon(r: f64) -> u64 { if r.is_nan() { f64::NAN.to_bits() } else { r.to_bits() } }

#[derive(serde::Deserialize)]
struct Vectors { n: usize, chunk: usize, fns: BTreeMap<String, Vec<String>> }

#[test]
fn every_function_matches_javascript() {
    let v: Vectors = serde_json::from_str(include_str!("../../../assets/parity-jsmath.json")).unwrap();
    let full = std::env::var("JSMATH_VECTORS").ok();
    let mut bad = Vec::new();
    for (name, chunks) in &v.fns {
        let (mut r, k) = (Rng(fnv32(name)), arity(name));
        let want: Option<Vec<f64>> = full.as_ref().map(|d| {
            std::fs::read(format!("{d}/{name}.bin")).unwrap().chunks_exact(8).map(|c| f64::from_le_bytes(c.try_into().unwrap())).collect()
        });
        let (mut h, mut wrong, mut shown) = (0xcbf29ce484222325u64, 0usize, 0);
        for i in 0..v.n {
            let xs: Vec<f64> = (0..k).map(|_| input(&mut r)).collect();
            let got = call(name, &xs);
            for b in canon(got).to_le_bytes() { h = (h ^ b as u64).wrapping_mul(0x100000001b3); }
            if let Some(w) = &want {
                let o = i * (k + 1);
                assert_eq!(&w[o..o + k].iter().map(|x| canon(*x)).collect::<Vec<_>>(), &xs.iter().map(|x| canon(*x)).collect::<Vec<_>>(), "{name} case {i}: the inputs differ (the generators disagree)");
                if canon(w[o + k]) != canon(got) {
                    if shown < 5 { eprintln!("{name}{xs:?}: JS {:e} ({:#018x}), here {:e} ({:#018x})", w[o + k], w[o + k].to_bits(), got, got.to_bits()); shown += 1; }
                    wrong += 1;
                }
            }
            if (i + 1) % v.chunk == 0 {
                let c = (i + 1) / v.chunk - 1;
                if format!("{h:016x}") != chunks[c] { bad.push(format!("{name} chunk {c}")); }
                h = 0xcbf29ce484222325;
            }
        }
        if full.is_some() && wrong > 0 { eprintln!("{name}: {wrong} of {} differ", v.n); }
    }
    assert!(bad.is_empty(), "differ from JavaScript: {bad:?}");
}
