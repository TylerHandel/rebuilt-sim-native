//! The engine's f32 math as the WebAssembly build has it. Rapier and parry call f32::sin, exp,
//! atan2... directly; compiled to WebAssembly those are Rust's own libm, natively the platform's C
//! library (glibc, the MSVC runtime), whose last bits differ now and then. Defining the C symbols
//! here makes the native build's calls land on libm too, so the engine steps to the browser's bits
//! (crates/sim/tests/parity_scripts.rs).
#![allow(clippy::missing_safety_doc)]
macro_rules! f1 { ($($n:ident),*) => { $(#[unsafe(no_mangle)] pub extern "C" fn $n(x: f32) -> f32 { libm::$n(x) })* }; }
macro_rules! f2 { ($($n:ident),*) => { $(#[unsafe(no_mangle)] pub extern "C" fn $n(x: f32, y: f32) -> f32 { libm::$n(x, y) })* }; }
f1!(sinf, cosf, tanf, asinf, acosf, atanf, expf, exp2f, expm1f, logf, log2f, log10f, log1pf, sinhf, coshf, tanhf, cbrtf);
f2!(atan2f, powf, hypotf, fmodf);
