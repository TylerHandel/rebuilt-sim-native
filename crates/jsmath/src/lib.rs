//! JavaScript's `Math`, to the bit, for porting the JS game's simulation exactly (PLAN.md D3).
//!
//! The transcendental functions are V8's own (`v8/src/base/ieee754.cc`, compiled by build.rs):
//! fdlibm, as Node's V8 runs it. The rest follow V8's builtins (`src/builtins/math.tq` and the
//! machine operators they lower to). Rust's `f64::sin` and the `libm` crate give other bits for
//! some inputs, so sim code uses these and never those.
//!
//! What's exact in IEEE 754 anyway is plain Rust: `+ - * /`, `%` (fmod), `sqrt`, `floor`, `ceil`,
//! `trunc`, `abs`. Use `jsmath::` for everything else, and for `max`, `min`, `round`, `sign`,
//! `hypot`, `imul`, `fround` (JS semantics differ from Rust's).

mod v8 {
    unsafe extern "C" {
        pub fn jsmath_acos(x: f64) -> f64;
        pub fn jsmath_acosh(x: f64) -> f64;
        pub fn jsmath_asin(x: f64) -> f64;
        pub fn jsmath_asinh(x: f64) -> f64;
        pub fn jsmath_atan(x: f64) -> f64;
        pub fn jsmath_atan2(y: f64, x: f64) -> f64;
        pub fn jsmath_atanh(x: f64) -> f64;
        pub fn jsmath_cbrt(x: f64) -> f64;
        pub fn jsmath_cos(x: f64) -> f64;
        pub fn jsmath_cosh(x: f64) -> f64;
        pub fn jsmath_exp(x: f64) -> f64;
        pub fn jsmath_expm1(x: f64) -> f64;
        pub fn jsmath_log(x: f64) -> f64;
        pub fn jsmath_log1p(x: f64) -> f64;
        pub fn jsmath_log2(x: f64) -> f64;
        pub fn jsmath_log10(x: f64) -> f64;
        pub fn jsmath_pow(x: f64, y: f64) -> f64;
        pub fn jsmath_sin(x: f64) -> f64;
        pub fn jsmath_sinh(x: f64) -> f64;
        pub fn jsmath_tan(x: f64) -> f64;
        pub fn jsmath_tanh(x: f64) -> f64;
    }
}

macro_rules! unary {
    ($($name:ident => $c:ident),* $(,)?) => {
        $(#[doc = concat!("`Math.", stringify!($name), "`")]
        #[inline]
        pub fn $name(x: f64) -> f64 { unsafe { v8::$c(x) } })*
    };
}
unary!(acos => jsmath_acos, acosh => jsmath_acosh, asin => jsmath_asin, asinh => jsmath_asinh, atan => jsmath_atan,
    atanh => jsmath_atanh, cbrt => jsmath_cbrt, cos => jsmath_cos, cosh => jsmath_cosh, exp => jsmath_exp,
    expm1 => jsmath_expm1, log => jsmath_log, log1p => jsmath_log1p, log2 => jsmath_log2, log10 => jsmath_log10,
    sin => jsmath_sin, sinh => jsmath_sinh, tan => jsmath_tan, tanh => jsmath_tanh);

/// `Math.atan2(y, x)`
#[inline]
pub fn atan2(y: f64, x: f64) -> f64 { unsafe { v8::jsmath_atan2(y, x) } }

/// `Math.pow(x, y)`, and `x ** y`
#[inline]
pub fn pow(x: f64, y: f64) -> f64 { unsafe { v8::jsmath_pow(x, y) } }

/// `Math.sqrt` (correctly rounded, as IEEE 754 requires everywhere)
#[inline]
pub fn sqrt(x: f64) -> f64 { x.sqrt() }

/// `Math.abs`, `Math.floor`, `Math.ceil`, `Math.trunc` (exact everywhere)
#[inline]
pub fn abs(x: f64) -> f64 { x.abs() }
#[inline]
pub fn floor(x: f64) -> f64 { x.floor() }
#[inline]
pub fn ceil(x: f64) -> f64 { x.ceil() }
#[inline]
pub fn trunc(x: f64) -> f64 { x.trunc() }

/// `Math.round`: to the nearest integer, a half up (toward +Infinity), -0 for -0.5 <= x < 0.
/// V8's Float64Round: ceil(x), less one when that's more than half above x.
#[inline]
pub fn round(x: f64) -> f64 {
    let r = x.ceil();
    if r - 0.5 > x { r - 1.0 } else { r }
}

/// `Math.sign`: NaN, -0 and +0 as they are; otherwise -1 or 1.
#[inline]
pub fn sign(x: f64) -> f64 {
    if x > 0.0 { 1.0 } else if x < 0.0 { -1.0 } else { x }
}

/// `Math.max(a, b)`: NaN if either is; +0 over -0.
#[inline]
pub fn max(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() { return f64::NAN; }
    if a == b { return if a == 0.0 && a.is_sign_negative() { b } else { a }; }
    if a > b { a } else { b }
}

/// `Math.min(a, b)`: NaN if either is; -0 under +0.
#[inline]
pub fn min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() { return f64::NAN; }
    if a == b { return if a == 0.0 && a.is_sign_negative() { a } else { b }; }
    if a < b { a } else { b }
}

/// `Math.max(...xs)` (-Infinity for none)
pub fn max_of(xs: &[f64]) -> f64 { xs.iter().fold(f64::NEG_INFINITY, |m, &x| max(m, x)) }
/// `Math.min(...xs)` (Infinity for none)
pub fn min_of(xs: &[f64]) -> f64 { xs.iter().fold(f64::INFINITY, |m, &x| min(m, x)) }

/// `Math.hypot(...xs)`, as V8's builtin (math.tq): Infinity if any is infinite, else NaN if any is
/// NaN; each over the largest, squared and summed with Kahan's compensation, its root times the
/// largest.
pub fn hypot(xs: &[f64]) -> f64 {
    if xs.is_empty() { return 0.0; }
    let (mut max, mut nan) = (0.0f64, false);
    for &x in xs {
        if x.is_nan() { nan = true; } else if x.abs() > max { max = x.abs(); }
    }
    if max == f64::INFINITY { return f64::INFINITY; }
    if nan { return f64::NAN; }
    if max == 0.0 { return 0.0; }
    let (mut sum, mut comp) = (0.0f64, 0.0f64);
    for &x in xs {
        let n = if x.is_nan() { 0.0 } else { x.abs() } / max;
        let summand = n * n - comp;
        let pre = sum + summand;
        comp = (pre - sum) - summand;
        sum = pre;
    }
    sum.sqrt() * max
}
/// `Math.hypot(a, b)`
#[inline]
pub fn hypot2(a: f64, b: f64) -> f64 { hypot(&[a, b]) }
/// `Math.hypot(a, b, c)`
#[inline]
pub fn hypot3(a: f64, b: f64, c: f64) -> f64 { hypot(&[a, b, c]) }

/// `Math.fround`: to the nearest f32, back as f64
#[inline]
pub fn fround(x: f64) -> f64 { x as f32 as f64 }

/// JavaScript's ToInt32: an integer modulo 2^32, as a signed 32-bit value (0 for NaN and infinities)
pub fn to_int32(x: f64) -> i32 {
    if !x.is_finite() { return 0; }
    let t = x.trunc();
    let m = t % 4294967296.0;
    let m = if m < 0.0 { m + 4294967296.0 } else { m };
    m as u32 as i32
}
/// JavaScript's ToUint32
pub fn to_uint32(x: f64) -> u32 { to_int32(x) as u32 }

/// `Math.imul(a, b)`
#[inline]
pub fn imul(a: f64, b: f64) -> f64 { to_int32(a).wrapping_mul(to_int32(b)) as f64 }

pub const PI: f64 = std::f64::consts::PI;
pub const SQRT2: f64 = std::f64::consts::SQRT_2;
pub const SQRT1_2: f64 = std::f64::consts::FRAC_1_SQRT_2;
pub const E: f64 = std::f64::consts::E;
pub const LN2: f64 = std::f64::consts::LN_2;
