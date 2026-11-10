// V8's src/base/ieee754.cc (12.4.254.21, the V8 in the Node the JS game's tools run on), compiled
// as it is, with shims for the few V8 headers it includes. No floating-point contraction: a fused
// multiply-add rounds once where the C++ rounds twice, and gives other bits.
fn main() {
    println!("cargo:rerun-if-changed=v8");
    let mut b = cc::Build::new();
    b.cpp(true).include("v8").file("v8/jsmath.cc").file("v8/src/base/ieee754.cc").warnings(false);
    if b.get_compiler().is_like_msvc() {
        b.flag("/fp:precise").flag("/std:c++17");
    } else {
        b.flag("-ffp-contract=off").flag("-fno-fast-math").flag("-std=c++17");
    }
    b.compile("jsmath_v8");
}
