// C entry points to V8's ieee754 functions (src/base/ieee754.cc, as V8 12.4.254.21 has it), for
// crates/jsmath. Node's V8 is built without V8_USE_LIBM_TRIG_FUNCTIONS, so Math.sin and Math.cos
// are the fdlibm ones.
#include "src/base/ieee754.h"
namespace i = v8::base::ieee754;
extern "C" {
double jsmath_acos(double x) { return i::acos(x); }
double jsmath_acosh(double x) { return i::acosh(x); }
double jsmath_asin(double x) { return i::asin(x); }
double jsmath_asinh(double x) { return i::asinh(x); }
double jsmath_atan(double x) { return i::atan(x); }
double jsmath_atan2(double y, double x) { return i::atan2(y, x); }
double jsmath_atanh(double x) { return i::atanh(x); }
double jsmath_cbrt(double x) { return i::cbrt(x); }
double jsmath_cos(double x) { return i::cos(x); }
double jsmath_cosh(double x) { return i::cosh(x); }
double jsmath_exp(double x) { return i::exp(x); }
double jsmath_expm1(double x) { return i::expm1(x); }
double jsmath_log(double x) { return i::log(x); }
double jsmath_log1p(double x) { return i::log1p(x); }
double jsmath_log2(double x) { return i::log2(x); }
double jsmath_log10(double x) { return i::log10(x); }
double jsmath_pow(double x, double y) { return i::pow(x, y); }
double jsmath_sin(double x) { return i::sin(x); }
double jsmath_sinh(double x) { return i::sinh(x); }
double jsmath_tan(double x) { return i::tan(x); }
double jsmath_tanh(double x) { return i::tanh(x); }
}
