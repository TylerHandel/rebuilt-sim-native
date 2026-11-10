// Shim for compiling V8's ieee754.cc on its own (crates/jsmath): the few macros it uses, as V8
// defines them (src/base/macros.h, src/base/compiler-specific.h, include/v8config.h).
#ifndef V8_BASE_MACROS_H_
#define V8_BASE_MACROS_H_
#include <cstdint>
#include <cstring>
#if defined(_MSC_VER) && !defined(__clang__)
#define V8_CC_MSVC 1
#define V8_INLINE inline __forceinline
#define V8_WARN_UNUSED_RESULT
#else
#define V8_CC_MSVC 0
#define V8_INLINE inline __attribute__((always_inline))
#define V8_WARN_UNUSED_RESULT __attribute__((warn_unused_result))
#endif
namespace v8 {
namespace base {
template <class Dest, class Source>
V8_INLINE Dest bit_cast(Source const& source) {
  static_assert(sizeof(Dest) == sizeof(Source), "bit_cast sizes");
  Dest dest;
  memcpy(&dest, &source, sizeof(dest));
  return dest;
}
}  // namespace base
}  // namespace v8
#endif
