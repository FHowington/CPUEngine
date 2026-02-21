#pragma once

#ifdef USE_SIMD
  #define SIMDE_ENABLE_NATIVE_ALIASES
  #include <simde/x86/avx2.h>
  #include <simde/x86/fma.h>

  // SIMDe's native alias for _MM_FROUND_NO_EXC requires SSE4.1 alias
  // to be set before sse.h is first included. Since include order is
  // hard to guarantee, define any missing rounding constants here.
  #ifndef _MM_FROUND_TO_NEAREST_INT
    #define _MM_FROUND_TO_NEAREST_INT SIMDE_MM_FROUND_TO_NEAREST_INT
  #endif
  #ifndef _MM_FROUND_TO_NEG_INF
    #define _MM_FROUND_TO_NEG_INF SIMDE_MM_FROUND_TO_NEG_INF
  #endif
  #ifndef _MM_FROUND_TO_POS_INF
    #define _MM_FROUND_TO_POS_INF SIMDE_MM_FROUND_TO_POS_INF
  #endif
  #ifndef _MM_FROUND_TO_ZERO
    #define _MM_FROUND_TO_ZERO SIMDE_MM_FROUND_TO_ZERO
  #endif
  #ifndef _MM_FROUND_CUR_DIRECTION
    #define _MM_FROUND_CUR_DIRECTION SIMDE_MM_FROUND_CUR_DIRECTION
  #endif
  #ifndef _MM_FROUND_RAISE_EXC
    #define _MM_FROUND_RAISE_EXC SIMDE_MM_FROUND_RAISE_EXC
  #endif
  #ifndef _MM_FROUND_NO_EXC
    #define _MM_FROUND_NO_EXC SIMDE_MM_FROUND_NO_EXC
  #endif

  #ifndef __AVX__
    #define __AVX__
  #endif
  #ifndef __AVX2__
    #define __AVX2__
  #endif
  #ifndef __FMA__
    #define __FMA__
  #endif
#endif
