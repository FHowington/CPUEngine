#include <iostream>
#include "geometry.h"
#include <immintrin.h>

template <>
matrix<4,4> matrix<4,4>::identity() {
  matrix<4,4> res;
  for (unsigned idx = 0; idx < 4; ++idx) {
    res._m[idx * 4 + idx] = 1;
  }
  return res;
}


template <>
template <>
matrix<4,4> matrix<4,4>::operator*<4>(const matrix<4,4>& rhs) const {
  // This needs to be vectorized
  matrix<4,4> result;

  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      float res = 0;
      for (int k=0; k<4; k++) {
        res += _m[i * 4 + k] * rhs._m[k * 4 + j];
      }
      result._m[i * 4 + j] = res;
    }
  }

  return result;
}

template <>
template <>
matrix<4,1> matrix<4,4>::operator*<1>(const matrix<4,1>& rhs) const {
  matrix<4,1> result;

  __m128 res = _mm_set1_ps(0.0);
  __m128 v1 = _mm_load_ps(rhs._m);
  __m128 v2 = _mm_set_ps(_m[15], _m[10], _m[5], _m[0]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(_m[12], _m[11], _m[6], _m[1]);
  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(_m[13], _m[8], _m[7], _m[2]);
  res = _mm_fmadd_ps(v1, v2, res);


  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(_m[14], _m[9], _m[4], _m[3]);
  res = _mm_fmadd_ps(v1, v2, res);

  _mm_stream_ps(result._m, res);
  return result;
}
