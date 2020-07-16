#include "Window.h"
#include "geometry.h"
#include <array>
#include <immintrin.h>
#include <iostream>

#ifdef __AVX__
#define MakeShuffleMask(x,y,z,w)           (x | (y<<2) | (z<<4) | (w<<6))

// vec(0, 1, 2, 3) -> (vec[x], vec[y], vec[z], vec[w])
#define VecSwizzleMask(vec, mask)          _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(vec), mask))
#define VecSwizzle(vec, x, y, z, w)        VecSwizzleMask(vec, MakeShuffleMask(x,y,z,w))
#define VecSwizzle1(vec, x)                VecSwizzleMask(vec, MakeShuffleMask(x,x,x,x))
// special swizzle
#define VecSwizzle_0022(vec)               _mm_moveldup_ps(vec)
#define VecSwizzle_1133(vec)               _mm_movehdup_ps(vec)

// return (vec1[x], vec1[y], vec2[z], vec2[w])
#define VecShuffle(vec1, vec2, x,y,z,w)    _mm_shuffle_ps(vec1, vec2, MakeShuffleMask(x,y,z,w))
// special shuffle
#define VecShuffle_0101(vec1, vec2)        _mm_movelh_ps(vec1, vec2)
#define VecShuffle_2323(vec1, vec2)        _mm_movehl_ps(vec2, vec1)

// for row major matrix
// we use __m128 to represent 2x2 matrix as A = | A0  A1 |
//                                              | A2  A3 |
// 2x2 row major Matrix multiply A*B
 __m128 Mat2Mul(__m128 vec1, __m128 vec2)
{
  return
      _mm_add_ps(_mm_mul_ps(                     vec1, VecSwizzle(vec2, 0,3,0,3)),
                 _mm_mul_ps(VecSwizzle(vec1, 1,0,3,2), VecSwizzle(vec2, 2,1,2,1)));
}
// 2x2 row major Matrix adjugate multiply (A#)*B
 __m128 Mat2AdjMul(__m128 vec1, __m128 vec2)
{
  return
      _mm_sub_ps(_mm_mul_ps(VecSwizzle(vec1, 3,3,0,0), vec2),
                 _mm_mul_ps(VecSwizzle(vec1, 1,1,2,2), VecSwizzle(vec2, 2,3,0,1)));

}
// 2x2 row major Matrix multiply adjugate A*(B#)
__m128 Mat2MulAdj(__m128 vec1, __m128 vec2)
{
  return
      _mm_sub_ps(_mm_mul_ps(                     vec1, VecSwizzle(vec2, 3,0,3,0)),
                 _mm_mul_ps(VecSwizzle(vec1, 1,0,3,2), VecSwizzle(vec2, 2,1,2,1)));
}
#endif

template <>
matrix<4,4> matrix<4,4>::identity() {
  matrix<4,4> res;
  for (unsigned idx = 0; idx < 4; ++idx) {
    res._m[idx * 4 + idx] = 1;
  }
  return res;
}

template <>
matrix<4,4> matrix<4,4>::rotationY(float rotY) {
  matrix<4,4> res;
  res.set(1, 1, 1);
  res.set(3, 3, 1);

  res.set(0, 0, cos(rotY));
  res.set(0, 2, -sin(rotY));
  res.set(2, 0, sin(rotY));
  res.set(2, 2, cos(rotY));
  return res;
}

template <>
matrix<4,4> matrix<4,4>::rotationX(float rotX) {
  matrix<4,4> res;
  res.set(0, 0, 1);
  res.set(3, 3, 1);

  res.set(1, 1, cos(rotX));
  res.set(1, 2, sin(rotX));
  res.set(2, 1, -sin(rotX));
  res.set(2, 2, cos(rotX));
  return res;
}

template <>
matrix<4,4> matrix<4,4>::rotationZ(float rotZ) {
  matrix<4,4> res;
  res.set(2, 2, 1);
  res.set(3, 3, 1);

  res.set(0, 0, cos(rotZ));
  res.set(0, 1, sin(rotZ));
  res.set(1, 0, -sin(rotZ));
  res.set(1, 1, cos(rotZ));
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


#ifdef __FMA__
template <>
template <>
matrix<4,1> matrix<4,4>::operator*<1>(const matrix<4,1>& rhs) const {
  matrix<4,1> result;

  __m128 res = _mm_set1_ps(0.0);
  __m128 v1 = _mm_load_ps(rhs._m.data());
  __m128 v2 = _mm_set_ps(_m[15], _m[10], _m[5], _m[0]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(_m[3], _m[14], _m[9], _m[4]);
  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(_m[7], _m[2], _m[13], _m[8]);
  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(_m[11], _m[6], _m[1], _m[12]);
  res = _mm_fmadd_ps(v1, v2, res);

  _mm_stream_ps(result._m.data(), res);
  return result;
}
#else
template <>
template <>
matrix<4,1> matrix<4,4>::operator*<1>(const matrix<4,1>& rhs) const {
  matrix<4,1> result;

  result._m[0] = rhs._m[0] * _m[0] + rhs._m[1] * _m[4] + rhs._m[2] * _m[8] + rhs._m[3] * _m[12];
  result._m[1] = rhs._m[0] * _m[1] + rhs._m[1] * _m[5] + rhs._m[2] * _m[9] + rhs._m[3] * _m[13];
  result._m[2] = rhs._m[0] * _m[2] + rhs._m[1] * _m[6] + rhs._m[2] * _m[10] + rhs._m[3] * _m[14];
  result._m[3] = rhs._m[0] * _m[3] + rhs._m[1] * _m[7] + rhs._m[2] * _m[11] + rhs._m[3] * _m[15];

  return result;
}
#endif

#ifdef __AVX__
// Fast matrix inverse using SIMD
// I did NOT write this, only modified. Credit goes to Eric Zhang
// Taken from: https://lxjk.github.io/2017/09/03/Fast-4x4-Matrix-Inverse-with-SSE-SIMD-Explained.html
matrix<4,4> invert(const matrix<4,4>& inM)
{
  __m128 A = VecShuffle_0101(*((__m128*)&inM._m[0]), *((__m128*)&inM._m[4]));
  __m128 B = VecShuffle_2323(*((__m128*)&inM._m[0]), *((__m128*)&inM._m[4]));
  __m128 C = VecShuffle_0101(*((__m128*)&inM._m[8]), *((__m128*)&inM._m[12]));
  __m128 D = VecShuffle_2323(*((__m128*)&inM._m[8]), *((__m128*)&inM._m[12]));

  __m128 detSub = _mm_sub_ps(
      _mm_mul_ps(VecShuffle(*((__m128*)&inM._m[0]), *((__m128*)&inM._m[8]), 0,2,0,2), VecShuffle(*((__m128*)&inM._m[4]), *((__m128*)&inM._m[12]), 1,3,1,3)),
      _mm_mul_ps(VecShuffle(*((__m128*)&inM._m[0]), *((__m128*)&inM._m[8]), 1,3,1,3), VecShuffle(*((__m128*)&inM._m[4]), *((__m128*)&inM._m[12]), 0,2,0,2)));

  __m128 detA = VecSwizzle1(detSub, 0);
  __m128 detB = VecSwizzle1(detSub, 1);
  __m128 detC = VecSwizzle1(detSub, 2);
  __m128 detD = VecSwizzle1(detSub, 3);

  // let iM = 1/|M| * | X  Y |
  //                  | Z  W |

  // D#C
  __m128 D_C = Mat2AdjMul(D, C);
  // A#B
  __m128 A_B = Mat2AdjMul(A, B);
  // X# = |D|A - B(D#C)
  __m128 X_ = _mm_sub_ps(_mm_mul_ps(detD, A), Mat2Mul(B, D_C));
  // W# = |A|D - C(A#B)
  __m128 W_ = _mm_sub_ps(_mm_mul_ps(detA, D), Mat2Mul(C, A_B));

  // |M| = |A|*|D| + ... (continue later)
  __m128 detM = _mm_mul_ps(detA, detD);

  // Y# = |B|C - D(A#B)#
  __m128 Y_ = _mm_sub_ps(_mm_mul_ps(detB, C), Mat2MulAdj(D, A_B));
  // Z# = |C|B - A(D#C)#
  __m128 Z_ = _mm_sub_ps(_mm_mul_ps(detC, B), Mat2MulAdj(A, D_C));

  // |M| = |A|*|D| + |B|*|C| ... (continue later)
  detM = _mm_add_ps(detM, _mm_mul_ps(detB, detC));

  // tr((A#B)(D#C))
  __m128 tr = _mm_mul_ps(A_B, VecSwizzle(D_C, 0,2,1,3));
  tr = _mm_hadd_ps(tr, tr);
  tr = _mm_hadd_ps(tr, tr);
  // |M| = |A|*|D| + |B|*|C| - tr((A#B)(D#C)
  detM = _mm_sub_ps(detM, tr);

  const __m128 adjSignMask = _mm_setr_ps(1.F, -1.F, -1.F, 1.F);
  // (1/|M|, -1/|M|, -1/|M|, 1/|M|)
  __m128 rDetM = _mm_div_ps(adjSignMask, detM);

  X_ = _mm_mul_ps(X_, rDetM);
  Y_ = _mm_mul_ps(Y_, rDetM);
  Z_ = _mm_mul_ps(Z_, rDetM);
  W_ = _mm_mul_ps(W_, rDetM);

  matrix<4,4> r;

  // apply adjugate and store, here we combine adjugate shuffle and store shuffle
  *((__m128*)&r._m[0]) = VecShuffle(X_, Y_, 3,1,3,1);
  *((__m128*)&r._m[4]) = VecShuffle(X_, Y_, 2,0,2,0);
  *((__m128*)&r._m[8]) = VecShuffle(Z_, W_, 3,1,3,1);
  *((__m128*)&r._m[12]) = VecShuffle(Z_, W_, 2,0,2,0);

  return r;
}

#else
// 4x4 non-SIMD matrix inversion
// Lifted from the MESA implementation of GLU library
// https://www.mesa3d.org/
matrix<4,4> invert(const matrix<4,4>& in) {
  const auto* m = static_cast<const float*>(in._m.data());

  std::array<float, 16> inv{};
  float det;
  int i;

  inv[0] = m[5]  * m[10] * m[15] -
           m[5]  * m[11] * m[14] -
           m[9]  * m[6]  * m[15] +
           m[9]  * m[7]  * m[14] +
           m[13] * m[6]  * m[11] -
           m[13] * m[7]  * m[10];

  inv[4] = -m[4]  * m[10] * m[15] +
           m[4]  * m[11] * m[14] +
           m[8]  * m[6]  * m[15] -
           m[8]  * m[7]  * m[14] -
           m[12] * m[6]  * m[11] +
           m[12] * m[7]  * m[10];

  inv[8] = m[4]  * m[9] * m[15] -
           m[4]  * m[11] * m[13] -
           m[8]  * m[5] * m[15] +
           m[8]  * m[7] * m[13] +
           m[12] * m[5] * m[11] -
           m[12] * m[7] * m[9];

  inv[12] = -m[4]  * m[9] * m[14] +
            m[4]  * m[10] * m[13] +
            m[8]  * m[5] * m[14] -
            m[8]  * m[6] * m[13] -
            m[12] * m[5] * m[10] +
            m[12] * m[6] * m[9];

  inv[1] = -m[1]  * m[10] * m[15] +
           m[1]  * m[11] * m[14] +
           m[9]  * m[2] * m[15] -
           m[9]  * m[3] * m[14] -
           m[13] * m[2] * m[11] +
           m[13] * m[3] * m[10];

  inv[5] = m[0]  * m[10] * m[15] -
           m[0]  * m[11] * m[14] -
           m[8]  * m[2] * m[15] +
           m[8]  * m[3] * m[14] +
           m[12] * m[2] * m[11] -
           m[12] * m[3] * m[10];

  inv[9] = -m[0]  * m[9] * m[15] +
           m[0]  * m[11] * m[13] +
           m[8]  * m[1] * m[15] -
           m[8]  * m[3] * m[13] -
           m[12] * m[1] * m[11] +
           m[12] * m[3] * m[9];

  inv[13] = m[0]  * m[9] * m[14] -
            m[0]  * m[10] * m[13] -
            m[8]  * m[1] * m[14] +
            m[8]  * m[2] * m[13] +
            m[12] * m[1] * m[10] -
            m[12] * m[2] * m[9];

  inv[2] = m[1]  * m[6] * m[15] -
           m[1]  * m[7] * m[14] -
           m[5]  * m[2] * m[15] +
           m[5]  * m[3] * m[14] +
           m[13] * m[2] * m[7] -
           m[13] * m[3] * m[6];

  inv[6] = -m[0]  * m[6] * m[15] +
           m[0]  * m[7] * m[14] +
           m[4]  * m[2] * m[15] -
           m[4]  * m[3] * m[14] -
           m[12] * m[2] * m[7] +
           m[12] * m[3] * m[6];

  inv[10] = m[0]  * m[5] * m[15] -
            m[0]  * m[7] * m[13] -
            m[4]  * m[1] * m[15] +
            m[4]  * m[3] * m[13] +
            m[12] * m[1] * m[7] -
            m[12] * m[3] * m[5];

  inv[14] = -m[0]  * m[5] * m[14] +
            m[0]  * m[6] * m[13] +
            m[4]  * m[1] * m[14] -
            m[4]  * m[2] * m[13] -
            m[12] * m[1] * m[6] +
            m[12] * m[2] * m[5];

  inv[3] = -m[1] * m[6] * m[11] +
           m[1] * m[7] * m[10] +
           m[5] * m[2] * m[11] -
           m[5] * m[3] * m[10] -
           m[9] * m[2] * m[7] +
           m[9] * m[3] * m[6];

  inv[7] = m[0] * m[6] * m[11] -
           m[0] * m[7] * m[10] -
           m[4] * m[2] * m[11] +
           m[4] * m[3] * m[10] +
           m[8] * m[2] * m[7] -
           m[8] * m[3] * m[6];

  inv[11] = -m[0] * m[5] * m[11] +
            m[0] * m[7] * m[9] +
            m[4] * m[1] * m[11] -
            m[4] * m[3] * m[9] -
            m[8] * m[1] * m[7] +
            m[8] * m[3] * m[5];

  inv[15] = m[0] * m[5] * m[10] -
            m[0] * m[6] * m[9] -
            m[4] * m[1] * m[10] +
            m[4] * m[2] * m[9] +
            m[8] * m[1] * m[6] -
            m[8] * m[2] * m[5];

  det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

  matrix<4,4> res;

  if (det == 0) {
    std::cout << "Matrix inversion failed\n";
    return res;
  }

  det = 1.0F / det;


  for (i = 0; i < 16; i++) {
    res._m[i] = inv[i] * det;
  }

  return res;
}
#endif

matrix<4,1> v2m(const vertex<float>& v) {
  matrix<4,1> m;
  m.set(0, 0, v._x);
  m.set(0, 1, v._y);
  m.set(0, 2, v._z);
  m.set(0, 3, 1);
  return m;
}

vertex<int> m2v(const matrix<4,1> m) {
  return vertex<int>(m._m[0], m._m[1], m._m[2]);
}

 vertex<float> m2vf(const matrix<4,1> m) {
  return vertex<float>(m._m[0], m._m[1], m._m[2]);
}

#if defined(__AVX__) && defined(__FMA__)
bool pipelineSlow(const matrix<4,4>& cameraTransform, const matrix<4,4>& model, const vertex<float>& v, vertex<int>& retResult, vertex<float>& realResult) {
  float __attribute__((aligned(16))) result[4]; // NOLINT
  bool bounded = true;

  // Model transform
  __m128 res = _mm_set1_ps(0.0);
  __m128 v1 = _mm_set_ps(1, v._z, v._y, v._x);
  __m128 v2 = _mm_set_ps(model._m[15], model._m[10], model._m[5], model._m[0]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(model._m[3], model._m[14], model._m[9], model._m[4]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(model._m[7], model._m[2], model._m[13], model._m[8]);
  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(model._m[11], model._m[6], model._m[1], model._m[12]);

  v1 = _mm_fmadd_ps(v1, v2, res);

  _mm_stream_ps((float *)result, v1);
  realResult = vertex<float>(result[0], result[1], result[2]);

  // Camera transform (model space to camera space)
  v2 = _mm_set_ps(cameraTransform._m[15], cameraTransform._m[10], cameraTransform._m[5], cameraTransform._m[0]);
  res = _mm_set1_ps(0.0);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(cameraTransform._m[3], cameraTransform._m[14], cameraTransform._m[9], cameraTransform._m[4]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(cameraTransform._m[7], cameraTransform._m[2], cameraTransform._m[13], cameraTransform._m[8]);
  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(cameraTransform._m[11], cameraTransform._m[6], cameraTransform._m[1], cameraTransform._m[12]);
  res = _mm_fmadd_ps(v1, v2, res);

  float zDist = _mm_cvtss_f32(_mm_shuffle_ps(res, res, _MM_SHUFFLE(0, 0, 0, 2)));
  //Apply clip boundaries
  if (zDist >= -1) {
    bounded = false;
    res = _mm_insert_ps(res, _mm_set1_ps(-1.0), 32);
  } else if (zDist < -50) {
    bounded = false;
  }

  // Perspective projection
  v1 = _mm_permute_ps(res, 0b10111010);

  v2 = _mm_set_ps(-focalLength, 1.0, -focalLength, -focalLength);
  v1 = _mm_mul_ps(v1, v2);
  v1 = _mm_div_ps(res, v1);

  _mm_stream_ps((float *)result, v1);
  result[0] *= W * xZoom;
  result[0] += W * xFOV;
  result[1] *= H * yZoom;
  result[1] += H * yFOV;

  result[2] *= depth;
  retResult = vertex<int>(result[0], result[1], result[2]);
  return bounded;
}

bool pipelineFast(const matrix<4,4>& cameraTransform, const matrix<4,4>& model, const vertex<float>& v, vertex<int>& retResult, vertex<float>& realResult) {
  float __attribute__((aligned(16))) result[4]; // NOLINT

  // Model transform
  __m128 res = _mm_set1_ps(0.0);
  __m128 v1 = _mm_set_ps(1, v._z, v._y, v._x);
  __m128 v2 = _mm_set_ps(model._m[15], model._m[10], model._m[5], model._m[0]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(model._m[3], model._m[14], model._m[9], model._m[4]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(model._m[7], model._m[2], model._m[13], model._m[8]);
  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(model._m[11], model._m[6], model._m[1], model._m[12]);

  v1 = _mm_fmadd_ps(v1, v2, res);

  _mm_stream_ps((float *)result, v1);
  realResult = vertex<float>(result[0], result[1], result[2]);

  // Camera transform (model space to camera space)
  v2 = _mm_set_ps(cameraTransform._m[15], cameraTransform._m[10], cameraTransform._m[5], cameraTransform._m[0]);
  res = _mm_set1_ps(0.0);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(cameraTransform._m[3], cameraTransform._m[14], cameraTransform._m[9], cameraTransform._m[4]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(cameraTransform._m[7], cameraTransform._m[2], cameraTransform._m[13], cameraTransform._m[8]);
  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(cameraTransform._m[11], cameraTransform._m[6], cameraTransform._m[1], cameraTransform._m[12]);
  res = _mm_fmadd_ps(v1, v2, res);

  float zDist = _mm_cvtss_f32(_mm_shuffle_ps(res, res, _MM_SHUFFLE(0, 0, 0, 2)));
  //Apply clip boundaries
  if (zDist >= -1 || zDist < -50) {
    return false;
  }

  // Perspective projection
  v1 = _mm_permute_ps(res, 0b10111010);

  v2 = _mm_set_ps(-focalLength, 1.0, -focalLength, -focalLength);
  v1 = _mm_mul_ps(v1, v2);
  v1 = _mm_div_ps(res, v1);

  _mm_stream_ps((float *)result, v1);
  result[0] *= W * xZoom;
  result[0] += W * xFOV;
  result[1] *= H * yZoom;
  result[1] += H * yFOV;

  result[2] *= depth;
  retResult = vertex<int>(result[0], result[1], result[2]);
  return true;
}

#else
bool pipelineSlow(const matrix<4,4>& cameraTransform, const matrix<4,4>& model, const vertex<float>& v, vertex<int>& retResult, vertex<float>& realResult) {
  matrix<4,1> imres(model * v2m(v));
  bool bounded = true;

  realResult = vertex<float>(imres._m[0], imres._m[1], imres._m[2]);

  imres = (cameraTransform * imres);
  //printf("PRE: %f %f %f\n", imres._m[0], imres._m[1], imres._m[2]);

  // Apply clip boundaries
  if (imres._m[2] > -1) {
    imres._m[2] = -1;
    bounded = false;
  } else if (imres._m[2] < -50) {
    bounded = false;
  }

  imres._m[3] = 1 / (-focalLength * imres._m[2]);
  //printf("3: %f\n", imres._m[3]);

  imres._m[0] = imres._m[0] *= imres._m[3];
  imres._m[1] = imres._m[1] *= imres._m[3];

  imres._m[0] *= W * xZoom;
  imres._m[0] += W * xFOV;
  imres._m[1] *= H * yZoom;
  imres._m[1] += H * yFOV;

  imres._m[2] *= depth;

  retResult = vertex<int>(imres._m[0], imres._m[1], imres._m[2]);
  return bounded;;
}

bool pipelineFast(const matrix<4,4>& cameraTransform, const matrix<4,4>& model, const vertex<float>& v, vertex<int>& retResult, vertex<float>& realResult) {
  matrix<4,1> imres(model * v2m(v));

  realResult = vertex<float>(imres._m[0], imres._m[1], imres._m[2]);

  imres = (cameraTransform * imres);
  // Apply clip boundaries
  if (imres._m[2] >= -1 || imres._m[2] < -50) {
    return false;
  }

  imres._m[3] = 1 / (-focalLength * imres._m[2]);
  imres._m[0] = imres._m[0] *= imres._m[3];
  imres._m[1] = imres._m[1] *= imres._m[3];

  imres._m[0] *= W * xZoom;
  imres._m[0] += W * xFOV;
  imres._m[1] *= H * yZoom;
  imres._m[1] += H * yFOV;

  imres._m[2] *= depth;

  retResult = vertex<int>(imres._m[0], imres._m[1], imres._m[2]);
  return true;;
}
#endif


#ifdef __FMA__
vertex<float> multToVector(const matrix<4,4> m, const vertex<float>& v) {
  // Basically a streamlined approach to the vector multiplication
  // We are doing matrix multiplication between a 4x1 vector and a 4x4 matrix, yielding a 4x1 matrix/vector
  float __attribute__((aligned(16))) result[4]; // NOLINT

  __m128 res = _mm_set1_ps(0.0);
  __m128 v1 = _mm_set_ps(1, v._z, v._y, v._x);
  __m128 v2 = _mm_set_ps(m._m[15], m._m[10], m._m[5], m._m[0]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(m._m[3], m._m[14], m._m[9], m._m[4]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(m._m[7], m._m[2], m._m[13], m._m[8]);
  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(m._m[11], m._m[6], m._m[1], m._m[12]);
  res = _mm_fmadd_ps(v1, v2, res);

  _mm_stream_ps((float *)result, res);

  return vertex<float>(result[0], result[1], result[2]);
}
#else
vertex<float> multToVector(const matrix<4,4> m, const vertex<float>& v) {
  matrix<4,1> vec(v2m(v));
  vec = m * vec;
  return vertex<float>(vec._m[0], vec._m[1], vec._m[2]);
}
#endif

#ifdef __FMA__
vertex<float> rotateVector(const matrix<4,4> m, const vertex<float>& v) {
  // Basically a streamlined approach to the vector multiplication
  // We are doing matrix multiplication between a 4x1 vector and a 4x4 matrix, yielding a 4x1 matrix/vector
  __m128 res = _mm_set1_ps(0.0);
  __m128 v1 = _mm_set_ps(v._x, v._z, v._y, v._x);
  __m128 v2 = _mm_set_ps(0, m._m[10], m._m[5], m._m[0]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b00111001);
  v2 = _mm_set_ps(0, m._m[2], m._m[9], m._m[4]);

  res = _mm_fmadd_ps(v1, v2, res);

  v1 = _mm_permute_ps(v1, 0b11111001);
  v2 = _mm_set_ps(0, m._m[6], m._m[1], m._m[8]);
  res = _mm_fmadd_ps(v1, v2, res);

  vertex<float> resultV;
  _mm_stream_ps((float *)resultV.raw, res);

  return resultV;
}
#else
vertex<float> rotateVector(const matrix<4,4> m, const vertex<float>& v) {
  matrix<4,4> temp = m;
  temp._m[12] = temp._m[13] = temp._m[14] = temp._m[15] = 0;
  return m2vf(temp * v2m(v));
}
#endif
