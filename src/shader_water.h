#pragma once

#include <cmath>

constexpr unsigned WATER_FLAG_SHADER = 0x01000000;

extern float globalTime;

inline unsigned waterColor(float x, float z, const vertex<float>& norm, float luminance, float wx, float wy, float wz) {
  float ripple = sinf(x * 1.5f + globalTime * 2.0f) * cosf(z * 1.3f + globalTime * 1.7f) * 0.15f;
  vertex<float> pNorm(norm._x + ripple, norm._y, norm._z + ripple * 0.7f);
  pNorm.normalize();

  illumination il = getLight(pNorm, luminance, wx, wy, wz);
  unsigned r = fast_min(255, (int)(30 * il._R));
  unsigned g = fast_min(255, (int)(55 * il._G));
  unsigned b = fast_min(255, (int)(90 * il._B));
  return WATER_FLAG_SHADER | (r << 16) | (g << 8) | b;
}

#ifdef __AVX2__
// Fast vectorized sin approximation (Bhaskara-style, valid for all inputs).
// Wraps input to [-pi, pi] then uses a degree-5 minimax polynomial.
inline __m256 _mm256_sin_approx(__m256 x) {
  const __m256 tp = _mm256_set1_ps(1.0f / (2.0f * 3.14159265f));
  const __m256 half = _mm256_set1_ps(0.5f);
  const __m256 B = _mm256_set1_ps(4.0f);
  const __m256 C = _mm256_set1_ps(-4.0f);
  const __m256 P = _mm256_set1_ps(0.225f);

  // x = x * (1/(2*pi)) - floor(x * (1/(2*pi)) + 0.5)  → wrap to [-0.5, 0.5]
  __m256 t = _mm256_mul_ps(x, tp);
  t = _mm256_sub_ps(t, _mm256_floor_ps(_mm256_add_ps(t, half)));

  // Parabolic approximation: y = B*t + C*t*|t|, then refine: y = P*(y*|y| - y) + y
  __m256 absT = _mm256_andnot_ps(_mm256_set1_ps(-0.0f), t);
  __m256 y = _mm256_add_ps(_mm256_mul_ps(B, t), _mm256_mul_ps(C, _mm256_mul_ps(t, absT)));
  __m256 absY = _mm256_andnot_ps(_mm256_set1_ps(-0.0f), y);
  y = _mm256_add_ps(y, _mm256_mul_ps(P, _mm256_sub_ps(_mm256_mul_ps(y, absY), y)));
  return y;
}

inline __m256 _mm256_cos_approx(__m256 x) {
  return _mm256_sin_approx(_mm256_add_ps(x, _mm256_set1_ps(1.5707963f)));
}

inline __m256i waterColorV(const __m256& xV, const __m256& zV,
                           const vertex<float>& norm, float luminance,
                           const __m256& wx, const __m256& wy, const __m256& wz) {
  __m256 sinX = _mm256_sin_approx(_mm256_add_ps(_mm256_mul_ps(xV, _mm256_set1_ps(1.5f)), _mm256_set1_ps(globalTime * 2.0f)));
  __m256 cosZ = _mm256_cos_approx(_mm256_add_ps(_mm256_mul_ps(zV, _mm256_set1_ps(1.3f)), _mm256_set1_ps(globalTime * 1.7f)));
  __m256 ripple = _mm256_mul_ps(_mm256_mul_ps(sinX, cosZ), _mm256_set1_ps(0.15f));

  __m256 normX = _mm256_add_ps(_mm256_set1_ps(norm._x), ripple);
  __m256 normY = _mm256_set1_ps(norm._y);
  __m256 normZ = _mm256_add_ps(_mm256_set1_ps(norm._z), _mm256_mul_ps(ripple, _mm256_set1_ps(0.7f)));

  // Normalize
  __m256 len = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(normX, normX), _mm256_mul_ps(normY, normY)), _mm256_mul_ps(normZ, normZ));
  __m256 invLen = _mm256_rsqrt_ps(len);
  normX = _mm256_mul_ps(normX, invLen);
  normY = _mm256_mul_ps(normY, invLen);
  normZ = _mm256_mul_ps(normZ, invLen);

  __m256 rV, gV, bV;
  getLight(normX, normY, normZ, luminance, wx, wy, wz, rV, gV, bV);

  __m256 maxV = _mm256_set1_ps(255.0f);
  __m256i rI = _mm256_slli_epi32(_mm256_cvttps_epi32(_mm256_min_ps(maxV, _mm256_mul_ps(_mm256_set1_ps(30.0f), rV))), 16);
  __m256i gI = _mm256_slli_epi32(_mm256_cvttps_epi32(_mm256_min_ps(maxV, _mm256_mul_ps(_mm256_set1_ps(55.0f), gV))), 8);
  __m256i bI = _mm256_cvttps_epi32(_mm256_min_ps(maxV, _mm256_mul_ps(_mm256_set1_ps(90.0f), bV)));

  return _mm256_or_si256(_mm256_set1_epi32(WATER_FLAG_SHADER), _mm256_or_si256(rI, _mm256_or_si256(gI, bI)));
}
#endif

class WaterXZShader : public UntexturedShader, public BehindCamera {
 public:
  WaterXZShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }
  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    return waterColor(x, z, _norm, _luminance, x, y, z);
  }
#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    colorsData = waterColorV(xV, zV, _norm, _luminance, xV, yV, zV);
  }
#endif
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {}
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {}
 private:
  const float _luminance;
  const vertex<float>& _norm;
};
