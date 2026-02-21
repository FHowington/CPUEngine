#pragma once

#include <cmath>

// Procedural wood grain: vertical plank pattern with grain lines
inline unsigned woodColor(float u, float v, const vertex<float>& faceNorm, float luminance, float wx, float wy, float wz) {
  const float plankW = 0.8f;
  const float gap = 0.02f;

  int plank = (int)floorf(u / plankW);
  float uLocal = u - plank * plankW;

  // Per-plank variation
  int h = plank * 374761393;
  h = (h ^ (h >> 13)) * 1274126177;
  float pVar = (float)(h & 0xFFFF) / 65535.0f;

  // Gap between planks
  if (uLocal < gap || uLocal > plankW - gap) {
    illumination il = getLight(faceNorm, luminance, wx, wy, wz);
    unsigned r = fast_min(255, (int)(25 * il._R));
    unsigned g = fast_min(255, (int)(18 * il._G));
    unsigned b = fast_min(255, (int)(12 * il._B));
    return (r << 16) | (g << 8) | b;
  }

  // Base wood color: warm brown with per-plank hue shift
  float baseR = 120 + pVar * 35;
  float baseG = 75 + pVar * 25;
  float baseB = 40 + pVar * 15;

  // Grain: sinusoidal lines running along v, offset per plank
  float grain = sinf((v + pVar * 10.0f) * 12.0f) * 0.5f + 0.5f;
  // Fine grain detail
  float fine = sinf((v + pVar * 3.7f) * 47.0f) * 0.3f + 0.5f;
  // Combine
  float g_factor = grain * 0.6f + fine * 0.4f;

  baseR += (g_factor - 0.5f) * 30;
  baseG += (g_factor - 0.5f) * 20;
  baseB += (g_factor - 0.5f) * 10;

  // Knot: occasional dark spot
  int kh = plank * 668265263 + (int)(v * 0.5f) * 1274126177;
  kh = (kh ^ (kh >> 13)) * 374761393;
  float knotChance = (float)(kh & 0xFFFF) / 65535.0f;
  if (knotChance > 0.92f) {
    float knotV = fmodf(v * 0.5f, 1.0f);
    float knotDist = fabsf(knotV - 0.5f) * 2.0f;
    if (knotDist < 0.3f) {
      float dark = knotDist / 0.3f;
      baseR *= (0.4f + 0.6f * dark);
      baseG *= (0.4f + 0.6f * dark);
      baseB *= (0.4f + 0.6f * dark);
    }
  }

  baseR = fminf(255, fmaxf(0, baseR));
  baseG = fminf(255, fmaxf(0, baseG));
  baseB = fminf(255, fmaxf(0, baseB));

  illumination il = getLight(faceNorm, luminance, wx, wy, wz);
  unsigned r = fast_min(255, (int)(baseR * il._R));
  unsigned g = fast_min(255, (int)(baseG * il._G));
  unsigned b = fast_min(255, (int)(baseB * il._B));
  return (r << 16) | (g << 8) | b;
}

#ifdef __AVX2__
inline __m256i woodColorV(const __m256& uV, const __m256& vV,
                          const vertex<float>& faceNorm, float luminance,
                          const __m256& wx, const __m256& wy, const __m256& wz) {
  const __m256 plankW = _mm256_set1_ps(0.8f);
  const __m256 gapV = _mm256_set1_ps(0.02f);
  const __m256 zero = _mm256_setzero_ps();
  const __m256 maxV = _mm256_set1_ps(255.0f);
  const __m256 half = _mm256_set1_ps(0.5f);

  // plank = floor(u / plankW)
  __m256 plankF = _mm256_floor_ps(_mm256_div_ps(uV, plankW));
  __m256i plankI = _mm256_cvttps_epi32(plankF);
  __m256 uLocal = _mm256_sub_ps(uV, _mm256_mul_ps(plankF, plankW));

  // Per-plank hash
  __m256i hV = _mm256_mullo_epi32(plankI, _mm256_set1_epi32(374761393));
  hV = _mm256_mullo_epi32(_mm256_xor_si256(hV, _mm256_srli_epi32(hV, 13)), _mm256_set1_epi32(1274126177));
  __m256 pVar = _mm256_div_ps(_mm256_cvtepi32_ps(_mm256_and_si256(hV, _mm256_set1_epi32(0xFFFF))), _mm256_set1_ps(65535.0f));

  // Gap detection
  __m256 plankWmG = _mm256_sub_ps(plankW, gapV);
  __m256 isGap = _mm256_or_ps(_mm256_cmp_ps(uLocal, gapV, _CMP_LT_OQ),
                              _mm256_cmp_ps(uLocal, plankWmG, _CMP_GT_OQ));

  // Base wood color
  __m256 baseR = _mm256_add_ps(_mm256_set1_ps(120.0f), _mm256_mul_ps(pVar, _mm256_set1_ps(35.0f)));
  __m256 baseG = _mm256_add_ps(_mm256_set1_ps(75.0f), _mm256_mul_ps(pVar, _mm256_set1_ps(25.0f)));
  __m256 baseB = _mm256_add_ps(_mm256_set1_ps(40.0f), _mm256_mul_ps(pVar, _mm256_set1_ps(15.0f)));

  // Grain: sin((v + pVar*10) * 12) * 0.5 + 0.5
  // Approximate sin with polynomial for speed: sin(x) ≈ x - x³/6 (good enough for visual grain)
  // Actually, let's just use the scalar approach per-lane since this is visual-only
  // We'll compute a simple hash-based grain instead for SIMD friendliness
  __m256 grainPhase = _mm256_mul_ps(_mm256_add_ps(vV, _mm256_mul_ps(pVar, _mm256_set1_ps(10.0f))), _mm256_set1_ps(12.0f));
  // Wrap to [0,1] via fract
  __m256 grainFract = _mm256_sub_ps(grainPhase, _mm256_floor_ps(grainPhase));
  // Triangle wave: abs(fract*2 - 1)
  __m256 grain = _mm256_sub_ps(_mm256_mul_ps(grainFract, _mm256_set1_ps(2.0f)), _mm256_set1_ps(1.0f));
  // abs
  grain = _mm256_max_ps(grain, _mm256_sub_ps(zero, grain));

  __m256 finePhase = _mm256_mul_ps(_mm256_add_ps(vV, _mm256_mul_ps(pVar, _mm256_set1_ps(3.7f))), _mm256_set1_ps(47.0f));
  __m256 fineFract = _mm256_sub_ps(finePhase, _mm256_floor_ps(finePhase));
  __m256 fine = _mm256_sub_ps(_mm256_mul_ps(fineFract, _mm256_set1_ps(2.0f)), _mm256_set1_ps(1.0f));
  fine = _mm256_max_ps(fine, _mm256_sub_ps(zero, fine));

  __m256 gFactor = _mm256_add_ps(_mm256_mul_ps(grain, _mm256_set1_ps(0.6f)), _mm256_mul_ps(fine, _mm256_set1_ps(0.4f)));
  __m256 gCentered = _mm256_sub_ps(gFactor, half);

  baseR = _mm256_add_ps(baseR, _mm256_mul_ps(gCentered, _mm256_set1_ps(30.0f)));
  baseG = _mm256_add_ps(baseG, _mm256_mul_ps(gCentered, _mm256_set1_ps(20.0f)));
  baseB = _mm256_add_ps(baseB, _mm256_mul_ps(gCentered, _mm256_set1_ps(10.0f)));

  baseR = _mm256_min_ps(maxV, _mm256_max_ps(zero, baseR));
  baseG = _mm256_min_ps(maxV, _mm256_max_ps(zero, baseG));
  baseB = _mm256_min_ps(maxV, _mm256_max_ps(zero, baseB));

  // Blend gap pixels to dark brown
  baseR = _mm256_blendv_ps(baseR, _mm256_set1_ps(25.0f), isGap);
  baseG = _mm256_blendv_ps(baseG, _mm256_set1_ps(18.0f), isGap);
  baseB = _mm256_blendv_ps(baseB, _mm256_set1_ps(12.0f), isGap);

  // Pack and light
  __m256i rI = _mm256_slli_epi32(_mm256_cvttps_epi32(baseR), 16);
  __m256i gI = _mm256_slli_epi32(_mm256_cvttps_epi32(baseG), 8);
  __m256i bI = _mm256_cvttps_epi32(baseB);
  __m256i colorsData = _mm256_or_si256(rI, _mm256_or_si256(gI, bI));

  __m256 rV, gV, bV;
  getLight(_mm256_set1_ps(faceNorm._x), _mm256_set1_ps(faceNorm._y),
           _mm256_set1_ps(faceNorm._z), luminance, wx, wy, wz, rV, gV, bV);
  return vectorLight(colorsData, rV, gV, bV);
}
#endif

class WoodXZShader : public UntexturedShader, public BehindCamera {
 public:
  WoodXZShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }
  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    return woodColor(x, z, litNormal(_norm), _luminance, x, y, z);
  }
#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    colorsData = woodColorV(xV, zV, litNormal(_norm), _luminance, xV, yV, zV);
  }
#endif
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {}
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {}
 private:
  const float _luminance;
  const vertex<float>& _norm;
};

class WoodXYShader : public UntexturedShader, public BehindCamera {
 public:
  WoodXYShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }
  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    return woodColor(x, y, litNormal(_norm), _luminance, x, y, z);
  }
#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    colorsData = woodColorV(xV, yV, litNormal(_norm), _luminance, xV, yV, zV);
  }
#endif
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {}
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {}
 private:
  const float _luminance;
  const vertex<float>& _norm;
};

class WoodYZShader : public UntexturedShader, public BehindCamera {
 public:
  WoodYZShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }
  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    return woodColor(z, y, litNormal(_norm), _luminance, x, y, z);
  }
#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    colorsData = woodColorV(zV, yV, litNormal(_norm), _luminance, xV, yV, zV);
  }
#endif
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {}
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {}
 private:
  const float _luminance;
  const vertex<float>& _norm;
};
