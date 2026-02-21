#pragma once

#include <cmath>

// --- Procedural stone wall helpers ---
inline float stoneHash(int a, int b) {
  int h = a * 374761393 + b * 668265263;
  h = (h ^ (h >> 13)) * 1274126177;
  return (float)(h & 0xFFFF) / 65535.0f;
}

inline unsigned stoneColor(float u, float v, const vertex<float>& faceNorm, const vertex<float>& tanU, const vertex<float>& tanV, float luminance, float wx, float wy, float wz) {
  const float blockW = 2.0f;
  const float blockH = 1.0f;
  const float mortar = 0.06f;

  int row = (int)floorf(v / blockH);
  float uShifted = u + (row & 1) * (blockW * 0.5f);
  int col = (int)floorf(uShifted / blockW);

  float uLocal = uShifted - col * blockW;
  float vLocal = v - row * blockH;

  bool isMortar = (uLocal < mortar || uLocal > blockW - mortar || vLocal < mortar || vLocal > blockH - mortar);

  unsigned baseR, baseG, baseB;
  vertex<float> norm = faceNorm;

  if (isMortar) {
    baseR = 60; baseG = 58; baseB = 55;
  } else {
    float h = stoneHash(col, row);
    baseR = (unsigned)(140 + h * 40);
    baseG = (unsigned)(130 + h * 30);
    baseB = (unsigned)(115 + h * 25);

    float edgeU = fminf(uLocal - mortar, blockW - mortar - uLocal);
    float edgeV = fminf(vLocal - mortar, blockH - mortar - vLocal);
    float bevel = 0.15f;

    if (edgeU < bevel) {
      float t = 1.0f - edgeU / bevel;
      float sign = (uLocal - mortar < blockW - mortar - uLocal) ? -1.0f : 1.0f;
      norm._x += tanU._x * sign * t * 0.5f;
      norm._y += tanU._y * sign * t * 0.5f;
      norm._z += tanU._z * sign * t * 0.5f;
    }
    if (edgeV < bevel) {
      float t = 1.0f - edgeV / bevel;
      float sign = (vLocal - mortar < blockH - mortar - vLocal) ? -1.0f : 1.0f;
      norm._x += tanV._x * sign * t * 0.5f;
      norm._y += tanV._y * sign * t * 0.5f;
      norm._z += tanV._z * sign * t * 0.5f;
    }
    norm.normalize();

    float noise = stoneHash((int)(u * 7.3f), (int)(v * 7.7f));
    baseR = (unsigned)fminf(255, baseR + (noise - 0.5f) * 20);
    baseG = (unsigned)fminf(255, baseG + (noise - 0.5f) * 15);
    baseB = (unsigned)fminf(255, baseB + (noise - 0.5f) * 12);
  }

  illumination il = getLight(norm, luminance, wx, wy, wz);
  unsigned r = fast_min(255, (int)(baseR * il._R));
  unsigned g = fast_min(255, (int)(baseG * il._G));
  unsigned b = fast_min(255, (int)(baseB * il._B));
  return (r << 16) | (g << 8) | b;
}

#ifdef __AVX2__
// AVX2 integer hash: h = a*374761393 + b*668265263; h = (h^(h>>13))*1274126177; return (h&0xFFFF)/65535
inline __m256 stoneHashV(const __m256i a, const __m256i b) {
  __m256i h = _mm256_add_epi32(_mm256_mullo_epi32(a, _mm256_set1_epi32(374761393)),
                               _mm256_mullo_epi32(b, _mm256_set1_epi32(668265263)));
  h = _mm256_mullo_epi32(_mm256_xor_si256(h, _mm256_srli_epi32(h, 13)), _mm256_set1_epi32(1274126177));
  return _mm256_div_ps(_mm256_cvtepi32_ps(_mm256_and_si256(h, _mm256_set1_epi32(0xFFFF))), _mm256_set1_ps(65535.0f));
}

// Process 8 stone pixels. uV/vV are the UV coords, tanU/tanV/faceNorm are uniform per-triangle.
// Returns packed ARGB __m256i with lighting applied.
inline __m256i stoneColorV(const __m256& uV, const __m256& vV,
                           const vertex<float>& faceNorm,
                           const vertex<float>& tanU, const vertex<float>& tanV,
                           float luminance,
                           const __m256& wx, const __m256& wy, const __m256& wz) {
  const __m256 blockW = _mm256_set1_ps(2.0f);
  const __m256 mortarV = _mm256_set1_ps(0.06f);
  const __m256 halfBlockW = _mm256_set1_ps(1.0f);
  const __m256 bevelV = _mm256_set1_ps(0.15f);
  const __m256 half = _mm256_set1_ps(0.5f);
  const __m256 zero = _mm256_setzero_ps();
  const __m256 one = _mm256_set1_ps(1.0f);

  // row = floor(v / blockH) = floor(v)
  __m256 rowF = _mm256_floor_ps(vV);
  __m256i rowI = _mm256_cvttps_epi32(rowF);

  // uShifted = u + (row&1) * halfBlockW
  __m256 oddRow = _mm256_cvtepi32_ps(_mm256_and_si256(rowI, _mm256_set1_epi32(1)));
  __m256 uShifted = _mm256_add_ps(uV, _mm256_mul_ps(oddRow, halfBlockW));

  // col = floor(uShifted / blockW) = floor(uShifted / 2)
  __m256 colF = _mm256_floor_ps(_mm256_mul_ps(uShifted, half));
  __m256i colI = _mm256_cvttps_epi32(colF);

  // uLocal = uShifted - col * blockW
  __m256 uLocal = _mm256_sub_ps(uShifted, _mm256_mul_ps(colF, blockW));
  // vLocal = v - row * blockH
  __m256 vLocal = _mm256_sub_ps(vV, rowF);

  // Mortar detection: uLocal < mortar || uLocal > blockW-mortar || vLocal < mortar || vLocal > blockH-mortar
  __m256 blockWmM = _mm256_set1_ps(2.0f - 0.06f);
  __m256 blockHmM = _mm256_set1_ps(1.0f - 0.06f);
  __m256 m1 = _mm256_cmp_ps(uLocal, mortarV, _CMP_LT_OQ);
  __m256 m2 = _mm256_cmp_ps(uLocal, blockWmM, _CMP_GT_OQ);
  __m256 m3 = _mm256_cmp_ps(vLocal, mortarV, _CMP_LT_OQ);
  __m256 m4 = _mm256_cmp_ps(vLocal, blockHmM, _CMP_GT_OQ);
  __m256 isMortar = _mm256_or_ps(_mm256_or_ps(m1, m2), _mm256_or_ps(m3, m4));

  // Hash for per-block color variation
  __m256 hVal = stoneHashV(colI, rowI);

  // Base stone colors: R=140+h*40, G=130+h*30, B=115+h*25
  __m256 baseR = _mm256_add_ps(_mm256_set1_ps(140.0f), _mm256_mul_ps(hVal, _mm256_set1_ps(40.0f)));
  __m256 baseG = _mm256_add_ps(_mm256_set1_ps(130.0f), _mm256_mul_ps(hVal, _mm256_set1_ps(30.0f)));
  __m256 baseB = _mm256_add_ps(_mm256_set1_ps(115.0f), _mm256_mul_ps(hVal, _mm256_set1_ps(25.0f)));

  // Noise: stoneHash((int)(u*7.3), (int)(v*7.7))
  __m256i noiseA = _mm256_cvttps_epi32(_mm256_mul_ps(uV, _mm256_set1_ps(7.3f)));
  __m256i noiseB = _mm256_cvttps_epi32(_mm256_mul_ps(vV, _mm256_set1_ps(7.7f)));
  __m256 noise = stoneHashV(noiseA, noiseB);
  __m256 noiseCentered = _mm256_sub_ps(noise, half);
  baseR = _mm256_add_ps(baseR, _mm256_mul_ps(noiseCentered, _mm256_set1_ps(20.0f)));
  baseG = _mm256_add_ps(baseG, _mm256_mul_ps(noiseCentered, _mm256_set1_ps(15.0f)));
  baseB = _mm256_add_ps(baseB, _mm256_mul_ps(noiseCentered, _mm256_set1_ps(12.0f)));

  // Clamp to [0,255]
  __m256 maxV = _mm256_set1_ps(255.0f);
  baseR = _mm256_min_ps(maxV, _mm256_max_ps(zero, baseR));
  baseG = _mm256_min_ps(maxV, _mm256_max_ps(zero, baseG));
  baseB = _mm256_min_ps(maxV, _mm256_max_ps(zero, baseB));

  // Mortar: blend to mortar color
  __m256 mortarR = _mm256_set1_ps(60.0f);
  __m256 mortarG = _mm256_set1_ps(58.0f);
  __m256 mortarB = _mm256_set1_ps(55.0f);
  baseR = _mm256_blendv_ps(baseR, mortarR, isMortar);
  baseG = _mm256_blendv_ps(baseG, mortarG, isMortar);
  baseB = _mm256_blendv_ps(baseB, mortarB, isMortar);

  // Normal perturbation (for non-mortar pixels)
  // edgeU = min(uLocal - mortar, blockW-mortar - uLocal)
  __m256 fromLeft = _mm256_sub_ps(uLocal, mortarV);
  __m256 fromRight = _mm256_sub_ps(blockWmM, uLocal);
  __m256 edgeU = _mm256_min_ps(fromLeft, fromRight);
  // edgeV = min(vLocal - mortar, blockH-mortar - vLocal)
  __m256 fromBot = _mm256_sub_ps(vLocal, mortarV);
  __m256 fromTop = _mm256_sub_ps(blockHmM, vLocal);
  __m256 edgeV = _mm256_min_ps(fromBot, fromTop);

  // Normal starts as face normal
  __m256 normX = _mm256_set1_ps(faceNorm._x);
  __m256 normY = _mm256_set1_ps(faceNorm._y);
  __m256 normZ = _mm256_set1_ps(faceNorm._z);

  // Bevel U: if edgeU < bevel, perturb along tanU
  __m256 bevelMaskU = _mm256_andnot_ps(isMortar, _mm256_cmp_ps(edgeU, bevelV, _CMP_LT_OQ));
  __m256 tU = _mm256_sub_ps(one, _mm256_div_ps(edgeU, bevelV));
  __m256 signU = _mm256_blendv_ps(one, _mm256_set1_ps(-1.0f), _mm256_cmp_ps(fromLeft, fromRight, _CMP_LT_OQ));
  __m256 pertU = _mm256_mul_ps(_mm256_mul_ps(signU, tU), half);
  pertU = _mm256_and_ps(pertU, bevelMaskU);
  normX = _mm256_add_ps(normX, _mm256_mul_ps(_mm256_set1_ps(tanU._x), pertU));
  normY = _mm256_add_ps(normY, _mm256_mul_ps(_mm256_set1_ps(tanU._y), pertU));
  normZ = _mm256_add_ps(normZ, _mm256_mul_ps(_mm256_set1_ps(tanU._z), pertU));

  // Bevel V: if edgeV < bevel, perturb along tanV
  __m256 bevelMaskV = _mm256_andnot_ps(isMortar, _mm256_cmp_ps(edgeV, bevelV, _CMP_LT_OQ));
  __m256 tVV = _mm256_sub_ps(one, _mm256_div_ps(edgeV, bevelV));
  __m256 signV = _mm256_blendv_ps(one, _mm256_set1_ps(-1.0f), _mm256_cmp_ps(fromBot, fromTop, _CMP_LT_OQ));
  __m256 pertV = _mm256_mul_ps(_mm256_mul_ps(signV, tVV), half);
  pertV = _mm256_and_ps(pertV, bevelMaskV);
  normX = _mm256_add_ps(normX, _mm256_mul_ps(_mm256_set1_ps(tanV._x), pertV));
  normY = _mm256_add_ps(normY, _mm256_mul_ps(_mm256_set1_ps(tanV._y), pertV));
  normZ = _mm256_add_ps(normZ, _mm256_mul_ps(_mm256_set1_ps(tanV._z), pertV));

  // Normalize perturbed normals
  __m256 len = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(normX, normX), _mm256_mul_ps(normY, normY)), _mm256_mul_ps(normZ, normZ));
  __m256 invLen = _mm256_rsqrt_ps(len);
  normX = _mm256_mul_ps(normX, invLen);
  normY = _mm256_mul_ps(normY, invLen);
  normZ = _mm256_mul_ps(normZ, invLen);

  // Pack base color into ARGB int
  __m256i rI = _mm256_slli_epi32(_mm256_cvttps_epi32(baseR), 16);
  __m256i gI = _mm256_slli_epi32(_mm256_cvttps_epi32(baseG), 8);
  __m256i bI = _mm256_cvttps_epi32(baseB);
  __m256i colorsData = _mm256_or_si256(rI, _mm256_or_si256(gI, bI));

  // Lighting
  __m256 rV, gV, bV;
  getLight(normX, normY, normZ, luminance, wx, wy, wz, rV, gV, bV);
  return vectorLight(colorsData, rV, gV, bV);
}
#endif

class StoneXZShader : public UntexturedShader, public BehindCamera {
 public:
  StoneXZShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }
  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    return stoneColor(x, z, _norm, vertex<float>(1,0,0), vertex<float>(0,0,1), _luminance, x, y, z);
  }
#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    colorsData = stoneColorV(xV, zV, _norm, vertex<float>(1,0,0), vertex<float>(0,0,1), _luminance, xV, yV, zV);
  }
#endif
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {}
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {}
 private:
  const float _luminance;
  const vertex<float>& _norm;
};

class StoneXYShader : public UntexturedShader, public BehindCamera {
 public:
  StoneXYShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }
  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    return stoneColor(x, y, _norm, vertex<float>(1,0,0), vertex<float>(0,1,0), _luminance, x, y, z);
  }
#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    colorsData = stoneColorV(xV, yV, _norm, vertex<float>(1,0,0), vertex<float>(0,1,0), _luminance, xV, yV, zV);
  }
#endif
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {}
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {}
 private:
  const float _luminance;
  const vertex<float>& _norm;
};

class StoneYZShader : public UntexturedShader, public BehindCamera {
 public:
  StoneYZShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }
  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    return stoneColor(z, y, _norm, vertex<float>(0,0,1), vertex<float>(0,1,0), _luminance, x, y, z);
  }
#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    colorsData = stoneColorV(zV, yV, _norm, vertex<float>(0,0,1), vertex<float>(0,1,0), _luminance, xV, yV, zV);
  }
#endif
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {}
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {}
 private:
  const float _luminance;
  const vertex<float>& _norm;
};
