#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>
#include <limits>

extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;

// Depth fog: blends each pixel toward fogColor based on zbuffer distance.
// fogStart/fogEnd are in zbuffer units (0 = far plane, depth = near plane).
// Pixels with zbuff <= fogStart get full fog, >= fogEnd get none.
inline void applyDepthFog(unsigned fogColor = 0x8090A0,
                          float fogNear = 0.3f,
                          float fogFar  = 1.0f) {
  const float fogR = (float)((fogColor >> 16) & 0xFF);
  const float fogG = (float)((fogColor >> 8) & 0xFF);
  const float fogB = (float)(fogColor & 0xFF);

  // Find zbuffer range for this frame
  int zMin = std::numeric_limits<int>::max();
  int zMax = std::numeric_limits<int>::min();
  const int empty = std::numeric_limits<int>::min();
  for (unsigned i = 0; i < W * H; ++i) {
    int z = zbuff[i];
    if (z == empty) continue;
    if (z < zMin) zMin = z;
    if (z > zMax) zMax = z;
  }
  if (zMax <= zMin) return; // nothing rendered

  const float range = (float)(zMax - zMin);
  const float invRange = 1.0f / range;
  // dist: 0 = closest (zMax), 1 = farthest (zMin)
  // fog = clamp((dist - fogNear) / (fogFar - fogNear), 0, 1)
  const float fogRange = fogFar - fogNear;
  if (fogRange <= 0.0f) return;
  const float invFogRange = 1.0f / fogRange;

#if defined(__AVX2__) && defined(__FMA__)
  const __m256 vInvRange = _mm256_set1_ps(invRange);
  const __m256 vZMax = _mm256_set1_ps((float)zMax);
  const __m256 vOne = _mm256_set1_ps(1.0f);
  const __m256 vZero = _mm256_setzero_ps();
  const __m256 vFogNear = _mm256_set1_ps(fogNear);
  const __m256 vInvFogRange = _mm256_set1_ps(invFogRange);
  const __m256 vFogR = _mm256_set1_ps(fogR);
  const __m256 vFogG = _mm256_set1_ps(fogG);
  const __m256 vFogB = _mm256_set1_ps(fogB);
  const __m256 v255 = _mm256_set1_ps(255.0f);
  const __m256i maskFF = _mm256_set1_epi32(0xFF);
  const __m256i vEmpty = _mm256_set1_epi32(empty);

  for (unsigned i = 0; i < W * H; i += 8) {
    __m256i zVals = _mm256_loadu_si256((__m256i*)&zbuff[i]);
    __m256i hasgeo = _mm256_xor_si256(_mm256_cmpeq_epi32(zVals, vEmpty), _mm256_set1_epi32(-1));
    if (_mm256_testz_si256(hasgeo, hasgeo)) continue;

    __m256 zF = _mm256_cvtepi32_ps(zVals);
    // dist = (zMax - z) / range  → 0=close, 1=far
    __m256 dist = _mm256_mul_ps(_mm256_sub_ps(vZMax, zF), vInvRange);
    // fog = clamp((dist - fogNear) / fogRange, 0, 1)
    __m256 fog = _mm256_mul_ps(_mm256_sub_ps(dist, vFogNear), vInvFogRange);
    fog = _mm256_min_ps(vOne, _mm256_max_ps(vZero, fog));
    __m256 invFog = _mm256_sub_ps(vOne, fog);

    __m256i pix = _mm256_loadu_si256((__m256i*)&pixels[i]);
    __m256 pR = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(pix, 16), maskFF));
    __m256 pG = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(pix, 8), maskFF));
    __m256 pB = _mm256_cvtepi32_ps(_mm256_and_si256(pix, maskFF));

    __m256 rOut = _mm256_add_ps(_mm256_mul_ps(pR, invFog), _mm256_mul_ps(vFogR, fog));
    __m256 gOut = _mm256_add_ps(_mm256_mul_ps(pG, invFog), _mm256_mul_ps(vFogG, fog));
    __m256 bOut = _mm256_add_ps(_mm256_mul_ps(pB, invFog), _mm256_mul_ps(vFogB, fog));

    rOut = _mm256_min_ps(v255, rOut);
    gOut = _mm256_min_ps(v255, gOut);
    bOut = _mm256_min_ps(v255, bOut);

    __m256i result = _mm256_or_si256(
      _mm256_slli_epi32(_mm256_cvttps_epi32(rOut), 16),
      _mm256_or_si256(
        _mm256_slli_epi32(_mm256_cvttps_epi32(gOut), 8),
        _mm256_cvttps_epi32(bOut)));

    result = _mm256_blendv_epi8(pix, result, hasgeo);
    _mm256_storeu_si256((__m256i*)&pixels[i], result);
  }
#else
  for (unsigned i = 0; i < W * H; ++i) {
    int z = zbuff[i];
    if (z == empty) continue;

    float dist = (float)(zMax - z) * invRange;
    float fog = (dist - fogNear) * invFogRange;
    if (fog <= 0.0f) continue;
    if (fog > 1.0f) fog = 1.0f;
    float inv = 1.0f - fog;

    unsigned p = pixels[i];
    unsigned r = (unsigned)((float)((p >> 16) & 0xFF) * inv + fogR * fog);
    unsigned g = (unsigned)((float)((p >> 8) & 0xFF) * inv + fogG * fog);
    unsigned b = (unsigned)((float)(p & 0xFF) * inv + fogB * fog);

    pixels[i] = (r << 16) | (g << 8) | b;
  }
#endif
}
