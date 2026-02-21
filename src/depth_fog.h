#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>

extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;

// Depth fog: blends each pixel toward fogColor based on zbuffer distance.
// fogStart/fogEnd are in zbuffer units (0 = far plane, depth = near plane).
// Pixels with zbuff <= fogStart get full fog, >= fogEnd get none.
inline void applyDepthFog(unsigned fogColor = 0x8090A0,
                          float fogNear = 0.3f,   // fraction of depth where fog starts (0=near, 1=far)
                          float fogFar  = 1.0f) {  // fraction of depth where fog is fully opaque
  const float fogR = (float)((fogColor >> 16) & 0xFF);
  const float fogG = (float)((fogColor >> 8) & 0xFF);
  const float fogB = (float)(fogColor & 0xFF);

  // Convert fractions to zbuffer units (zbuff: high=close, 0=far)
  // fogNear: distance where fog begins (close to camera = high zbuff)
  // fogFar: distance where fog is 100% (far from camera = low zbuff)
  // We want: at zbuff=depth (very close), fog=0. At zbuff=0 (very far), fog=1.
  // Normalized distance: dist = 1.0 - zbuff/depth  (0=close, 1=far)
  // fog factor = clamp((dist - fogNear) / (fogFar - fogNear), 0, 1)
  const float invDepth = 1.0f / (float)depth;
  const float range = fogFar - fogNear;
  if (range <= 0.0f) return;
  const float invRange = 1.0f / range;

#if defined(__AVX2__) && defined(__FMA__)
  const __m256 vInvDepth = _mm256_set1_ps(invDepth);
  const __m256 vOne = _mm256_set1_ps(1.0f);
  const __m256 vZero = _mm256_setzero_ps();
  const __m256 vFogNear = _mm256_set1_ps(fogNear);
  const __m256 vInvRange = _mm256_set1_ps(invRange);
  const __m256 vFogR = _mm256_set1_ps(fogR);
  const __m256 vFogG = _mm256_set1_ps(fogG);
  const __m256 vFogB = _mm256_set1_ps(fogB);
  const __m256 v255 = _mm256_set1_ps(255.0f);
  const __m256i maskFF = _mm256_set1_epi32(0xFF);

  for (unsigned i = 0; i < W * H; i += 8) {
    // Load zbuffer values
    __m256i zVals = _mm256_loadu_si256((__m256i*)&zbuff[i]);
    // Skip if all zero (sky/empty)
    if (_mm256_testz_si256(zVals, zVals)) continue;

    __m256 zF = _mm256_cvtepi32_ps(zVals);
    // dist = 1 - z/depth
    __m256 dist = _mm256_sub_ps(vOne, _mm256_mul_ps(zF, vInvDepth));
    // fog = clamp((dist - fogNear) / range, 0, 1)
    __m256 fog = _mm256_mul_ps(_mm256_sub_ps(dist, vFogNear), vInvRange);
    fog = _mm256_min_ps(vOne, _mm256_max_ps(vZero, fog));
    __m256 invFog = _mm256_sub_ps(vOne, fog);

    // Load pixel colors
    __m256i pix = _mm256_loadu_si256((__m256i*)&pixels[i]);
    __m256 pR = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(pix, 16), maskFF));
    __m256 pG = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(pix, 8), maskFF));
    __m256 pB = _mm256_cvtepi32_ps(_mm256_and_si256(pix, maskFF));

    // Lerp: result = pixel * (1-fog) + fogColor * fog
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

    // Only write where zbuff > 0 (geometry present)
    __m256i hasgeo = _mm256_cmpgt_epi32(zVals, _mm256_setzero_si256());
    result = _mm256_blendv_epi8(pix, result, hasgeo);
    _mm256_storeu_si256((__m256i*)&pixels[i], result);
  }
#else
  for (unsigned i = 0; i < W * H; ++i) {
    int z = zbuff[i];
    if (z <= 0) continue; // sky

    float dist = 1.0f - (float)z * invDepth;
    float fog = (dist - fogNear) * invRange;
    if (fog <= 0.0f) continue;
    if (fog > 1.0f) fog = 1.0f;
    float invFog = 1.0f - fog;

    unsigned p = pixels[i];
    float pR = (float)((p >> 16) & 0xFF);
    float pG = (float)((p >> 8) & 0xFF);
    float pB = (float)(p & 0xFF);

    unsigned r = (unsigned)(pR * invFog + fogR * fog);
    unsigned g = (unsigned)(pG * invFog + fogG * fog);
    unsigned b = (unsigned)(pB * invFog + fogB * fog);

    pixels[i] = (r << 16) | (g << 8) | b;
  }
#endif
}
