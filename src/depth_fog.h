#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>
#include <limits>

extern std::array<unsigned, BUF_SZ> pixels;
extern std::array<int, BUF_SZ> zbuff;

// Depth fog: blends pixels toward fogColor based on camera-space distance.
// zbuffer stores camZ * depth (negative; higher = closer).
// fogStart/fogEnd are world-distance fractions: 0 = at camera, 1 = at far clip.
inline void applyDepthFog(float nearClip, float farClip,
                          unsigned fogColor = 0x8090A0,
                          float fogNear = 0.3f,
                          float fogFar  = 1.0f) {
  const float fogR = (float)((fogColor >> 16) & 0xFF);
  const float fogG = (float)((fogColor >> 8) & 0xFF);
  const float fogB = (float)(fogColor & 0xFF);

  // zbuff = camZ * depth, camZ is negative for visible geometry
  // worldDist = -camZ = -zbuff / depth
  // Normalize: t = (worldDist - nearClip) / (farClip - nearClip)  → 0 at near, 1 at far
  // fog = clamp((t - fogNear) / (fogFar - fogNear), 0, 1)
  const float clipRange = farClip - nearClip;
  if (clipRange <= 0.0f) return;
  const float fogRange = fogFar - fogNear;
  if (fogRange <= 0.0f) return;

  const float negInvDepth = -1.0f / (float)depth;  // to convert zbuff → worldDist
  const float invClipRange = 1.0f / clipRange;
  const float invFogRange = 1.0f / fogRange;
  const int empty = std::numeric_limits<int>::min();

  // NOTE: pixels[] is Y-flipped (row 0 = screen bottom) but zbuff[] is not
  // (row 0 = screen top). Must read zbuff from the flipped row.

#if defined(__AVX2__) && defined(__FMA__)
  const __m256 vNegInvDepth = _mm256_set1_ps(negInvDepth);
  const __m256 vNearClip = _mm256_set1_ps(nearClip);
  const __m256 vInvClipRange = _mm256_set1_ps(invClipRange);
  const __m256 vFogNear = _mm256_set1_ps(fogNear);
  const __m256 vInvFogRange = _mm256_set1_ps(invFogRange);
  const __m256 vOne = _mm256_set1_ps(1.0f);
  const __m256 vZero = _mm256_setzero_ps();
  const __m256 vFogR = _mm256_set1_ps(fogR);
  const __m256 vFogG = _mm256_set1_ps(fogG);
  const __m256 vFogB = _mm256_set1_ps(fogB);
  const __m256 v255 = _mm256_set1_ps(255.0f);
  const __m256i maskFF = _mm256_set1_epi32(0xFF);
  const __m256i vEmpty = _mm256_set1_epi32(empty);

  for (unsigned py = 0; py < rH; ++py) {
    unsigned pixRow = py * W;
    unsigned zRow = (rH - 1 - py) * W;
    for (unsigned x = 0; x < rW; x += 8) {
      __m256i zVals = _mm256_loadu_si256((__m256i*)&zbuff[zRow + x]);
      __m256i hasgeo = _mm256_xor_si256(_mm256_cmpeq_epi32(zVals, vEmpty), _mm256_set1_epi32(-1));
      if (_mm256_testz_si256(hasgeo, hasgeo)) continue;

      __m256 worldDist = _mm256_mul_ps(_mm256_cvtepi32_ps(zVals), vNegInvDepth);
      __m256 t = _mm256_mul_ps(_mm256_sub_ps(worldDist, vNearClip), vInvClipRange);
      __m256 fog = _mm256_mul_ps(_mm256_sub_ps(t, vFogNear), vInvFogRange);
      fog = _mm256_min_ps(vOne, _mm256_max_ps(vZero, fog));
      __m256 inv = _mm256_sub_ps(vOne, fog);

      __m256i pix = _mm256_loadu_si256((__m256i*)&pixels[pixRow + x]);
      __m256 pR = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(pix, 16), maskFF));
      __m256 pG = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(pix, 8), maskFF));
      __m256 pB = _mm256_cvtepi32_ps(_mm256_and_si256(pix, maskFF));

      __m256 rOut = _mm256_fmadd_ps(pR, inv, _mm256_mul_ps(vFogR, fog));
      __m256 gOut = _mm256_fmadd_ps(pG, inv, _mm256_mul_ps(vFogG, fog));
      __m256 bOut = _mm256_fmadd_ps(pB, inv, _mm256_mul_ps(vFogB, fog));

      rOut = _mm256_min_ps(v255, rOut);
      gOut = _mm256_min_ps(v255, gOut);
      bOut = _mm256_min_ps(v255, bOut);

      __m256i result = _mm256_or_si256(
        _mm256_slli_epi32(_mm256_cvttps_epi32(rOut), 16),
        _mm256_or_si256(
          _mm256_slli_epi32(_mm256_cvttps_epi32(gOut), 8),
          _mm256_cvttps_epi32(bOut)));

      result = _mm256_blendv_epi8(pix, result, hasgeo);
      _mm256_storeu_si256((__m256i*)&pixels[pixRow + x], result);
    }
  }
#else
  for (unsigned py = 0; py < rH; ++py) {
    unsigned pixRow = py * W;
    unsigned zRow = (rH - 1 - py) * W;
    for (unsigned x = 0; x < rW; ++x) {
      int z = zbuff[zRow + x];
      if (z == empty) continue;

      float worldDist = (float)z * negInvDepth;
      float t = (worldDist - nearClip) * invClipRange;
      float fog = (t - fogNear) * invFogRange;
      if (fog <= 0.0f) continue;
      if (fog > 1.0f) fog = 1.0f;
      float inv = 1.0f - fog;

      unsigned p = pixels[pixRow + x];
      unsigned r = (unsigned)((float)((p >> 16) & 0xFF) * inv + fogR * fog);
      unsigned g = (unsigned)((float)((p >> 8) & 0xFF) * inv + fogG * fog);
      unsigned b = (unsigned)((float)(p & 0xFF) * inv + fogB * fog);

      pixels[pixRow + x] = (r << 16) | (g << 8) | b;
    }
  }
#endif
}
