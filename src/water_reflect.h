#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>

constexpr unsigned WATER_FLAG = 0x01000000;
constexpr float WATER_Y = -5.2f;

extern std::array<unsigned, BUF_SZ> pixels;
extern std::array<unsigned, BUF_SZ> reflectionBuf;
extern std::array<int, BUF_SZ> reflectionZBuf;

inline void applyWaterReflection(bool reflections = true) {
  if (!reflections) {
    // Just strip the water flag from all pixels
    for (unsigned py = 0; py < rH; ++py) {
      unsigned off = py * W;
#ifdef __AVX2__
      const __m256i vMask = _mm256_set1_epi32(~WATER_FLAG);
      for (unsigned x = 0; x + 8 <= rW; x += 8) {
        __m256i px = _mm256_loadu_si256((__m256i*)&pixels[off + x]);
        _mm256_storeu_si256((__m256i*)&pixels[off + x], _mm256_and_si256(px, vMask));
      }
#endif
      for (unsigned x = 0; x < rW; ++x)
        pixels[off + x] &= ~WATER_FLAG;
    }
    return;
  }
#if defined(__AVX2__) && defined(__FMA__)
  const __m256i vFlag = _mm256_set1_epi32(WATER_FLAG);
  const __m256i vZero = _mm256_setzero_si256();
  const __m256i maskFF = _mm256_set1_epi32(0xFF);
  const __m256 vReflW = _mm256_set1_ps(100.0f / 256.0f);
  const __m256 vWaterW = _mm256_set1_ps(156.0f / 256.0f);
  const __m256 v255 = _mm256_set1_ps(255.0f);

  for (unsigned py = 0; py < rH; ++py) {
    unsigned off = py * W;
    unsigned x = 0;
    for (; x + 8 <= rW; x += 8) {
      __m256i px = _mm256_loadu_si256((__m256i*)&pixels[off + x]);
      __m256i isWater = _mm256_cmpeq_epi32(_mm256_and_si256(px, vFlag), vFlag);
      if (_mm256_testz_si256(isWater, isWater)) continue;

      __m256i reflPx = _mm256_loadu_si256((__m256i*)&reflectionBuf[off + x]);
      __m256i waterPx = _mm256_andnot_si256(vFlag, px);
      __m256i reflBad = _mm256_cmpeq_epi32(reflPx, vZero);

      // Extract channels as float for FMA blend
      __m256 wR = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(waterPx, 16), maskFF));
      __m256 wG = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(waterPx, 8), maskFF));
      __m256 wB = _mm256_cvtepi32_ps(_mm256_and_si256(waterPx, maskFF));
      __m256 rR = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(reflPx, 16), maskFF));
      __m256 rG = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(reflPx, 8), maskFF));
      __m256 rB = _mm256_cvtepi32_ps(_mm256_and_si256(reflPx, maskFF));

      // blend = refl * (100/256) + water * (156/256)
      __m256 bR = _mm256_min_ps(v255, _mm256_fmadd_ps(rR, vReflW, _mm256_mul_ps(wR, vWaterW)));
      __m256 bG = _mm256_min_ps(v255, _mm256_fmadd_ps(rG, vReflW, _mm256_mul_ps(wG, vWaterW)));
      __m256 bB = _mm256_min_ps(v255, _mm256_fmadd_ps(rB, vReflW, _mm256_mul_ps(wB, vWaterW)));

      __m256i blended = _mm256_or_si256(
        _mm256_slli_epi32(_mm256_cvttps_epi32(bR), 16),
        _mm256_or_si256(
          _mm256_slli_epi32(_mm256_cvttps_epi32(bG), 8),
          _mm256_cvttps_epi32(bB)));

      // For bad reflection pixels, use plain water color
      blended = _mm256_blendv_epi8(blended, waterPx, reflBad);
      // Only apply to water pixels
      __m256i result = _mm256_blendv_epi8(px, blended, isWater);
      _mm256_storeu_si256((__m256i*)&pixels[off + x], result);
    }
    // Scalar tail
    for (; x < rW; ++x) {
      unsigned px = pixels[off + x];
      if (!(px & WATER_FLAG)) continue;
      unsigned reflPx = reflectionBuf[off + x];
      unsigned wr = (px >> 16) & 0xFF, wg = (px >> 8) & 0xFF, wb = px & 0xFF;
      if (reflPx == 0) { pixels[off + x] = (wr << 16) | (wg << 8) | wb; continue; }
      unsigned rr = (reflPx >> 16) & 0xFF, rg = (reflPx >> 8) & 0xFF, rb = reflPx & 0xFF;
      unsigned fr = (rr * 100 + wr * 156) >> 8;
      unsigned fg = (rg * 100 + wg * 156) >> 8;
      unsigned fb = (rb * 100 + wb * 156) >> 8;
      if (fr > 255) fr = 255; if (fg > 255) fg = 255; if (fb > 255) fb = 255;
      pixels[off + x] = (fr << 16) | (fg << 8) | fb;
    }
  }
#else
  for (unsigned py = 0; py < rH; ++py) {
    unsigned off = py * W;
    for (unsigned x = 0; x < rW; ++x) {
      unsigned px = pixels[off + x];
      if (!(px & WATER_FLAG)) continue;
      unsigned reflPx = reflectionBuf[off + x];
      unsigned wr = (px >> 16) & 0xFF, wg = (px >> 8) & 0xFF, wb = px & 0xFF;
      if (reflPx == 0) { pixels[off + x] = (wr << 16) | (wg << 8) | wb; continue; }
      unsigned rr = (reflPx >> 16) & 0xFF, rg = (reflPx >> 8) & 0xFF, rb = reflPx & 0xFF;
      unsigned fr = (rr * 100 + wr * 156) >> 8;
      unsigned fg = (rg * 100 + wg * 156) >> 8;
      unsigned fb = (rb * 100 + wb * 156) >> 8;
      if (fr > 255) fr = 255; if (fg > 255) fg = 255; if (fb > 255) fb = 255;
      pixels[off + x] = (fr << 16) | (fg << 8) | fb;
    }
  }
#endif
}
