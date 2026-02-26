#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>
#include <cstring>

extern std::array<unsigned, BUF_SZ> pixels;

inline void applyAA(float threshold = 24.0f) {
  static std::array<unsigned, W * H> copy;
  std::memcpy(copy.data(), pixels.data(), W * rH * sizeof(unsigned));

  const float invThresh = 1.0f / threshold;

#if defined(__AVX2__) && defined(__FMA__)
  const __m256i maskFF = _mm256_set1_epi32(0xFF);
  const __m256 vInvThresh = _mm256_set1_ps(invThresh);
  const __m256 vHalf = _mm256_set1_ps(0.5f);
  const __m256 vQuarter = _mm256_set1_ps(0.25f);
  const __m256 vOne = _mm256_set1_ps(1.0f);
  const __m256 v255 = _mm256_set1_ps(255.0f);
  const __m256 vFour = _mm256_set1_ps(4.0f);
  // Luminance weights: R*2 + G*5 + B, then >>3
  const __m256 wR = _mm256_set1_ps(2.0f / 8.0f);
  const __m256 wG = _mm256_set1_ps(5.0f / 8.0f);
  const __m256 wB = _mm256_set1_ps(1.0f / 8.0f);

  for (unsigned y = 1; y < rH - 1; ++y) {
    unsigned x = 1;
    // Process 8 pixels at a time, stop before right edge
    for (; x + 8 <= rW - 1; x += 8) {
      const unsigned idx = y * W + x;

      // Load center and 4 neighbors
      __m256i cPix = _mm256_loadu_si256((__m256i*)&copy[idx]);
      __m256i nUp  = _mm256_loadu_si256((__m256i*)&copy[idx - W]);
      __m256i nDn  = _mm256_loadu_si256((__m256i*)&copy[idx + W]);
      __m256i nLt  = _mm256_loadu_si256((__m256i*)&copy[idx - 1]);
      __m256i nRt  = _mm256_loadu_si256((__m256i*)&copy[idx + 1]);

      // Extract center RGB as floats
      __m256 cR = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(cPix, 16), maskFF));
      __m256 cG = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(cPix, 8), maskFF));
      __m256 cB = _mm256_cvtepi32_ps(_mm256_and_si256(cPix, maskFF));

      // Center luminance
      __m256 cL = _mm256_fmadd_ps(cR, wR, _mm256_fmadd_ps(cG, wG, _mm256_mul_ps(cB, wB)));

      // Helper: compute luminance from packed pixel
      #define LUM(p) _mm256_fmadd_ps( \
        _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(p, 16), maskFF)), wR, \
        _mm256_fmadd_ps( \
          _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(p, 8), maskFF)), wG, \
          _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_and_si256(p, maskFF)), wB)))

      __m256 lUp = LUM(nUp);
      __m256 lDn = LUM(nDn);
      __m256 lLt = LUM(nLt);
      __m256 lRt = LUM(nRt);
      #undef LUM

      // Absolute differences
      __m256 signMask = _mm256_set1_ps(-0.0f);
      __m256 d0 = _mm256_andnot_ps(signMask, _mm256_sub_ps(cL, lUp));
      __m256 d1 = _mm256_andnot_ps(signMask, _mm256_sub_ps(cL, lDn));
      __m256 d2 = _mm256_andnot_ps(signMask, _mm256_sub_ps(cL, lLt));
      __m256 d3 = _mm256_andnot_ps(signMask, _mm256_sub_ps(cL, lRt));

      __m256 maxD = _mm256_max_ps(_mm256_max_ps(d0, d1), _mm256_max_ps(d2, d3));

      // Skip if all below minimum edge threshold (4)
      __m256 edgeMask = _mm256_cmp_ps(maxD, vFour, _CMP_GE_OQ);
      if (_mm256_testz_ps(edgeMask, edgeMask)) continue;

      // blend = min(maxD * invThresh, 1) * 0.5
      __m256 blend = _mm256_mul_ps(_mm256_min_ps(_mm256_mul_ps(maxD, vInvThresh), vOne), vHalf);
      // Zero out blend for non-edge pixels
      blend = _mm256_and_ps(blend, edgeMask);
      __m256 inv = _mm256_sub_ps(vOne, blend);

      // Average neighbor RGB
      #define CH(p, shift) _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(p, shift), maskFF))
      __m256 nR = _mm256_mul_ps(_mm256_add_ps(_mm256_add_ps(CH(nUp,16), CH(nDn,16)), _mm256_add_ps(CH(nLt,16), CH(nRt,16))), vQuarter);
      __m256 nG = _mm256_mul_ps(_mm256_add_ps(_mm256_add_ps(CH(nUp,8), CH(nDn,8)), _mm256_add_ps(CH(nLt,8), CH(nRt,8))), vQuarter);
      __m256 nB = _mm256_mul_ps(_mm256_add_ps(_mm256_add_ps(CH(nUp,0), CH(nDn,0)), _mm256_add_ps(CH(nLt,0), CH(nRt,0))), vQuarter);
      #undef CH

      __m256 rOut = _mm256_fmadd_ps(cR, inv, _mm256_mul_ps(nR, blend));
      __m256 gOut = _mm256_fmadd_ps(cG, inv, _mm256_mul_ps(nG, blend));
      __m256 bOut = _mm256_fmadd_ps(cB, inv, _mm256_mul_ps(nB, blend));

      __m256i result = _mm256_or_si256(
        _mm256_slli_epi32(_mm256_cvttps_epi32(_mm256_min_ps(v255, rOut)), 16),
        _mm256_or_si256(
          _mm256_slli_epi32(_mm256_cvttps_epi32(_mm256_min_ps(v255, gOut)), 8),
          _mm256_cvttps_epi32(_mm256_min_ps(v255, bOut))));

      // Only write edge pixels, keep non-edge original
      result = _mm256_blendv_epi8(cPix, result, _mm256_castps_si256(edgeMask));
      _mm256_storeu_si256((__m256i*)&pixels[idx], result);
    }

    // Scalar tail
    for (; x < rW - 1; ++x) {
      const unsigned idx = y * W + x;
      const unsigned c = copy[idx];
      const int cR = (c >> 16) & 0xFF, cG = (c >> 8) & 0xFF, cB = c & 0xFF;
      const int cL = (cR * 2 + cG * 5 + cB) >> 3;

      const unsigned n0 = copy[idx-W], n1 = copy[idx+W], n2 = copy[idx-1], n3 = copy[idx+1];
      auto lum = [](unsigned p) -> int { return (((p>>16)&0xFF)*2+((p>>8)&0xFF)*5+(p&0xFF))>>3; };

      int d0 = std::abs(cL-lum(n0)), d1 = std::abs(cL-lum(n1));
      int d2 = std::abs(cL-lum(n2)), d3 = std::abs(cL-lum(n3));
      int maxD = d0; if(d1>maxD) maxD=d1; if(d2>maxD) maxD=d2; if(d3>maxD) maxD=d3;
      if (maxD < 4) continue;

      float blend = (float)maxD * invThresh;
      if (blend > 1.0f) blend = 1.0f;
      blend *= 0.5f;
      float inv = 1.0f - blend;

      float nR = (float)(((n0>>16)&0xFF)+((n1>>16)&0xFF)+((n2>>16)&0xFF)+((n3>>16)&0xFF))*0.25f;
      float nG = (float)(((n0>>8)&0xFF)+((n1>>8)&0xFF)+((n2>>8)&0xFF)+((n3>>8)&0xFF))*0.25f;
      float nB = (float)((n0&0xFF)+(n1&0xFF)+(n2&0xFF)+(n3&0xFF))*0.25f;

      pixels[idx] = ((unsigned)(cR*inv+nR*blend)<<16)|((unsigned)(cG*inv+nG*blend)<<8)|(unsigned)(cB*inv+nB*blend);
    }
  }

#else
  for (unsigned y = 1; y < rH - 1; ++y) {
    for (unsigned x = 1; x < rW - 1; ++x) {
      const unsigned idx = y * W + x;
      const unsigned c = copy[idx];
      const int cR = (c >> 16) & 0xFF, cG = (c >> 8) & 0xFF, cB = c & 0xFF;
      const int cL = (cR * 2 + cG * 5 + cB) >> 3;

      const unsigned n0 = copy[idx-W], n1 = copy[idx+W], n2 = copy[idx-1], n3 = copy[idx+1];
      auto lum = [](unsigned p) -> int { return (((p>>16)&0xFF)*2+((p>>8)&0xFF)*5+(p&0xFF))>>3; };

      int d0 = std::abs(cL-lum(n0)), d1 = std::abs(cL-lum(n1));
      int d2 = std::abs(cL-lum(n2)), d3 = std::abs(cL-lum(n3));
      int maxD = d0; if(d1>maxD) maxD=d1; if(d2>maxD) maxD=d2; if(d3>maxD) maxD=d3;
      if (maxD < 4) continue;

      float blend = (float)maxD * invThresh;
      if (blend > 1.0f) blend = 1.0f;
      blend *= 0.5f;
      float inv = 1.0f - blend;

      float nR = (float)(((n0>>16)&0xFF)+((n1>>16)&0xFF)+((n2>>16)&0xFF)+((n3>>16)&0xFF))*0.25f;
      float nG = (float)(((n0>>8)&0xFF)+((n1>>8)&0xFF)+((n2>>8)&0xFF)+((n3>>8)&0xFF))*0.25f;
      float nB = (float)((n0&0xFF)+(n1&0xFF)+(n2&0xFF)+(n3&0xFF))*0.25f;

      pixels[idx] = ((unsigned)(cR*inv+nR*blend)<<16)|((unsigned)(cG*inv+nG*blend)<<8)|(unsigned)(cB*inv+nB*blend);
    }
  }
#endif
}
