#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>
#include <cstring>

extern std::array<unsigned, W * H> pixels;

inline void applyAA(float threshold = 24.0f) {
  // 3-row ring buffer instead of full-screen copy
  static unsigned ring[3][W];
  const float invThresh = 1.0f / threshold;

  // Seed ring with rows 0 and 1
  std::memcpy(ring[0], pixels.data(), W * sizeof(unsigned));
  std::memcpy(ring[1], pixels.data() + W, W * sizeof(unsigned));

#if defined(__AVX2__) && defined(__FMA__)
  const __m256i maskFF = _mm256_set1_epi32(0xFF);
  const __m256 vInvThresh = _mm256_set1_ps(invThresh);
  const __m256 vHalf = _mm256_set1_ps(0.5f);
  const __m256 vQuarter = _mm256_set1_ps(0.25f);
  const __m256 vOne = _mm256_set1_ps(1.0f);
  const __m256 v255 = _mm256_set1_ps(255.0f);
  const __m256 vFour = _mm256_set1_ps(4.0f);
  const __m256 wR = _mm256_set1_ps(2.0f / 8.0f);
  const __m256 wG = _mm256_set1_ps(5.0f / 8.0f);
  const __m256 wB = _mm256_set1_ps(1.0f / 8.0f);
  const __m256 signMask = _mm256_set1_ps(-0.0f);

  for (unsigned y = 1; y < H - 1; ++y) {
    // Load next row into ring
    unsigned nextSlot = (y + 1) % 3;
    std::memcpy(ring[nextSlot], pixels.data() + (y + 1) * W, W * sizeof(unsigned));

    unsigned curSlot = y % 3;
    unsigned prevSlot = (y - 1) % 3;
    const unsigned* rowUp = ring[prevSlot];
    const unsigned* rowCur = ring[curSlot];
    const unsigned* rowDn = ring[nextSlot];

    unsigned x = 1;
    for (; x + 8 <= W - 1; x += 8) {
      __m256i cPix = _mm256_loadu_si256((__m256i*)&rowCur[x]);
      __m256i nUp  = _mm256_loadu_si256((__m256i*)&rowUp[x]);
      __m256i nDn  = _mm256_loadu_si256((__m256i*)&rowDn[x]);
      __m256i nLt  = _mm256_loadu_si256((__m256i*)&rowCur[x - 1]);
      __m256i nRt  = _mm256_loadu_si256((__m256i*)&rowCur[x + 1]);

      __m256 cR = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(cPix, 16), maskFF));
      __m256 cG = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(cPix, 8), maskFF));
      __m256 cB = _mm256_cvtepi32_ps(_mm256_and_si256(cPix, maskFF));
      __m256 cL = _mm256_fmadd_ps(cR, wR, _mm256_fmadd_ps(cG, wG, _mm256_mul_ps(cB, wB)));

      #define LUM(p) _mm256_fmadd_ps( \
        _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(p, 16), maskFF)), wR, \
        _mm256_fmadd_ps( \
          _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(p, 8), maskFF)), wG, \
          _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_and_si256(p, maskFF)), wB)))

      __m256 d0 = _mm256_andnot_ps(signMask, _mm256_sub_ps(cL, LUM(nUp)));
      __m256 d1 = _mm256_andnot_ps(signMask, _mm256_sub_ps(cL, LUM(nDn)));
      __m256 d2 = _mm256_andnot_ps(signMask, _mm256_sub_ps(cL, LUM(nLt)));
      __m256 d3 = _mm256_andnot_ps(signMask, _mm256_sub_ps(cL, LUM(nRt)));
      #undef LUM

      __m256 maxD = _mm256_max_ps(_mm256_max_ps(d0, d1), _mm256_max_ps(d2, d3));
      __m256 edgeMask = _mm256_cmp_ps(maxD, vFour, _CMP_GE_OQ);
      if (_mm256_testz_ps(edgeMask, edgeMask)) continue;

      __m256 blend = _mm256_mul_ps(_mm256_min_ps(_mm256_mul_ps(maxD, vInvThresh), vOne), vHalf);
      blend = _mm256_and_ps(blend, edgeMask);
      __m256 inv = _mm256_sub_ps(vOne, blend);

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

      result = _mm256_blendv_epi8(cPix, result, _mm256_castps_si256(edgeMask));
      _mm256_storeu_si256((__m256i*)&pixels[y * W + x], result);
    }

    for (; x < W - 1; ++x) {
      const unsigned c = rowCur[x];
      const int cR = (c >> 16) & 0xFF, cG = (c >> 8) & 0xFF, cB = c & 0xFF;
      const int cL = (cR * 2 + cG * 5 + cB) >> 3;

      const unsigned n0 = rowUp[x], n1 = rowDn[x], n2 = rowCur[x-1], n3 = rowCur[x+1];
      auto lum = [](unsigned p) -> int { return (((p>>16)&0xFF)*2+((p>>8)&0xFF)*5+(p&0xFF))>>3; };

      int dd0 = std::abs(cL-lum(n0)), dd1 = std::abs(cL-lum(n1));
      int dd2 = std::abs(cL-lum(n2)), dd3 = std::abs(cL-lum(n3));
      int maxD = dd0; if(dd1>maxD) maxD=dd1; if(dd2>maxD) maxD=dd2; if(dd3>maxD) maxD=dd3;
      if (maxD < 4) continue;

      float bl = (float)maxD * invThresh;
      if (bl > 1.0f) bl = 1.0f;
      bl *= 0.5f;
      float inv = 1.0f - bl;

      float nR = (float)(((n0>>16)&0xFF)+((n1>>16)&0xFF)+((n2>>16)&0xFF)+((n3>>16)&0xFF))*0.25f;
      float nG = (float)(((n0>>8)&0xFF)+((n1>>8)&0xFF)+((n2>>8)&0xFF)+((n3>>8)&0xFF))*0.25f;
      float nB = (float)((n0&0xFF)+(n1&0xFF)+(n2&0xFF)+(n3&0xFF))*0.25f;

      pixels[y * W + x] = ((unsigned)(cR*inv+nR*bl)<<16)|((unsigned)(cG*inv+nG*bl)<<8)|(unsigned)(cB*inv+nB*bl);
    }
  }

#else
  for (unsigned y = 1; y < H - 1; ++y) {
    unsigned nextSlot = (y + 1) % 3;
    std::memcpy(ring[nextSlot], pixels.data() + (y + 1) * W, W * sizeof(unsigned));

    unsigned curSlot = y % 3;
    unsigned prevSlot = (y - 1) % 3;
    const unsigned* rowUp = ring[prevSlot];
    const unsigned* rowCur = ring[curSlot];
    const unsigned* rowDn = ring[nextSlot];

    for (unsigned x = 1; x < W - 1; ++x) {
      const unsigned c = rowCur[x];
      const int cR = (c >> 16) & 0xFF, cG = (c >> 8) & 0xFF, cB = c & 0xFF;
      const int cL = (cR * 2 + cG * 5 + cB) >> 3;

      const unsigned n0 = rowUp[x], n1 = rowDn[x], n2 = rowCur[x-1], n3 = rowCur[x+1];
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

      pixels[y * W + x] = ((unsigned)(cR*inv+nR*blend)<<16)|((unsigned)(cG*inv+nG*blend)<<8)|(unsigned)(cB*inv+nB*blend);
    }
  }
#endif
}
