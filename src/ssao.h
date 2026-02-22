#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;

inline void applySSAO(float radius = 10.0f, float strength = 0.5f) {
  const int zMin = std::numeric_limits<int>::min();
  const int R = (int)radius;

  // Sample offsets: 8 directions at 3 different radii for stability
  const int R1 = std::max(1, R / 3);
  const int R2 = std::max(2, R * 2 / 3);
  const int R3 = R;
  const int offsets[][2] = {
    {-R1,0},{R1,0},{0,-R1},{0,R1},
    {-R2,-R2},{R2,-R2},{-R2,R2},{R2,R2},
    {-R3,0},{R3,0},{0,-R3},{0,R3},
    {-R3,-R3},{R3,-R3},{-R3,R3},{R3,R3},
  };
  const int nOffsets = 16;

#ifdef __AVX2__
  const __m256i zMinV = _mm256_set1_epi32(zMin);
  const __m256 strengthV = _mm256_set1_ps(strength);
  const __m256 oneV = _mm256_set1_ps(1.0f);
  const __m256 minAO = _mm256_set1_ps(0.3f);
  const __m256 hundredV = _mm256_set1_ps(100.0f);
  const __m256 threshScale = _mm256_set1_ps(0.01f);
  const __m256i mask8 = _mm256_set1_epi32(0xFF);

  for (unsigned y = 0; y < rH; ++y) {
    unsigned pxRow = (rH - 1 - y) * W;
    for (unsigned x = 0; x + 7 < rW; x += 8) {
      unsigned idx = y * W + x;
      __m256i centerZ = _mm256_loadu_si256((__m256i*)(zbuff.data() + idx));

      __m256i isSky = _mm256_cmpeq_epi32(centerZ, zMinV);
      if (_mm256_testc_si256(isSky, _mm256_set1_epi32(-1))) continue;

      __m256 centerZf = _mm256_cvtepi32_ps(centerZ);
      __m256 absCenterZ = _mm256_andnot_ps(_mm256_set1_ps(-0.0f), centerZf);
      __m256i thresh = _mm256_cvttps_epi32(_mm256_mul_ps(absCenterZ, threshScale));

      __m256 rcpNegCenterZ = _mm256_rcp_ps(_mm256_sub_ps(_mm256_setzero_ps(), centerZf));

      __m256 totalWeight = _mm256_setzero_ps();
      __m256i totalSamples = _mm256_setzero_si256();

      for (int s = 0; s < nOffsets; ++s) {
        int sx = (int)x + offsets[s][0];
        int sy = (int)y + offsets[s][1];
        if (sx < 0 || sx + 7 >= (int)rW || sy < 0 || sy >= (int)rH) continue;

        __m256i sampleZ = _mm256_loadu_si256((__m256i*)(zbuff.data() + sy * W + sx));

        __m256i sampleValid = _mm256_xor_si256(_mm256_cmpeq_epi32(sampleZ, zMinV), _mm256_set1_epi32(-1));
        __m256i valid = _mm256_andnot_si256(isSky, sampleValid);

        totalSamples = _mm256_add_epi32(totalSamples, _mm256_and_si256(valid, _mm256_set1_epi32(1)));

        __m256i diff = _mm256_sub_epi32(sampleZ, centerZ);
        __m256i occluding = _mm256_and_si256(_mm256_cmpgt_epi32(diff, thresh), valid);

        __m256 w = _mm256_mul_ps(_mm256_mul_ps(_mm256_cvtepi32_ps(diff), hundredV), rcpNegCenterZ);
        w = _mm256_min_ps(w, oneV);
        w = _mm256_and_ps(w, _mm256_castsi256_ps(occluding));
        totalWeight = _mm256_add_ps(totalWeight, w);
      }

      __m256 hasOcc = _mm256_cmp_ps(totalWeight, _mm256_setzero_ps(), _CMP_GT_OQ);
      __m256 totalSamplesF = _mm256_cvtepi32_ps(totalSamples);
      __m256 hasSamples = _mm256_cmp_ps(totalSamplesF, _mm256_setzero_ps(), _CMP_GT_OQ);
      __m256 applyMask = _mm256_and_ps(hasOcc, hasSamples);
      if (_mm256_testz_ps(applyMask, applyMask)) continue;

      __m256 frac = _mm256_mul_ps(totalWeight, _mm256_rcp_ps(_mm256_max_ps(totalSamplesF, oneV)));
      __m256 ao = _mm256_max_ps(minAO, _mm256_sub_ps(oneV, _mm256_mul_ps(strengthV, frac)));
      ao = _mm256_blendv_ps(oneV, ao, applyMask);

      __m256 needsWork = _mm256_cmp_ps(ao, oneV, _CMP_LT_OQ);
      __m256i px = _mm256_loadu_si256((__m256i*)(pixels.data() + pxRow + x));

      __m256 rf = _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(px, 16), mask8)), ao);
      __m256 gf = _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(px, 8), mask8)), ao);
      __m256 bf = _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_and_si256(px, mask8)), ao);

      __m256i result = _mm256_or_si256(
        _mm256_slli_epi32(_mm256_cvttps_epi32(rf), 16),
        _mm256_or_si256(_mm256_slli_epi32(_mm256_cvttps_epi32(gf), 8), _mm256_cvttps_epi32(bf)));
      result = _mm256_blendv_epi8(px, result, _mm256_castps_si256(needsWork));
      _mm256_storeu_si256((__m256i*)(pixels.data() + pxRow + x), result);
    }
  }
#else
  for (unsigned y = 0; y < rH; ++y) {
    unsigned pxRow = (rH - 1 - y) * W;
    for (unsigned x = 0; x < rW; ++x) {
      int centerZ = zbuff[y * W + x];
      if (centerZ == zMin) continue;

      int closerCount = 0;
      int totalSamples = 0;
      float totalWeight = 0.0f;
      int thresh = (int)(fabsf((float)centerZ) * 0.01f);

      for (int s = 0; s < nOffsets; ++s) {
        int sx = (int)x + offsets[s][0];
        int sy = (int)y + offsets[s][1];
        if (sx < 0 || sx >= (int)rW || sy < 0 || sy >= (int)rH) continue;

        int sZ = zbuff[sy * W + sx];
        if (sZ == zMin) continue;
        totalSamples++;

        int diff = sZ - centerZ;
        if (diff > thresh) {
          float w = (float)diff / (float)(-centerZ) * 100.0f;
          if (w > 1.0f) w = 1.0f;
          totalWeight += w;
          closerCount++;
        }
      }

      if (closerCount == 0 || totalSamples == 0) continue;

      float frac = totalWeight / (float)totalSamples;
      float ao = 1.0f - strength * frac;
      if (ao < 0.3f) ao = 0.3f;
      if (ao >= 1.0f) continue;

      unsigned px = pixels[pxRow + x];
      unsigned pr = (unsigned)((float)((px >> 16) & 0xFF) * ao);
      unsigned pg = (unsigned)((float)((px >> 8) & 0xFF) * ao);
      unsigned pb = (unsigned)((float)(px & 0xFF) * ao);
      pixels[pxRow + x] = (pr << 16) | (pg << 8) | pb;
    }
  }
#endif
}
