#pragma once

#include "light.h"
#include "Window.h"
#include "geometry.h"
#include "simd_compat.h"
#include <array>
#include <cmath>

extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;
extern matrix<4,4> cameraTransform;
extern float focalLength;

bool pipelineFast(const matrix<4,4>& cameraTransform, const matrix<4,4>& model, const vertex<float>& v, vertex<int>& retResult, vertex<float>& realResult);

inline void applyLightFog(const matrix<4,4>& camTransform, float intensity = 0.35f, float radius = 200.0f) {
  static const matrix<4,4> identity = matrix<4,4>::identity();

  for (const auto& light : Light::sceneLights) {
    if (light._type != LightType::Point) continue;

    vertex<float> lp(light._x, light._y, light._z);
    vertex<int> screenPos;
    vertex<float> worldPos;

    if (!pipelineFast(camTransform, identity, lp, screenPos, worldPos))
      continue;

    float sx = (float)screenPos._x;
    float sy = (float)(H - screenPos._y);
    int lightZ = screenPos._z;

    // Skip fog if light center is off-screen or occluded by geometry
    if (screenPos._x < 0 || screenPos._x >= (int)W ||
        screenPos._y < 0 || screenPos._y >= (int)H) continue;
    int zAtCenter = zbuff[(H - 1 - screenPos._y) * W + screenPos._x];
    if (zAtCenter > lightZ) continue;

    float camZ = (float)lightZ / (float)depth;
    float distScale = 5.0f / fmaxf(1.0f, -camZ);
    float r = radius * distScale;
    if (r < 10.0f) continue;
    float rSq = r * r;

    int x0 = fast_max(0, (int)(sx - r));
    int x1 = fast_min((int)W - 1, (int)(sx + r));
    int y0 = fast_max(0, (int)(sy - r));
    int y1 = fast_min((int)H - 1, (int)(sy + r));

    float lR = light._R * light._strength * intensity * 255.0f;
    float lG = light._G * light._strength * intensity * 255.0f;
    float lB = light._B * light._strength * intensity * 255.0f;

#ifdef __AVX2__
    const __m256 sxV = _mm256_set1_ps(sx);
    const __m256 rSqV = _mm256_set1_ps(rSq);
    const __m256i lightZV = _mm256_set1_epi32(lightZ);
    const __m256 lRV = _mm256_set1_ps(lR);
    const __m256 lGV = _mm256_set1_ps(lG);
    const __m256 lBV = _mm256_set1_ps(lB);
    const __m256 oneV = _mm256_set1_ps(1.0f);
    const __m256i maxChar = _mm256_set1_epi32(255);

    // Align x0 to 8-pixel boundary for cleaner SIMD
    int x0a = x0 & ~7;

    for (int y = y0; y <= y1; ++y) {
      float dy = y - sy;
      __m256 dySqV = _mm256_set1_ps(dy * dy);
      int rowOff = y * W;
      int zRowOff = (H - y) * W;  // zbuff uses non-flipped Y

      for (int x = x0a; x <= x1; x += 8) {
        __m256 dxV = _mm256_sub_ps(
          _mm256_add_ps(_mm256_set1_ps((float)x), _mm256_set_ps(7,6,5,4,3,2,1,0)),
          sxV);
        __m256 dSqV = _mm256_add_ps(_mm256_mul_ps(dxV, dxV), dySqV);

        // Mask: dSq < rSq
        __m256 inRadius = _mm256_cmp_ps(dSqV, rSqV, _CMP_LT_OQ);

        // Mask: x in [x0, x1]
        __m256i xIdxV = _mm256_add_epi32(_mm256_set1_epi32(x), _mm256_set_epi32(7,6,5,4,3,2,1,0));
        __m256 xInRange = _mm256_and_ps(
          _mm256_cmp_ps(_mm256_cvtepi32_ps(xIdxV), _mm256_set1_ps((float)x0), _CMP_GE_OQ),
          _mm256_cmp_ps(_mm256_cvtepi32_ps(xIdxV), _mm256_set1_ps((float)x1), _CMP_LE_OQ));
        inRadius = _mm256_and_ps(inRadius, xInRange);

        if (_mm256_testz_ps(inRadius, inRadius)) continue;

        // Depth occlusion: zbuff[idx] <= lightZ + 2000
        int base = rowOff + x;
        int zbase = zRowOff + x;
        __m256i pzV = _mm256_loadu_si256((__m256i*)(zbuff.data() + zbase));
        __m256i depthOk = _mm256_or_si256(
          _mm256_cmpgt_epi32(lightZV, pzV),
          _mm256_cmpeq_epi32(lightZV, pzV));
        __m256 activeMask = _mm256_and_ps(inRadius, _mm256_castsi256_ps(depthOk));

        if (_mm256_testz_ps(activeMask, activeMask)) continue;

        // fog = (1 - dSq/rSq)^3 — cubic for softer translucent falloff
        __m256 t = _mm256_sub_ps(oneV, _mm256_div_ps(dSqV, rSqV));
        __m256 fog = _mm256_mul_ps(_mm256_mul_ps(t, t), t);
        fog = _mm256_and_ps(fog, activeMask);

        // Load existing pixels
        __m256i pxV = _mm256_loadu_si256((__m256i*)(pixels.data() + base));

        // Extract R, G, B
        __m256 prF = _mm256_cvtepi32_ps(_mm256_srli_epi32(_mm256_and_si256(pxV, _mm256_set1_epi32(0xFF0000)), 16));
        __m256 pgF = _mm256_cvtepi32_ps(_mm256_srli_epi32(_mm256_and_si256(pxV, _mm256_set1_epi32(0x00FF00)), 8));
        __m256 pbF = _mm256_cvtepi32_ps(_mm256_and_si256(pxV, _mm256_set1_epi32(0x0000FF)));

        // Add fog contribution
        prF = _mm256_add_ps(prF, _mm256_mul_ps(fog, lRV));
        pgF = _mm256_add_ps(pgF, _mm256_mul_ps(fog, lGV));
        pbF = _mm256_add_ps(pbF, _mm256_mul_ps(fog, lBV));

        // Clamp to 255
        __m256i prI = _mm256_min_epi32(_mm256_cvttps_epi32(prF), maxChar);
        __m256i pgI = _mm256_min_epi32(_mm256_cvttps_epi32(pgF), maxChar);
        __m256i pbI = _mm256_min_epi32(_mm256_cvttps_epi32(pbF), maxChar);

        // Pack back
        __m256i result = _mm256_or_si256(_mm256_slli_epi32(prI, 16),
                           _mm256_or_si256(_mm256_slli_epi32(pgI, 8), pbI));

        // Blend: only write active pixels
        result = _mm256_blendv_epi8(pxV, result, _mm256_castps_si256(activeMask));
        _mm256_storeu_si256((__m256i*)(pixels.data() + base), result);
      }
    }
#else
    for (int y = y0; y <= y1; ++y) {
      float dy = y - sy;
      float dySq = dy * dy;
      int rowOff = y * W;
      int zRowOff = (H - y) * W;
      for (int x = x0; x <= x1; ++x) {
        float dx = x - sx;
        float dSq = dx * dx + dySq;
        if (dSq >= rSq) continue;

        float t = 1.0f - dSq / rSq;
        float fog = t * t * t;

        int idx = rowOff + x;
        int pz = zbuff[zRowOff + x];
        if (pz > lightZ) continue;

        unsigned px = pixels[idx];
        unsigned pr = (px >> 16) & 0xFF;
        unsigned pg = (px >> 8) & 0xFF;
        unsigned pb = px & 0xFF;

        pr = fast_min(255, (int)(pr + fog * lR));
        pg = fast_min(255, (int)(pg + fog * lG));
        pb = fast_min(255, (int)(pb + fog * lB));

        pixels[idx] = (pr << 16) | (pg << 8) | pb;
      }
    }
#endif
  }
}
