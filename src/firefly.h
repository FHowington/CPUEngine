#pragma once

#include "entity.h"
#include "geometry.h"
#include "simd_compat.h"
#include "Window.h"
#include <array>
#include <cmath>
#include <limits>

extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;
extern float focalLength;

// Firefly — a slow-drifting tiny light with a long sparkle trail.
// Renders as additive screen-space glow (no lighting calculations).
class Firefly : public Entity {
 public:
  static constexpr unsigned TRAIL_LEN = 20;

  Firefly(float cx, float cy, float cz, float radius, float speed,
          float r, float g, float b, float headIntensity, float headRadius,
          float phase = 0.0f)
      : _cx(cx), _cy(cy), _cz(cz), _radius(radius), _speed(speed),
        _r(r), _g(g), _b(b), _headIntensity(headIntensity), _headRadius(headRadius),
        _angle(phase), _phaseY(phase * 1.7f), _phaseR(phase * 0.6f) {
    float x = _cx + _radius * cosf(_angle);
    float z = _cz + _radius * sinf(_angle);
    float y = _cy + 0.3f * sinf(_phaseY);
    for (auto& p : _trail) p = {x, y, z};
  }

  void update(float dt) override {
    _angle  += _speed * dt;
    _phaseY += _speed * 0.6f * dt;
    _phaseR += _speed * 0.3f * dt;

    float r = _radius + 0.3f * sinf(_phaseR);
    float x = _cx + r * cosf(_angle);
    float z = _cz + r * sinf(_angle);
    float y = _cy + 0.3f * sinf(_phaseY);

    for (unsigned i = TRAIL_LEN - 1; i > 0; --i)
      _trail[i] = _trail[i - 1];
    _trail[0] = {x, y, z};
  }

  void draw(const matrix<4,4>& camTransform) const {
    for (unsigned i = 0; i < TRAIL_LEN; ++i) {
      float t = (float)i / TRAIL_LEN;
      float intensity, radius;
      if (i == 0) {
        intensity = _headIntensity;
        radius = _headRadius;
      } else {
        float fade = (1.0f - t);
        intensity = _headIntensity * 0.3f * fade * fade;
        radius = _headRadius * 0.3f;
      }
      if (intensity < 0.01f) break;
      splatGlow(camTransform, _trail[i].x, _trail[i].y, _trail[i].z,
                intensity, radius);
    }
  }

 private:
  float _cx, _cy, _cz;
  float _radius, _speed;
  float _r, _g, _b, _headIntensity, _headRadius;
  float _angle, _phaseY, _phaseR;

  struct Pos { float x, y, z; };
  std::array<Pos, TRAIL_LEN> _trail;

  // Project world position to screen; returns false if behind camera
  bool project(const matrix<4,4>& cam, float wx, float wy, float wz,
               float& sx, float& sy, float& cz) const {
    vertex<float> v(wx, wy, wz);
    matrix<4,1> m = cam * v2m(v);
    cz = m._m[2];
    if (cz >= -1.0f) return false;
    float scale = 1.0f / (-focalLength * cz);
    sx = m._m[0] * scale * rW * xZoom + rW * xFOV;
    sy = m._m[1] * scale * rH * yZoom + rH * yFOV;
    return true;
  }

  void splatGlow(const matrix<4,4>& cam, float wx, float wy, float wz,
                 float intensity, float glowR) const {
    float sx, sy, cz;
    if (!project(cam, wx, wy, wz, sx, sy, cz)) return;

    float scale = 1.0f / (-focalLength * cz);
    int ix = (int)sx;
    int iy = (int)(rH - 1 - sy);
    int fireflyZ = (int)(cz * depth);

    float screenR = glowR * scale * rH * yZoom;
    if (screenR < 1.0f) screenR = 1.0f;
    int rad = (int)screenR;

    int x0 = std::max(0, ix - rad);
    int x1 = std::min((int)rW - 1, ix + rad);
    int y0 = std::max(0, iy - rad);
    int y1 = std::min((int)rH - 1, iy + rad);
    if (x0 > x1 || y0 > y1) return;

    float screenR2 = screenR * screenR;
    float invR2 = 1.0f / screenR2;

#if defined(__AVX2__) && defined(__FMA__)
    splatGlowSIMD(ix, iy, fireflyZ, screenR2, invR2, intensity,
                  x0, x1, y0, y1);
#else
    splatGlowScalar(ix, iy, fireflyZ, screenR2, invR2, intensity,
                    x0, x1, y0, y1);
#endif
  }

#if defined(__AVX2__) && defined(__FMA__)
  void splatGlowSIMD(int ix, int iy, int fireflyZ, float screenR2,
                     float invR2, float intensity,
                     int x0, int x1, int y0, int y1) const {
    const __m256 vScreenR2 = _mm256_set1_ps(screenR2);
    const __m256 vInvR2 = _mm256_set1_ps(invR2);
    const __m256 vIntensity = _mm256_set1_ps(intensity);
    const __m256 vOne = _mm256_set1_ps(1.0f);
    const __m256 v255 = _mm256_set1_ps(255.0f);
    const __m256 vCR = _mm256_set1_ps(_r * 255.0f);
    const __m256 vCG = _mm256_set1_ps(_g * 255.0f);
    const __m256 vCB = _mm256_set1_ps(_b * 255.0f);
    const __m256i vFireflyZ = _mm256_set1_epi32(fireflyZ);
    const __m256i vEmpty = _mm256_set1_epi32(std::numeric_limits<int>::min());
    const __m256i maskFF = _mm256_set1_epi32(0xFF);
    const __m256 vIx = _mm256_set1_ps((float)ix);

    for (int py = y0; py <= y1; ++py) {
      float dy = (float)(py - iy);
      __m256 vDy2 = _mm256_set1_ps(dy * dy);
      unsigned* pixRow = pixels.data() + py * W;
      int* zRow = zbuff.data() + (int)(rH - 1 - py) * W;

      // Process 8 pixels at a time
      int px = x0;
      for (; px + 7 <= x1; px += 8) {
        __m256 vPx = _mm256_add_ps(_mm256_set1_ps((float)px),
                       _mm256_set_ps(7,6,5,4,3,2,1,0));
        __m256 vDx = _mm256_sub_ps(vPx, vIx);
        __m256 vD2 = _mm256_fmadd_ps(vDx, vDx, vDy2);

        // Radius mask: d2 < screenR2
        __m256 radiusMask = _mm256_cmp_ps(vD2, vScreenR2, _CMP_LT_OQ);

        // Depth test: skip pixels where geometry is closer
        __m256i zVals = _mm256_loadu_si256((__m256i*)(zRow + px));
        __m256i notEmpty = _mm256_xor_si256(
            _mm256_cmpeq_epi32(zVals, vEmpty), _mm256_set1_epi32(-1));
        __m256i geomCloser = _mm256_cmpgt_epi32(zVals, vFireflyZ);
        __m256i occluded = _mm256_and_si256(notEmpty, geomCloser);
        // Combine: visible = radiusMask AND NOT occluded
        __m256 depthMask = _mm256_castsi256_ps(
            _mm256_xor_si256(occluded, _mm256_set1_epi32(-1)));
        __m256 mask = _mm256_and_ps(radiusMask, depthMask);

        if (_mm256_testz_ps(mask, mask)) continue;

        // falloff = (1 - d2*invR2)^2
        __m256 falloff = _mm256_sub_ps(vOne, _mm256_mul_ps(vD2, vInvR2));
        falloff = _mm256_mul_ps(falloff, falloff);
        __m256 a = _mm256_mul_ps(vIntensity, falloff);

        // Load existing pixels
        __m256i pix = _mm256_loadu_si256((__m256i*)(pixRow + px));
        __m256 pR = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(pix, 16), maskFF));
        __m256 pG = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(pix, 8), maskFF));
        __m256 pB = _mm256_cvtepi32_ps(_mm256_and_si256(pix, maskFF));

        // Additive blend
        __m256 rOut = _mm256_min_ps(v255, _mm256_fmadd_ps(vCR, a, pR));
        __m256 gOut = _mm256_min_ps(v255, _mm256_fmadd_ps(vCG, a, pG));
        __m256 bOut = _mm256_min_ps(v255, _mm256_fmadd_ps(vCB, a, pB));

        __m256i result = _mm256_or_si256(
            _mm256_slli_epi32(_mm256_cvttps_epi32(rOut), 16),
            _mm256_or_si256(
                _mm256_slli_epi32(_mm256_cvttps_epi32(gOut), 8),
                _mm256_cvttps_epi32(bOut)));

        // Blend: keep original pixel where masked out
        result = _mm256_blendv_epi8(pix, result, _mm256_castps_si256(mask));
        _mm256_storeu_si256((__m256i*)(pixRow + px), result);
      }

      // Scalar tail
      for (; px <= x1; ++px) {
        scalarPixel(px, py, ix, iy, fireflyZ, screenR2, invR2, intensity,
                    pixRow, zRow);
      }
    }
  }
#endif

  void splatGlowScalar(int ix, int iy, int fireflyZ, float screenR2,
                       float invR2, float intensity,
                       int x0, int x1, int y0, int y1) const {
    for (int py = y0; py <= y1; ++py) {
      unsigned* pixRow = pixels.data() + py * W;
      int* zRow = zbuff.data() + (int)(rH - 1 - py) * W;
      for (int px = x0; px <= x1; ++px) {
        scalarPixel(px, py, ix, iy, fireflyZ, screenR2, invR2, intensity,
                    pixRow, zRow);
      }
    }
  }

  inline void scalarPixel(int px, int py, int ix, int iy, int fireflyZ,
                          float screenR2, float invR2, float intensity,
                          unsigned* pixRow, int* zRow) const {
    int zVal = zRow[px];
    if (zVal != std::numeric_limits<int>::min() && zVal > fireflyZ) return;

    float dx = (float)(px - ix), dy = (float)(py - iy);
    float d2 = dx * dx + dy * dy;
    if (d2 >= screenR2) return;

    float falloff = 1.0f - d2 * invR2;
    falloff *= falloff;
    float a = intensity * falloff;

    unsigned& p = pixRow[px];
    unsigned pr = (p >> 16) & 0xFF;
    unsigned pg = (p >> 8) & 0xFF;
    unsigned pb = p & 0xFF;
    pr = std::min(255u, pr + (unsigned)(_r * 255.0f * a));
    pg = std::min(255u, pg + (unsigned)(_g * 255.0f * a));
    pb = std::min(255u, pb + (unsigned)(_b * 255.0f * a));
    p = (pr << 16) | (pg << 8) | pb;
  }
};
