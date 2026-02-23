#pragma once

#include "entity.h"
#include "geometry.h"
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

    // Gentle wandering orbit with slight radius variation
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
      // Head: bright small glow. Trail: tiny dim sparkles.
      float intensity, radius;
      if (i == 0) {
        intensity = _headIntensity;
        radius = _headRadius;
      } else {
        float fade = (1.0f - t);
        intensity = _headIntensity * 0.3f * fade * fade;
        radius = _headRadius * 0.3f;  // tiny sparkle dots
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

  void splatGlow(const matrix<4,4>& cam, float wx, float wy, float wz,
                 float intensity, float glowR) const {
    vertex<float> v(wx, wy, wz);
    matrix<4,1> m = cam * v2m(v);
    float cz = m._m[2];
    if (cz >= -1.0f) return;

    float scale = 1.0f / (-focalLength * cz);
    float sx = m._m[0] * scale * rW * xZoom + rW * xFOV;
    float sy = m._m[1] * scale * rH * yZoom + rH * yFOV;

    // Flip Y: projection has Y-up, pixel buffer has Y=0 at top
    int ix = (int)sx;
    int iy = (int)(rH - 1 - sy);

    // Depth value comparable to zbuffer (camZ * depth, negative)
    int fireflyZ = (int)(cz * depth);

    float screenR = glowR * scale * rH * yZoom;
    if (screenR < 1.0f) screenR = 1.0f;
    int r = (int)screenR;

    int x0 = std::max(0, ix - r);
    int x1 = std::min((int)rW - 1, ix + r);
    int y0 = std::max(0, iy - r);
    int y1 = std::min((int)rH - 1, iy + r);

    float invR2 = 1.0f / (screenR * screenR);
    unsigned cr = (unsigned)(_r * 255.0f);
    unsigned cg = (unsigned)(_g * 255.0f);
    unsigned cb = (unsigned)(_b * 255.0f);

    for (int py = y0; py <= y1; ++py) {
      float dy = (float)(py - iy);
      int zRow = (int)(rH - 1 - py) * W;
      for (int px = x0; px <= x1; ++px) {
        // Depth test: skip if geometry is closer than the firefly
        int zVal = zbuff[zRow + px];
        if (zVal != std::numeric_limits<int>::min() && zVal > fireflyZ)
          continue;

        float dx = (float)(px - ix);
        float d2 = dx * dx + dy * dy;
        if (d2 >= screenR * screenR) continue;

        float falloff = 1.0f - d2 * invR2;
        falloff *= falloff;
        float a = intensity * falloff;

        unsigned& p = pixels[py * W + px];
        unsigned pr = (p >> 16) & 0xFF;
        unsigned pg = (p >> 8) & 0xFF;
        unsigned pb = p & 0xFF;
        pr = std::min(255u, pr + (unsigned)(cr * a));
        pg = std::min(255u, pg + (unsigned)(cg * a));
        pb = std::min(255u, pb + (unsigned)(cb * a));
        p = (pr << 16) | (pg << 8) | pb;
      }
    }
  }
};
