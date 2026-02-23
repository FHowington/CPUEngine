#pragma once

#include "entity.h"
#include "geometry.h"
#include "Window.h"
#include <array>
#include <cmath>

extern std::array<unsigned, W * H> pixels;
extern float focalLength;

// Firefly — a glowing point that orbits a center with a fading trail.
// Renders as an additive screen-space glow (no lighting calculations).
class Firefly : public Entity {
 public:
  static constexpr unsigned TRAIL_LEN = 6;

  Firefly(float cx, float cy, float cz, float radius, float speed,
          float r, float g, float b, float strength, float glowRadius,
          float phase = 0.0f)
      : _cx(cx), _cy(cy), _cz(cz), _radius(radius), _speed(speed),
        _r(r), _g(g), _b(b), _strength(strength), _glowRadius(glowRadius),
        _angle(phase), _phaseY(phase * 1.7f) {
    float x = _cx + _radius * cosf(_angle);
    float z = _cz + _radius * sinf(_angle);
    float y = _cy + 0.5f * sinf(_phaseY);
    for (auto& p : _trail) p = {x, y, z};
  }

  void update(float dt) override {
    _angle += _speed * dt;
    _phaseY += _speed * 0.7f * dt;

    float x = _cx + _radius * cosf(_angle);
    float z = _cz + _radius * sinf(_angle);
    float y = _cy + 0.5f * sinf(_phaseY);

    for (unsigned i = TRAIL_LEN - 1; i > 0; --i)
      _trail[i] = _trail[i - 1];
    _trail[0] = {x, y, z};
  }

  // Splat additive glow into the pixel buffer for each trail point.
  // Must be called while pixels are in rW×rH space (before upscale).
  void draw(const matrix<4,4>& camTransform) const {
    for (unsigned i = 0; i < TRAIL_LEN; ++i) {
      float fade = 1.0f - (float)i / TRAIL_LEN;
      float intensity = _strength * fade * fade;
      if (intensity < 0.02f) break;
      splatGlow(camTransform, _trail[i].x, _trail[i].y, _trail[i].z,
                intensity, _glowRadius * fade);
    }
  }

 private:
  float _cx, _cy, _cz;
  float _radius, _speed;
  float _r, _g, _b, _strength, _glowRadius;
  float _angle, _phaseY;

  struct Pos { float x, y, z; };
  std::array<Pos, TRAIL_LEN> _trail;

  void splatGlow(const matrix<4,4>& cam, float wx, float wy, float wz,
                 float intensity, float radius) const {
    // World → camera space
    vertex<float> v(wx, wy, wz);
    matrix<4,1> m = cam * v2m(v);
    float cz = m._m[2];
    if (cz >= -1.0f) return;  // behind camera

    // Camera → screen (same math as pipelineSlowPartTwo)
    float scale = 1.0f / (-focalLength * cz);
    float sx = m._m[0] * scale * rW * xZoom + rW * xFOV;
    float sy = m._m[1] * scale * rH * yZoom + rH * yFOV;

    // Screen-space glow radius scales with perspective
    float screenR = radius * scale * rH * yZoom;
    if (screenR < 1.0f) screenR = 1.0f;

    int r = (int)screenR;
    int ix = (int)sx, iy = (int)sy;

    // Y is flipped in the pixel buffer
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
      for (int px = x0; px <= x1; ++px) {
        float dx = (float)(px - ix);
        float d2 = dx * dx + dy * dy;
        if (d2 >= screenR * screenR) continue;

        float falloff = 1.0f - d2 * invR2;
        falloff *= falloff;  // squared for softer edge
        float a = intensity * falloff;

        // Additive blend
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
