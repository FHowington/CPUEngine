#pragma once

#include "entity.h"
#include "light.h"
#include <array>
#include <cmath>
#include <list>

// Firefly — a glowing point light that orbits a center point with a fading trail.
// Each firefly maintains a short history of positions; older positions emit dimmer light.
class Firefly : public Entity {
 public:
  static constexpr unsigned TRAIL_LEN = 6;

  Firefly(float cx, float cy, float cz, float radius, float speed,
          float r, float g, float b, float strength, float phase = 0.0f)
      : _cx(cx), _cy(cy), _cz(cz), _radius(radius), _speed(speed),
        _r(r), _g(g), _b(b), _strength(strength), _angle(phase),
        _phaseY(phase * 1.7f) {
    // Initialize trail to starting position
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

    // Shift trail: oldest falls off, newest goes to front
    for (unsigned i = TRAIL_LEN - 1; i > 0; --i)
      _trail[i] = _trail[i - 1];
    _trail[0] = {x, y, z};
  }

  // Append this firefly's lights (head + trail) to the scene light list.
  void emitLights(std::list<Light>& lights) const {
    for (unsigned i = 0; i < TRAIL_LEN; ++i) {
      float fade = 1.0f - (float)i / TRAIL_LEN;  // 1.0 at head, ~0 at tail
      float s = _strength * fade * fade;           // quadratic falloff
      if (s < 0.5f) break;
      lights.emplace_back(LightType::Point,
                          _trail[i].x, _trail[i].y, _trail[i].z,
                          s, _r, _g, _b);
    }
  }

 private:
  float _cx, _cy, _cz;   // orbit center
  float _radius, _speed;
  float _r, _g, _b, _strength;
  float _angle, _phaseY;

  struct Pos { float x, y, z; };
  std::array<Pos, TRAIL_LEN> _trail;
};
