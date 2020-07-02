// Created by Forbes Howington 5/19/20
#pragma once
#include <list>
#include "immintrin.h"

enum class LightType { Directional, Point, Spot };

struct illumination {
  float _R;
  float _G;
  float _B;
};

class Light {
 public:
  Light (const LightType lt, const float strength, const float R, const float G, const float B) :
      _type(lt), _strength(strength), _R(R), _G(G), _B(B) {}

  Light (const LightType lt, const vertex<float> direction, const float R, const float G, const float B) :
      _type(lt), _direction(direction), _R(R), _G(G), _B(B) {
    assert(lt == LightType::Directional && "Only directional light may have a vector defined");
  }

  Light (const LightType lt, const float x, const float y, const float z, const float strength, const float R, const float G, const float B) :
      _type(lt), _x(x), _y(y), _z(z), _strength(strength), _R(R), _G(G), _B(B) {
    assert(lt == LightType::Point && "Only point light may position without direction");
  }

  static std::list<Light> sceneLights;

  LightType _type;
  float _strength;
  float _R;
  float _G;
  float _B;
  // Needed for point lights, may be removed in the future.
  float _x;
  float _y;
  float _z;
  // Needed for shadow stencils
  matrix<4,4> _orientation;
  vertex<float> _direction; // Only used for directional light
};

const illumination getLight(const vertex<float>& norm, const float ambient, const float x, const float y, const float z);

#ifdef __AVX2__
void getLight(const __m256& xNorm, const __m256& yNorm, const __m256& zNorm, float ambient,
              const __m256& x, const __m256& y, const __m256& z,
              __m256& R, __m256& G, __m256& B);
#endif
