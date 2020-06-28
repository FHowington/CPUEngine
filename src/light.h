// Created by Forbes Howington 5/19/20
#pragma once
#include <list>
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

  static std::list<Light> sceneLights;

  LightType _type;
  float _strength;
  float _R;
  float _G;
  float _B;
  const matrix<4,4> _orientation;
  const vertex<float> _direction; // Only used for directional light
};

const illumination getLight(const vertex<float>& norm, const float ambient, const float x, const float y, const float z);
