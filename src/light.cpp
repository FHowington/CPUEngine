#include "geometry.h"
#include "light.h"

std::list<Light> Light::sceneLights;

const illumination getLight(const vertex<float>& norm, const float ambient, const float x, const float y, const float z) {
  float R = 0, G = 0, B = 0;
  for (const Light& l : Light::sceneLights) {
    switch (l._type) {
      case LightType::Directional: {
        float d = -dot(l._direction, norm);
        if (d > 0) {
          R += d * l._R;
          G += d * l._G;
          B += d * l._B;
        }
        break;
      }

      case LightType::Point: {
        float d = dot(vertex<float>(l._x - x, l._y - y, l._z - z), norm);
        if (d > 0) {
          // Falls off according to inverse square law
          float dist = pow(l._x - x, 2) +  pow(l._y - y, 2) +  pow(l._z - z, 2);
          d /= dist;
          d *= l._strength;
          R += d * l._R;
          G += d * l._G;
          B += d * l._B;
        }
        break;
      }
    }
  }
  if (R < ambient || G < ambient || B < ambient) {
    return {ambient, ambient, ambient};
  }

  return {R, G, B};
}
