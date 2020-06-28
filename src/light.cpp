#include "geometry.h"
#include "light.h"

std::list<Light> Light::sceneLights;

const illumination getLight(const vertex<float>& norm, const float ambient, const float x, const float y, const float z) {
  float R = 0, G = 0, B = 0;
  for (const Light& l : Light::sceneLights) {
    switch (l._type) {
      case LightType::Directional:
        float d = -dot(l._direction, norm);
        if (d > 0) {
          R += d * l._R;
          G += d * l._G;
          B += d * l._B;
        }
        break;
    }
  }
  if (R < ambient || G < ambient || B < ambient) {
    return {ambient, ambient, ambient};
  }

  return {R, G, B};
}
