#include "geometry.h"
#include "light.h"

std::list<Light> Light::sceneLights;

illumination getLight(const vertex<float>& norm, const float ambient, const float x, const float y, const float z) {
  float R = 0;
  float G = 0;
  float B = 0;
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
        float d = dot(vertex<float>(l._x - x, l._y - y, l._z - z).normalize(), norm);
        if (d > 0) {
          // Falls off according to inverse square law
          auto dist = (float)(pow(l._x - x, 2) +  pow(l._y - y, 2) +  pow(l._z - z, 2));
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

#if defined(__AVX2__) && defined(__FMA__)
void getLight(const __m256& xNorm, const __m256& yNorm, const __m256& zNorm, float ambient,
              const __m256& x, const __m256& y, const __m256& z,
              __m256& R, __m256& G, __m256& B) {

  R = _mm256_setzero_si256();
  G = _mm256_setzero_si256();
  B = _mm256_setzero_si256();

  for (const Light& l : Light::sceneLights) {
    switch (l._type) {
      case LightType::Directional: {
        __m256 dot = _mm256_mul_ps(xNorm, _mm256_set1_ps(-l._direction._x));
        dot = _mm256_fmsub_ps(yNorm, _mm256_set1_ps(l._direction._y), dot);
        dot = _mm256_fmsub_ps(zNorm, _mm256_set1_ps(l._direction._z), dot);

        __m256 mask = _mm256_cmpgt_epi32(_mm256_setzero_si256(), dot);
        dot = _mm256_blendv_ps(dot, _mm256_setzero_si256(), mask);

        R = _mm256_fmadd_ps(dot, _mm256_set1_ps(l._R), R);
        G = _mm256_fmadd_ps(dot, _mm256_set1_ps(l._G), G);
        B = _mm256_fmadd_ps(dot, _mm256_set1_ps(l._B), B);
        break;
      }
      case LightType::Point: {
        // Need to calculate xNorm, yNorm, zNorm.
        __m256 lXNorm = _mm256_sub_ps(_mm256_set1_ps(l._x), x);
        __m256 lYNorm = _mm256_sub_ps(_mm256_set1_ps(l._y), y);
        __m256 lZNorm = _mm256_sub_ps(_mm256_set1_ps(l._z), z);

        __m256 dist = _mm256_mul_ps(lXNorm, lXNorm);
        dist = _mm256_fmadd_ps(lYNorm, lYNorm, dist);
        dist = _mm256_fmadd_ps(lZNorm, lZNorm, dist);

        const __m256 recipRoot = _mm256_rsqrt_ps(dist);
        lXNorm = _mm256_mul_ps(lXNorm, recipRoot);
        lYNorm = _mm256_mul_ps(lYNorm, recipRoot);
        lZNorm = _mm256_mul_ps(lZNorm, recipRoot);

        __m256 dot = _mm256_mul_ps(lXNorm, xNorm);
        dot = _mm256_fmadd_ps(lYNorm, yNorm, dot);
        dot = _mm256_fmadd_ps(lZNorm, zNorm, dot);

        dot = _mm256_div_ps(dot, dist);
        dot = _mm256_mul_ps(dot, _mm256_set1_ps(l._strength));

        const __m256 mask = _mm256_cmpgt_epi32(_mm256_setzero_si256(), dot);
        dot = _mm256_blendv_ps(dot, _mm256_setzero_si256(), mask);

        R = _mm256_fmadd_ps(dot, _mm256_set1_ps(l._R), R);
        G = _mm256_fmadd_ps(dot, _mm256_set1_ps(l._G), G);
        B = _mm256_fmadd_ps(dot, _mm256_set1_ps(l._B), B);
        break;
      }
    }
  }

  const __m256 ambientV =_mm256_set1_ps(ambient);
  __m256 mask = _mm256_cmpgt_epi32(ambientV, R);
  mask = _mm256_or_si256(_mm256_cmpgt_epi32(ambientV, G), mask);
  mask = _mm256_or_si256(_mm256_cmpgt_epi32(ambientV, B), mask);
  R = _mm256_blendv_ps(R, ambientV, mask);
  G = _mm256_blendv_ps(G, ambientV, mask);
  B = _mm256_blendv_ps(B, ambientV, mask);
}
#endif
