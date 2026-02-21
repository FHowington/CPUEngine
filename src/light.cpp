#include "geometry.h"
#include "light.h"
#include <algorithm>
#include <cmath>

std::list<Light> Light::sceneLights;
vertex<float> cameraPos;
bool specularEnabled = false;
float specularShininess = 32.0f;

illumination getLight(const vertex<float>& norm, const float ambient, const float x, const float y, const float z) {
  float R = 0;
  float G = 0;
  float B = 0;

  // View direction for specular (computed once per fragment)
  vertex<float> viewDir;
  if (specularEnabled) {
    viewDir = vertex<float>(cameraPos._x - x, cameraPos._y - y, cameraPos._z - z);
    viewDir.normalize();
  }

  for (const Light& l : Light::sceneLights) {
    switch (l._type) {
      case LightType::Directional: {
        float d = -dot(l._direction, norm);
        if (d > 0) {
          R += d * l._R;
          G += d * l._G;
          B += d * l._B;

          if (specularEnabled) {
            // reflect = 2 * dot(N, L) * N - L, where L = -direction
            float dn2 = 2.0f * d;
            vertex<float> refl(dn2 * norm._x + l._direction._x,
                               dn2 * norm._y + l._direction._y,
                               dn2 * norm._z + l._direction._z);
            float spec = dot(refl, viewDir);
            if (spec > 0) {
              spec = powf(spec, specularShininess);
              R += spec * l._R;
              G += spec * l._G;
              B += spec * l._B;
            }
          }
        }
        break;
      }

      case LightType::Point: {
        vertex<float> toLight(l._x - x, l._y - y, l._z - z);
        auto dist = (float)(toLight._x * toLight._x + toLight._y * toLight._y + toLight._z * toLight._z);
        toLight.normalize();
        float d = dot(toLight, norm);
        if (d > 0) {
          float atten = d * l._strength / dist;
          R += atten * l._R;
          G += atten * l._G;
          B += atten * l._B;

          if (specularEnabled) {
            float dn2 = 2.0f * d;
            vertex<float> refl(dn2 * norm._x - toLight._x,
                               dn2 * norm._y - toLight._y,
                               dn2 * norm._z - toLight._z);
            float spec = dot(refl, viewDir);
            if (spec > 0) {
              spec = powf(spec, specularShininess) * l._strength / dist;
              R += spec * l._R;
              G += spec * l._G;
              B += spec * l._B;
            }
          }
        }
        break;
      }
    }
  }
  R = std::max(ambient, R);
  G = std::max(ambient, G);
  B = std::max(ambient, B);

  return {R, G, B};
}

#if defined(__AVX2__) && defined(__FMA__)

// Fast approximate pow for specular: spec^n via repeated squaring
// Works well for typical shininess values (16, 32, 64)
static inline __m256 fastPow(__m256 base, float exponent) {
  // Use exp2(exponent * log2(base)) approximation
  // log2(x) ≈ exponent bits trick, but simpler: just do repeated squaring for power-of-2
  // For arbitrary exponent, fall back to scalar
  // For common case (32), this is 5 multiplies
  int n = (int)exponent;
  __m256 result = _mm256_set1_ps(1.0f);
  __m256 cur = base;
  while (n > 0) {
    if (n & 1) result = _mm256_mul_ps(result, cur);
    cur = _mm256_mul_ps(cur, cur);
    n >>= 1;
  }
  return result;
}

void getLight(const __m256& xNorm, const __m256& yNorm, const __m256& zNorm, float ambient,
              const __m256& x, const __m256& y, const __m256& z,
              __m256& R, __m256& G, __m256& B) {

  R = _mm256_setzero_si256();
  G = _mm256_setzero_si256();
  B = _mm256_setzero_si256();

  // View direction for specular
  __m256 vdX, vdY, vdZ;
  if (specularEnabled) {
    vdX = _mm256_sub_ps(_mm256_set1_ps(cameraPos._x), x);
    vdY = _mm256_sub_ps(_mm256_set1_ps(cameraPos._y), y);
    vdZ = _mm256_sub_ps(_mm256_set1_ps(cameraPos._z), z);
    __m256 vdLen = _mm256_rsqrt_ps(_mm256_fmadd_ps(vdZ, vdZ, _mm256_fmadd_ps(vdY, vdY, _mm256_mul_ps(vdX, vdX))));
    vdX = _mm256_mul_ps(vdX, vdLen);
    vdY = _mm256_mul_ps(vdY, vdLen);
    vdZ = _mm256_mul_ps(vdZ, vdLen);
  }

  const __m256 zero = _mm256_setzero_ps();

  for (const Light& l : Light::sceneLights) {
    switch (l._type) {
      case LightType::Directional: {
        __m256 d = _mm256_mul_ps(xNorm, _mm256_set1_ps(-l._direction._x));
        d = _mm256_fmsub_ps(yNorm, _mm256_set1_ps(l._direction._y), d);
        d = _mm256_fmsub_ps(zNorm, _mm256_set1_ps(l._direction._z), d);

        __m256 mask = _mm256_cmpgt_epi32(_mm256_setzero_si256(), d);
        d = _mm256_blendv_ps(d, zero, mask);

        R = _mm256_fmadd_ps(d, _mm256_set1_ps(l._R), R);
        G = _mm256_fmadd_ps(d, _mm256_set1_ps(l._G), G);
        B = _mm256_fmadd_ps(d, _mm256_set1_ps(l._B), B);

        if (specularEnabled) {
          // reflect = 2*dot(N,L)*N - L where L = -direction
          __m256 dn2 = _mm256_add_ps(d, d);
          __m256 rX = _mm256_fmadd_ps(dn2, xNorm, _mm256_set1_ps(l._direction._x));
          __m256 rY = _mm256_fmadd_ps(dn2, yNorm, _mm256_set1_ps(l._direction._y));
          __m256 rZ = _mm256_fmadd_ps(dn2, zNorm, _mm256_set1_ps(l._direction._z));
          __m256 spec = _mm256_fmadd_ps(rZ, vdZ, _mm256_fmadd_ps(rY, vdY, _mm256_mul_ps(rX, vdX)));
          spec = _mm256_max_ps(spec, zero);
          spec = fastPow(spec, specularShininess);
          R = _mm256_fmadd_ps(spec, _mm256_set1_ps(l._R), R);
          G = _mm256_fmadd_ps(spec, _mm256_set1_ps(l._G), G);
          B = _mm256_fmadd_ps(spec, _mm256_set1_ps(l._B), B);
        }
        break;
      }
      case LightType::Point: {
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

        __m256 d = _mm256_mul_ps(lXNorm, xNorm);
        d = _mm256_fmadd_ps(lYNorm, yNorm, d);
        d = _mm256_fmadd_ps(lZNorm, zNorm, d);

        // Clamp negative to zero (one-sided lighting)
        const __m256 mask = _mm256_cmpgt_epi32(_mm256_setzero_si256(), d);
        d = _mm256_blendv_ps(d, zero, mask);

        __m256 atten = _mm256_div_ps(d, dist);
        atten = _mm256_mul_ps(atten, _mm256_set1_ps(l._strength));

        R = _mm256_fmadd_ps(atten, _mm256_set1_ps(l._R), R);
        G = _mm256_fmadd_ps(atten, _mm256_set1_ps(l._G), G);
        B = _mm256_fmadd_ps(atten, _mm256_set1_ps(l._B), B);

        if (specularEnabled) {
          __m256 dn2 = _mm256_add_ps(d, d);
          __m256 rX = _mm256_fmsub_ps(dn2, xNorm, lXNorm);
          __m256 rY = _mm256_fmsub_ps(dn2, yNorm, lYNorm);
          __m256 rZ = _mm256_fmsub_ps(dn2, zNorm, lZNorm);
          __m256 spec = _mm256_fmadd_ps(rZ, vdZ, _mm256_fmadd_ps(rY, vdY, _mm256_mul_ps(rX, vdX)));
          spec = _mm256_max_ps(spec, zero);
          spec = fastPow(spec, specularShininess);
          __m256 specAtten = _mm256_mul_ps(spec, _mm256_set1_ps(l._strength));
          specAtten = _mm256_div_ps(specAtten, dist);
          R = _mm256_fmadd_ps(specAtten, _mm256_set1_ps(l._R), R);
          G = _mm256_fmadd_ps(specAtten, _mm256_set1_ps(l._G), G);
          B = _mm256_fmadd_ps(specAtten, _mm256_set1_ps(l._B), B);
        }
        break;
      }
    }
  }

  const __m256 ambientV =_mm256_set1_ps(ambient);
  R = _mm256_max_ps(R, ambientV);
  G = _mm256_max_ps(G, ambientV);
  B = _mm256_max_ps(B, ambientV);
}
#endif
