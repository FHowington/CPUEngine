// Created by Forbes Howington 5/19/20
#pragma once
#include "geometry.h"
#ifdef __AVX__
#include <immintrin.h>
#endif
#include <cassert>
#include <list>

enum class LightType { Directional, Point };

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
      _type(lt), _R(R), _G(G), _B(B), _direction(direction) {
    _direction.normalize();
    assert(lt == LightType::Directional && "Only directional light may have a vector defined"); // NOLINT
  }

  Light (const LightType lt, const float x, const float y, const float z, const float strength, const float R, const float G, const float B) :
      _type(lt), _strength(strength), _R(R), _G(G), _B(B), _x(x), _y(y), _z(z) {
    assert(lt == LightType::Point && "Only point light may position without direction"); // NOLINT
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

illumination getLight(const vertex<float>& norm, float ambient, float x, float y, float z);

#if defined(__AVX2__) && defined(__FMA__)
void getLight(const __m256& xNorm, const __m256& yNorm, const __m256& zNorm, float ambient,
              const __m256& x, const __m256& y, const __m256& z,
              __m256& R, __m256& G, __m256& B);

inline __m256i vectorLight(const __m256i colorsData, const __m256& rV, const __m256& gV, const __m256& bV) {
  __m256 rBytes = _mm256_blendv_epi8(_mm256_setzero_si256(), colorsData,
                                     _mm256_set_epi8 (0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0));
    rBytes = _mm256_srli_epi32(rBytes, 16);
    rBytes = _mm256_cvtepi32_ps(rBytes);
    rBytes = _mm256_mul_ps(rV, rBytes);
    rBytes = _mm256_cvttps_epi32(rBytes);

    static const __m256i maxChar = _mm256_set1_epi32(255);
    __m256i mask = _mm256_cmpgt_epi32(rBytes, maxChar);
    rBytes = _mm256_blendv_ps(rBytes, maxChar, mask);
    rBytes = _mm256_slli_epi32(rBytes, 16);

    __m256 gBytes = _mm256_blendv_epi8(_mm256_setzero_si256(), colorsData,
                                       _mm256_set_epi8 (0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0));
    gBytes = _mm256_srli_epi32(gBytes, 8);
    gBytes = _mm256_cvtepi32_ps(gBytes);
    gBytes = _mm256_mul_ps(gV, gBytes);

    gBytes = _mm256_cvttps_epi32(gBytes);

    mask = _mm256_cmpgt_epi32(gBytes, maxChar);
    gBytes = _mm256_blendv_ps(gBytes, maxChar, mask);
    gBytes = _mm256_slli_epi32(gBytes, 8);

    __m256 bBytes = _mm256_blendv_epi8(_mm256_setzero_si256(), colorsData,
                                       _mm256_set_epi8 (0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF,0,0,0,0xFF));
    bBytes = _mm256_cvtepi32_ps(bBytes);
    bBytes = _mm256_mul_ps(bV, bBytes);

    bBytes = _mm256_cvttps_epi32(bBytes);

    mask = _mm256_cmpgt_epi32(bBytes, maxChar);
    bBytes = _mm256_blendv_ps(bBytes, maxChar, mask);

    return  _mm256_or_si256(bBytes, _mm256_or_si256(rBytes, gBytes));
}
#endif
