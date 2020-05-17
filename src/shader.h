//
// Created by Forbes Howington on 5/9/20.
//

#pragma once

#include "geometry.h"
#include <immintrin.h>
#include "loader.h"

// We do this instead of the typical inheritance route because dynamic binding of function calls
// incurs a perf. hit for shaders. They are simply called too many time.

// Textured shaders take advantage of looking up textures before they are needed using vectorization
// This is a class defining a shader.
// vertexShader called on all vertices per triangle
// Traditional implementation would pass in barycentric coordinates to
// the fragment shader. However, this is much slower than just using deltas for interpolation
// so instead, we will guarantee that stepX and stepY are called for respective
// steps, and depend on implementation to update their values appropriately

enum class shaderType { FlatShader, GouraudShader, InterpFlatShader, InterpGouraudShader };

class Shader {
 public:
  virtual ~Shader() {};

  virtual const fcolor fragmentShader(const unsigned color = 0) = 0;

#ifdef __AVX2__
  virtual const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData) { return; }
#endif

  virtual void stepXForX(const unsigned step) = 0;
  virtual void stepYForX(const unsigned step) = 0;
};

class TexturedShader : public Shader {
};

class UntexturedShader : public Shader {
};
// NOTE: Using this flat shader causes a serious perf. impact versus the naive
// approach of just doing these calculations inline
// The difference is due to the cost of calling a virtual function MANY times
// So instead, we will NOT call the functions via a Shader ref or pointer
// The type of shader will be a compile time CONSTANT!
class FlatShader : public TexturedShader {
 public:
  FlatShader(const ModelInstance& m, const face& f, const vertex<float>& light, const short A12, const short A20, const short A01,
                    const short B12, const short B20, const short B01, const float wTotal, int w0, int w1, int w2) {
    const vertex<float> v0iLight(multToVector(m._position, m._baseModel.getVertex(f._v0)));
    const vertex<float> v1iLight(multToVector(m._position, m._baseModel.getVertex(f._v1)));
    const vertex<float> v2iLight(multToVector(m._position, m._baseModel.getVertex(f._v2)));
    vertex<float> vLight = cross(v0iLight, v1iLight, v2iLight);
    vLight.normalize();
    _light = dot(vLight, light);

    // Effectively, this is the global illumination
    if (_light < 0.2) {
      _light = 0.2;
    };
  }

  const inline __attribute__((always_inline)) fcolor fragmentShader(const unsigned color = 0) override {
    return fcolor(color, _light);
  }

#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData) override {
    unsigned __attribute__((aligned(32))) colorTemp[8];
    const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);

    _mm256_stream_si256((__m256i *)(colorTemp), colorsData);

    // TODO: Figure out to do this all in registers
    for (unsigned idx = 0; idx < 8; ++idx) {
      colorTemp[idx] = ((int)(((colorTemp[idx] >> 16) & 0xff) * _light)) << 16 |
                       ((int)(((colorTemp[idx] >> 8) & 0xff) * _light)) << 8 |
                       (int)(((colorTemp[idx]) & 0xff) * _light);
    }

    colorsData = _mm256_load_si256((__m256i*)(colorTemp));

  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 0) override { return; }
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override { return; }

 private:
  float _light;
};


class GouraudShader : public TexturedShader {
 public:
  GouraudShader(const ModelInstance& m, const face& f, const vertex<float>& light, const short A12, const short A20, const short A01,
                    const short B12, const short B20, const short B01, const float wTotal, int w0, int w1, int w2) {
    // Change in the texture coordinated for x/y, used for interpolation
    const vertex<float> v0iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v0)));
    const vertex<float> v1iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v1)));
    const vertex<float> v2iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v2)));

    float light0 = -dot(light, v0iNorm);
    if (light0 < 0.2) {
      light0 = 0.2;
    }
    float light1 = -dot(light, v1iNorm);
    if (light1 < 0.2) {
      light1 = 0.2;
    }
    float light2 = -dot(light, v2iNorm);
    if (light2 < 0.2) {
      light2 = 0.2;
    }

    _lDx = (light0 * A12 + light1 * A20 + light2 * A01) / wTotal;
    _lDy = (light0 * B12 + light1 * B20 + light2 * B01) / wTotal;

    _light = (light0 * w0 + light1 * w1 + light2 * w2) / wTotal;
    _lightRow = _light;
  }

  const inline __attribute__((always_inline)) fcolor fragmentShader(const unsigned color = 0) override {
    return fcolor(color, _light);
  }

#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData) override {

    unsigned __attribute__((aligned(32))) colorTemp[8];
    float __attribute__((aligned(32))) lightTemp[8];
    const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);
    const __m256i lightAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_lDx));
    const __m256i lightRow = _mm256_add_ps(_mm256_set1_ps(_light), lightAdd);

    _mm256_stream_si256((__m256i *)(colorTemp), colorsData);
    _mm256_stream_si256((__m256i *)(lightTemp), lightRow);

    // TODO: Figure out to do this all in registers
    for (unsigned idx = 0; idx < 8; ++idx) {
      colorTemp[idx] = ((int)(((colorTemp[idx] >> 16) & 0xff) * lightTemp[idx])) << 16 |
                       ((int)(((colorTemp[idx] >> 8) & 0xff) * lightTemp[idx])) << 8 |
                       (int)(((colorTemp[idx]) & 0xff) * lightTemp[idx]);

    }

    _light += _lDx * 8;
    colorsData = _mm256_load_si256((__m256i*)(colorTemp));
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
    _light += _lDx * step;
  }

  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 1) override
  {
    _lightRow += _lDy * step;
    _light = _lightRow;
  }

 private:
  float _lDx;
  float _lDy;
  float _light;
  float _lightRow;
};

class InterpFlatShader : public UntexturedShader {
 public:
  InterpFlatShader(const ModelInstance& m, const face& f, const vertex<float>& light, const short A12, const short A20, const short A01,
                        const short B12, const short B20, const short B01, const float wTotal, int w0, int w1, int w2) {
    const vertex<float> v0iLight(multToVector(m._position, m._baseModel.getVertex(f._v0)));
    const vertex<float> v1iLight(multToVector(m._position, m._baseModel.getVertex(f._v1)));
    const vertex<float> v2iLight(multToVector(m._position, m._baseModel.getVertex(f._v2)));
    vertex<float> vLight = cross(v0iLight, v1iLight, v2iLight);
    vLight.normalize();
    float _light = dot(vLight, light);

    // Effectively, this is the global illumination
    if (_light < 0.2) {
      _light = 0.2;
    };

    const unsigned col0 = m._texture->fast_get(f._t0x, f._t0y);
    const unsigned col1 = m._texture->fast_get(f._t1x, f._t1y);
    const unsigned col2 = m._texture->fast_get(f._t2x, f._t2y);

    const float R0 = ((col0 >> 16) & 0xff) * _light;
    const float G0 = ((col0 >> 8) & 0xff) * _light;
    const float B0 = (col0 & 0xff) * _light;

    const float R1 = ((col1 >> 16) & 0xff) * _light;
    const float G1 = ((col1 >> 8) & 0xff) * _light;
    const float B1 = (col1 & 0xff) * _light;

    const float R2 = ((col2 >> 16) & 0xff) * _light;
    const float G2 = ((col2 >> 8) & 0xff) * _light;
    const float B2 = (col2 & 0xff) * _light;

    _Rdx = (R0 * A12 + R1 * A20 + R2 * A01) / wTotal;
    _Rdy = (R0 * B12 + R1 * B20 + R2 * B01) / wTotal;

    _Gdx = (G0 * A12 + G1 * A20 + G2 * A01) / wTotal;
    _Gdy = (G0 * B12 + G1 * B20 + G2 * B01) / wTotal;

    _Bdx = (B0 * A12 + B1 * A20 + B2 * A01) / wTotal;
    _Bdy = (B0 * B12 + B1 * B20 + B2 * B01) / wTotal;

    _R = (R0 * w0 + R1 * w1 + R2 * w2) / wTotal;
    _G = (G0 * w0 + G1 * w1 + G2 * w2) / wTotal;
    _B = (B0 * w0 + B1 * w1 + B2 * w2) / wTotal;
    _rowR = _R;
    _rowG = _G;
    _rowB = _B;
  }

  const inline __attribute__((always_inline)) fcolor fragmentShader(const unsigned color = 0) override {
    return fcolor(0, _R, _G, _B);
  }


#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData) override {
    unsigned __attribute__((aligned(32))) colorTemp[8];
    for (unsigned idx = 0; idx < 8; ++idx) {
      colorTemp[idx] = (int)_R << 16 |
                       (int)_G << 8 |
                       (int)_B;
          _R += _Rdx;
          _G += _Gdx;
          _B += _Bdx;
    }
    colorsData = _mm256_load_si256((__m256i*)(colorTemp));
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
    _R += _Rdx * step;
    _G += _Gdx * step;
    _B += _Bdx * step;
  }
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {
    _rowR += _Rdy;
    _rowG += _Gdy;
    _rowB += _Bdy;

    _R = _rowR;
    _G = _rowG;
    _B = _rowB;
  }

 private:
  float _R;
  float _G;
  float _B;
  float _Rdx;
  float _Rdy;
  float _Gdx;
  float _Gdy;
  float _Bdx;
  float _Bdy;

  float _rowR;
  float _rowG;
  float _rowB;
};


class InterpGouraudShader : public UntexturedShader {
 public:
  InterpGouraudShader(const ModelInstance& m, const face& f, const vertex<float>& light, const short A12, const short A20, const short A01,
                        const short B12, const short B20, const short B01, const float wTotal, int w0, int w1, int w2) {

    const vertex<float> v0iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v0)));
    const vertex<float> v1iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v1)));
    const vertex<float> v2iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v2)));

    float light0 = -dot(light, v0iNorm);
    if (light0 < 0.2) {
      light0 = 0.2;
    }
    float light1 = -dot(light, v1iNorm);
    if (light1 < 0.2) {
      light1 = 0.2;
    }
    float light2 = -dot(light, v2iNorm);
    if (light2 < 0.2) {
      light2 = 0.2;
    }

    const unsigned col0 = m._texture->fast_get(f._t0x, f._t0y);
    const unsigned col1 = m._texture->fast_get(f._t1x, f._t1y);
    const unsigned col2 = m._texture->fast_get(f._t2x, f._t2y);

    const float R0 = ((col0 >> 16) & 0xff) * light0;
    const float G0 = ((col0 >> 8) & 0xff) * light0;
    const float B0 = (col0 & 0xff) * light0;

    const float R1 = ((col1 >> 16) & 0xff) * light1;
    const float G1 = ((col1 >> 8) & 0xff) * light1;
    const float B1 = (col1 & 0xff) * light1;

    const float R2 = ((col2 >> 16) & 0xff) * light2;
    const float G2 = ((col2 >> 8) & 0xff) * light2;
    const float B2 = (col2 & 0xff) * light2;

    _Rdx = (R0 * A12 + R1 * A20 + R2 * A01) / wTotal;
    _Rdy = (R0 * B12 + R1 * B20 + R2 * B01) / wTotal;

    _Gdx = (G0 * A12 + G1 * A20 + G2 * A01) / wTotal;
    _Gdy = (G0 * B12 + G1 * B20 + G2 * B01) / wTotal;

    _Bdx = (B0 * A12 + B1 * A20 + B2 * A01) / wTotal;
    _Bdy = (B0 * B12 + B1 * B20 + B2 * B01) / wTotal;

    _R = (R0 * w0 + R1 * w1 + R2 * w2) / wTotal;
    _G = (G0 * w0 + G1 * w1 + G2 * w2) / wTotal;
    _B = (B0 * w0 + B1 * w1 + B2 * w2) / wTotal;
    _rowR = _R;
    _rowG = _G;
    _rowB = _B;
  }

  const inline __attribute__((always_inline)) fcolor fragmentShader(const unsigned color = 0) override {
    return fcolor(0, _R, _G, _B);
  }

#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData) override {
    unsigned __attribute__((aligned(32))) colorTemp[8];
    for (unsigned idx = 0; idx < 8; ++idx) {
      colorTemp[idx] = (int)_R << 16 |
                       (int)_G << 8 |
                       (int)_B;
          _R += _Rdx;
          _G += _Gdx;
          _B += _Bdx;
    }
    colorsData = _mm256_load_si256((__m256i*)(colorTemp));
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
    _R += _Rdx * step;
    _G += _Gdx * step;
    _B += _Bdx * step;
  }

  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {
    _rowR += _Rdy;
    _rowG += _Gdy;
    _rowB += _Bdy;

    _R = _rowR;
    _G = _rowG;
    _B = _rowB;
  }

 private:
  float _R;
  float _G;
  float _B;
  float _Rdx;
  float _Rdy;
  float _Gdx;
  float _Gdy;
  float _Bdx;
  float _Bdy;

  float _rowR;
  float _rowG;
  float _rowB;
};
