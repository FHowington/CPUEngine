//
// Created by Forbes Howington on 5/9/20.
//

#pragma once

#include "geometry.h"
#include <immintrin.h>
#include "loader.h"
#include "light.h"

// We do this instead of the typical inheritance route because dynamic binding of function calls
// incurs a perf. hit for shaders. They are simply called too many time.

// Textured shaders take advantage of looking up textures before they are needed using vectorization
// This is a class defining a shader.
// vertexShader called on all vertices per triangle
// Traditional implementation would pass in barycentric coordinates to
// the fragment shader. However, this is much slower than just using deltas for interpolation
// so instead, we will guarantee that stepX and stepY are called for respective
// steps, and depend on implementation to update their values appropriately

enum class shaderType { FlatShader, GouraudShader, InterpFlatShader, InterpGouraudShader, PlaneShader };

class Shader {
 public:
  virtual ~Shader() {};

  virtual const fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) = 0;

#ifdef __AVX2__
  virtual const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& x, const __m256i& y, const __m256i& z) { return; }
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
  FlatShader(const ModelInstance& m, const face& f,
             const short A12, const short A20, const short A01,
             const short B12, const short B20, const short B01,
             const float wTotal, int w0, int w1, int w2,
             const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination) {

    const vertex<float> v0iLight(multToVector(m._position, m._baseModel.getVertex(f._v0)));
    const vertex<float> v1iLight(multToVector(m._position, m._baseModel.getVertex(f._v1)));
    const vertex<float> v2iLight(multToVector(m._position, m._baseModel.getVertex(f._v2)));
    _norm = cross(v2iLight, v1iLight, v0iLight);
    _norm.normalize();
  }

  const inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    illumination il = getLight(_norm, _luminance, x, y, z);
    return fcolor(color, il._R, il._G, il._B);
  }

#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& x, const __m256i& y, const __m256i& z) override {
    __m256 rV;
    __m256 gV;
    __m256 bV;

    getLight(_mm256_set1_ps(_norm._x), _mm256_set1_ps(_norm._y),
             _mm256_set1_ps(_norm._z), _luminance,
             x, y, z, rV, gV, bV);


    colorsData = vectorLight(colorsData, rV, gV, bV);
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 0) override { return; }
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override { return; }

 private:
  vertex<float> _norm;
  const float _luminance;
};


class GouraudShader : public TexturedShader {
 public:
  GouraudShader(const ModelInstance& m, const face& f,
                const short A12, const short A20, const short A01,
                const short B12, const short B20, const short B01,
                const float wTotal, int w0, int w1, int w2,
                const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination) {

    // Change in the texture coordinated for x/y, used for interpolation
    const vertex<float> v0iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v0)));
    const vertex<float> v1iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v1)));
    const vertex<float> v2iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v2)));

    _aDxX = (v0iNorm._x * A12 + v1iNorm._x * A20 + v2iNorm._x * A01) / wTotal;
    _aDxY = (v0iNorm._y * A12 + v1iNorm._y * A20 + v2iNorm._y * A01) / wTotal;
    _aDxZ = (v0iNorm._z * A12 + v1iNorm._z * A20 + v2iNorm._z * A01) / wTotal;
    _aDyX = (v0iNorm._x * B12 + v1iNorm._x * B20 + v2iNorm._x * B01) / wTotal;
    _aDyY = (v0iNorm._y * B12 + v1iNorm._y * B20 + v2iNorm._y * B01) / wTotal;
    _aDyZ = (v0iNorm._z * B12 + v1iNorm._z * B20 + v2iNorm._z * B01) / wTotal;

    _angleX = (v0iNorm._x * w0 + v1iNorm._x * w1 + v2iNorm._x * w2) / wTotal;
    _angleY = (v0iNorm._y * w0 + v1iNorm._y * w1 + v2iNorm._y * w2) / wTotal;
    _angleZ = (v0iNorm._z * w0 + v1iNorm._z * w1 + v2iNorm._z * w2) / wTotal;
    _angleRowX = _angleX;
    _angleRowY = _angleY;
    _angleRowZ = _angleZ;
  }

  const inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    illumination il = getLight(vertex<float>(_angleX, _angleY, _angleZ), _luminance, x, y, z);
    return fcolor(color, il._R, il._G, il._B);
  }

#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& x, const __m256i& y, const __m256i& z) override {
    static const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);
    const __m256i angleAddX = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_aDxX));
    const __m256i angleRowX = _mm256_add_ps(_mm256_set1_ps(_angleX), angleAddX);

    const __m256i angleAddY = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_aDxY));
    const __m256i angleRowY = _mm256_add_ps(_mm256_set1_ps(_angleY), angleAddY);

    const __m256i angleAddZ = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_aDxZ));
    const __m256i angleRowZ = _mm256_add_ps(_mm256_set1_ps(_angleZ), angleAddZ);

    __m256 rV;
    __m256 gV;
    __m256 bV;

    getLight(angleRowX, angleRowY,
             angleRowZ, _luminance,
             x, y, z, rV, gV, bV);


    colorsData = vectorLight(colorsData, rV, gV, bV);

    _angleX += _aDxX * 8;
    _angleY += _aDxY * 8;
    _angleZ += _aDxZ * 8;
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
    _angleX += _aDxX * step;
    _angleY += _aDxY * step;
    _angleZ += _aDxZ * step;
  }

  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 1) override
  {
    _angleRowX += _aDyX * step;
    _angleX = _angleRowX;
    _angleRowY += _aDyY * step;
    _angleY = _angleRowY;
    _angleRowZ += _aDyZ * step;
    _angleZ = _angleRowZ;
  }

 private:
  float _aDxX;
  float _aDxY;
  float _aDxZ;
  float _aDyX;
  float _aDyY;
  float _aDyZ;
  float _angleX;
  float _angleY;
  float _angleZ;
  float _angleRowX;
  float _angleRowY;
  float _angleRowZ;
  const float _luminance;
};

class InterpGouraudShader : public UntexturedShader {
 public:
  InterpGouraudShader(const ModelInstance& m, const face& f,
                   const short A12, const short A20, const short A01,
                   const short B12, const short B20, const short B01,
                   const float wTotal, int w0, int w1, int w2,
                   const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination) {

    // Change in the texture coordinated for x/y, used for interpolation
    const vertex<float> v0iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v0)));
    const vertex<float> v1iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v1)));
    const vertex<float> v2iNorm = (rotateVector(m._position, m._baseModel.getVertexNormal(f._v2)));

    _aDxX = (v0iNorm._x * A12 + v1iNorm._x * A20 + v2iNorm._x * A01) / wTotal;
    _aDxY = (v0iNorm._y * A12 + v1iNorm._y * A20 + v2iNorm._y * A01) / wTotal;
    _aDxZ = (v0iNorm._z * A12 + v1iNorm._z * A20 + v2iNorm._z * A01) / wTotal;
    _aDyX = (v0iNorm._x * B12 + v1iNorm._x * B20 + v2iNorm._x * B01) / wTotal;
    _aDyY = (v0iNorm._y * B12 + v1iNorm._y * B20 + v2iNorm._y * B01) / wTotal;
    _aDyZ = (v0iNorm._z * B12 + v1iNorm._z * B20 + v2iNorm._z * B01) / wTotal;

    _angleX = (v0iNorm._x * w0 + v1iNorm._x * w1 + v2iNorm._x * w2) / wTotal;
    _angleY = (v0iNorm._y * w0 + v1iNorm._y * w1 + v2iNorm._y * w2) / wTotal;
    _angleZ = (v0iNorm._z * w0 + v1iNorm._z * w1 + v2iNorm._z * w2) / wTotal;
    _angleRowX = _angleX;
    _angleRowY = _angleY;
    _angleRowZ = _angleZ;

    const unsigned col0 = m._texture->fast_get(f._t0x, f._t0y);
    const unsigned col1 = m._texture->fast_get(f._t1x, f._t1y);
    const unsigned col2 = m._texture->fast_get(f._t2x, f._t2y);

    const float R0 = (col0 >> 16) & 0xff;
    const float G0 = (col0 >> 8) & 0xff;
    const float B0 = col0 & 0xff;

    const float R1 = (col1 >> 16) & 0xff;
    const float G1 = (col1 >> 8) & 0xff;
    const float B1 = col1 & 0xff;

    const float R2 = (col2 >> 16) & 0xff;
    const float G2 = (col2 >> 8) & 0xff;
    const float B2 = col2 & 0xff;

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

  const inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    illumination il = getLight(vertex<float>(_angleX, _angleY, _angleZ), _luminance, x, y, z);
    return fcolor(((unsigned)_R << 16) | ((unsigned)_G << 8) | (unsigned)_B, il._R, il._G, il._B);
  }


#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& x, const __m256i& y, const __m256i& z) override {
    static const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);

    const __m256i angleAddX = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_aDxX));
    const __m256i angleRowX = _mm256_add_ps(_mm256_set1_ps(_angleX), angleAddX);

    const __m256i angleAddY = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_aDxY));
    const __m256i angleRowY = _mm256_add_ps(_mm256_set1_ps(_angleY), angleAddY);

    const __m256i angleAddZ = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_aDxZ));
    const __m256i angleRowZ = _mm256_add_ps(_mm256_set1_ps(_angleZ), angleAddZ);

    const __m256i rAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_Rdx));
    __m256i rRow = _mm256_add_ps(_mm256_set1_ps(_R), rAdd);
    rRow = _mm256_cvttps_epi32(rRow);
    rRow = _mm256_slli_epi32(rRow, 16);

    const __m256i gAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_Gdx));
    __m256i gRow = _mm256_add_ps(_mm256_set1_ps(_G), gAdd);
    gRow = _mm256_cvttps_epi32(gRow);
    gRow = _mm256_slli_epi32(gRow, 8);

    const __m256i bAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_Bdx));
    __m256i bRow = _mm256_add_ps(_mm256_set1_ps(_B), bAdd);
    bRow = _mm256_cvttps_epi32(bRow);

    __m256 rV;
    __m256 gV;
    __m256 bV;

    getLight(angleRowX, angleRowY,
             angleRowZ, _luminance,
             x, y, z, rV, gV, bV);

    colorsData = _mm256_or_si256(rRow, _mm256_or_si256(gRow, bRow));
    colorsData = vectorLight(colorsData, rV, gV, bV);

    _angleX += _aDxX * 8;
    _angleY += _aDxY * 8;
    _angleZ += _aDxZ * 8;
    _R += _Rdx * 8;
    _G += _Gdx * 8;
    _B += _Bdx * 8;
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
    _R += _Rdx * step;
    _G += _Gdx * step;
    _B += _Bdx * step;

    _angleX += _aDxX * step;
    _angleY += _aDxY * step;
    _angleZ += _aDxZ * step;
  }
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {
    _rowR += _Rdy;
    _rowG += _Gdy;
    _rowB += _Bdy;

    _R = _rowR;
    _G = _rowG;
    _B = _rowB;

    _angleRowX += _aDyX * step;
    _angleX = _angleRowX;
    _angleRowY += _aDyY * step;
    _angleY = _angleRowY;
    _angleRowZ += _aDyZ * step;
    _angleZ = _angleRowZ;
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

  float _aDxX;
  float _aDxY;
  float _aDxZ;
  float _aDyX;
  float _aDyY;
  float _aDyZ;
  float _angleX;
  float _angleY;
  float _angleZ;
  float _angleRowX;
  float _angleRowY;
  float _angleRowZ;
  const float _luminance;
};


class InterpFlatShader : public UntexturedShader {
 public:
  InterpFlatShader(const ModelInstance& m, const face& f,
                   const short A12, const short A20, const short A01,
                   const short B12, const short B20, const short B01,
                   const float wTotal, int w0, int w1, int w2,
                   const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination) {

    const vertex<float> v0iLight(multToVector(m._position, m._baseModel.getVertex(f._v0)));
    const vertex<float> v1iLight(multToVector(m._position, m._baseModel.getVertex(f._v1)));
    const vertex<float> v2iLight(multToVector(m._position, m._baseModel.getVertex(f._v2)));
    _norm = cross(v2iLight, v1iLight, v0iLight);
    _norm.normalize();

    // Change in the texture coordinated for x/y, used for interpolation
    const unsigned col0 = m._texture->fast_get(f._t0x, f._t0y);
    const unsigned col1 = m._texture->fast_get(f._t1x, f._t1y);
    const unsigned col2 = m._texture->fast_get(f._t2x, f._t2y);

    const float R0 = (col0 >> 16) & 0xff;
    const float G0 = (col0 >> 8) & 0xff;
    const float B0 = col0 & 0xff;

    const float R1 = (col1 >> 16) & 0xff;
    const float G1 = (col1 >> 8) & 0xff;
    const float B1 = col1 & 0xff;

    const float R2 = (col2 >> 16) & 0xff;
    const float G2 = (col2 >> 8) & 0xff;
    const float B2 = col2 & 0xff;

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

  const inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    illumination il = getLight(_norm, _luminance, x, y, z);
    return fcolor(((unsigned)_R << 16) | ((unsigned)_G << 8) | (unsigned)_B, il._R, il._G, il._B);
  }


#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& x, const __m256i& y, const __m256i& z) override {
    static const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);
    const __m256i rAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_Rdx));
    __m256i rRow = _mm256_add_ps(_mm256_set1_ps(_R), rAdd);
    rRow = _mm256_cvttps_epi32(rRow);
    rRow = _mm256_slli_epi32(rRow, 16);

    const __m256i gAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_Gdx));
    __m256i gRow = _mm256_add_ps(_mm256_set1_ps(_G), gAdd);
    gRow = _mm256_cvttps_epi32(gRow);
    gRow = _mm256_slli_epi32(gRow, 8);

    const __m256i bAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_Bdx));
    __m256i bRow = _mm256_add_ps(_mm256_set1_ps(_B), bAdd);
    bRow = _mm256_cvttps_epi32(bRow);

    __m256 rV;
    __m256 gV;
    __m256 bV;

    getLight(_mm256_set1_ps(_norm._x), _mm256_set1_ps(_norm._y),
             _mm256_set1_ps(_norm._z), _luminance,
             x, y, z, rV, gV, bV);

    colorsData = _mm256_or_si256(rRow, _mm256_or_si256(gRow, bRow));
    colorsData = vectorLight(colorsData, rV, gV, bV);

    _R += _Rdx * 8;
    _G += _Gdx * 8;
    _B += _Bdx * 8;
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

  vertex<float> _norm;
  const float _luminance;
};


// The goal of this texture is to create a grid pattern based on the global x/z coords of each pixel
class PlaneShader : public UntexturedShader {
 public:
  PlaneShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }


  const inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    unsigned res = (((unsigned)floor(x)) & 0x1) ? 123456 : 4321;
    illumination il = getLight(_norm, _luminance, x, y, z);

    res = fast_min(255, ((int)(((res >> 16) & 0xff) * il._R))) << 16 |
                       fast_min(255, ((int)(((res >> 8) & 0xff) * il._G))) << 8 |
                       fast_min(255, (int)(((res) & 0xff) * il._B));
    return res;
  }

#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    __m256i xVals = xVals = _mm256_round_ps (xV, (_MM_FROUND_TO_NEG_INF |_MM_FROUND_NO_EXC) );
    xVals = _mm256_cvtps_epi32 (xVals);

    __m256i oddMask = _mm256_set1_epi32(1);

    xVals = _mm256_and_si256(oddMask, xVals);
    xVals = _mm256_cmpeq_epi32(xVals, _mm256_setzero_si256());

    __m256i blue = _mm256_set1_epi32(4321);
    __m256i green = _mm256_set1_epi32(123456);
    colorsData = _mm256_blendv_ps(blue, green, xVals);

    __m256 rV;
    __m256 gV;
    __m256 bV;

    getLight(_mm256_set1_ps(_norm._x), _mm256_set1_ps(_norm._y),
             _mm256_set1_ps(_norm._z), _luminance,
             xV, yV, zV, rV, gV, bV);


    colorsData = vectorLight(colorsData, rV, gV, bV);
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
  }

  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {
  }

 private:
  const float _luminance;
  const vertex<float>& _norm;
};
