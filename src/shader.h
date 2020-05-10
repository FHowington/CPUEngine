//
// Created by Forbes Howington on 5/9/20.
//

#pragma once

#include "geometry.h"
#include "loader.h"

// This is a class defining a shader.
// vertexShader called on all vertices per triangle
// Traditional implementation would pass in barycentric coordinates to
// the fragment shader. However, this is much slower than just using deltas for interpolation
// so instead, we will guarantee that stepX and stepY are called for respective
// steps, and depend on implementation to update their values appropriately

class Shader {
 public:
  virtual ~Shader() {};
  virtual void vertexShader(const ModelInstance& m, const face& f, const vertex<float>& light, const short A12, const short A20, const short A01,
                            const short B12, const short B20, const short B01, const float wTotal, int w0, int w1, int w2) = 0;
  virtual const fcolor fragmentShader(const unsigned color) = 0;
  virtual void stepXForX(const unsigned step) = 0;
  virtual void stepYForX(const unsigned step) = 0;
};


// NOTE: Using this default shader causes a serious perf. impact versus the naive
// approach of just doing these calculations inline
// The difference is due to the cost of calling a virtual function MANY times
// So instead, we will NOT call the functions via a Shader ref or pointer
// The type of shader will be a compile time CONSTANT!
class DefaultShader : public Shader {
 public:
  void vertexShader(const ModelInstance& m, const face& f, const vertex<float>& light, const short A12, const short A20, const short A01,
                    const short B12, const short B20, const short B01, const float wTotal, int w0, int w1, int w2) override {
    const vertex<float> v0iLight(multToVector(m.position, m.baseModel.getVertex(f._v0)));
    const vertex<float> v1iLight(multToVector(m.position, m.baseModel.getVertex(f._v1)));
    const vertex<float> v2iLight(multToVector(m.position, m.baseModel.getVertex(f._v2)));
    vertex<float> vLight = cross(v0iLight, v1iLight, v2iLight);
    vLight.normalize();
    _light = dot(vLight, light);

    // Effectively, this is the global illumination
    if (_light < 0.2) {
      _light = 0.2;
    };
  }

  const inline __attribute__((always_inline)) fcolor fragmentShader(const unsigned color) override {
    return fcolor(color, _light);
  }

  DefaultShader() : _light(0) {}
  inline __attribute__((always_inline)) void stepXForX(const unsigned step) override { return; }
  inline __attribute__((always_inline)) void stepYForX(const unsigned step) override { return; }

 private:
  float _light;
};


class GourandShader : public Shader {
 public:
  GourandShader()  {}

  void vertexShader(const ModelInstance& m, const face& f, const vertex<float>& light, const short A12, const short A20, const short A01,
                    const short B12, const short B20, const short B01, const float wTotal, int w0, int w1, int w2) override {
    // Change in the texture coordinated for x/y, used for interpolation
    v0iNorm = (rotateVector(m.position, m.baseModel.getVertexNormal(f._v0)));
    v1iNorm = (rotateVector(m.position, m.baseModel.getVertexNormal(f._v1)));
    v2iNorm = (rotateVector(m.position, m.baseModel.getVertexNormal(f._v2)));

    _xDx = (v0iNorm._x * A12 + v1iNorm._x * A20 + v2iNorm._x * A01) / wTotal;
    _yDx = (v0iNorm._y * A12 + v1iNorm._y * A20 + v2iNorm._y * A01) / wTotal;
    _zDx = (v0iNorm._z * A12 + v1iNorm._z * A20 + v2iNorm._z * A01) / wTotal;

    _xDy = (v0iNorm._x * B12 + v1iNorm._x * B20 + v2iNorm._x * B01) / wTotal;
    _yDy = (v0iNorm._y * B12 + v1iNorm._y * B20 + v2iNorm._y * B01) / wTotal;
    _zDy = (v0iNorm._z * B12 + v1iNorm._z * B20 + v2iNorm._z * B01) / wTotal;

    _normX = (v0iNorm._x * w0 + v1iNorm._x * w1 + v2iNorm._x * w2) / wTotal;
    _normY = (v0iNorm._y * w0 + v1iNorm._y * w1 + v2iNorm._y * w2) / wTotal;
    _normZ = (v0iNorm._z * w0 + v1iNorm._z * w1 + v2iNorm._z * w2) / wTotal;
    _rowX = _normX;
    _rowY = _normY;
    _rowZ = _normZ;

    _light = light;
  }
  const inline __attribute__((always_inline)) fcolor fragmentShader(const unsigned color) override {
    vertex<float> normalV(_normX, _normY, _normZ);

    float res = -dot(_light, normalV);
    if (res < 0.2) {
      res = 0.2;
    }
    return fcolor(color, res);
  }

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
    _normX += _xDx * step; _normY += _yDx * step; _normZ += _zDx * step;
  }

  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 1) override
  {
    _rowX += _xDy * step;
    _rowY += _yDy * step;
    _rowZ += _zDy * step;
    _normX = _rowX;
    _normY = _rowY;
    _normZ = _rowZ;
  }

 private:
  float _normX;
  float _normY;
  float _normZ;

  float _xDx;
  float _yDx;
  float _zDx;

  float _xDy;
  float _yDy;
  float _zDy;

  float _rowX;
  float _rowY;
  float _rowZ;

  vertex<float> _light;
  vertex<float> v0iNorm;
  vertex<float> v1iNorm;
  vertex<float> v2iNorm;
};
