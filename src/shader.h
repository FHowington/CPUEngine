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


// NOTE: Using this flat shader causes a serious perf. impact versus the naive
// approach of just doing these calculations inline
// The difference is due to the cost of calling a virtual function MANY times
// So instead, we will NOT call the functions via a Shader ref or pointer
// The type of shader will be a compile time CONSTANT!
class FlatShader : public Shader {
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

  FlatShader() : _light(0) {}
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 0) override { return; }
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override { return; }

 private:
  float _light;
};


class GouraudShader : public Shader {
 public:
  GouraudShader()  {}

  void vertexShader(const ModelInstance& m, const face& f, const vertex<float>& light, const short A12, const short A20, const short A01,
                    const short B12, const short B20, const short B01, const float wTotal, int w0, int w1, int w2) override {
    // Change in the texture coordinated for x/y, used for interpolation
    // OMG I'm a fool, instead just interpolate the intensity
    vertex<float> v0iNorm = (rotateVector(m.position, m.baseModel.getVertexNormal(f._v0)));
    vertex<float> v1iNorm = (rotateVector(m.position, m.baseModel.getVertexNormal(f._v1)));
    vertex<float> v2iNorm = (rotateVector(m.position, m.baseModel.getVertexNormal(f._v2)));

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
  const inline __attribute__((always_inline)) fcolor fragmentShader(const unsigned color) override {
    return fcolor(color, _light);
  }

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
