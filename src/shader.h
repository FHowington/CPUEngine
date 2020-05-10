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
  virtual void vertexShader(const ModelInstance& m, const face& f, const vertex<float>& light) = 0;
  virtual const fcolor fragmentShader(const unsigned color) = 0;
  virtual void stepXForX() = 0;
  virtual void stepYForX() = 0;
  virtual void stepXForY() = 0;
  virtual void stepYForY() = 0;
};


// NOTE: Using this default shader causes a serious perf. impact versus the naive
// approach of just doing these calculations inline
// The difference is due to the cost of calling a virtual function MANY times
// So instead, we will NOT call the functions via a Shader ref or pointer
// The type of shader will be a compile time CONSTANT!
class DefaultShader : public Shader {
 public:
  void vertexShader(const ModelInstance& m, const face& f, const vertex<float>& light) override {
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
  inline __attribute__((always_inline)) void stepXForX() override { return; }
  inline __attribute__((always_inline)) void stepYForX() override { return; }
  inline __attribute__((always_inline)) void stepXForY() override { return; }
  inline __attribute__((always_inline)) void stepYForY() override { return; }

 private:
  float _light;
};


class GourandShader : public Shader {
 public:
  GourandShader()  {}

  void vertexShader(const ModelInstance& m, const face& f, const vertex<float>& light) override {
  }
  const inline __attribute__((always_inline)) fcolor fragmentShader(const unsigned color) override {
    return fcolor(color, _light);
  }
  inline __attribute__((always_inline)) void stepXForX() override { _normX += _xDx; _normY += _yDx; _normZ += _zDx; }

  inline __attribute__((always_inline)) void stepYForX() override
  {
    _rowX += _xDy;
    _rowY += _yDy;
    _rowZ += _zDy;
    _normX = _rowX;
    _normY = _rowY;
    _normZ = _rowZ;
  }

  inline __attribute__((always_inline)) void stepXForY() override {
    _rowX += _xDx;
    _rowY += _yDx;
    _rowZ += _zDx;
    _normX = _colX;
    _normY = _colY;
    _normZ = _colZ;
  }

  inline __attribute__((always_inline)) void stepYForY() override { _normX += _xDy; _normY += _yDy; _normZ += _zDy; }

 private:
  float _light;
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

  float _colX;
  float _colY;
  float _colZ;
};
