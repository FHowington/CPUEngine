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
  virtual void vertexShader(const Model& m, const face& f, float light) = 0;
  virtual const fcolor fragmentShader(const unsigned color) = 0;
  virtual void stepX() = 0;
  virtual void stepY() = 0;
};


// NOTE: Using this default shader causes a serious perf. impact versus the naive
// approach of just doing these calculations inline
// The difference is due to the cost of calling a virtual function MANY times
// So instead, we will NOT call the functions via a Shader ref or pointer
// The type of shader will be a compile time CONSTANT!
class DefaultShader : public Shader {
 public:
  void vertexShader(const Model& m, const face& f, float light) override {
    _light = light;
  }
  const inline __attribute__((always_inline)) fcolor fragmentShader(const unsigned color) override {
    return fcolor(color, _light);
  }
  DefaultShader() : _light(0) {}
  inline __attribute__((always_inline)) void stepX() override { return; }
  inline __attribute__((always_inline)) void stepY() override { return; }
 private:
  float _light;
};
