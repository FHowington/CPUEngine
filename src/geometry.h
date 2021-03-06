//
// Created by Forbes Howington on 4/12/20.
//
#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <vector>

// Fast bitwise algorithms for finding max/min3
// Requires that highest bit not be used
inline __attribute__((always_inline)) int fast_max(int a, int b) {
  int diff = a - b;
  int dsgn = diff >> 31;
  return a - (diff & dsgn);
}

inline __attribute__((always_inline)) int fast_min(int a, int b) {
  int diff = a - b;
  int dsgn = diff >> 31;
  return b + (diff & dsgn);
}

// This defines a 4x4 matrix
template <unsigned N, unsigned M>
struct matrix {
  __attribute__((aligned(16))) std::array<float, N * M>_m{};

  static auto identity() -> matrix;
  static auto rotationX(float rotX) -> matrix;
  static auto rotationY(float rotY) -> matrix;
  static auto rotationZ(float rotZ) -> matrix;

  template <unsigned O>
  matrix<N,O> operator*(const matrix<M,O>& rhs) const;

  [[nodiscard]] float at(const unsigned i, const unsigned j) const { return _m[M * i + j]; }
  void set(const unsigned i, const unsigned j, const float val) { _m[M * i + j] = val; }

  matrix() {
    memset(_m.data(), 0, N * M * 4);
  }
};


struct fcolor {
  static const unsigned blank = 0xFFFFFF;
  static const unsigned duplicate = 0xFFAA55;
  union {
    struct {
      unsigned char b, g, r, a;
    };
    unsigned _color;
  };
  fcolor (uint8_t a, uint8_t r, uint8_t g, uint8_t b) : _color((a << 24) | (r << 16) | (g << 8) | b) {}
  fcolor (const unsigned color) { _color = color; }
  fcolor (const unsigned color, const float light) : _color(color) {
    r = fast_min(255, (int)((float)r * light));
    g = fast_min(255, (int)((float)g * light));
    b = fast_min(255, (int)((float)b * light));
  }
  fcolor (const unsigned color, const float R, const float G, const float B) : _color(color) {
    r = fast_min(255, (int)((float)r * R));
    g = fast_min(255, (int)((float)g * G));
    b = fast_min(255, (int)((float)b * B));
  }
  operator unsigned() const { return _color; }
};


template <typename T>
struct vertex {
  union {
    struct {
      T _x, _y, _z;
      float _e;
    };
    // This should ONLY be used if T is float
    float raw[4]; // NOLINT
  };


  vertex (const T x, const T y, const T z, const float e = 0) : _x(x), _y(y), _z(z), _e(e) {}
  vertex () : _x(), _y(), _z() {};

  // This should be switched to the fast approximation
  T norm() { return std::sqrt(_x*_x+_y*_y+_z*_z); }

  vertex<T>& normalize(T l = 1) {
    T lnorm = norm();
    lnorm = l/lnorm;
    this->_x *= lnorm;
    this->_y *= lnorm;
    this->_z *= lnorm;
    return *this;
  }
};


struct face {
  // Mapping to texture coords per vertex

  face(unsigned v0, unsigned v1, unsigned v2, const int t0x, const int t0y,
       const int t1x, const int t1y, const int t2x, const int t2y) :
      _v0(v0), _v1(v1), _v2(v2), _t0x(t0x), _t0y(t0y), _t1x(t1x), _t1y(t1y), _t2x(t2x), _t2y(t2y) {}

  face(unsigned v0, unsigned v1, unsigned v2) : _v0(v0), _v1(v1), _v2(v2) {}

  unsigned _v0;
  unsigned _v1;
  unsigned _v2;

  // Texture coordinates
  int _t0x;
  int _t0y;
  int _t1x;
  int _t1y;
  int _t2x;
  int _t2y;
};


template <typename T>
vertex<float> cross(const vertex<T>& v0, const vertex<T>& v1, const vertex<T>& v2) {
  vertex<float> l(v1._x - v0._x, v1._y - v0._y, v1._z - v0._z);
  vertex<float> r(v1._x - v2._x, v1._y - v2._y, v1._z - v2._z);
  vertex<float> v;
  v._x = (l._y * r._z) - (l._z * r._y);
  v._y = (l._z * r._x) - (l._x * r._z);
  v._z = (l._x * r._y) - (l._y * r._x);
  return v;
}

template<typename T>
T dot(const vertex<T>& l, const vertex<T>& r) {
  return l._x * r._x + l._y * r._y + l._z * r._z;
}

matrix<4,4> inverseNoTranslate(const matrix<4,4>& in);
matrix<4,4> invert(const matrix<4,4>& in);
vertex<float> multToVector(matrix<4,4> m, const vertex<float>& v);
matrix<4,1> v2m(const vertex<float>& v);
vertex<int> m2v(matrix<4,1> m);
vertex<float> m2vf(matrix<4,1> m);
int pipelineSlow(const matrix<4,4>& cameraTransform, const matrix<4,4>& model, const vertex<float>& v, vertex<float>& realResult, vertex<float>& camResult);
vertex<int> pipelineSlowPartTwo(vertex<float> cameraResult);
bool pipelineFast(const matrix<4,4>& cameraTransform, const matrix<4,4>& model, const vertex<float>& v, vertex<int>& retResult, vertex<float>& realResult);
vertex<float> rotateVector(matrix<4,4> m, const vertex<float>& v);
