//
// Created by Forbes Howington on 3/28/20.
//

#pragma once

#include <fstream>
#include <iostream>
#include <cmath>
#include <memory>
#include <sstream>
#include <vector>
#include "Window.h"

struct fcolor {
  static const unsigned blank = 0xFFFFFF;
  static const unsigned duplicate = 0xFFAA55;
  union {
    struct {
      unsigned char b, g, r, a;
    };
    unsigned char raw[4];
    unsigned _color;
  };
  fcolor (uint8_t a, uint8_t r, uint8_t g, uint8_t b) : _color((a << 24) | (r << 16) | (g << 8) | b) {}
  fcolor (const unsigned color) { _color = color; }
  operator unsigned() const { return _color; }
};

template <typename T>
struct vertex {
  T _x;
  T _y;
  T _z;

  vertex (const T x, const T y, const T z) : _x(x), _y(y), _z(z) {}
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
  //Mapping to texture coords per vertex

  face(vertex<float> &v0, vertex<float>& v1, vertex<float>& v2, const unsigned t0x, const unsigned t0y,
       const unsigned t1x, const unsigned t1y, const unsigned t2x, const unsigned t2y) :
      _v0(v0), _v1(v1), _v2(v2), _t0x(t0x), _t0y(t0y), _t1x(t1x), _t1y(t1y), _t2x(t2x), _t2y(t2y) {}

  face(vertex<float>& v0, vertex<float>& v1, vertex<float>& v2) : face(v0, v1, v2, 0, 0, 0, 0, 0, 0) {}

  vertex<float>& _v0;
  vertex<float>& _v1;
  vertex<float>& _v2;

  int _t0x;
  int _t1x;
  int _t2x;
  int _t0y;
  int _t1y;
  int _t2y;
};

class Model {
 public:
  Model(const std::string& fileName, const unsigned width, const unsigned height) { loadModel(fileName, width, height); }

  const std::vector<face>& getFaces() const { return faces; }

 private:
  std::vector<vertex<float>> vertices;
  std::vector<face> faces;

  void loadModel(const std::string& fileName, const unsigned width, const unsigned height);
};

template <typename T>
vertex<T> cross(const vertex<T>& v0, const vertex<T>& v1, const vertex<T>& v2) {
  vertex<T> l(v1._x - v0._x, v1._y - v0._y, v1._z - v0._z);
  vertex<T> r(v1._x - v2._x, v1._y - v2._y, v1._z - v2._z);
  vertex<T> v;
  v._x = (l._y * r._z) - (l._z * r._y);
  v._y = (l._z * r._x) - (l._x * r._z);
  v._z = (l._x * r._y) - (l._y * r._x);
  return v;
}

template<typename T>
T dot(const vertex<T>& l, const vertex<T>& r) {
  return l._x * r._x + l._y * r._y + l._z * r._z;
}
