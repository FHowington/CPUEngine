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

struct vertex {
  float _x;
  float _y;
  float _z;

  vertex (const float x, const float y, const float z) : _x(x), _y(y), _z(z) {}
  vertex () : _x(), _y(), _z() {};

  // This should be switched to the fast approximation
  float norm() { return std::sqrt(_x*_x+_y*_y+_z*_z); }

  vertex& normalize(float l = 1);
};

struct face {
  //Mapping to texture coords per vertex

  face(vertex &v0, vertex& v1, vertex& v2, const unsigned t0x, const unsigned t0y,
       const unsigned t1x, const unsigned t1y, const unsigned t2x, const unsigned t2y) :
      _v0(v0), _v1(v1), _v2(v2), _t0x(t0x), _t0y(t0y), _t1x(t1x), _t1y(t1y), _t2x(t2x), _t2y(t2y) {}

  face(vertex &v0, vertex& v1, vertex& v2) : face(v0, v1, v2, 0, 0, 0, 0, 0, 0) {}

  vertex& _v0;
  vertex& _v1;
  vertex& _v2;

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
  std::vector<vertex> vertices;
  std::vector<face> faces;

  void loadModel(const std::string& fileName, const unsigned width, const unsigned height);
};

vertex cross(const vertex& v0, const vertex& v1, const vertex& v2);

// Remember: this is vectors, not vertices!
float dot(const vertex& l, const vertex& r);
