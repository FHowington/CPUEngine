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
  unsigned _color;
  fcolor (uint8_t r, uint8_t g, uint8_t b, uint8_t a) : _color((r << 24) | (g << 16) | (b << 8) | a) {}
  operator unsigned() const { return _color; }
};

struct vertex {
  float _x;
  float _y;
  float _z;

  vertex (const float x, const float y, const float z) : _x(x), _y(y), _z(z) {}
  vertex () : _x(), _y(), _z() {};

  float norm() { return std::sqrt(_x*_x+_y*_y+_z*_z); }
  vertex& normalize(float l = 1) {
    float lnorm = norm();
    lnorm = l/lnorm;
    this->_x *= lnorm;
    this->_y *= lnorm;
    this->_z *= lnorm;
    return *this;
  }
};

struct face {
  //Mapping to texture coords per vertex
  face(vertex &v0, vertex& v1, vertex& v2) : _v0(v0), _v1(v1), _v2(v2) {}
  vertex& _v0;
  vertex& _v1;
  vertex& _v2;
};

class Model {
 public:
  Model(const std::string& fileName) {
    loadModel(fileName);
  }

  const std::vector<face>& getFaces() const {
    return faces;
  }

 private:
  std::vector<vertex> vertices;
  std::vector<face> faces;

  void loadModel(const std::string& fileName);
};

vertex cross(const vertex& v0, const vertex& v1, const vertex& v2);

// Remember: this is vectors, not vertices!
float dot(const vertex& l, const vertex& r);
