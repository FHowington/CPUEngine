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

struct vertex {
  int _x;
  int _y;
  int _z;
  vertex (const unsigned x, const unsigned y, const unsigned z) : _x(x), _y(y), _z(z) {}
  vertex () : _x(), _y(), _z() {};

  float norm() { return std::sqrt(_x*_x+_y*_y+_z*_z); }
  vertex& normalize(unsigned l = 1) {
    float lnorm = norm();
    lnorm = l/lnorm;
    this->_x *= lnorm;
    this->_y *= lnorm;
    this->_z *= lnorm;
    return *this;
  }
};

vertex cross(const vertex& v0, const vertex& v1, const vertex& v2) {
  vertex l(v1._x - v0._x, v1._y - v0._y, v1._z - v1._z);
  vertex r(v1._x - v2._x, v1._y - v2._y, v1._z - v2._z);
  vertex v;
  v._x = (l._y * r._z) - (l._z * r._y);
  v._y = (l._z * r._x) - (l._x * r._z);
  v._z = (l._x * r._y) - (l._y * r._x);
  return v;
}

// Remeber: this is vectors, not vertices!
int dot(const vertex& l, const vertex& r) {
  return l._x * r._x + l._y * r._y + l._z + r._z;
}

std::vector<std::shared_ptr<vertex>> getVerticesFromWave(const std::string& fileName) {
  std::string line;
  std::ifstream infile(fileName);
  std::vector<std::shared_ptr<vertex>> result;

  if (!infile) {
    printf("File failed\n");
    return result;
  }
  while (std::getline(infile, line))
  {
    if (line.size() > 1 && line[0] == 'v' && line[1] == ' ') {
      line = line.substr(1, line.size());
      std::istringstream iss(line);
      float x;
      float y;
      float z;
      if (!(iss >> x >> y >> z)) {
        printf("Parsing for vectors failed at line: %s\n", line.c_str());
        break;
      }

      // w/2 is 0
      x = x * (W/2) + (W/2);
      y = y * (H/2) + (H/2);
      // For now, consider 16 bits of depth
      z = z * (0xFFFF / 2) + (0xFFFF / 2);

      // For now, z is ignored
      result.emplace_back(std::make_shared<vertex>(x, y, z));
    }
  }
  return result;
}


std::vector<std::tuple<const std::shared_ptr<vertex>, const std::shared_ptr<vertex>, const std::shared_ptr<vertex>>> getWireframe(const std::string& fileName) {
  std::vector<std::shared_ptr<vertex>> vertices = getVerticesFromWave(fileName);

  std::string line;
  std::ifstream infile(fileName);
  std::vector<std::tuple<const std::shared_ptr<vertex>, const std::shared_ptr<vertex>, const std::shared_ptr<vertex>>> result;

  if (!infile) {
    printf("File failed\n");
    return result;
  }

  while (std::getline(infile, line))
  {
    if (line.size() > 1 && line[0] == 'f' && line[1] == ' ') {

      std::replace(line.begin(), line.end(), '/', ' ');
      line = line.substr(1, line.size());

      std::istringstream iss(line);
      unsigned v0;
      unsigned v1;
      unsigned v2;
      unsigned trash;


      if (!(iss >> v0 >> trash >> trash >> v1 >> trash >> trash >> v2)) {
        printf("Parsing for faces failed at line: %s\n", line.c_str());
        break;
      }

      // For now, z is ignored
      result.emplace_back(vertices[v0 - 1], vertices[v1 - 1], vertices[v2-1]);
    }
  }
  return result;
}
