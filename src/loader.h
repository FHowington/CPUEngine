//
// Created by Forbes Howington on 3/28/20.
//

#pragma once

#include <cmath>
#include <fstream>
#include "geometry.h"
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include "Window.h"

class Model {
 public:
  Model(const std::string& fileName, const unsigned width, const unsigned height) { loadModel(fileName, width, height); }

  const std::vector<face>& getFaces() const { return faces; }
  const vertex<float>& getVertex(const unsigned idx) const { return vertices[idx]; }

 private:
  std::vector<vertex<float>> vertices;
  std::vector<vertex<float>> vertexNormals;
  std::vector<face> faces;

  void loadModel(const std::string& fileName, const unsigned width, const unsigned height);
};
