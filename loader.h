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

 private:
  std::vector<vertex<float>> vertices;
  std::vector<face> faces;

  void loadModel(const std::string& fileName, const unsigned width, const unsigned height);
};
