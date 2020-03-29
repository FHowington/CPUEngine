//
// Created by Forbes Howington on 3/28/20.
//
#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include "Window.h"

struct vertex {
  unsigned _x;
  unsigned _y;
  unsigned _z;
  vertex (const unsigned x, const unsigned y, const unsigned z) : _x(x), _y(y), _z(z) {}
};

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

      // For now, z is ignored
      result.emplace_back(std::make_shared<vertex>(x, y, 0));
    }
  }
  return result;
}


std::vector<std::pair<const std::shared_ptr<vertex>, const std::shared_ptr<vertex>>> getWireframe(const std::string& fileName) {
  std::vector<std::shared_ptr<vertex>> vertices = getVerticesFromWave(fileName);

  std::string line;
  std::ifstream infile(fileName);
  std::vector<std::pair<const std::shared_ptr<vertex>, const std::shared_ptr<vertex>>> result;

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
      result.emplace_back(vertices[v0 - 1], vertices[v1 - 1]);
      result.emplace_back(vertices[v1 - 1], vertices[v2 - 1]);
      result.emplace_back(vertices[v2 - 1], vertices[v0 - 1]);
    }
  }
  return result;
}
