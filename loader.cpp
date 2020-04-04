//
// Created by Forbes Howington on 4/4/20.
//

#include "loader.h"

void  Model::loadModel(const std::string& fileName, const unsigned width, const unsigned height) {
  std::string line;
  std::ifstream infile(fileName);

  if (!infile) {
    printf("File failed\n");
    return;
  }

  std::vector<vertex> textures;

  // Implicit assumption that all faces come after the vertices so this may be done in a single pass
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

      vertices.emplace_back(x, y, z);

    } else if (line.size() > 2 && line[0] == 'v' && line[1] == 't' && line[2] == ' ') {
      line = line.substr(2, line.size());
      std::istringstream iss(line);
      float x;
      float y;
      if (!(iss >> x >> y)) {
        printf("Parsing for textures failed at line: %s\n", line.c_str());
        break;
      }

      textures.emplace_back(x, y, 0);

    } else if (line.size() > 1 && line[0] == 'f' && line[1] == ' ') {

      std::replace(line.begin(), line.end(), '/', ' ');
      line = line.substr(1, line.size());

      std::istringstream iss(line);
      unsigned v0;
      unsigned v1;
      unsigned v2;

      unsigned t0;
      unsigned t1;
      unsigned t2;
      unsigned trash;

      if (!(iss >> v0 >> t0 >> trash >> v1 >> t1 >> trash >> v2 >> t2)) {
        printf("Parsing for faces failed at line: %s\n", line.c_str());
        break;
      }

      faces.emplace_back(vertices[v0 - 1], vertices[v1 - 1], vertices[v2-1],
                         textures.at(t0-1)._x * width, textures.at(t0-1)._y * height,
                         textures.at(t1-1)._x * width, textures.at(t1-1)._y * height,
                         textures.at(t2-1)._x * width, textures.at(t2-1)._y * height);
    }
  }
}

vertex& vertex::normalize(float l) {
  float lnorm = norm();
  lnorm = l/lnorm;
  this->_x *= lnorm;
  this->_y *= lnorm;
  this->_z *= lnorm;
  return *this;
}

vertex cross(const vertex& v0, const vertex& v1, const vertex& v2) {
  vertex l(v1._x - v0._x, v1._y - v0._y, v1._z - v0._z);
  vertex r(v1._x - v2._x, v1._y - v2._y, v1._z - v2._z);
  vertex v;
  v._x = (l._y * r._z) - (l._z * r._y);
  v._y = (l._z * r._x) - (l._x * r._z);
  v._z = (l._x * r._y) - (l._y * r._x);
  return v;
}

float dot(const vertex& l, const vertex& r) {
  return l._x * r._x + l._y * r._y + l._z * r._z;
}
