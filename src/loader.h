//
// Created by Forbes Howington on 3/28/20.
//

#pragma once

#include "Window.h"
#include "geometry.h"
#include "tgaimage.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

class TGAImage;
enum class shaderType;

class Model {
 public:
  Model(const std::string& fileName, const unsigned width, const unsigned height) { loadModel(fileName, width, height); }
  Model() = default;

  [[nodiscard]] const std::vector<face>& getFaces() const { return faces; }
  [[nodiscard]] const vertex<float>& getVertex(const unsigned idx) const { return vertices[idx]; }
  [[nodiscard]] const vertex<float>& getVertexNormal(const unsigned idx) const { return vertexNormals[idx]; }

  template<typename T>
  void setVertices(T&& newVertices)
  {
    vertices = std::forward<std::vector<vertex<float>>>(newVertices);
  }

  template<typename T>
  void setNormals(T&& newNormals)
  {
    vertexNormals = std::forward<std::vector<vertex<float>>>(newNormals);
  }

  template<typename T>
  void setFaces(T&& newFaces)
  {
    faces = std::forward<std::vector<face>>(newFaces);
  }

 private:
  std::vector<vertex<float>> vertices;
  std::vector<vertex<float>> vertexNormals;
  std::vector<face> faces;
  void loadModel(const std::string& fileName, unsigned width, unsigned height);
};


struct ModelInstance {
  ModelInstance (const Model& mod, const TGAImage* text, const shaderType shader, const float globalIllumination = 0.2) : _baseModel(mod), _texture(text), _shader(shader), _globalIllumination(globalIllumination) {}
  const Model& _baseModel;
  const TGAImage* _texture;
  matrix<4,4> _position;
  const shaderType _shader;
  const float _globalIllumination;
};
