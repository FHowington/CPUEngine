//
// Created by Forbes Howington on 3/28/20.
//

#pragma once

#include "Window.h"
#include "geometry.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <vector>

enum class shaderType;
class TGAImage;

class Model {
 public:
  Model(const std::string& fileName, const TGAImage* texture);
  Model() : _texture(nullptr) {}

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

  const TGAImage* _texture;

 private:
  std::vector<vertex<float>> vertices;
  std::vector<vertex<float>> vertexNormals;
  std::vector<face> faces;
  void loadModel(const std::string& fileName, unsigned width, unsigned height);
};


struct ModelInstance {
  ModelInstance (const Model& mod, const shaderType shader, const float globalIllumination = 0.2) : _baseModel(mod), _shader(shader), _globalIllumination(globalIllumination) {}
  const Model& _baseModel;
  matrix<4,4> _position;
  const shaderType _shader;
  const float _globalIllumination;
};

void loadScene(std::vector<std::shared_ptr<ModelInstance>>& modelInstances, std::map<const std::string, Model>& models, std::map<const std::string, TGAImage>& textures, const std::string& sceneFile);
