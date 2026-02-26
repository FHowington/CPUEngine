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
class SceneNode;
class CollisionWorld;

class Model {
 public:
  Model(const std::string& fileName, const TGAImage* texture);
  Model() : _texture(nullptr) {}

  [[nodiscard]] const std::vector<face>& getFaces() const { return faces; }
  [[nodiscard]] const std::vector<vertex<float>>& getVertices() const { return vertices; }
  [[nodiscard]] const std::vector<vertex<float>>& getVertexNormals() const { return vertexNormals; }
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
  ModelInstance (const Model& mod, const shaderType shader, const float globalIllumination = 0.2, const bool doubleSided = false) : _baseModel(mod), _shader(shader), _globalIllumination(globalIllumination), _doubleSided(doubleSided) {
    computeBounds();
  }
  void computeBounds() {
    const auto& faces = _baseModel.getFaces();
    if (faces.empty()) return;
    float cx = 0, cy = 0, cz = 0;
    unsigned n = 0;
    for (const auto& f : faces) {
      for (unsigned idx : {f._v0, f._v1, f._v2}) {
        const auto& v = _baseModel.getVertex(idx);
        cx += v._x; cy += v._y; cz += v._z; ++n;
      }
    }
    cx /= n; cy /= n; cz /= n;
    _bCenter = vertex<float>(cx, cy, cz);
    _bRadius = 0;
    for (const auto& f : faces) {
      for (unsigned idx : {f._v0, f._v1, f._v2}) {
        const auto& v = _baseModel.getVertex(idx);
        float dx = v._x - cx, dy = v._y - cy, dz = v._z - cz;
        float d2 = dx*dx + dy*dy + dz*dz;
        if (d2 > _bRadius) _bRadius = d2;
      }
    }
    _bRadius = sqrtf(_bRadius);
  }
  const Model& _baseModel;
  matrix<4,4> _position;
  const shaderType _shader;
  const float _globalIllumination;
  const bool _doubleSided;
  vertex<float> _bCenter;
  float _bRadius = 0;
};

void loadScene(std::vector<std::shared_ptr<ModelInstance>>& modelInstances, std::map<const std::string, Model>& models, std::map<const std::string, TGAImage>& textures, const std::string& sceneFile, std::vector<std::shared_ptr<SceneNode>>& roots, std::map<std::string, std::shared_ptr<SceneNode>>& nodesByName, CollisionWorld& collision);
