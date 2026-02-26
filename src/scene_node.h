#pragma once

#include "geometry.h"
#include "loader.h"
#include <memory>
#include <string>
#include <vector>

class SceneNode {
 public:
  explicit SceneNode(const std::string& name) : _name(name) {}

  void addChild(std::shared_ptr<SceneNode> child) {
    child->_parent = this;
    _children.push_back(std::move(child));
  }

  // Recursively compute world transforms and push into ModelInstance::_position
  void updateWorldTransforms(const matrix<4,4>& parentWorld) {
    _worldTransform = _localTransform * parentWorld;
    if (_model) {
      _model->_position = _worldTransform;
      _model->computeBounds();
    }
    for (auto& c : _children)
      c->updateWorldTransforms(_worldTransform);
  }

  void updateWorldTransforms() {
    updateWorldTransforms(matrix<4,4>::identity());
  }

  matrix<4,4> _localTransform = matrix<4,4>::identity();
  matrix<4,4> _worldTransform = matrix<4,4>::identity();
  std::shared_ptr<ModelInstance> _model;  // may be null (group node)
  const std::string _name;
  SceneNode* _parent = nullptr;
  std::vector<std::shared_ptr<SceneNode>> _children;
};
