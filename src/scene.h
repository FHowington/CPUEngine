#pragma once

#include "collision.h"
#include "light.h"
#include "loader.h"
#include "scene_node.h"
#include "tgaimage.h"
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

class Scene {
 public:
  void load(const std::string& path);

  // Update all world transforms from the node tree into ModelInstance::_position
  void updateTransforms();

  // Find a node by name (returns nullptr if not found)
  std::shared_ptr<SceneNode> findNode(const std::string& name) const;

  std::vector<std::shared_ptr<ModelInstance>> models;
  std::list<Light> lights;
  std::vector<std::shared_ptr<SceneNode>> roots;  // top-level scene nodes
  std::map<std::string, std::shared_ptr<SceneNode>> nodesByName;
  CollisionWorld collision;

 private:
  std::map<const std::string, Model> _modelAssets;
  std::map<const std::string, TGAImage> _textureAssets;
};
