#include "scene.h"

void Scene::load(const std::string& path) {
  loadScene(models, _modelAssets, _textureAssets, path, roots, nodesByName, collision);
}

void Scene::updateTransforms() {
  for (auto& root : roots)
    root->updateWorldTransforms();
}

std::shared_ptr<SceneNode> Scene::findNode(const std::string& name) const {
  auto it = nodesByName.find(name);
  return it != nodesByName.end() ? it->second : nullptr;
}
