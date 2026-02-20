#include "scene.h"

void Scene::load(const std::string& path) {
  loadScene(models, _modelAssets, _textureAssets, path);
}
