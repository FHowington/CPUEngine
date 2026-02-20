#pragma once

#include "light.h"
#include "loader.h"
#include "tgaimage.h"
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

class Scene {
 public:
  void load(const std::string& path);

  std::vector<std::shared_ptr<ModelInstance>> models;
  std::list<Light> lights;

 private:
  // Asset maps kept private: ModelInstance and Model hold raw pointers/refs
  // into these, so their lifetime must be tied to the Scene.
  std::map<const std::string, Model> _modelAssets;
  std::map<const std::string, TGAImage> _textureAssets;
};
