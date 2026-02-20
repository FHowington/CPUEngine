//
// DemoGame — the original scene viewer, now as a Game implementation.
// Shows how to build a game on top of the Engine.
//

#pragma once

#include "engine.h"
#include "light.h"
#include "tgaimage.h"
#include <chrono>
#include <map>

class DemoGame : public Game {
 public:
  void init(Engine& engine) override;
  void handleEvent(const SDL_Event& event, bool& quit) override;
  void update(float deltaTime, Engine& engine) override;
  const std::vector<std::shared_ptr<ModelInstance>>& getModels() const override;

 private:
  std::vector<std::shared_ptr<ModelInstance>> _modelsInScene;
  std::map<const std::string, Model> _models;
  std::map<const std::string, TGAImage> _textures;
  std::shared_ptr<ModelInstance> _modInstance;

  bool _wireframe = false;
  bool _fps = false;
  unsigned _frame = 0;

  float _lightX = 1;
  float _lightY = -3;
  float _rot = 0;

  float _cameraRotX = 0;
  float _cameraRotY = 0;

  bool _lLeft = false;
  bool _lRight = false;
  bool _lUp = false;
  bool _lDown = false;

  bool _mForward = false;
  bool _mBackward = false;
  bool _mLeft = false;
  bool _mRight = false;

  float _cameraX = 0;
  float _cameraY = 0;
  float _cameraZ = 0;

  std::chrono::high_resolution_clock::time_point _fpsStart;
};
