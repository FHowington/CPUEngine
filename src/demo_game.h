//
// DemoGame — the original scene viewer, now as a Game implementation.
// Shows how to build a game on top of Engine, Scene, and Camera.
//

#pragma once

#include "camera.h"
#include "engine.h"
#include "overlay.h"
#include "scene.h"
#include <chrono>

class DemoGame : public Game {
 public:
  void init(Engine& engine) override;
  void handleEvent(const SDL_Event& event, bool& quit) override;
  void update(float deltaTime, Engine& engine) override;
  const std::vector<std::shared_ptr<ModelInstance>>& getModels() const override;
  void drawOverlay() override;

 private:
  Scene _scene;
  Camera _camera;

  // Demo-specific state: light controls, model spin, toggles
  float _lightX = 1;
  float _lightY = -3;
  float _rot = 0;
  bool _wireframe = false;
  bool _fps = false;
  unsigned _frame = 0;
  float _lastFPS = 0.0f;
  std::chrono::high_resolution_clock::time_point _fpsStart;
};
