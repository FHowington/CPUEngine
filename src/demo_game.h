//
// DemoGame — the original scene viewer, now as a Game implementation.
// Shows how to build a game on top of Engine, Scene, and Camera.
//

#pragma once

#include "camera.h"
#include "engine.h"
#include "menu.h"
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

  // Camera options
  float _cameraSpeed = 1.0f;  // Movement speed & look sensitivity multiplier
  bool _showSettings = true;  // Show/hide settings panel (Tab key)
  bool _lightFog = true;      // Volumetric light glow (G key)
  matrix<4,4> _renderCameraTransform;  // Camera transform used for the rendered frame

  // Menu system
  Menu _mainMenu{"Settings"};
  Menu _renderMenu{"Render"};
  Menu _controlsMenu{"Controls"};
  MenuStack _menuStack;
  void buildMenus();
};
