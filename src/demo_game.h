//
// DemoGame — the original scene viewer, now as a Game implementation.
// Shows how to build a game on top of Engine, Scene, and Camera.
//

#pragma once

#include "camera.h"
#include "engine.h"
#include "firefly.h"
#include "menu.h"
#include "overlay.h"
#include "scene.h"
#include <chrono>
#include <vector>

class DemoGame : public Game {
 public:
  void init(Engine& engine) override;
  void handleEvent(const SDL_Event& event, bool& quit) override;
  void update(float deltaTime, Engine& engine) override;
  const std::vector<std::shared_ptr<ModelInstance>>& getModels() const override;
  void drawOverlay() override;
  void postProcess() override;

 private:
  Scene _scene;
  Camera _camera;

  // Demo-specific state: light controls, model spin, toggles
  float _lightX = 1;
  float _lightY = -0.5;
  float _rot = 0;
  bool _wireframe = false;
  bool _fps = false;
  unsigned _frame = 0;
  float _lastFPS = 0.0f;
  std::chrono::high_resolution_clock::time_point _fpsStart;

  // Camera options
  float _cameraSpeed = 1.0f;
  float _fov = 41.0f;
  float _nearClip = 2.0f;
  float _farClip = 100.0f;
  bool _lightFog = true;
  bool _depthFog = true;
  float _depthFogNear = 0.15f;
  float _depthFogFar = 0.85f;
  bool _aa = true;
  float _aaThreshold = 24.0f;
  bool _ssao = true;
  float _ssaoRadius = 3.0f;
  float _ssaoStrength = 0.5f;
  bool _showNormals = false;
  bool _frustumCull = true;
  bool _dynamicLights = true;
  bool _specular = true;
  float _shininess = 64.0f;
  float _specStrength = 1.0f;
  float _resolution = 1080.0f;
  matrix<4,4> _renderCameraTransform;

  // Animated entities
  std::vector<Firefly> _fireflies;

  // Menu system
  Menu _mainMenu{"Settings"};
  Menu _renderMenu{"Render"};
  Menu _cameraMenu{"Camera"};
  Menu _fogMenu{"Fog"};
  MenuStack _menuStack;
  void buildMenus();
};
