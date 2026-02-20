#include "demo_game.h"
#include <iostream>

void DemoGame::init(Engine& engine) {
  _scene.load("../src/scene1.scn");

  _scene.lights.emplace_back(LightType::Point, -5, 0, 7, 50, 1, 0, 0);
  _scene.lights.emplace_back(LightType::Point, 5, 0, 7, 50, 0, 0, 1);
  _scene.lights.emplace_back(LightType::Directional, vertex<float>(5, _lightY, -1.5), 1, 1, 1);
  Light::sceneLights = _scene.lights;

  _fpsStart = std::chrono::high_resolution_clock::now();
}

void DemoGame::handleEvent(const SDL_Event& event, bool& quit) {
  _camera.handleEvent(event);

  if (event.type == SDL_KEYDOWN) {
    switch (event.key.keysym.sym) {
      case SDLK_p: _wireframe = !_wireframe; break;
      case SDLK_f: _fps = !_fps;             break;
      case SDLK_e: _rot += 0.05f;            break;
      case SDLK_q: _rot -= 0.05f;            break;
      case SDLK_i: _lightY += 0.2f;          break;
      case SDLK_k: _lightY -= 0.2f;          break;
      case SDLK_l: _lightX += 0.2f;          break;
      case SDLK_j: _lightX -= 0.2f;          break;
    }
  }
}

void DemoGame::update(float deltaTime, Engine& engine) {
  _camera.update(deltaTime);
  engine.setCameraTransform(_camera.getTransform());

  // Update directional light and sync to the rendering global
  _scene.lights.back()._direction = vertex<float>(_lightX, _lightY, -1.5);
  _scene.lights.back()._direction.normalize();
  Light::sceneLights = _scene.lights;

  // Spin the first model via e/q keys
  matrix<4,4> newPosition = matrix<4,4>::rotationY(_rot);
  static matrix<4,4> scaler;
  scaler.set(0, 0, .8f);
  scaler.set(1, 1, .8f);
  scaler.set(2, 2, .8f);
  scaler.set(3, 3, 1);
  newPosition = scaler * newPosition;
  newPosition.set(3, 2, -5);
  newPosition.set(3, 0, -5);
  _scene.models.front()->_position = newPosition;

  // FPS counter
  if (_fps) {
    ++_frame;
    if (_frame == 100) {
      _frame = 0;
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - _fpsStart);
      float fpsVal = (100.0F / duration.count()) * 1000000;
      std::cout << fpsVal << " FPS" << std::endl;
      _fpsStart = std::chrono::high_resolution_clock::now();
    }
  }
}

const std::vector<std::shared_ptr<ModelInstance>>& DemoGame::getModels() const {
  return _scene.models;
}
