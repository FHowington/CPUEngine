#include "demo_game.h"
#include "Window.h"
#include "rasterize.h"
#include "shader.h"
#include <iostream>

void DemoGame::init(Engine& engine) {
  Light::sceneLights.emplace_back(LightType::Point, -5, 0, 7, 50, 1, 0, 0);
  Light::sceneLights.emplace_back(LightType::Point, 5, 0, 7, 50, 0, 0, 1);
  Light::sceneLights.emplace_back(LightType::Directional, vertex<float>(5, _lightY, -1.5), 1, 1, 1);

  loadScene(_modelsInScene, _models, _textures, "../src/scene1.scn");

  _modInstance = _modelsInScene.front();

  _fpsStart = std::chrono::high_resolution_clock::now();
}

void DemoGame::handleEvent(const SDL_Event& event, bool& quit) {
  switch (event.type) {
    case SDL_KEYDOWN:
      switch (event.key.keysym.sym) {
        case SDLK_DOWN:  _lDown = true;     break;
        case SDLK_UP:    _lUp = true;       break;
        case SDLK_LEFT:  _lLeft = true;     break;
        case SDLK_RIGHT: _lRight = true;    break;
        case SDLK_p:     _wireframe = !_wireframe; break;
        case SDLK_f:     _fps = !_fps;      break;
        case SDLK_e:     _rot += 0.05;      break;
        case SDLK_q:     _rot -= 0.05;      break;
        case SDLK_w:     _mForward = true;  break;
        case SDLK_s:     _mBackward = true; break;
        case SDLK_a:     _mLeft = true;     break;
        case SDLK_d:     _mRight = true;    break;
        case SDLK_i:     _lightY += 0.2;    break;
        case SDLK_k:     _lightY -= 0.2;    break;
        case SDLK_l:     _lightX += 0.2;    break;
        case SDLK_j:     _lightX -= 0.2;    break;
      }
      break;

    case SDL_KEYUP:
      switch (event.key.keysym.sym) {
        case SDLK_DOWN:  _lDown = false;     break;
        case SDLK_UP:    _lUp = false;       break;
        case SDLK_LEFT:  _lLeft = false;     break;
        case SDLK_RIGHT: _lRight = false;    break;
        case SDLK_w:     _mForward = false;  break;
        case SDLK_s:     _mBackward = false; break;
        case SDLK_a:     _mLeft = false;     break;
        case SDLK_d:     _mRight = false;    break;
      }
      break;
  }
}

void DemoGame::update(float deltaTime, Engine& engine) {
  // Camera rotation
  if (_lUp)    _cameraRotX -= 0.03;
  if (_lDown)  _cameraRotX += 0.03;
  if (_lRight) _cameraRotY -= 0.03;
  if (_lLeft)  _cameraRotY += 0.03;

  matrix<4,4> cameraRotXM = matrix<4,4>::rotationX(_cameraRotX);
  matrix<4,4> cameraRotYM = matrix<4,4>::rotationY(_cameraRotY);
  matrix<4,4> cameraRot(cameraRotXM * cameraRotYM);

  // Camera translation (deltaTime is already divided by SPEED in engine)
  if (_mForward) {
    _cameraX -= deltaTime * cameraRot._m[8];
    _cameraY -= deltaTime * cameraRot._m[9];
    _cameraZ -= deltaTime * cameraRot._m[10];
  } else if (_mBackward) {
    _cameraX += deltaTime * cameraRot._m[8];
    _cameraY += deltaTime * cameraRot._m[9];
    _cameraZ += deltaTime * cameraRot._m[10];
  }

  if (_mLeft) {
    _cameraX -= deltaTime * cameraRot._m[0];
    _cameraY -= deltaTime * cameraRot._m[1];
    _cameraZ -= deltaTime * cameraRot._m[2];
  } else if (_mRight) {
    _cameraX += deltaTime * cameraRot._m[0];
    _cameraY += deltaTime * cameraRot._m[1];
    _cameraZ += deltaTime * cameraRot._m[2];
  }

  cameraRot.set(3, 0, _cameraX);
  cameraRot.set(3, 1, _cameraY);
  cameraRot.set(3, 2, _cameraZ);

  engine.setCameraTransform(invert(cameraRot));

  // Update directional light
  auto& dirLight = Light::sceneLights.back();
  dirLight._direction = vertex<float>(_lightX, _lightY, -1.5);
  dirLight._direction.normalize();

  // Update model position (rotation via e/q)
  matrix<4,4> newPosition = matrix<4,4>::rotationY(_rot);
  static matrix<4,4> scaler;
  scaler.set(0, 0, .8);
  scaler.set(1, 1, .8);
  scaler.set(2, 2, .8);
  scaler.set(3, 3, 1);

  newPosition = scaler * newPosition;
  newPosition.set(3, 2, -5);
  newPosition.set(3, 0, -5);
  _modInstance->_position = newPosition;

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
  return _modelsInScene;
}
