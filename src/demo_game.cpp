#include "demo_game.h"
#include <cstdio>
#include <iostream>

void DemoGame::init(Engine& engine) {
  _scene.load("../src/scene1.scn");

  _scene.lights.emplace_back(LightType::Point, -5, 0, 7, 50, 1, 0, 0);
  _scene.lights.emplace_back(LightType::Point, 5, 0, 7, 50, 0, 0, 1);
  _scene.lights.emplace_back(LightType::Directional, vertex<float>(5, _lightY, -1.5), 1, 1, 1);
  Light::sceneLights = _scene.lights;

  _camera.setSensitivity(_cameraSpeed);
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
      // Camera speed controls (- and =)
      case SDLK_MINUS:
      case SDLK_UNDERSCORE:
        _cameraSpeed *= 0.8f;
        if (_cameraSpeed < 0.1f) _cameraSpeed = 0.1f;
        _camera.setSensitivity(_cameraSpeed);
        break;
      case SDLK_EQUALS:
      case SDLK_PLUS:
        _cameraSpeed *= 1.25f;
        if (_cameraSpeed > 4.0f) _cameraSpeed = 4.0f;
        _camera.setSensitivity(_cameraSpeed);
        break;
      // Camera sensitivity controls (Ctrl+- and Ctrl+=)
      case SDLK_LCTRL:
      case SDLK_RCTRL:
        // Modifier only, handle in other cases
        break;
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

  // FPS counter — always running so the overlay can show it
  ++_frame;
  if (_frame == 100) {
    _frame = 0;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - _fpsStart);
    _lastFPS = (100.0F / duration.count()) * 1000000;
    if (_fps) {
      std::cout << _lastFPS << " FPS" << std::endl;
    }
    _fpsStart = std::chrono::high_resolution_clock::now();
  }
}

const std::vector<std::shared_ptr<ModelInstance>>& DemoGame::getModels() const {
  return _scene.models;
}

// ─── Overlay ─────────────────────────────────────────────────────────────────
// Panel sits in the top-left corner.
// Layout: 8px outer margin, 4px inner padding, 8×8 font, 10px line pitch.

void DemoGame::drawOverlay() {
  // Panel geometry
  constexpr int MARGIN  = 8;   // distance from window edge
  constexpr int PAD     = 4;   // inner padding
  constexpr int CHAR_W  = 8;
  constexpr int LINE_H  = 10;  // 8px glyph + 2px gap
  constexpr int COLS    = 28;  // max characters per line (increased from 18)
  constexpr int ROWS    = 24;  // number of text rows (increased from 16)
  constexpr int PW      = COLS * CHAR_W + PAD * 2;
  constexpr int PH      = ROWS * LINE_H + PAD * 2;

  const int px = MARGIN;
  const int py = MARGIN;

  // Semi-transparent dark background + border
  Overlay::fillRect(px, py, PW, PH, 0x1A1A2E, 210);
  Overlay::drawRect(px, py, PW, PH, 0x4A4A6A);

  // Text origin (inside padding)
  const int tx = px + PAD;
  int ty = py + PAD;
  char buf[64];

  // ── Camera ──────────────────────────────────────────────────────────────
  Overlay::drawText(tx, ty, "-- Camera --", 0xAAAAFF); ty += LINE_H;

  snprintf(buf, sizeof(buf), "X:   %7.2f", _camera.getX());
  Overlay::drawText(tx, ty, buf, 0xFFFFFF); ty += LINE_H;

  snprintf(buf, sizeof(buf), "Y:   %7.2f", _camera.getY());
  Overlay::drawText(tx, ty, buf, 0xFFFFFF); ty += LINE_H;

  snprintf(buf, sizeof(buf), "Z:   %7.2f", _camera.getZ());
  Overlay::drawText(tx, ty, buf, 0xFFFFFF); ty += LINE_H;

  snprintf(buf, sizeof(buf), "Pitch:%6.2f", _camera.getPitch());
  Overlay::drawText(tx, ty, buf, 0xFFFFFF); ty += LINE_H;

  snprintf(buf, sizeof(buf), "Yaw:  %6.2f", _camera.getYaw());
  Overlay::drawText(tx, ty, buf, 0xFFFFFF); ty += LINE_H;

  snprintf(buf, sizeof(buf), "Speed: %5.2fx", _cameraSpeed);
  Overlay::drawText(tx, ty, buf, 0x88FFFF); ty += LINE_H;

  ty += LINE_H;  // blank separator

  // ── Toggles ─────────────────────────────────────────────────────────────
  Overlay::drawText(tx, ty, "-- Toggles --", 0xAAAAFF); ty += LINE_H;

  snprintf(buf, sizeof(buf), "[F] FPS:  %5.1f", _lastFPS);
  Overlay::drawText(tx, ty, buf, _fps ? 0x55FF55 : 0x888888); ty += LINE_H;

  Overlay::drawText(tx, ty,
      _wireframe ? "[P] Wire:  ON " : "[P] Wire: OFF",
      _wireframe ? 0x55FF55 : 0x888888); ty += LINE_H;

  ty += LINE_H;  // blank separator

  // ── Controls ────────────────────────────────────────────────────────────
  Overlay::drawText(tx, ty, "-- Controls --", 0xAAAAFF); ty += LINE_H;
  Overlay::drawText(tx, ty, "WASD  Move",       0xCCCCCC); ty += LINE_H;
  Overlay::drawText(tx, ty, "Arrs  Look",       0xCCCCCC); ty += LINE_H;
  Overlay::drawText(tx, ty, "Q/E   Spin",       0xCCCCCC); ty += LINE_H;
  Overlay::drawText(tx, ty, "IJKL  Light",      0xCCCCCC); ty += LINE_H;
  Overlay::drawText(tx, ty, "-/=   Speed",      0xFFAAAA); ty += LINE_H;
}
