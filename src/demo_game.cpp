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
      // Settings panel toggle
      case SDLK_TAB:
        _showSettings = !_showSettings;
        break;
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
      // FOV controls ([ and ])
      case SDLK_LEFTBRACKET:
        _camera.setFOV(_camera.getFOV() - 5.0f);
        if (_camera.getFOV() < 30.0f) _camera.setFOV(30.0f);
        break;
      case SDLK_RIGHTBRACKET:
        _camera.setFOV(_camera.getFOV() + 5.0f);
        if (_camera.getFOV() > 120.0f) _camera.setFOV(120.0f);
        break;
    }
  }
}

void DemoGame::update(float deltaTime, Engine& engine) {
  _camera.update(deltaTime);
  engine.setCameraTransform(_camera.getTransform());
  engine.setWireframeMode(_wireframe);
  engine.setFOV(_camera.getFOV());

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
// 2x scaled font for readability. Panel sits in the top-left corner.
// At 2x scale: each glyph is 16x16 in the 1080px framebuffer → 64x64 on the
// 4x SDL window. Line pitch = 18px (16px glyph + 2px gap).

void DemoGame::drawOverlay() {
  constexpr int S       = 2;           // font scale factor
  constexpr int MARGIN  = 12;
  constexpr int PAD     = 8;
  constexpr int CHAR_W  = 8 * S;      // 16px per glyph
  constexpr int LINE_H  = 8 * S + 2;  // 18px per line
  constexpr int COLS    = 30;          // characters per line
  constexpr int ROWS    = 22;          // max text rows (with settings open)
  constexpr int PW      = COLS * CHAR_W + PAD * 2;   // 496px
  constexpr int PH      = ROWS * LINE_H + PAD * 2;   // 412px

  const int px = MARGIN;
  const int py = MARGIN;

  // Semi-transparent dark background + border
  Overlay::fillRect(px, py, PW, PH, 0x1A1A2E, 210);
  Overlay::drawRect(px, py, PW, PH, 0x4A4A6A);

  const int tx = px + PAD;
  int ty = py + PAD;
  char buf[96];

  // ── Title ───────────────────────────────────────────────────────────────
  Overlay::drawText(tx, ty, "CPU Engine", 0xFFFFFF, S); ty += LINE_H;
  ty += LINE_H / 2;

  // ── Camera Position ─────────────────────────────────────────────────────
  Overlay::drawText(tx, ty, "-- Camera --", 0xAAAAFF, S); ty += LINE_H;

  snprintf(buf, sizeof(buf), "X: %8.2f", _camera.getX());
  Overlay::drawText(tx, ty, buf, 0xFFFFFF, S); ty += LINE_H;

  snprintf(buf, sizeof(buf), "Y: %8.2f", _camera.getY());
  Overlay::drawText(tx, ty, buf, 0xFFFFFF, S); ty += LINE_H;

  snprintf(buf, sizeof(buf), "Z: %8.2f", _camera.getZ());
  Overlay::drawText(tx, ty, buf, 0xFFFFFF, S); ty += LINE_H;

  ty += LINE_H / 2;

  // ── Render ──────────────────────────────────────────────────────────────
  Overlay::drawText(tx, ty, "-- Render --", 0xAAAAFF, S); ty += LINE_H;

  snprintf(buf, sizeof(buf), "FPS: %6.1f [F]", _lastFPS);
  Overlay::drawText(tx, ty, buf, _fps ? 0x55FF55 : 0xCCCCCC, S); ty += LINE_H;

  snprintf(buf, sizeof(buf), "Wire: %s  [P]", _wireframe ? "ON " : "OFF");
  Overlay::drawText(tx, ty, buf, _wireframe ? 0x55FF55 : 0xCCCCCC, S); ty += LINE_H;

  snprintf(buf, sizeof(buf), "FOV:  %.0f     [/]", _camera.getFOV());
  Overlay::drawText(tx, ty, buf, 0x88FFFF, S); ty += LINE_H;

  snprintf(buf, sizeof(buf), "Speed:%.1fx  [-/=]", _cameraSpeed);
  Overlay::drawText(tx, ty, buf, 0x88FFFF, S); ty += LINE_H;

  ty += LINE_H / 2;

  // ── Controls (toggle with Tab) ──────────────────────────────────────────
  if (_showSettings) {
    Overlay::drawText(tx, ty, "-- Controls [TAB] --", 0xFFDD00, S); ty += LINE_H;
    Overlay::drawText(tx, ty, "WASD  Move camera", 0xCCCCCC, S); ty += LINE_H;
    Overlay::drawText(tx, ty, "Arrows  Look around", 0xCCCCCC, S); ty += LINE_H;
    Overlay::drawText(tx, ty, "Q/E  Spin model", 0xCCCCCC, S); ty += LINE_H;
    Overlay::drawText(tx, ty, "IJKL Move light", 0xCCCCCC, S); ty += LINE_H;
    Overlay::drawText(tx, ty, "[/]  FOV  -/= Speed", 0xCCCCCC, S); ty += LINE_H;
    Overlay::drawText(tx, ty, "P Wire  F FPS", 0xCCCCCC, S); ty += LINE_H;
  } else {
    Overlay::drawText(tx, ty, "[TAB] Show controls", 0x666666, S); ty += LINE_H;
  }
}
