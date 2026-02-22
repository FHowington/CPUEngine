#include "antialias.h"
#include "demo_game.h"
#include "depth_fog.h"
#include "ssao.h"
#include "geometry.h"
#include "light.h"
#include "light_fog.h"
#include "loader.h"
#include "shader.h"
#include <algorithm>
#include <cstdio>
#include <iostream>

void DemoGame::init(Engine& engine) {
  _scene.load("../src/scene1.scn");

  _scene.lights.emplace_back(LightType::Point, -4, 2, 13, 40, 1.0, 0.6, 0.2);
  _scene.lights.emplace_back(LightType::Point, 4, 2, 13, 40, 1.0, 0.6, 0.2);
  _scene.lights.emplace_back(LightType::Point, 0, 3, -5, 200, 0.3, 0.3, 0.8);
  Light::sceneLights = _scene.lights;

  _camera.setSensitivity(_cameraSpeed);
  _fpsStart = std::chrono::high_resolution_clock::now();

  buildMenus();
}

void DemoGame::buildMenus() {
  _renderMenu.addItem(MenuItem::toggle("Wireframe", &_wireframe));
  _renderMenu.addItem(MenuItem::toggle("FPS Log", &_fps));
  _renderMenu.addItem(MenuItem::toggle("AA", &_aa));
  _renderMenu.addItem(MenuItem::slider("AA Thresh", &_aaThreshold, 4.0f, 64.0f, 4.0f));
  _renderMenu.addItem(MenuItem::toggle("SSAO", &_ssao));
  _renderMenu.addItem(MenuItem::slider("SSAO Radius", &_ssaoRadius, 1.0f, 10.0f, 1.0f));
  _renderMenu.addItem(MenuItem::slider("SSAO Str", &_ssaoStrength, 0.1f, 1.0f, 0.1f));
  _renderMenu.addItem(MenuItem::toggle("Normals", &_showNormals));
  _renderMenu.addItem(MenuItem::toggle("Frustum Cull", &_frustumCull));
  _renderMenu.addItem(MenuItem::toggle("Dynamic Lights", &_dynamicLights));
  _renderMenu.addItem(MenuItem::toggle("Specular", &_specular));
  _renderMenu.addItem(MenuItem::slider("Shininess", &_shininess, 4.0f, 512.0f, 8.0f));
  _renderMenu.addItem(MenuItem::slider("Spec Str", &_specStrength, 0.1f, 2.0f, 0.1f));

  _cameraMenu.addItem(MenuItem::slider("Speed", &_cameraSpeed, 0.1f, 4.0f, 0.1f));
  _cameraMenu.addItem(MenuItem::slider("FOV", &_fov, 30.0f, 120.0f, 5.0f));
  _cameraMenu.addItem(MenuItem::separator());
  _cameraMenu.addItem(MenuItem::slider("Near Clip", &_nearClip, 0.5f, 10.0f, 0.5f));
  _cameraMenu.addItem(MenuItem::slider("Far Clip", &_farClip, 20.0f, 500.0f, 10.0f));

  _fogMenu.addItem(MenuItem::toggle("Light Fog", &_lightFog));
  _fogMenu.addItem(MenuItem::toggle("Depth Fog", &_depthFog));
  _fogMenu.addItem(MenuItem::separator());
  _fogMenu.addItem(MenuItem::slider("Fog Near", &_depthFogNear, 0.0f, 1.0f, 0.05f));
  _fogMenu.addItem(MenuItem::slider("Fog Far", &_depthFogFar, 0.1f, 1.0f, 0.05f));

  _mainMenu.addItem(MenuItem::sub("Render", &_renderMenu));
  _mainMenu.addItem(MenuItem::sub("Camera", &_cameraMenu));
  _mainMenu.addItem(MenuItem::sub("Fog", &_fogMenu));

  _menuStack.setRoot(&_mainMenu);
  _menuStack.setPosition(W - 420, 12);
}

void DemoGame::handleEvent(const SDL_Event& event, bool& quit) {
  if (event.type == SDL_KEYDOWN) {
    // Menu toggle
    if (event.key.keysym.sym == SDLK_ESCAPE) {
      if (_menuStack.isOpen()) _menuStack.close();
      else _menuStack.open();
      return;
    }
    // If menu is open, it consumes arrow/enter/backspace keys
    if (_menuStack.handleKey(event.key.keysym.sym))
      return;
  }

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
      case SDLK_g: _lightFog = !_lightFog;   break;
      // Camera speed controls (- and =)
      case SDLK_MINUS:
      case SDLK_UNDERSCORE:
        _cameraSpeed = std::max(0.1f, _cameraSpeed * 0.8f);
        break;
      case SDLK_EQUALS:
      case SDLK_PLUS:
        _cameraSpeed = std::min(4.0f, _cameraSpeed * 1.25f);
        break;
      case SDLK_LEFTBRACKET:
        _fov = std::max(30.0f, _fov - 5.0f);
        break;
      case SDLK_RIGHTBRACKET:
        _fov = std::min(120.0f, _fov + 5.0f);
        break;
    }
  }
}

void DemoGame::update(float deltaTime, Engine& engine) {
  _renderCameraTransform = engine.getRenderCameraTransform();

  // Sync menu-controlled camera params
  _camera.setSensitivity(_cameraSpeed);
  _camera.setFOV(_fov);
  _camera.setNearClip(_nearClip);
  _camera.setFarClip(_farClip);

  _camera.update(deltaTime);
  engine.setCameraTransform(_camera.getTransform());
  engine.setWireframeMode(_wireframe);
  engine.setFrustumCulling(_frustumCull);
  engine.setFOV(_camera.getFOV());
  engine.setClipDistances(_camera.getNearClip(), _camera.getFarClip());

  // Sync lights to the rendering global
  if (_dynamicLights)
    Light::sceneLights = _scene.lights;
  else
    Light::sceneLights.clear();

  // Specular lighting
  specularEnabled = _specular;
  specularShininess = _shininess;
  specularStrength = _specStrength;
  cameraPos = vertex<float>(_camera.getX(), _camera.getY(), _camera.getZ());

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
  // Draw normal arrows before post-processing so they're clearly visible
  if (_showNormals) {
    const auto identity = matrix<4,4>::identity();
    for (const auto& m : _scene.models) {
      // Skip loaded mesh models (only show normals for planes)
      if (m->_shader == shaderType::FlatShader || m->_shader == shaderType::GouraudShader ||
          m->_shader == shaderType::InterpFlatShader || m->_shader == shaderType::InterpGouraudShader)
        continue;

      const auto& faces = m->_baseModel.getFaces();
      if (faces.empty()) continue;

      // Compute centroid from all unique vertices of first face
      const auto& v0 = m->_baseModel.getVertex(faces[0]._v0);
      const auto& v1 = m->_baseModel.getVertex(faces[0]._v1);
      const auto& v2 = m->_baseModel.getVertex(faces[0]._v2);
      // Use last face's third vertex to get the opposite corner of the quad
      const auto& v3 = m->_baseModel.getVertex(faces.back()._v2);
      vertex<float> center((v0._x + v1._x + v2._x + v3._x) * 0.25f,
                           (v0._y + v1._y + v2._y + v3._y) * 0.25f,
                           (v0._z + v1._z + v2._z + v3._z) * 0.25f);

      const auto& norm = m->_baseModel.getVertexNormal(faces[0]._v2);
      vertex<float> tip(center._x + norm._x * 0.5f,
                        center._y + norm._y * 0.5f,
                        center._z + norm._z * 0.5f);

      vertex<int> sBase, sTip;
      vertex<float> rDummy;
      if (!pipelineFast(_renderCameraTransform, identity, center, sBase, rDummy)) continue;
      if (!pipelineFast(_renderCameraTransform, identity, tip, sTip, rDummy)) continue;

      // Arrow shaft
      Overlay::drawLine(sBase._x, sBase._y, sTip._x, sTip._y, 0x00FF00, 3);

      // Arrowhead — two short lines at ±45° from the tip
      int dx = sTip._x - sBase._x, dy = sTip._y - sBase._y;
      float len = sqrtf((float)(dx * dx + dy * dy));
      if (len > 4) {
        float ux = dx / len, uy = dy / len;
        float headLen = len * 0.3f;
        if (headLen > 8) headLen = 8;
        int hx1 = (int)(sTip._x - headLen * (ux + uy));
        int hy1 = (int)(sTip._y - headLen * (uy - ux));
        int hx2 = (int)(sTip._x - headLen * (ux - uy));
        int hy2 = (int)(sTip._y - headLen * (uy + ux));
        Overlay::drawLine(sTip._x, sTip._y, hx1, hy1, 0x00FF00, 3);
        Overlay::drawLine(sTip._x, sTip._y, hx2, hy2, 0x00FF00, 3);
      }
    }
  }

  if (_ssao) applySSAO(_ssaoRadius, _ssaoStrength);
  if (_depthFog) applyDepthFog(_nearClip, _farClip, 0x8090A0, _depthFogNear, _depthFogFar);
  if (_aa) applyAA(_aaThreshold);
  if (_lightFog) applyLightFog(_renderCameraTransform, 0.2f, 80.0f);

  // ── Compact always-visible HUD (top-left) ──────────────────────────────
  constexpr int S      = 2;
  constexpr int CHAR_W = 8 * S;
  constexpr int LINE_H = 8 * S + 2;
  constexpr int PAD    = 6;
  constexpr int MARGIN = 8;

  char buf[64];
  int hudLines = 4; // FPS + X + Y + Z
  int hudW = 20 * CHAR_W + PAD * 2;
  int hudH = hudLines * LINE_H + PAD * 2;

  Overlay::fillRect(MARGIN, MARGIN, hudW, hudH, 0x1A1A2E, 180);
  Overlay::drawRect(MARGIN, MARGIN, hudW, hudH, 0x4A4A6A);

  int tx = MARGIN + PAD;
  int ty = MARGIN + PAD;

  snprintf(buf, sizeof(buf), "FPS: %.1f", _lastFPS);
  Overlay::drawText(tx, ty, buf, 0x55FF55, S); ty += LINE_H;

  snprintf(buf, sizeof(buf), "X:%7.1f Y:%7.1f", _camera.getX(), _camera.getY());
  Overlay::drawText(tx, ty, buf, 0xCCCCCC, S); ty += LINE_H;

  snprintf(buf, sizeof(buf), "Z:%7.1f", _camera.getZ());
  Overlay::drawText(tx, ty, buf, 0xCCCCCC, S); ty += LINE_H;

  if (!_menuStack.isOpen()) {
    Overlay::drawText(tx, ty, "[ESC] Settings", 0x666666, S);
  }

  // Menu overlay (Escape to toggle)
  _menuStack.draw();
}
