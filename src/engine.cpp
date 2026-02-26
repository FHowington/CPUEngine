#include "engine.h"
#include "cpu_backend.h"
#ifdef USE_METAL
#include "metal_backend.h"
#endif
#include "shader.h"
#include "water_reflect.h"
#include "Window.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <thread>

// Globals that the rendering pipeline depends on.
// These are referenced via extern by pool.cpp and rasterize.h.
constexpr float SPEED = 80000000.0;
std::array<unsigned, BUF_SZ> pixels;
std::array<int, BUF_SZ> zbuff;
std::array<unsigned, BUF_SZ> reflectionBuf;
std::array<int, BUF_SZ> reflectionZBuf;
unsigned rW = W;
unsigned rH = H;
matrix<4,4> cameraTransform;
std::atomic<unsigned> remaining_models;
bool renderWireframe = false;
bool frustumCulling = true;
float focalLength = 1.5f;  // Default projection focal length (updated by setFOV)
float globalTime = 0.0f;

Engine::Engine() {
  remaining_models = 0;
  _window = SDL_CreateWindow("CPUEngine", SDL_WINDOWPOS_UNDEFINED,
                             SDL_WINDOWPOS_UNDEFINED, W * 4, H * 4,
                             SDL_WINDOW_RESIZABLE);
  _renderer = SDL_CreateRenderer(_window, -1, 0);
  _texture = SDL_CreateTexture(_renderer, SDL_PIXELFORMAT_ARGB8888,
                               SDL_TEXTUREACCESS_STREAMING, W, H);
  _backend = std::make_unique<CPUBackend>();
#ifdef USE_METAL
  _backend = std::make_unique<MetalBackend>();
#endif
}

Engine::~Engine() {
  _backend.reset();
  if (_texture)  SDL_DestroyTexture(_texture);
  if (_renderer) SDL_DestroyRenderer(_renderer);
  if (_window)   SDL_DestroyWindow(_window);
}

void Engine::setCameraTransform(const matrix<4,4>& transform) {
  cameraTransform = transform;
}

void Engine::setWireframeMode(bool enabled) {
  renderWireframe = enabled;
}

void Engine::setFrustumCulling(bool enabled) {
  frustumCulling = enabled;
}

void Engine::setFOV(float degrees) {
  float radians = degrees * (float)M_PI / 180.0f;
  focalLength = tanf(radians * 0.5f) * (float)xZoom / (1.0f - xFOV);
}

extern float nearClipDist;
extern float farClipDist;

void Engine::setClipDistances(float near, float far) {
  nearClipDist = near;
  farClipDist = far;
}

void Engine::setResolution(unsigned w, unsigned h) {
  if (w > W) w = W;
  if (h > H) h = H;
  rW = w;
  rH = h;
}

void Engine::run(Game& game) {
  game.init(*this);

  auto lastFrame = std::chrono::high_resolution_clock::now();

  for (bool quit = false; !quit;) {
    const auto& models = game.getModels();

    // === Reflection pass: render scene with camera mirrored across water plane ===
    if (game.needsReflectionPass()) {
      _backend->clearBuffers();
      matrix<4,4> savedCam = cameraTransform;
      matrix<4,4> reflectM = matrix<4,4>::identity();
      reflectM.set(1, 1, -1.0f);
      reflectM.set(3, 1, 2.0f * WATER_Y);
      cameraTransform = reflectM * savedCam;
      _reflectionCameraTransform = cameraTransform;
      _backend->renderModels(models, true);
      _backend->snapshotReflection();
      cameraTransform = savedCam;
    }

    // === Main pass: render scene normally ===
    _backend->clearBuffers();
    _backend->renderModels(models, false);

    _renderCameraTransform = cameraTransform;

    // Process SDL events
    SDL_Event ev;
    while (SDL_PollEvent(&ev) != 0) {
      if (ev.type == SDL_QUIT) {
        quit = true;
      }
      game.handleEvent(ev, quit);
    }

    // Compute frame delta (raw nanosecond count as float, matching original SPEED divisor)
    auto now = std::chrono::high_resolution_clock::now();
    auto d = now - lastFrame;
    lastFrame = now;
    float deltaTime = static_cast<float>(d.count()) / SPEED;
    globalTime += static_cast<float>(d.count()) / 1e9f;

    // Let the game update state for the next frame
    game.update(deltaTime, *this);

    // Post-process the 3D framebuffer at rW×rH (before upscale)
    game.postProcess();

    // Upscale 3D framebuffer from rW×rH to W×H (nearest-neighbor) so
    // the overlay can draw at full resolution and always look the same.
    if (rW < W || rH < H) {
      // Work bottom-up so we don't overwrite source rows we still need
      for (int dy = (int)H - 1; dy >= 0; --dy) {
        const unsigned sy = dy * rH / H;
        const unsigned* srcRow = pixels.data() + sy * W;
        unsigned* dstRow = pixels.data() + dy * W;
        for (int dx = (int)W - 1; dx >= 0; --dx)
          dstRow[dx] = srcRow[dx * rW / W];
      }
    }

    // Draw any 2D overlay on top of the finished 3D frame
    game.drawOverlay();

    // Present the full-resolution frame
    SDL_UpdateTexture(_texture, nullptr, pixels.data(), 4 * W);
    SDL_RenderCopy(_renderer, _texture, nullptr, nullptr);
    SDL_RenderPresent(_renderer);
  }
}
