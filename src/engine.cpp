#include "engine.h"
#include "pool.h"
#include "Window.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

// Globals that the rendering pipeline depends on.
// These are referenced via extern by pool.cpp and rasterize.h.
constexpr float SPEED = 80000000.0;
std::array<unsigned, W * H> pixels;
std::array<int, W * H> zbuff;
matrix<4,4> cameraTransform;
std::atomic<unsigned> remaining_models;
bool renderWireframe = false;
float focalLength = 1.5f;  // Default projection focal length (updated by setFOV)

Engine::Engine() {
  remaining_models = 0;
  _window = SDL_CreateWindow("CPUEngine", SDL_WINDOWPOS_UNDEFINED,
                             SDL_WINDOWPOS_UNDEFINED, W * 4, H * 4,
                             SDL_WINDOW_RESIZABLE);
  _renderer = SDL_CreateRenderer(_window, -1, 0);
  _texture = SDL_CreateTexture(_renderer, SDL_PIXELFORMAT_ARGB8888,
                               SDL_TEXTUREACCESS_STREAMING, W, H);
  _pool = std::make_unique<Pool>(std::thread::hardware_concurrency());
}

Engine::~Engine() {
  _pool.reset();
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

void Engine::setFOV(float degrees) {
  // Relationship: tan(hfov/2) = (1-xFOV) * focalLength / xZoom
  // Solving: focalLength = tan(hfov/2) * xZoom / (1-xFOV)
  float radians = degrees * (float)M_PI / 180.0f;
  focalLength = tanf(radians * 0.5f) * (float)xZoom / (1.0f - xFOV);
}

extern float nearClipDist;
extern float farClipDist;

void Engine::setClipDistances(float near, float far) {
  nearClipDist = near;
  farClipDist = far;
}

void Engine::run(Game& game) {
  game.init(*this);

  auto lastFrame = std::chrono::high_resolution_clock::now();

  for (bool quit = false; !quit;) {
    // Clear framebuffer and z-buffer
    for (auto& p : pixels) { p = 0; }
    for (auto& p : zbuff)  { p = std::numeric_limits<int>::min(); }

    // Submit all models in the scene for rendering
    const auto& models = game.getModels();
    for (const auto& m : models) {
      ++remaining_models;
      Pool::enqueue_model(m);
    }

    // TODO(forbes): Change this to something..better. A conditional perhaps.
    while (remaining_models != 0U) { ; }

    // Save the camera transform that was actually used for this frame's rendering
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

    // Let the game update state for the next frame
    game.update(deltaTime, *this);

    // Draw any 2D overlay on top of the finished 3D frame
    game.drawOverlay();

    // Present the rendered frame
    SDL_UpdateTexture(_texture, nullptr, pixels.data(), 4 * W);
    SDL_RenderCopy(_renderer, _texture, nullptr, nullptr);
    SDL_RenderPresent(_renderer);
  }
}
