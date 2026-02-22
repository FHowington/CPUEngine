#include "engine.h"
#include "pool.h"
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
std::array<unsigned, W * H> pixels;
std::array<int, W * H> zbuff;
unsigned rW = W;
unsigned rH = H;
matrix<4,4> cameraTransform;
std::atomic<unsigned> remaining_models;
bool renderWireframe = false;
bool frustumCulling = true;
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
    // Clear framebuffer and z-buffer (only active region, stride is still W)
    for (unsigned y = 0; y < rH; ++y) {
      std::memset(pixels.data() + y * W, 0, rW * sizeof(unsigned));
      std::fill(zbuff.begin() + y * W, zbuff.begin() + y * W + rW, std::numeric_limits<int>::min());
    }

    // Submit all models in the scene for rendering
    const auto& models = game.getModels();
    for (const auto& m : models) {
      if (frustumCulling) {
        // Frustum cull: transform bounding sphere center to camera space
        matrix<4,1> wc(m->_position * v2m(m->_bCenter));
        matrix<4,1> cc(cameraTransform * wc);
        float cz = cc._m[2];
        float r = m->_bRadius;

        // Behind camera or beyond far plane
        if (cz - r > -nearClipDist || cz + r < -farClipDist) continue;

        // Side frustum planes: visible range at depth -cz is ±halfW*(-cz), ±halfH*(-cz)
        float halfW = xFOV * focalLength / xZoom;
        float halfH = yFOV * focalLength / yZoom;
        float extent = -cz; // positive
        float xLimit = halfW * extent + r;
        float yLimit = halfH * extent + r;
        if (cc._m[0] > xLimit || cc._m[0] < -xLimit) continue;
        if (cc._m[1] > yLimit || cc._m[1] < -yLimit) continue;
      }

      ++remaining_models;
      Pool::enqueue_model(m);
    }

    // Wait for all models to finish rendering
    Pool::wait_for_render();

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

    // Present the rendered frame — source rect selects active region
    SDL_Rect src = {0, 0, (int)rW, (int)rH};
    SDL_UpdateTexture(_texture, nullptr, pixels.data(), 4 * W);
    SDL_RenderCopy(_renderer, _texture, &src, nullptr);
    SDL_RenderPresent(_renderer);
  }
}
