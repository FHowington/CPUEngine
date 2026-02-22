//
// Engine / Game interface — separates renderer from game logic
//

#pragma once

#include "geometry.h"
#include "loader.h"
#include <SDL.h>
#include <memory>
#include <vector>

class Pool;
class Engine;

class Game {
 public:
  virtual ~Game() = default;
  virtual void init(Engine& engine) = 0;
  virtual void handleEvent(const SDL_Event& event, bool& quit) = 0;
  virtual void update(float deltaTime, Engine& engine) = 0;
  virtual const std::vector<std::shared_ptr<ModelInstance>>& getModels() const = 0;

  // Optional: draw 2D overlay into the pixel framebuffer after 3D rendering.
  // Called after all models are rendered, before the frame is presented.
  virtual void drawOverlay() {}
};

class Engine {
 public:
  Engine();
  ~Engine();

  void run(Game& game);
  void setCameraTransform(const matrix<4,4>& transform);
  void setWireframeMode(bool enabled);
  void setFrustumCulling(bool enabled);
  void setFOV(float degrees);  // Sets projection focalLength from FOV angle
  void setClipDistances(float near, float far);
  void setResolution(unsigned w, unsigned h);
  const matrix<4,4>& getRenderCameraTransform() const { return _renderCameraTransform; }

 private:
  SDL_Window* _window;
  SDL_Renderer* _renderer;
  SDL_Texture* _texture;
  std::unique_ptr<Pool> _pool;
  matrix<4,4> _renderCameraTransform;
};
