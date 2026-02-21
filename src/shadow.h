#pragma once

#include "geometry.h"
#include <array>
#include <vector>
#include <memory>

struct ModelInstance;

// Shadow map resolution (power of 2 for fast modulo)
constexpr unsigned SM_SIZE = 512;

struct ShadowMap {
  // Depth buffer from light's perspective (stores closest depth per texel)
  std::array<float, SM_SIZE * SM_SIZE> depth;

  // Light-space view-projection matrix (world → light clip space)
  matrix<4,4> lightVP;

  // Inverse: maps light clip → world (for debug)
  float bias = 0.05f;

  // Ortho projection bounds (world units)
  float orthoHalf = 35.0f;
  float nearPlane = 0.1f;
  float farPlane  = 80.0f;

  void clear();

  // Build the light view-projection from a directional light direction
  void buildFromDirection(const vertex<float>& lightDir, const vertex<float>& sceneCenter);

  // Rasterize all models into the shadow depth buffer
  void render(const std::vector<std::shared_ptr<ModelInstance>>& models);

  // Test whether a world-space point is in shadow. Returns 0.0 (fully shadowed) or 1.0 (lit).
  float test(float wx, float wy, float wz) const;
};

// Global shadow map instance
extern ShadowMap shadowMap;
extern bool shadowsEnabled;
