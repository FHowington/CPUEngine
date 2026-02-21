#pragma once

#include "light.h"
#include "Window.h"
#include "geometry.h"
#include <array>
#include <cmath>

extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;
extern matrix<4,4> cameraTransform;
extern float focalLength;

bool pipelineFast(const matrix<4,4>& cameraTransform, const matrix<4,4>& model, const vertex<float>& v, vertex<int>& retResult, vertex<float>& realResult);

// Render a volumetric glow/fog around each point light in screen space.
// Projects lights to screen using the same pipeline as the renderer.
// intensity: overall glow strength (0.0-1.0), radius: screen-space glow radius in pixels
inline void applyLightFog(const matrix<4,4>& camTransform, float intensity = 0.35f, float radius = 200.0f) {
  static const matrix<4,4> identity = matrix<4,4>::identity();

  for (const auto& light : Light::sceneLights) {
    if (light._type != LightType::Point) continue;

    vertex<float> lp(light._x, light._y, light._z);
    vertex<int> screenPos;
    vertex<float> worldPos;

    // Use the exact same projection pipeline as the renderer
    if (!pipelineFast(camTransform, identity, lp, screenPos, worldPos))
      continue;

    float sx = (float)screenPos._x;
    float sy = (float)(H - screenPos._y);  // Renderer flips Y when writing pixels
    int lightZ = screenPos._z;

    // Scale radius by depth — closer lights have bigger, more diffuse glow
    // screenPos._z is depth*zCam, larger = closer. Use inverse relationship.
    float camZ = (float)lightZ / (float)depth;
    float distScale = 5.0f / fmaxf(1.0f, -camZ);
    float r = radius * distScale;
    if (r < 10.0f) continue;
    float rSq = r * r;

    int x0 = fast_max(0, (int)(sx - r));
    int x1 = fast_min((int)W - 1, (int)(sx + r));
    int y0 = fast_max(0, (int)(sy - r));
    int y1 = fast_min((int)H - 1, (int)(sy + r));

    float lR = light._R * light._strength * intensity;
    float lG = light._G * light._strength * intensity;
    float lB = light._B * light._strength * intensity;

    for (int y = y0; y <= y1; ++y) {
      float dy = y - sy;
      float dySq = dy * dy;
      int rowOff = y * W;
      for (int x = x0; x <= x1; ++x) {
        float dx = x - sx;
        float dSq = dx * dx + dySq;
        if (dSq >= rSq) continue;

        float t = 1.0f - dSq / rSq;
        float fog = t * t;

        // Block glow if geometry is closer than the light
        int idx = rowOff + x;
        int pz = zbuff[idx];
        if (pz > lightZ + 2000) continue;

        unsigned px = pixels[idx];
        unsigned pr = (px >> 16) & 0xFF;
        unsigned pg = (px >> 8) & 0xFF;
        unsigned pb = px & 0xFF;

        pr = fast_min(255, (int)(pr + fog * lR * 255));
        pg = fast_min(255, (int)(pg + fog * lG * 255));
        pb = fast_min(255, (int)(pb + fog * lB * 255));

        pixels[idx] = (pr << 16) | (pg << 8) | pb;
      }
    }
  }
}
