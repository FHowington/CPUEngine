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

// Camera transform snapshot from the frame that was actually rendered.
// Must be captured before game.update() overwrites cameraTransform.
inline matrix<4,4> fogCameraTransform;

// Call once per frame after rendering but before game.update() overwrites cameraTransform.
inline void captureFogCamera() { fogCameraTransform = cameraTransform; }

// Render a volumetric glow/fog around each point light in screen space.
// Projects lights to screen, then applies radial color falloff.
// intensity: overall glow strength (0.0-1.0), radius: screen-space glow radius in pixels
inline void applyLightFog(float intensity = 0.35f, float radius = 120.0f) {
  for (const auto& light : Light::sceneLights) {
    if (light._type != LightType::Point) continue;

    // Transform light world position into camera space
    vertex<float> lp(light._x, light._y, light._z);
    matrix<4,1> cam = fogCameraTransform * v2m(lp);
    float cz = cam._m[2];
    if (cz >= -0.5f) continue;  // behind camera

    // Project to screen
    float scale = 1.0f / (-focalLength * cz);
    float sx = cam._m[0] * scale * W * xZoom + W * xFOV;
    float sy = cam._m[1] * scale * H * yZoom + H * yFOV;

    // Zbuffer value this light would have
    int lightZ = (int)(cz * depth);

    // Scale radius by distance — closer lights have bigger, more diffuse glow
    float distScale = 5.0f / (-cz);
    float r = radius * distScale;
    if (r < 10.0f) continue;
    float rSq = r * r;

    // Bounding box on screen
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

        // Smooth falloff: (1 - d²/r²)²
        float t = 1.0f - dSq / rSq;
        float fog = t * t;

        // Depth occlusion: if geometry is closer than the light, block the glow
        int idx = rowOff + x;
        int pz = zbuff[idx];
        if (pz > lightZ + 2000) {
          fog = 0;
          continue;
        }

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
