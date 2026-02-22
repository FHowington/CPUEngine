#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;

// Screen-Space Ambient Occlusion: darkens corners and crevices.
// Compares depth of nearby samples to the center pixel — if a neighbor
// is significantly closer to the camera, the center is in a "crease."
inline void applySSAO(float radius = 6.0f, float strength = 0.6f) {
  // 12 sample offsets at varying distances and angles
  static const float offsets[][2] = {
    {1,0},{-1,0},{0,1},{0,-1},
    {0.7f,0.7f},{-0.7f,0.7f},{0.7f,-0.7f},{-0.7f,-0.7f},
    {0.5f,0.87f},{-0.5f,0.87f},{0.87f,-0.5f},{-0.87f,-0.5f},
  };
  static const int nSamples = 12;

  const int zMin = std::numeric_limits<int>::min();

  // Work on a copy so reads don't see modified pixels
  static std::array<unsigned, W * H> pixCopy;
  std::memcpy(pixCopy.data(), pixels.data(), W * H * sizeof(unsigned));

  // zbuff is non-flipped (row 0 = screen top), pixels is flipped (row 0 = screen bottom)
  for (unsigned y = 1; y < H - 1; ++y) {
    unsigned pxRow = (H - 1 - y) * W;  // pixels row (flipped)

    for (unsigned x = 1; x < W - 1; ++x) {
      int centerZ = zbuff[y * W + x];
      if (centerZ == zMin) continue;

      // Scale radius by depth — further objects get smaller sample radius
      // centerZ is negative, larger (less negative) = closer
      float depthScale = (float)(-centerZ) / (float)(depth * 10);
      if (depthScale < 0.1f) depthScale = 0.1f;
      float r = radius / depthScale;
      if (r < 1.0f) r = 1.0f;
      if (r > 20.0f) r = 20.0f;

      // Threshold: how much closer a sample must be to count as occluding
      // Scale with depth so it works at all distances
      int thresh = (int)(fabsf((float)centerZ) * 0.02f);
      if (thresh < 500) thresh = 500;

      int occluded = 0;
      for (int s = 0; s < nSamples; ++s) {
        int sx = (int)x + (int)(offsets[s][0] * r);
        int sy = (int)y + (int)(offsets[s][1] * r);
        if (sx < 0 || sx >= (int)W || sy < 0 || sy >= (int)H) continue;

        int sampleZ = zbuff[sy * W + sx];
        if (sampleZ == zMin) continue;

        // Neighbor is closer to camera (larger z) by more than threshold
        if (sampleZ - centerZ > thresh) {
          ++occluded;
        }
      }

      if (occluded == 0) continue;

      float ao = 1.0f - strength * (float)occluded / (float)nSamples;
      if (ao < 0.3f) ao = 0.3f;

      unsigned px = pixCopy[pxRow + x];
      unsigned pr = (unsigned)((float)((px >> 16) & 0xFF) * ao);
      unsigned pg = (unsigned)((float)((px >> 8) & 0xFF) * ao);
      unsigned pb = (unsigned)((float)(px & 0xFF) * ao);
      pixels[pxRow + x] = (pr << 16) | (pg << 8) | pb;
    }
  }
}
