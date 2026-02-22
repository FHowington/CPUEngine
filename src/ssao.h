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
inline void applySSAO(float radius = 12.0f, float strength = 0.6f) {
  static const float offsets[][2] = {
    {1,0},{-1,0},{0,1},{0,-1},
    {0.7f,0.7f},{-0.7f,0.7f},{0.7f,-0.7f},{-0.7f,-0.7f},
    {0.5f,0.87f},{-0.5f,0.87f},{0.87f,-0.5f},{-0.87f,-0.5f},
  };
  static const int nSamples = 12;
  const int zMin = std::numeric_limits<int>::min();

  static std::array<unsigned, W * H> pixCopy;
  std::memcpy(pixCopy.data(), pixels.data(), W * H * sizeof(unsigned));

  // zbuff: non-flipped (row 0 = screen top), pixels: flipped (row 0 = screen bottom)
  for (unsigned y = 0; y < H; ++y) {
    unsigned pxRow = (H - 1 - y) * W;

    for (unsigned x = 0; x < W; ++x) {
      int centerZ = zbuff[y * W + x];
      if (centerZ == zMin) continue;

      // Threshold scales with depth — 0.5% of |centerZ|, min 100
      int thresh = (int)(fabsf((float)centerZ) * 0.005f);
      if (thresh < 100) thresh = 100;

      int occluded = 0;
      for (int s = 0; s < nSamples; ++s) {
        int sx = (int)x + (int)(offsets[s][0] * radius);
        int sy = (int)y + (int)(offsets[s][1] * radius);
        if (sx < 0 || sx >= (int)W || sy < 0 || sy >= (int)H) continue;

        int sampleZ = zbuff[sy * W + sx];
        if (sampleZ == zMin) continue;

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
