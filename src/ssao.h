#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;

// Crease darkening: darkens pixels near depth discontinuities on the
// recessed (further) side. At a wall-floor junction, the floor pixels
// near the wall get darkened because the wall is closer.
inline void applySSAO(float radius = 10.0f, float strength = 0.5f) {
  const int zMin = std::numeric_limits<int>::min();

  // Compute per-pixel occlusion factor into a float buffer
  static std::array<float, W * H> aoMap;
  std::fill(aoMap.begin(), aoMap.end(), 1.0f);

  const int R = (int)radius;

  for (unsigned y = 0; y < H; ++y) {
    for (unsigned x = 0; x < W; ++x) {
      int centerZ = zbuff[y * W + x];
      if (centerZ == zMin) continue;

      // Count how many nearby pixels are significantly closer (larger z)
      // Only sample in 4 cardinal + 4 diagonal directions at the radius distance
      int closerCount = 0;
      int totalSamples = 0;
      float totalWeight = 0.0f;

      for (int dy = -R; dy <= R; dy += R) {
        for (int dx = -R; dx <= R; dx += R) {
          if (dx == 0 && dy == 0) continue;
          int sx = (int)x + dx;
          int sy = (int)y + dy;
          if (sx < 0 || sx >= (int)W || sy < 0 || sy >= (int)H) continue;

          int sZ = zbuff[sy * W + sx];
          if (sZ == zMin) continue;
          totalSamples++;

          int diff = sZ - centerZ;
          if (diff > 0) {
            // Neighbor is closer — weight by how much closer
            float w = (float)diff / (float)(-centerZ) * 100.0f;
            if (w > 1.0f) w = 1.0f;
            totalWeight += w;
            closerCount++;
          }
        }
      }

      if (closerCount == 0 || totalSamples == 0) continue;

      // Only darken if at least 1 neighbor is closer (we're on the recessed side)
      // Scale by fraction of closer neighbors and their weight
      float frac = totalWeight / (float)totalSamples;
      float ao = 1.0f - strength * frac;
      if (ao < 0.3f) ao = 0.3f;
      if (ao >= 1.0f) continue;

      aoMap[y * W + x] = ao;
    }
  }

  // Apply AO to pixels (pixels are Y-flipped relative to zbuff)
  for (unsigned y = 0; y < H; ++y) {
    unsigned pxRow = (H - 1 - y) * W;
    for (unsigned x = 0; x < W; ++x) {
      float ao = aoMap[y * W + x];
      if (ao >= 1.0f) continue;

      unsigned px = pixels[pxRow + x];
      unsigned pr = (unsigned)((float)((px >> 16) & 0xFF) * ao);
      unsigned pg = (unsigned)((float)((px >> 8) & 0xFF) * ao);
      unsigned pb = (unsigned)((float)(px & 0xFF) * ao);
      pixels[pxRow + x] = (pr << 16) | (pg << 8) | pb;
    }
  }
}
