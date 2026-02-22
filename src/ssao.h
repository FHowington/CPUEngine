#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;

// Screen-Space Ambient Occlusion: darkens corners and creases.
// Uses opposite-pair sampling: if BOTH sides of a pixel have closer
// geometry, the pixel is in a concavity (corner/crease).
inline void applySSAO(float radius = 8.0f, float strength = 0.7f) {
  const int zMin = std::numeric_limits<int>::min();

  static std::array<unsigned, W * H> pixCopy;
  std::memcpy(pixCopy.data(), pixels.data(), W * H * sizeof(unsigned));

  // 6 opposite pairs at varying angles
  static const float pairs[][4] = {
    { 1, 0, -1,  0},   // horizontal
    { 0, 1,  0, -1},   // vertical
    { 1, 1, -1, -1},   // diagonal
    { 1,-1, -1,  1},   // anti-diagonal
    { 2, 1, -2, -1},   // wide angle
    { 1, 2, -1, -2},   // wide angle 2
  };
  static const int nPairs = 6;

  for (unsigned y = 0; y < H; ++y) {
    unsigned pxRow = (H - 1 - y) * W;

    for (unsigned x = 0; x < W; ++x) {
      int centerZ = zbuff[y * W + x];
      if (centerZ == zMin) continue;

      // Threshold: how much closer a neighbor must be to count
      int thresh = (int)(fabsf((float)centerZ) * 0.003f);
      if (thresh < 50) thresh = 50;

      int occPairs = 0;
      for (int p = 0; p < nPairs; ++p) {
        int ax = (int)x + (int)(pairs[p][0] * radius);
        int ay = (int)y + (int)(pairs[p][1] * radius);
        int bx = (int)x + (int)(pairs[p][2] * radius);
        int by = (int)y + (int)(pairs[p][3] * radius);

        // Clamp to screen
        if (ax < 0 || ax >= (int)W || ay < 0 || ay >= (int)H) continue;
        if (bx < 0 || bx >= (int)W || by < 0 || by >= (int)H) continue;

        int zA = zbuff[ay * W + ax];
        int zB = zbuff[by * W + bx];
        if (zA == zMin || zB == zMin) continue;

        // Both sides of the pair are closer than center → concavity
        if (zA - centerZ > thresh && zB - centerZ > thresh) {
          ++occPairs;
        }
      }

      if (occPairs == 0) continue;

      float ao = 1.0f - strength * (float)occPairs / (float)nPairs;
      if (ao < 0.25f) ao = 0.25f;

      unsigned px = pixCopy[pxRow + x];
      unsigned pr = (unsigned)((float)((px >> 16) & 0xFF) * ao);
      unsigned pg = (unsigned)((float)((px >> 8) & 0xFF) * ao);
      unsigned pb = (unsigned)((float)(px & 0xFF) * ao);
      pixels[pxRow + x] = (pr << 16) | (pg << 8) | pb;
    }
  }
}
