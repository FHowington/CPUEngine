#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>
#include <cstring>

extern std::array<unsigned, W * H> pixels;

// Lightweight post-process anti-aliasing.
// Detects edges via luminance contrast with neighbors, then blends
// the pixel with its 4 direct neighbors weighted by edge strength.
// threshold: luminance difference (0-255) to consider an edge.
//            Lower = more smoothing, higher = sharper.

inline void applyAA(float threshold = 24.0f) {
  // Work on a copy so reads don't see already-blended values
  static std::array<unsigned, W * H> copy;
  std::memcpy(copy.data(), pixels.data(), W * H * sizeof(unsigned));

  const float invThresh = 1.0f / threshold;

  for (unsigned y = 1; y < H - 1; ++y) {
    for (unsigned x = 1; x < W - 1; ++x) {
      const unsigned idx = y * W + x;
      const unsigned c = copy[idx];

      // Luminance approximation: (R*2 + G*5 + B) >> 3
      const int cR = (c >> 16) & 0xFF;
      const int cG = (c >> 8) & 0xFF;
      const int cB = c & 0xFF;
      const int cL = (cR * 2 + cG * 5 + cB) >> 3;

      // Sample 4 neighbors
      const unsigned n0 = copy[idx - W]; // up
      const unsigned n1 = copy[idx + W]; // down
      const unsigned n2 = copy[idx - 1]; // left
      const unsigned n3 = copy[idx + 1]; // right

      auto lum = [](unsigned p) -> int {
        return (((p >> 16) & 0xFF) * 2 + ((p >> 8) & 0xFF) * 5 + (p & 0xFF)) >> 3;
      };

      // Max luminance difference with any neighbor
      int d0 = std::abs(cL - lum(n0));
      int d1 = std::abs(cL - lum(n1));
      int d2 = std::abs(cL - lum(n2));
      int d3 = std::abs(cL - lum(n3));
      int maxD = d0;
      if (d1 > maxD) maxD = d1;
      if (d2 > maxD) maxD = d2;
      if (d3 > maxD) maxD = d3;

      if (maxD < 4) continue; // not an edge, skip

      // Blend strength: 0 at threshold, 0.5 at max contrast
      float blend = (float)maxD * invThresh;
      if (blend > 1.0f) blend = 1.0f;
      blend *= 0.5f; // max 50% neighbor contribution

      // Weighted average of neighbors (equal weight)
      float nR = (float)(((n0>>16)&0xFF) + ((n1>>16)&0xFF) + ((n2>>16)&0xFF) + ((n3>>16)&0xFF)) * 0.25f;
      float nG = (float)(((n0>>8)&0xFF) + ((n1>>8)&0xFF) + ((n2>>8)&0xFF) + ((n3>>8)&0xFF)) * 0.25f;
      float nB = (float)((n0&0xFF) + (n1&0xFF) + (n2&0xFF) + (n3&0xFF)) * 0.25f;

      float inv = 1.0f - blend;
      unsigned r = (unsigned)(cR * inv + nR * blend);
      unsigned g = (unsigned)(cG * inv + nG * blend);
      unsigned b = (unsigned)(cB * inv + nB * blend);

      pixels[idx] = (r << 16) | (g << 8) | b;
    }
  }
}
