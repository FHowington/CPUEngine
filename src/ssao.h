#pragma once

#include "Window.h"
#include "simd_compat.h"
#include <array>
#include <cmath>

extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;

// Screen-Space Ambient Occlusion: darkens corners and crevices
// by sampling the z-buffer around each pixel.
inline void applySSAO(float radius = 3.0f, float strength = 0.5f, int samples = 8) {
  // Pre-compute sample offsets in a circle
  float offsets[16][2];
  int nSamples = samples < 16 ? samples : 16;
  for (int i = 0; i < nSamples; ++i) {
    float angle = (float)i / (float)nSamples * 2.0f * (float)M_PI;
    offsets[i][0] = cosf(angle) * radius;
    offsets[i][1] = sinf(angle) * radius;
  }

  // Work on a copy so reads don't see modified pixels
  static std::array<unsigned, W * H> pixelsCopy;
  std::memcpy(pixelsCopy.data(), pixels.data(), W * H * sizeof(unsigned));

  const int zMin = std::numeric_limits<int>::min();

  for (unsigned y = 0; y < H; ++y) {
    unsigned rowOff = y * W;
    // zbuff row: zbuff is stored with y=0 at screen top
    unsigned zRow = y * W;

    for (unsigned x = 0; x < W; ++x) {
      int centerZ = zbuff[zRow + x];
      if (centerZ == zMin) continue; // sky pixel

      int occluded = 0;
      for (int s = 0; s < nSamples; ++s) {
        int sx = (int)x + (int)offsets[s][0];
        int sy = (int)y + (int)offsets[s][1];
        if (sx < 0 || sx >= (int)W || sy < 0 || sy >= (int)H) {
          continue;
        }
        int sampleZ = zbuff[sy * W + sx];
        // If sample is closer to camera (larger z), this pixel is occluded
        if (sampleZ > centerZ + 200) {
          ++occluded;
        }
      }

      if (occluded == 0) continue;

      float ao = 1.0f - strength * (float)occluded / (float)nSamples;
      if (ao >= 1.0f) continue;
      if (ao < 0.2f) ao = 0.2f;

      unsigned px = pixelsCopy[rowOff + x];
      unsigned r = (unsigned)((float)((px >> 16) & 0xFF) * ao);
      unsigned g = (unsigned)((float)((px >> 8) & 0xFF) * ao);
      unsigned b = (unsigned)((float)(px & 0xFF) * ao);
      pixels[rowOff + x] = (r << 16) | (g << 8) | b;
    }
  }
}
