#pragma once

#include <algorithm>
#include <vector>

struct CollisionBox {
  float minX, minY, minZ;
  float maxX, maxY, maxZ;
};

class CollisionWorld {
 public:
  void addBox(float x0, float y0, float z0, float x1, float y1, float z1) {
    _boxes.push_back({std::min(x0,x1), std::min(y0,y1), std::min(z0,z1),
                      std::max(x0,x1), std::max(y0,y1), std::max(z0,z1)});
  }

  // Test a player AABB (center + half-extents) against all boxes.
  // Returns corrected position after sliding.
  static constexpr float SKIN = 0.001f;

  void resolveMove(float& x, float& y, float& z,
                   float hx, float hy, float hz) const {
    for (int iter = 0; iter < 4; ++iter) {
      bool resolved = false;
      for (const auto& b : _boxes) {
        if (!overlaps(x, y, z, hx, hy, hz, b)) continue;

        float penXp = (x + hx) - b.minX;
        float penXn = b.maxX - (x - hx);
        float penYp = (y + hy) - b.minY;
        float penYn = b.maxY - (y - hy);
        float penZp = (z + hz) - b.minZ;
        float penZn = b.maxZ - (z - hz);

        float minPen = penXp;
        int axis = 0; int dir = -1;
        if (penXn < minPen) { minPen = penXn; axis = 0; dir = 1; }
        if (penYp < minPen) { minPen = penYp; axis = 1; dir = -1; }
        if (penYn < minPen) { minPen = penYn; axis = 1; dir = 1; }
        if (penZp < minPen) { minPen = penZp; axis = 2; dir = -1; }
        if (penZn < minPen) { minPen = penZn; axis = 2; dir = 1; }

        float push = minPen + SKIN;
        if (axis == 0) x += dir * push;
        else if (axis == 1) y += dir * push;
        else z += dir * push;
        resolved = true;
      }
      if (!resolved) break;
    }
  }

  void clear() { _boxes.clear(); }
  const std::vector<CollisionBox>& boxes() const { return _boxes; }

 private:
  std::vector<CollisionBox> _boxes;

  static bool overlaps(float x, float y, float z,
                       float hx, float hy, float hz,
                       const CollisionBox& b) {
    return (x + hx > b.minX && x - hx < b.maxX &&
            y + hy > b.minY && y - hy < b.maxY &&
            z + hz > b.minZ && z - hz < b.maxZ);
  }
};
