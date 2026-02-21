#include "shadow.h"
#include "loader.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

ShadowMap shadowMap;
bool shadowsEnabled = false;

void ShadowMap::clear() {
  std::fill(depth.begin(), depth.end(), std::numeric_limits<float>::max());
}

void ShadowMap::buildFromDirection(const vertex<float>& lightDir, const vertex<float>& sceneCenter) {
  // Light "position" far along the light direction (directional light = parallel rays)
  // lightDir points FROM light TO scene, so light position is sceneCenter - lightDir * distance
  float dist = 40.0f;
  vertex<float> lightPos(sceneCenter._x - lightDir._x * dist,
                         sceneCenter._y - lightDir._y * dist,
                         sceneCenter._z - lightDir._z * dist);

  // Build look-at matrix: light looks toward sceneCenter
  vertex<float> forward = lightDir;  // already normalized
  // Choose an up vector that isn't parallel to forward
  vertex<float> up(0, 1, 0);
  if (std::fabs(forward._y) > 0.99f) up = vertex<float>(0, 0, 1);

  // right = forward x up
  vertex<float> right(forward._y * up._z - forward._z * up._y,
                      forward._z * up._x - forward._x * up._z,
                      forward._x * up._y - forward._y * up._x);
  right.normalize();

  // Recompute up = right x forward
  up = vertex<float>(right._y * forward._z - right._z * forward._y,
                     right._z * forward._x - right._x * forward._z,
                     right._x * forward._y - right._y * forward._x);

  // View matrix (world → light view space)
  matrix<4,4> view;
  view.set(0, 0, right._x);    view.set(0, 1, right._y);    view.set(0, 2, right._z);
  view.set(1, 0, up._x);       view.set(1, 1, up._y);       view.set(1, 2, up._z);
  view.set(2, 0, forward._x);  view.set(2, 1, forward._y);  view.set(2, 2, forward._z);
  view.set(3, 3, 1.0f);
  // Translation: -dot(axis, lightPos)
  view.set(0, 3, -(right._x * lightPos._x + right._y * lightPos._y + right._z * lightPos._z));
  view.set(1, 3, -(up._x * lightPos._x + up._y * lightPos._y + up._z * lightPos._z));
  view.set(2, 3, -(forward._x * lightPos._x + forward._y * lightPos._y + forward._z * lightPos._z));

  // Orthographic projection: maps [-orthoHalf, orthoHalf] x [-orthoHalf, orthoHalf] x [nearPlane, farPlane]
  // to [0, 1] range for x,y and [0,1] for depth
  float h = orthoHalf;
  float n = nearPlane;
  float f = farPlane;
  matrix<4,4> proj;
  proj.set(0, 0, 1.0f / h);
  proj.set(1, 1, 1.0f / h);
  proj.set(2, 2, 1.0f / (f - n));
  proj.set(2, 3, -n / (f - n));
  proj.set(3, 3, 1.0f);

  lightVP = proj * view;
}

// Minimal depth-only rasterizer for shadow pass
static void shadowRasterizeTri(ShadowMap& sm,
                                float x0, float y0, float z0,
                                float x1, float y1, float z1,
                                float x2, float y2, float z2) {
  // Convert from [-1,1] clip space to [0, SM_SIZE] texel coords
  float halfS = SM_SIZE * 0.5f;
  x0 = (x0 + 1.0f) * halfS; y0 = (y0 + 1.0f) * halfS;
  x1 = (x1 + 1.0f) * halfS; y1 = (y1 + 1.0f) * halfS;
  x2 = (x2 + 1.0f) * halfS; y2 = (y2 + 1.0f) * halfS;

  // Bounding box
  int minX = std::max(0, (int)std::min({x0, x1, x2}));
  int minY = std::max(0, (int)std::min({y0, y1, y2}));
  int maxX = std::min((int)SM_SIZE - 1, (int)std::max({x0, x1, x2}));
  int maxY = std::min((int)SM_SIZE - 1, (int)std::max({y0, y1, y2}));

  float area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
  if (std::fabs(area) < 0.001f) return;
  float invArea = 1.0f / area;

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      float px = x + 0.5f, py = y + 0.5f;
      float w0 = ((x1 - x0) * (py - y0) - (px - x0) * (y1 - y0)) * invArea;
      float w1 = ((x2 - x1) * (py - y1) - (px - x1) * (y2 - y1)) * invArea;
      float w2 = 1.0f - w0 - w1;
      // Use the same winding test as the main rasterizer
      if (area > 0) {
        if (w0 < 0 || w1 < 0 || w2 < 0) continue;
      } else {
        if (w0 > 0 || w1 > 0 || w2 > 0) continue;
        w0 = -w0; w1 = -w1; w2 = -w2;
      }
      float depth = z0 * w2 + z1 * w0 + z2 * w1;
      unsigned idx = y * SM_SIZE + x;
      if (depth < sm.depth[idx]) {
        sm.depth[idx] = depth;
      }
    }
  }
}

void ShadowMap::render(const std::vector<std::shared_ptr<ModelInstance>>& models) {
  clear();

  for (const auto& m : models) {
    const auto& faces = m->_baseModel.getFaces();
    for (const auto& f : faces) {
      // Transform each vertex: model → world → light clip
      const auto& mv0 = m->_baseModel.getVertex(f._v0);
      const auto& mv1 = m->_baseModel.getVertex(f._v1);
      const auto& mv2 = m->_baseModel.getVertex(f._v2);

      // Model to world
      matrix<4,1> w0(m->_position * v2m(mv0));
      matrix<4,1> w1(m->_position * v2m(mv1));
      matrix<4,1> w2(m->_position * v2m(mv2));

      // World to light clip
      matrix<4,1> c0(lightVP * w0);
      matrix<4,1> c1(lightVP * w1);
      matrix<4,1> c2(lightVP * w2);

      // Clip: skip triangles entirely outside [0,1] depth range
      if (c0._m[2] < 0 && c1._m[2] < 0 && c2._m[2] < 0) continue;
      if (c0._m[2] > 1 && c1._m[2] > 1 && c2._m[2] > 1) continue;

      shadowRasterizeTri(*this,
                         c0._m[0], c0._m[1], c0._m[2],
                         c1._m[0], c1._m[1], c1._m[2],
                         c2._m[0], c2._m[1], c2._m[2]);
    }
  }
}

float ShadowMap::test(float wx, float wy, float wz) const {
  // Transform world position to light clip space
  matrix<4,1> wv;
  wv._m[0] = wx; wv._m[1] = wy; wv._m[2] = wz; wv._m[3] = 1.0f;
  matrix<4,1> lv(lightVP * wv);

  // Convert to shadow map texel coordinates
  float halfS = SM_SIZE * 0.5f;
  float sx = (lv._m[0] + 1.0f) * halfS;
  float sy = (lv._m[1] + 1.0f) * halfS;
  float sz = lv._m[2];

  int ix = (int)sx;
  int iy = (int)sy;

  // Outside shadow map → lit
  if (ix < 0 || ix >= (int)SM_SIZE || iy < 0 || iy >= (int)SM_SIZE) return 1.0f;

  float mapDepth = depth[iy * SM_SIZE + ix];
  return (sz <= mapDepth + bias) ? 1.0f : 0.0f;
}
