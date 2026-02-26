#pragma once

#include "render_backend.h"
#include "pool.h"
#include "shader.h"
#include "Window.h"
#include "water_reflect.h"
#include <array>
#include <atomic>
#include <cstring>
#include <limits>
#include <memory>
#include <thread>

extern std::array<unsigned, BUF_SZ> pixels;
extern std::array<int, BUF_SZ> zbuff;
extern std::array<unsigned, BUF_SZ> reflectionBuf;
extern std::array<int, BUF_SZ> reflectionZBuf;
extern matrix<4,4> cameraTransform;
extern std::atomic<unsigned> remaining_models;
extern bool frustumCulling;
extern float focalLength;
extern float nearClipDist;
extern float farClipDist;

class CPUBackend : public RenderBackend {
 public:
  CPUBackend()
    : _pool(std::make_unique<Pool>(std::thread::hardware_concurrency())) {}
  ~CPUBackend() override = default;

  void clearBuffers() override {
    for (unsigned y = 0; y < rH; ++y) {
      std::memset(pixels.data() + y * W, 0, rW * sizeof(unsigned));
      std::fill(zbuff.begin() + y * W, zbuff.begin() + y * W + rW, std::numeric_limits<int>::min());
    }
  }

  void renderModels(const std::vector<std::shared_ptr<ModelInstance>>& models,
                    bool reflectionPass) override {
    for (const auto& m : models) {
      if (reflectionPass) {
        if (m->_shader == shaderType::WaterXZShader) continue;
        float worldY = m->_position.at(3, 1) + m->_bCenter._y;
        if (worldY < WATER_Y + 2.0f) continue;
      }
      if (frustumCulling) {
        matrix<4,1> wc(m->_position * v2m(m->_bCenter));
        matrix<4,1> cc(cameraTransform * wc);
        float cz = cc._m[2];
        float r = m->_bRadius;
        if (cz - r > -nearClipDist || cz + r < -farClipDist) continue;
        float hw = xFOV * focalLength / xZoom;
        float hh = yFOV * focalLength / yZoom;
        float extent = -cz;
        float xLimit = hw * extent + r;
        float yLimit = hh * extent + r;
        if (cc._m[0] > xLimit || cc._m[0] < -xLimit) continue;
        if (cc._m[1] > yLimit || cc._m[1] < -yLimit) continue;
      }
      ++remaining_models;
      Pool::enqueue_model(m);
    }
    Pool::wait_for_render();
  }

  void snapshotReflection() override {
    std::memcpy(reflectionBuf.data(), pixels.data(), rH * W * sizeof(unsigned));
    std::memcpy(reflectionZBuf.data(), zbuff.data(), rH * W * sizeof(int));
  }

  unsigned* pixelData() override { return pixels.data(); }
  int* depthData() override { return zbuff.data(); }

 private:
  std::unique_ptr<Pool> _pool;
};
