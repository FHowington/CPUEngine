#pragma once

#include "render_backend.h"
#include "Window.h"
#include <array>
#include <atomic>

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

class MetalBackend : public RenderBackend {
 public:
  MetalBackend();
  ~MetalBackend() override;

  void clearBuffers() override;
  void renderModels(const std::vector<std::shared_ptr<ModelInstance>>& models,
                    bool reflectionPass) override;
  void snapshotReflection() override;
  unsigned* pixelData() override;
  int* depthData() override;

 private:
  struct Impl;
  Impl* _impl;
};
