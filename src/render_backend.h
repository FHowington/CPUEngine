#pragma once

#include "geometry.h"
#include "loader.h"
#include <memory>
#include <vector>

// Abstract rendering backend.
// The CPU implementation uses the existing thread-pool rasterizer.
// A CUDA implementation would upload geometry and run kernels on the GPU.
class RenderBackend {
 public:
  virtual ~RenderBackend() = default;

  // Clear pixel and depth buffers for the active render region.
  virtual void clearBuffers() = 0;

  // Rasterize models with the current camera transform.
  // reflectionPass: if true, skip water geometry and below-water models.
  virtual void renderModels(const std::vector<std::shared_ptr<ModelInstance>>& models,
                            bool reflectionPass) = 0;

  // Copy current pixel/zbuff into the reflection buffers.
  virtual void snapshotReflection() = 0;

  // Access the output framebuffer (W×H with BUF_SZ padding).
  virtual unsigned* pixelData() = 0;
  virtual int* depthData() = 0;
};
