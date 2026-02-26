#import "metal_backend.h"
#import "light.h"
#import "shader.h"
#import "water_reflect.h"
#import <Metal/Metal.h>
#import <Foundation/Foundation.h>
#import <cstring>
#import <limits>
#import <vector>

// Must match shaders.metal
struct DeviceFace {
  uint32_t v0, v1, v2;
  int32_t t0x, t0y, t1x, t1y, t2x, t2y;
};

struct ScreenTri {
  int32_t sx0, sy0, sz0;
  int32_t sx1, sy1, sz1;
  int32_t sx2, sy2, sz2;
  float wx0, wy0, wz0;
  float wx1, wy1, wz1;
  float wx2, wy2, wz2;
  float t0x, t0y;
  float t1x, t1y;
  float t2x, t2y;
  int32_t shaderType;
  float globalIllum;
  int32_t textureIdx;
  int32_t texW, texH;
  int32_t doubleSided;
};

struct DeviceLight {
  int32_t type;
  float strength;
  float r, g, b;
  float dx, dy, dz;
};

struct VertexUniforms {
  float model[16];
  float cam[16];
  float focalLength;
  float nearClip;
  float farClip;
  uint32_t rW;
  uint32_t rH;
  int32_t shaderType;
  float globalIllum;
  int32_t doubleSided;
  int32_t textureIdx;
  int32_t texW;
  int32_t texH;
  int32_t faceCount;
};

struct RasterUniforms {
  uint32_t rW;
  uint32_t rH;
  uint32_t pitch;
  uint32_t tilesX;
  uint32_t tilesY;
  int32_t numLights;
};

static constexpr int TILE_SIZE = 32;
static constexpr int MAX_TRIS_PER_TILE = 1024;
static constexpr int MAX_SCREEN_TRIS = 500000;

struct MetalBackend::Impl {
  id<MTLDevice> device;
  id<MTLCommandQueue> queue;
  id<MTLLibrary> library;

  // Pipelines
  id<MTLComputePipelineState> clearPipeline;
  id<MTLComputePipelineState> vertexPipeline;
  id<MTLComputePipelineState> binPipeline;
  id<MTLComputePipelineState> rasterPipeline;

  // Persistent buffers
  id<MTLBuffer> pixelBuf;
  id<MTLBuffer> zbuffBuf;
  id<MTLBuffer> screenTriBuf;
  id<MTLBuffer> triCountBuf;
  id<MTLBuffer> tileCountsBuf;
  id<MTLBuffer> tileTrisBuf;
  id<MTLBuffer> lightBuf;

  // Per-model upload buffers (resized as needed)
  id<MTLBuffer> vertBuf;
  id<MTLBuffer> faceBuf;

  unsigned tilesX, tilesY;
};

static id<MTLComputePipelineState> makePipeline(id<MTLDevice> dev, id<MTLLibrary> lib, NSString* name) {
  id<MTLFunction> fn = [lib newFunctionWithName:name];
  if (!fn) {
    fprintf(stderr, "Metal: function '%s' not found\n", [name UTF8String]);
    return nil;
  }
  NSError* err = nil;
  id<MTLComputePipelineState> ps = [dev newComputePipelineStateWithFunction:fn error:&err];
  if (err) fprintf(stderr, "Metal pipeline error: %s\n", [[err localizedDescription] UTF8String]);
  return ps;
}

MetalBackend::MetalBackend() {
  _impl = new Impl();
  auto& m = *_impl;

  m.device = MTLCreateSystemDefaultDevice();
  m.queue = [m.device newCommandQueue];

  // Load shader source at runtime and compile
  NSError* err = nil;
  NSString* execPath = [NSString stringWithUTF8String:[[[NSProcessInfo processInfo] arguments][0] UTF8String]];
  NSString* execDir = [execPath stringByDeletingLastPathComponent];
  NSString* shaderPath = [execDir stringByAppendingPathComponent:@"shaders.metal"];
  NSString* shaderSrc = [NSString stringWithContentsOfFile:shaderPath encoding:NSUTF8StringEncoding error:&err];
  if (!shaderSrc) {
    fprintf(stderr, "Metal: failed to load shaders.metal: %s\n", [[err localizedDescription] UTF8String]);
    return;
  }
  MTLCompileOptions* opts = [[MTLCompileOptions alloc] init];
  m.library = [m.device newLibraryWithSource:shaderSrc options:opts error:&err];
  if (err) fprintf(stderr, "Metal shader compile: %s\n", [[err localizedDescription] UTF8String]);

  m.clearPipeline = makePipeline(m.device, m.library, @"clearBuffers");
  m.vertexPipeline = makePipeline(m.device, m.library, @"vertexTransform");
  m.binPipeline = makePipeline(m.device, m.library, @"binTriangles");
  m.rasterPipeline = makePipeline(m.device, m.library, @"rasterizeTiles");

  m.tilesX = (rW + TILE_SIZE - 1) / TILE_SIZE;
  m.tilesY = (rH + TILE_SIZE - 1) / TILE_SIZE;

  // Allocate persistent GPU buffers
  m.pixelBuf = [m.device newBufferWithLength:W * H * sizeof(unsigned) options:MTLResourceStorageModeShared];
  m.zbuffBuf = [m.device newBufferWithLength:W * H * sizeof(int) options:MTLResourceStorageModeShared];
  m.screenTriBuf = [m.device newBufferWithLength:MAX_SCREEN_TRIS * sizeof(ScreenTri) options:MTLResourceStorageModeShared];
  m.triCountBuf = [m.device newBufferWithLength:sizeof(int32_t) options:MTLResourceStorageModeShared];
  m.tileCountsBuf = [m.device newBufferWithLength:m.tilesX * m.tilesY * sizeof(int32_t) options:MTLResourceStorageModeShared];
  m.tileTrisBuf = [m.device newBufferWithLength:m.tilesX * m.tilesY * MAX_TRIS_PER_TILE * sizeof(int32_t) options:MTLResourceStorageModeShared];
  m.lightBuf = [m.device newBufferWithLength:32 * sizeof(DeviceLight) options:MTLResourceStorageModeShared];
  m.vertBuf = nil;
  m.faceBuf = nil;
}

MetalBackend::~MetalBackend() {
  delete _impl;
}

void MetalBackend::clearBuffers() {
  auto& m = *_impl;
  if (!m.clearPipeline) return;

  id<MTLCommandBuffer> cmd = [m.queue commandBuffer];
  id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
  [enc setComputePipelineState:m.clearPipeline];
  [enc setBuffer:m.pixelBuf offset:0 atIndex:0];
  [enc setBuffer:m.zbuffBuf offset:0 atIndex:1];
  uint32_t rWVal = rW, rHVal = rH, pitchVal = W;
  [enc setBytes:&rWVal length:sizeof(uint32_t) atIndex:2];
  [enc setBytes:&rHVal length:sizeof(uint32_t) atIndex:3];
  [enc setBytes:&pitchVal length:sizeof(uint32_t) atIndex:4];

  MTLSize grid = MTLSizeMake(rW, rH, 1);
  MTLSize group = MTLSizeMake(16, 16, 1);
  [enc dispatchThreads:grid threadsPerThreadgroup:group];
  [enc endEncoding];
  [cmd commit];
  [cmd waitUntilCompleted];

  // Reset tile counts and tri counter
  memset([m.tileCountsBuf contents], 0, m.tilesX * m.tilesY * sizeof(int32_t));
  *(int32_t*)[m.triCountBuf contents] = 0;
}

void MetalBackend::renderModels(const std::vector<std::shared_ptr<ModelInstance>>& models,
                                 bool reflectionPass) {
  auto& m = *_impl;
  if (!m.vertexPipeline || !m.binPipeline || !m.rasterPipeline) return;

  // Upload lights
  std::vector<DeviceLight> hostLights;
  for (const auto& l : Light::sceneLights) {
    DeviceLight dl;
    dl.type = (l._type == LightType::Directional) ? 0 : 1;
    dl.strength = l._strength;
    dl.r = l._R; dl.g = l._G; dl.b = l._B;
    if (l._type == LightType::Directional) {
      dl.dx = l._direction._x; dl.dy = l._direction._y; dl.dz = l._direction._z;
    } else {
      dl.dx = l._x; dl.dy = l._y; dl.dz = l._z;
    }
    hostLights.push_back(dl);
  }
  if (!hostLights.empty()) {
    memcpy([m.lightBuf contents], hostLights.data(), hostLights.size() * sizeof(DeviceLight));
  }

  // Reset tri counter
  *(int32_t*)[m.triCountBuf contents] = 0;

  // Vertex transform: one dispatch per model
  id<MTLCommandBuffer> cmd = [m.queue commandBuffer];
  id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
  [enc setComputePipelineState:m.vertexPipeline];

  for (const auto& model : models) {
    if (reflectionPass) {
      if (model->_shader == shaderType::WaterXZShader) continue;
      float worldY = model->_position.at(3, 1) + model->_bCenter._y;
      if (worldY < WATER_Y + 2.0f) continue;
    }

    const auto& srcVerts = model->_baseModel.getVertices();
    const auto& srcFaces = model->_baseModel.getFaces();
    int numVerts = (int)srcVerts.size();
    int numFaces = (int)srcFaces.size();
    if (numFaces == 0 || numVerts == 0) continue;

    // Resize/create per-model buffers if needed
    NSUInteger vertSize = numVerts * 4 * sizeof(float);
    NSUInteger faceSize = numFaces * sizeof(DeviceFace);
    if (!m.vertBuf || [m.vertBuf length] < vertSize)
      m.vertBuf = [m.device newBufferWithLength:vertSize options:MTLResourceStorageModeShared];
    if (!m.faceBuf || [m.faceBuf length] < faceSize)
      m.faceBuf = [m.device newBufferWithLength:faceSize options:MTLResourceStorageModeShared];

    // Upload vertices (vertex<float> has raw[4] = {x,y,z,e})
    memcpy([m.vertBuf contents], srcVerts.data(), vertSize);

    // Convert and upload faces
    DeviceFace* df = (DeviceFace*)[m.faceBuf contents];
    for (int i = 0; i < numFaces; ++i) {
      df[i] = {srcFaces[i]._v0, srcFaces[i]._v1, srcFaces[i]._v2,
                srcFaces[i]._t0x, srcFaces[i]._t0y,
                srcFaces[i]._t1x, srcFaces[i]._t1y,
                srcFaces[i]._t2x, srcFaces[i]._t2y};
    }

    VertexUniforms vu;
    memcpy(vu.model, model->_position._m.data(), 16 * sizeof(float));
    memcpy(vu.cam, cameraTransform._m.data(), 16 * sizeof(float));
    vu.focalLength = focalLength;
    vu.nearClip = nearClipDist;
    vu.farClip = farClipDist;
    vu.rW = rW;
    vu.rH = rH;
    vu.shaderType = (int)model->_shader;
    vu.globalIllum = model->_globalIllumination;
    vu.doubleSided = model->_doubleSided ? 1 : 0;
    vu.textureIdx = -1;
    vu.texW = 0; vu.texH = 0;
    if (model->_baseModel._texture) {
      vu.texW = model->_baseModel._texture->get_width();
      vu.texH = model->_baseModel._texture->get_height();
    }
    vu.faceCount = numFaces;

    [enc setBuffer:m.vertBuf offset:0 atIndex:0];
    [enc setBuffer:m.faceBuf offset:0 atIndex:1];
    [enc setBytes:&vu length:sizeof(vu) atIndex:2];
    [enc setBuffer:m.screenTriBuf offset:0 atIndex:3];
    [enc setBuffer:m.triCountBuf offset:0 atIndex:4];

    MTLSize grid = MTLSizeMake(numFaces, 1, 1);
    NSUInteger tw = m.vertexPipeline.threadExecutionWidth;
    MTLSize group = MTLSizeMake(tw, 1, 1);
    [enc dispatchThreads:grid threadsPerThreadgroup:group];
  }
  [enc endEncoding];
  [cmd commit];
  [cmd waitUntilCompleted];

  int totalTris = *(int32_t*)[m.triCountBuf contents];
  if (totalTris <= 0) return;
  if (totalTris > MAX_SCREEN_TRIS) totalTris = MAX_SCREEN_TRIS;

  // Bin triangles
  memset([m.tileCountsBuf contents], 0, m.tilesX * m.tilesY * sizeof(int32_t));
  cmd = [m.queue commandBuffer];
  enc = [cmd computeCommandEncoder];
  [enc setComputePipelineState:m.binPipeline];
  [enc setBuffer:m.screenTriBuf offset:0 atIndex:0];
  [enc setBuffer:m.tileCountsBuf offset:0 atIndex:1];
  [enc setBuffer:m.tileTrisBuf offset:0 atIndex:2];
  uint32_t txVal = m.tilesX, tyVal = m.tilesY;
  [enc setBytes:&txVal length:sizeof(uint32_t) atIndex:3];
  [enc setBytes:&tyVal length:sizeof(uint32_t) atIndex:4];
  [enc setBytes:&totalTris length:sizeof(int32_t) atIndex:5];
  {
    MTLSize grid = MTLSizeMake(totalTris, 1, 1);
    NSUInteger tw = m.binPipeline.threadExecutionWidth;
    MTLSize group = MTLSizeMake(tw, 1, 1);
    [enc dispatchThreads:grid threadsPerThreadgroup:group];
  }
  [enc endEncoding];
  [cmd commit];
  [cmd waitUntilCompleted];

  // Rasterize
  RasterUniforms ru;
  ru.rW = rW; ru.rH = rH; ru.pitch = W;
  ru.tilesX = m.tilesX; ru.tilesY = m.tilesY;
  ru.numLights = (int)hostLights.size();

  cmd = [m.queue commandBuffer];
  enc = [cmd computeCommandEncoder];
  [enc setComputePipelineState:m.rasterPipeline];
  [enc setBuffer:m.screenTriBuf offset:0 atIndex:0];
  [enc setBuffer:m.tileCountsBuf offset:0 atIndex:1];
  [enc setBuffer:m.tileTrisBuf offset:0 atIndex:2];
  [enc setBuffer:m.pixelBuf offset:0 atIndex:3];
  [enc setBuffer:m.zbuffBuf offset:0 atIndex:4];
  [enc setBuffer:m.lightBuf offset:0 atIndex:5];
  [enc setBytes:&ru length:sizeof(ru) atIndex:6];
  {
    MTLSize grid = MTLSizeMake(rW, rH, 1);
    MTLSize group = MTLSizeMake(16, 16, 1);
    [enc dispatchThreads:grid threadsPerThreadgroup:group];
  }
  [enc endEncoding];
  [cmd commit];
  [cmd waitUntilCompleted];

  // Copy GPU results to host arrays (shared memory — just memcpy)
  memcpy(pixels.data(), [m.pixelBuf contents], rH * W * sizeof(unsigned));
  memcpy(zbuff.data(), [m.zbuffBuf contents], rH * W * sizeof(int));
}

void MetalBackend::snapshotReflection() {
  auto& m = *_impl;
  memcpy(reflectionBuf.data(), [m.pixelBuf contents], rH * W * sizeof(unsigned));
  memcpy(reflectionZBuf.data(), [m.zbuffBuf contents], rH * W * sizeof(int));
}

unsigned* MetalBackend::pixelData() { return pixels.data(); }
int* MetalBackend::depthData() { return zbuff.data(); }
