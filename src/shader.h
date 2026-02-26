//
// Created by Forbes Howington on 5/9/20.
//

#pragma once

#include "geometry.h"
#include "light.h"
#include "loader.h"
#include "tgaimage.h"
#include "simd_compat.h"

// We do this instead of the typical inheritance route because dynamic binding of function calls
// incurs a perf. hit for shaders. They are simply called too many time.

// Textured shaders take advantage of looking up textures before they are needed using vectorization
// This is a class defining a shader.
// vertexShader called on all vertices per triangle
// Traditional implementation would pass in barycentric coordinates to
// the fragment shader. However, this is much slower than just using deltas for interpolation
// so instead, we will guarantee that stepX and stepY are called for respective
// steps, and depend on implementation to update their values appropriately

enum class shaderType { FlatShader, GouraudShader, InterpFlatShader, InterpGouraudShader, PlaneXZShader, PlaneXYShader, PlaneYZShader, StoneXZShader, StoneXYShader, StoneYZShader, WoodXZShader, WoodXYShader, WoodYZShader, WaterXZShader };

class Shader { // NOLINT
 public:
  virtual ~Shader() = default;;

  virtual fcolor fragmentShader(float x, float y, float z, unsigned color = 0) = 0;

#ifdef __AVX2__
  virtual const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& x, const __m256i& y, const __m256i& z) { return; }
#endif

  virtual void stepXForX(unsigned step) = 0;
  virtual void stepYForX(unsigned step) = 0;
};


// Shaders that are only intended to appear "in front" of the camera inherit from this.
// No clipping at view frustum boundaries occurs for these shaders, and interpolation of
// real world coordinates and texture coordinates  passed to the shaders is affine
// (no perspective correction applied to interpolated coordinates)
class BehindCamera {};

// Shaders for which part of the polygon may be behind the camera and therefore require clipping inherit from this.
// These textures will also use perspective corrected interpolation for real world and texture coordinates
class InFrontCamera {};

class TexturedShader : public Shader {
};

class UntexturedShader : public Shader {
};

#include "shader_flat.h"
#include "shader_gouraud.h"
#include "shader_plane.h"
#include "shader_stone.h"
#include "shader_wood.h"
#include "shader_water.h"
