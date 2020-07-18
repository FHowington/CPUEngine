//
// Created by Forbes Howington on 3/20/20.
//

#pragma once

#include "Window.h"
#include "geometry.h"
#include "loader.h"
#include "pool.h"
#include "shader.h"
#include "tgaimage.h"
#include <SDL2/SDL.h>
#include <algorithm>
#include <array>
#include <utility>
#include <vector>

const unsigned halfW = W/2;
const unsigned halfH = H/2;


extern std::array<unsigned, W * H> pixels;
extern std::array<int, W * H> zbuff;

inline __attribute__((always_inline)) void plot(unsigned x, unsigned y, unsigned color);

// We deal only with counter-clockwise triangles
// Therefore, see if it is horizontal & higher than the other points (p1 is left of p0)
// Or
template <typename T>
inline bool isTopLeft(T& p0, T& p1) {
  return (p0._y == p1._y && p1._x < p0._x) || (p0._y < p1._y);
}

inline int orient2d(const int x0, const int x1, const int x2, const int y0, const int y1, const int y2)
{
  return (x1 - x0)*(y2 - y0) - (y1 - y0)*(x2 - x0);
}

inline int min3(const int v0, const int v1, const int v2)
{
  return fast_max(0, fast_min(fast_min(v0, v1), v2));
}

inline int max3(const int v0, const int v1, const int v2, const int border)
{
  return fast_min(border, fast_max(fast_max(v0, v1), v2));
}

template <typename T>
inline int directionality(const T& p0, const T& p1, const T& p2)
{
  return p0._x*(p1._y - p2._y) - p0._y*(p1._x - p2._x) + p2._y*p1._x - p1._y*p2._x;
}

inline float zPos(const long long cx, const long long bx, const long long ax, const long long cy, const long long by, const long long ay, const long long cz, const long long bz, const long long az, const long long x, const long long y) {
  return ((float)(ax*(cz*(by - y) - bz*(cy - y)) + bx*(cz*(-ay + y) + az*(cy - y)) + cx*(bz*(ay - y) + az*(y - by)) + x*(ay*(cz - bz) + by*(az - cz) + cy*(bz - az))))/((float)(ax*(by - cy) + bx*(cy - ay) + cx*(ay - by)));
}

inline bool colinear(const int x0, const int x1, const int x2, const int y0, const int y1,const int y2) {
  return x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1) == 0;
}

// Implementation of Bresenham's line algo
// This code is rather long to remove as many conditions, mults, divs, and floats as possible
void line(const vertex<int>& v0, const vertex<int>& v1, unsigned color);

matrix<4,4> GetInverse(const matrix<4,4>& inM);

template<typename T, typename std::enable_if<std::is_base_of<TexturedShader, T>::value, int>::type* = nullptr>
void drawTri(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2);

template<typename T, typename std::enable_if<std::is_base_of<UntexturedShader, T>::value, int>::type* = nullptr>
void drawTri(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2);


template <typename T, typename std::enable_if<std::is_base_of<InFrontCamera, T>::value, int>::type* = nullptr>
void renderModel(std::shared_ptr<const ModelInstance> model, const matrix<4,4>& cameraTransform) {
  for (auto t : model->_baseModel.getFaces()) {
    vertex<int> v0i;
    vertex<int> v1i;
    vertex<int> v2i;
    vertex<float> v0;
    vertex<float> v1;
    vertex<float> v2;

    if (pipelineFast(cameraTransform, model->_position, model->_baseModel.getVertex(t._v0), v0i, v0) &&
        pipelineFast(cameraTransform, model->_position, model->_baseModel.getVertex(t._v1), v1i, v1) &&
        pipelineFast(cameraTransform, model->_position, model->_baseModel.getVertex(t._v2), v2i, v2)) {
      // We get the normal vector for every triangle
      const vertex<float> v = cross(v0i, v1i, v2i);

      // If it is backfacing, vector will be pointing in +z, so cull it
      if (v._z < 0) {
        drawTri<T>(*model, t, v0i, v1i, v2i, v0, v1, v2);
      }
    }
  }
}

template <typename T, typename std::enable_if<std::is_base_of<BehindCamera, T>::value, int>::type* = nullptr>
void renderModel (std::shared_ptr<const ModelInstance> model, const matrix<4,4>& cameraTransform);
