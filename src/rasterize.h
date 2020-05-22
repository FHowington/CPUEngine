//
// Created by Forbes Howington on 3/20/20.
//

#pragma once

#include <algorithm>
#include <array>
#include "geometry.h"
#include "loader.h"
#include <SDL2/SDL.h>
#include "shader.h"
#include "tgaimage.h"
#include <utility>
#include <vector>
#include "pool.h"
#include "Window.h"

const unsigned halfW = W/2;
const unsigned halfH = H/2;


extern unsigned pixels[W * H];
extern int zbuff[W * H];

// Fast bitwise algorithms for finding max/min3
// Requires that highest bit not be used
inline __attribute__((always_inline)) int fast_max(int a, int b);
inline __attribute__((always_inline)) int fast_min(int a, int b);

typedef std::pair<double, double> SlopePair;
inline __attribute__((always_inline)) void plot(unsigned x, unsigned y, const unsigned color);

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

inline const int min3(const int v0, const int v1, const int v2)
{
  return fast_max(0, fast_min(fast_min(v0, v1), v2));
}

inline const int max3(const int v0, const int v1, const int v2, const int border)
{
  return fast_min(border, fast_max(fast_max(v0, v1), v2));
}

template <typename T>
inline int directionality(const T& p0, const T& p1, const T& p2)
{
  return p0._x*(p1._y - p2._y) - p0._y*(p1._x - p2._x) + p2._y*p1._x - p1._y*p2._x;
}

float zPos(const int cx, const int bx, const int ax, const int cy, const int by, const int ay, const int cz, const int bz, const int az, const int x, const int y);

inline bool colinear(const int x0, const int x1, const int x2, const int y0, const int y1,const int y2) {
  return x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1) == 0;
}

// Implementation of Bresenham's line algo
// This code is rather long to remove as many conditions, mults, divs, and floats as possible
void line(const vertex<int>& v0, const vertex<int>& v1, const unsigned color);

const matrix<4,4> getProjection(float focalLength);

const matrix<4,4> viewport(const int x, const int y, const int w, const int h);

matrix<4,4> GetInverse(const matrix<4,4>& inM);

template<typename T, typename std::enable_if<std::is_base_of<TexturedShader, T>::value, int>::type* = nullptr>
void drawTri(const ModelInstance& m, const face& f, const vertex<float>& light,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i);

template<typename T, typename std::enable_if<std::is_base_of<UntexturedShader, T>::value, int>::type* = nullptr>
void drawTri(const ModelInstance& m, const face& f, const vertex<float>& light,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i);

template <typename T>
inline void renderModel(const ModelInstance* model, const matrix<4,4>& cameraTransform, const matrix<4,4>& viewClip, const vertex<float>& light) {
  for (auto t : model->_baseModel.getFaces()) {
    const vertex<int> v0i(pipeline(cameraTransform, model->_position, viewClip, model->_baseModel.getVertex(t._v0), 1.5));
    const vertex<int> v1i(pipeline(cameraTransform, model->_position, viewClip, model->_baseModel.getVertex(t._v1), 1.5));
    const vertex<int> v2i(pipeline(cameraTransform, model->_position, viewClip, model->_baseModel.getVertex(t._v2), 1.5));

    // We get the normal vector for every triangle
    const vertex<float> v = cross(v0i, v1i, v2i);

    // If it is backfacing, vector will be pointing in +z, so cull it
    if (v._z < 0) {

      drawTri<T>(*model, t, light, v0i, v1i, v2i);
    }
  }
}
