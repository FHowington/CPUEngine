//
// Created by Forbes Howington on 3/20/20.
//
#pragma once
#include <algorithm>
#include <array>
#include "loader.h"
#include <SDL2/SDL.h>
#include <utility>
#include <vector>
#include "Window.h"

unsigned blank = 0xFFFFFF, duplicate = 0xFFAA55;
const unsigned halfW = W/2;
const unsigned halfH = H/2;

extern unsigned pixels[W*H];

typedef std::pair<double, double> SlopePair;
void plot(unsigned x, unsigned y, const unsigned color)
{
  pixels[(H-y)*W+x] = color;
}


// We deal only with counter-clockwise triangles
// Therefore, see if it is horizontal & higher than the other points (p1 is left of p0)
// Or
template <typename T>
inline bool isTopLeft(T& p0, T& p1) {
  if (p0._y == p1._y && p1._x < p0._x) {
    return true;
  } else if (p0._y < p1._y) {
    return true;
  }
  return false;
}

inline bool orient2d(const int x0, const int x1, const int x2, const int y0, const int y1, const int y2, const int bias)
{
  return (x1 - x0)*(y2 - y0) - (y1 - y0)*(x2 - x0) + bias > 0;
}

template <typename T>
inline const T& min3(const T& v0, const T& v1, const T& v2)
{
  return std::max<T>(0, v0 <= v1 ? (v0 <= v2 ? v0 : v2) : (v1 <= v2 ? v1 : v2));
}

template <typename T>
inline const T& max3(const T& v0, const T& v1, const T& v2, const T& border)
{
  const T& max = v0 >= v1 ? (v0 >= v2 ? v0 : v2) : (v1 >= v2 ? v1 : v2);
  return max > border ? border : max;
}

template <typename T>
// > 0, clockwise, <0, counterclockwise, 0, colinear
inline bool directionality(const T& p0, const T& p1, const T& p2)
{
  return p0->_x*(p1->_y - p2->_y) - p0->_y*(p1->_x - p2->_x) + p2->_y*p1->_x - p1->_y*p2->_x;
}

template <typename T>
void drawTri(const T& p0, const T& p1, const T& p2,  unsigned color)
{
  static const unsigned xs[16]{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

  // Compute triangle bounding box


  //int direction = directionality(p0, p1, p2);
  //if (direction == false) {
  //  printf("Failed\n");
  //  return;
  //}
  //bool clockwise = direction > 0 ? true : false;
  unsigned x0 = p0._x * halfW + halfW, x1 = p1._x * halfW + halfW, x2 = p2._x * halfW + halfW;
  unsigned y0 = p0._y * halfH + halfH, y1 = p1._y * halfH + halfH, y2 = p2._y * halfH + halfH;

  // TODO: make bounding box more efficient
  const unsigned minX = min3(x0, x1, x2);
  const unsigned minY = min3(y0, y1, y2);
  const unsigned maxX = max3(x0, x1, x2, W-1) + 1;
  const unsigned maxY = max3(y0, y1, y2, H-1);

  unsigned x, y, xVal, xValInner, numInner, xInner;
  unsigned bias0 = isTopLeft(p1, p2) ? 0 : 1;
  unsigned bias1 = isTopLeft(p2, p0) ? 0 : 1;
  unsigned bias2 = isTopLeft(p0, p1) ? 0 : 1;

  // Lets slice this up to allow for vectorization
  // Unroll by a factor of 16 (found this based on profiling, may be different for different CPUs)
  for (y = minY; y <= maxY; ++y) {
    numInner = (maxX - minX) / 16;
    for (xInner = 0; xInner < numInner; ++xInner) {
      xVal = 16 * xInner + minX;

#pragma clang loop vectorize(enable) interleave(enable)
      for (unsigned x = 0; x < 16; ++x) {
        xValInner = xVal + xs[x];

        auto w0 = orient2d(x2, x1, xValInner, y2, y1, y, bias0);
        auto w1 = orient2d(x0, x2, xValInner, y0, y2, y, bias1);
        auto w2 = orient2d(x1, x0, xValInner, y1, y0, y, bias2);

        // If p is on or inside all edges, render pixel
        if (w0 && w1 && w2)
          plot(xValInner, y, color);
      }
    }

    xVal = 16 * numInner + minX;
    for (x = 0; x < ((maxX - minX) % 16); ++x) {
      xValInner = xVal + x;
      auto w0 = orient2d(x2, x1, xValInner, y2, y1, y, bias0);
      auto w1 = orient2d(x0, x2, xValInner, y0, y2, y, bias1);
      auto w2 = orient2d(x1, x0, xValInner, y1, y0, y, bias2);

      // If p is on or inside all edges, render pixel
      if (w0 && w1 && w2)
          plot(xValInner, y, color);
    }
  }
}

// Implementation of Bresenham's line algo
// This code is rather long to remove as many conditions, mults, divs, and floats as possible
void line(const vertex& v0, const vertex& v1, const unsigned color) {
  // We must find whether x is longer, or y is longer
  // If true, indicates that x will have pixels on same row

  unsigned x0 = v0._x * halfW + halfW;
  unsigned y0 = v0._y * halfH + halfH;
  unsigned x1 = v1._x * halfW + halfW;
  unsigned y1 = v1._y * halfH + halfH;

  bool longX = std::abs((long)x1 - x0) >= std::abs((long)y1 - y0);
  if (x1 < x0) {
    std::swap(x1, x0);
    std::swap(y1, y0);
  }

  if (longX) {
    if (x1 < x0) {
      std::swap(x1, x0);
      std::swap(y1, y0);
    }

    // How much the y changes per x
    int dx = x1 - x0;
    int dy = y1 - y0;
    int derror = dy + dy; // Or dy*2
    unsigned absderror = std::abs(derror);
    int offset = (derror > 0 ? 1 : -1);
    int error = 0;

    unsigned y = y0;
    for (int x = x0; x < x1; ++x) {
      plot(x, y, color);
      // We increment the y "error" by twice the change in y
      error += absderror;
      if (error > dx) {
        y += offset;
        error -= (dx + dx);
      }
    }
  } else {
    if (y1 < y0) {
      std::swap(x1, x0);
      std::swap(y1, y0);
    }
    int dx = x1 - x0;
    int dy = y1 - y0;
    int derror = dx + dx; // derror is division error
    unsigned absderror = std::abs(derror);
    int offset = (derror > 0 ? 1 : -1);
    int error = 0;

    // How much the x changes per y
    unsigned x = x0;

    for (unsigned y = y0; y < y1; ++y) {
      plot(x, y, color);
      error += absderror;
      if (error > dy) {
        x += offset;
        error -= (dy + dy);
      }
    }
  }
}
