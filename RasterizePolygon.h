//
// Created by Forbes Howington on 3/20/20.
//

#pragma once

#include <algorithm>
#include <array>
#include "loader.h"
#include <SDL2/SDL.h>
#include "tgaimage.h"
#include <utility>
#include <vector>
#include "Window.h"

const unsigned halfW = W/2;
const unsigned halfH = H/2;

extern unsigned pixels[W*H];
extern unsigned zbuff[W*H];

typedef std::pair<double, double> SlopePair;
void plot(unsigned x, unsigned y, const unsigned color);

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

inline int orient2d(const int x0, const int x1, const int x2, const int y0, const int y1, const int y2)
{
  return (x1 - x0)*(y2 - y0) - (y1 - y0)*(x2 - x0);
}

template <typename T>
inline const T min3(const T& v0, const T& v1, const T& v2, bool y = false)
{
  const T result = v0 <= v1 ? (v0 <= v2 ? v0 : v2) : (v1 <= v2 ? v1 : v2);
  return y ? ((result > 0) ? result : 1) : (result > 0) ? result : 0;
}

template <typename T>
inline const T max3(const T& v0, const T& v1, const T& v2, const T& border)
{
  const T& max = v0 >= v1 ? (v0 >= v2 ? v0 : v2) : (v1 >= v2 ? v1 : v2);
  return max > border ? border : max;
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

void drawTri(const face& f,  const float light, const TGAImage& img);

// Implementation of Bresenham's line algo
// This code is rather long to remove as many conditions, mults, divs, and floats as possible
void line(const vertex& v0, const vertex& v1, const unsigned color);
