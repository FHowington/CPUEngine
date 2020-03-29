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

extern unsigned pixels[W*H];

typedef std::pair<double, double> SlopePair;
void Plot(unsigned x, unsigned y, const unsigned color)
{
  if(pixels[y*W+x] != blank)
    pixels[y*W+x] = duplicate;
  else
    pixels[y*W+x] = (color&0xFFFFFF);
}


// We deal only with counter-clockwise triangles
// Therefore, see if it is horizontal & higher than the other points (p1 is left of p0)
// Or
template <typename T>
inline bool isTopLeft(T& p0, T& p1) {
  if (std::get<1>(p0) == std::get<1>(p1) && std::get<0>(p1) < std::get<0>(p0)) {
    return true;
  } else if (std::get<1>(p0) < std::get<1>(p1)) {
    return true;
  }
  return false;
}

template <typename T>
inline bool orient2d(const T& p0, const T& p1, const T& p2, int bias)
{
  return (std::get<0>(p1)-std::get<0>(p0))*(std::get<1>(p2)-std::get<1>(p0)) - (std::get<1>(p1)-std::get<1>(p0))*(std::get<0>(p2)-std::get<0>(p0)) + bias > 0;
}

template <typename T>
inline const T& min3(const T& v0, const T& v1, const T& v2)
{
  return v0 <= v1 ? (v0 <= v2 ? v0 : v2) : (v1 <= v2 ? v1 : v2);
}

template <typename T>
inline const T& max3(const T& v0, const T& v1, const T& v2)
{
  return v0 >= v1 ? (v0 >= v2 ? v0 : v2) : (v1 >= v2 ? v1 : v2);
}

template <typename T>
// > 0, clockwise, <0, counterclockwise, 0, colinear
inline bool directionality(const T& p0, const T& p1, const T& p2)
{
  return std::get<0>(p0)*(std::get<1>(p1) - std::get<1>(p2)) - std::get<1>(p0)*(std::get<0>(p1) - std::get<0>(p2)) + std::get<1>(p2)*std::get<0>(p1) - std::get<1>(p1)*std::get<0>(p2);
}

template <typename T>
void drawTri(const T& p0, const T& p1, const T& p2,  unsigned color)
{
  // Compute triangle bounding box

  //int direction = directionality(p0, p1, p2);
  //if (direction == 0) {
  //  return;
  //}
  //bool clockwise = direction > 0 ? true : false;
  auto minX = min3(std::get<0>(p0), std::get<0>(p1),std::get<0>(p2));
  auto minY = min3(std::get<1>(p0), std::get<1>(p1),std::get<1>(p2));
  auto maxX = max3(std::get<0>(p0), std::get<0>(p1),std::get<0>(p2));
  auto maxY = max3(std::get<1>(p0), std::get<1>(p1),std::get<1>(p2));

  // Clip against screen bounds
  minX = std::max(minX, (decltype(minX))0);
  minY = std::max(minY, (decltype(minX))0);
  maxX = std::min(maxX, (decltype(minX))W - 1);
  maxY = std::min(maxY, (decltype(minX))H - 1);

  // Rasterize
  T p;

  int bias0 = isTopLeft(p1, p2) ? 0 : 1;
  int bias1 = isTopLeft(p2, p0) ? 0 : 1;
  int bias2 = isTopLeft(p0, p1) ? 0 : 1;

  for (std::get<1>(p) = minY; std::get<1>(p) <= maxY; ++std::get<1>(p)) {
    for (std::get<0>(p) = minX; std::get<0>(p) <= maxX; ++std::get<0>(p)) {

      // Determine barycentric coordinates
      auto w0 = orient2d(p1, p2, p, bias0);
      auto w1 = orient2d(p2, p0, p, bias1);
      auto w2 = orient2d(p0, p1, p, bias2);

      // If p is on or inside all edges, render pixel.
      if (w0 && w1 && w2)
        Plot(std::get<0>(p), std::get<1>(p), color);
    }
  }
}

// Sort high to low, based on y-val first, then x-val
template <typename T>
inline void hToL (const T*& p0, const T*& p1) {
    if (std::get<1>(*p1) < std::get<1>(*p0)) {
      std::swap(p0, p1);
    } else if (std::get<1>(*p1) == std::get<1>(*p0) && std::get<0>(*p1) < std::get<0>(*p0)) {
      std::swap(p0, p1);
    }
}

template <typename T>
inline SlopePair slope(const T& from, const T& to, unsigned steps) {
    // Read this as, beginning at x pos of from, we increase (second) pixels per scanline
    return std::make_pair(std::get<0>(from), ((double)std::get<0>(to) - (double)std::get<0>(from)) / steps);
}

template <typename T>
inline void drawScanline(T& left, T& right, unsigned y, unsigned color) {
    unsigned begx = left.first, endx = right.first;

    for (; begx < endx; ++begx) {
      Plot(begx, y, color);
    }
    left.first += left.second;
    right.first += right.second;

    const std::array<int, 2>* p = new std::array<int, 2>;
    const std::array<int, 2>* p2 = new std::array<int, 2>;
}

template <typename T>
void rasterizePolygon (const T* p0, const T* p1, const T* p2, unsigned color) {
  hToL(p0, p1);
  hToL(p0, p2);
  hToL(p1, p2);

  // If highest and lowest are the same, return early as
  // triangle has no area
  if (std::get<1>(*p0) == std::get<1>(*p2)) {
    return;
  }

  // True = right, false = left
  // Cross-product magic to determine which side it is on
  bool rightIsShortSide = (std::get<1>(*p1) - std::get<1>(*p0)) * (std::get<0>(*p2) - std::get<0>(*p0)) <
    (std::get<0>(*p1) - std::get<0>(*p0)) * (std::get<1>(*p2) - std::get<1>(*p0));

  SlopePair longSide = slope(*p0, *p2, std::get<1>(*p2) - std::get<1>(*p0));

  // Combine these 2 loops later
  if (std::get<1>(*p0) < std::get<1>(*p1)) {
    SlopePair shortSide = slope(*p0, *p1, std::get<1>(*p1) - std::get<1>(*p0));
    for (auto y = std::get<1>(*p0); y < std::get<1>(*p1); ++y) {
      if (rightIsShortSide) {
        drawScanline(longSide, shortSide, y, color);
      } else {
        drawScanline(shortSide, longSide, y, color);
      }
    }
  }

  if (std::get<1>(*p1) < std::get<1>(*p2)) {
    SlopePair shortSide = slope(*p1, *p2, std::get<1>(*p2) - std::get<1>(*p1));
    for (auto y = std::get<1>(*p1); y < std::get<1>(*p2); ++y) {
      if (rightIsShortSide) {
        drawScanline(longSide, shortSide, y, color);
      } else {
        drawScanline(shortSide, longSide, y, color);
      }
    }
  }
}

void drawPoly(std::array<int, 2> p0, std::array<int, 2> p1, std::array<int, 2> p2, unsigned color) {
  rasterizePolygon(&p0, &p1, &p2, color);
}

// Implementation of Bresenham's line algo
// This code is rather long to remove as many conditions, mults, divs, and floats as possible
void line(unsigned x0, unsigned x1, unsigned y0, unsigned y1, const unsigned color) {
  // We must find whether x is longer, or y is longer
  // If true, indicates that x will have pixels on same row

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
      Plot(x, y, color);
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
      Plot(x, y, color);
      error += absderror;
      if (error > dy) {
        x += offset;
        error -= (dy + dy);
      }
    }
  }
}
