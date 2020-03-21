//
// Created by Forbes Howington on 3/20/20.
//
#pragma once
#include <vector>
#include <utility>
#include <array>
#include <SDL2/SDL.h>
#include "Window.h"

unsigned blank = 0xFFFFFF, duplicate = 0xFFAA55;

extern unsigned pixels[W*H];

typedef std::pair<double, double> SlopePair;
void Plot(unsigned x, unsigned y, unsigned color)
{
  if(pixels[y*W+x] != blank)
    pixels[y*W+x] = duplicate;
  else
    pixels[y*W+x] = (color&0xFFFFFF);
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
