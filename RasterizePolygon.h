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
extern unsigned zbuff[W*H];

typedef std::pair<double, double> SlopePair;
void plot(unsigned x, unsigned y, const unsigned color)
{
  // We want to flip pos y to mean "up"
#ifdef DEBUG
  if(pixels[(H-y)*W+x] != blank)
    pixels[(H-y)*W+x] = duplicate;
  else
#endif
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


unsigned zPos(const unsigned cx, const unsigned bx, const unsigned ax, const unsigned cy, const unsigned by, const unsigned ay, const unsigned cz, const unsigned bz, const unsigned az, const unsigned x, const unsigned y) {
  unsigned z = az;
  unsigned div1 = (bx-ax) * (cy-ay) - (cx-ax) * (by-ay);
  if (div1 == 0) {
    return 0xFFFFFFFF;
  }
   unsigned div2 = (bx-ax)*(cy-ay) - (cx-ax) * (by-ay);
  if (div1 == 0) {
    return 0xFFFFFFFF;
  }

  z += ((bx - ax) * (cz - az) - (cx - ax) * (bz - az))*(y - ay)/div1     -      ((by-ay) * (cz-az) - (cy-ay)*(bz-az))*(x-ax)/div2;
  return z;
}


template <typename T>
void drawTri(const T& p0, const T& p1, const T& p2,  const unsigned color)
{

#ifdef DEBUG
  int direction = directionality(p0, p1, p2);
  if (direction < 0) {
    printf("Failed\n");
    return;
  }
#endif

  const unsigned x0 = p0._x * halfW + halfW;
  const unsigned x1 = p1._x * halfW + halfW;
  const unsigned x2 = p2._x * halfW + halfW;

  const unsigned y0 = p0._y * halfH + halfH;
  const unsigned y1 = p1._y * halfH + halfH;
  const unsigned y2 = p2._y * halfH + halfH;

  const unsigned z0 = p0._z * 0xFFFF + 0xFFFF;
  const unsigned z1 = p1._z * 0xFFFF + 0xFFFF;
  const unsigned z2 = p2._z * 0xFFFF + 0xFFFF;

  const int A01 = y0 - y1;
  const int A12 = y1 - y2;
  const int A20 = y2 - y0;
  const int B01 = x1 - x0;
  const int B12 = x2 - x1;
  const int B20 = x0 - x2;

  const int minX = min3(x0, x1, x2);
  const int minY = min3(y0, y1, y2, true);

  const int maxX = max3(x0, x1, x2, W);
  const int maxY = max3(y0, y1, y2, H);

  const int bias0 = isTopLeft(p1, p2) ? 0 : -1;
  const int bias1 = isTopLeft(p2, p0) ? 0 : -1;
  const int bias2 = isTopLeft(p0, p1) ? 0 : -1;

  int w0_row = orient2d(x1, x2, minX, y1, y2, minY) + bias0;
  int w1_row = orient2d(x2, x0, minX, y2, y0, minY) + bias1;
  int w2_row = orient2d(x0, x1, minX, y0, y1, minY) + bias2;

  unsigned x, y, xVal, xValInner, numInner, xInner, numOuter;
  int w0, w1, w2;


  //float zdx = (p0._y *(p2._z - p1._z) + p1._y*(p0._z - p2._z) + p2._y*(p1._z - p2._z)) / (p0._x*(p1._y - p2._y) + p1._x*(p2._y-p0._y) + p2._x*(p0._y-p1._y));
  //zdx /= (1920/2);
  //float zdy = (-((-p0._x + p2._x)*(-p0._z + p1._z)) + (-p0._x + p1._x)*(-p0._z + p2._z))/((-p0._x + p1._x)*(p2._y - p0._y) - (-p0._x + p2._x)*(-p0._y + p1._y));



  for (y = minY; y <= maxY; ++y) {
    w0 = w0_row;
    w1 = w1_row;
    w2 = w2_row;

    numInner = (maxX - minX) / 8;
    numOuter = (maxX - minX) % 8 + 1;

    //float z = w0 * p0._z + w1 * p1._z + w2 * p2._z;
    //z /= (w0 + w1 + w2);

    //printf("Calculated initial z to be %f, zdx = %f\n", z, zdx);

    for (xInner = 0; xInner < numInner; ++xInner) {
      xVal = 8 * xInner + minX;

      // We have AVX2, lets take advantage of it!
      // We break the loop out to enable the compiler to vectorize it
#pragma clang loop vectorize(enable) interleave(enable)
      for (unsigned x = 0; x < 8; ++x) {
        xValInner = xVal + x;


        //float z2 = w0 * p0._z + w1 * p1._z + w2 * p2._z;
        //z2 /= (w0 + w1 + w2);

        //printf("Z is now %f correct z: %f\n", z, z2);


        // If p is on or inside all edges, render pixel
        unsigned z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);
        if ((w0 | w1 | w2) >= 0){
          if (zbuff[xValInner + W*y] < z) {
            zbuff[xValInner + W*y] = z;
            plot(xValInner, y, color);
          }
        }
        w0 += A12;
        w1 += A20;
        w2 += A01;
        //z += zdx;
      }
    }

    xVal = 8 * numInner + minX;

    for (x = 0; x < numOuter; ++x) {
      xValInner = xVal + x;

      // If p is on or inside all edges, render pixel
      unsigned z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);

      if ((w0 | w1 | w2) >= 0) {
        if (zbuff[xValInner + W*y] < z) {
          zbuff[xValInner + W*y] = z;
          plot(xValInner, y, color);
        }
      }
      w0 += A12;
      w1 += A20;
      w2 += A01;
      //z += zdx;

    }
    w0_row += B12;
    w1_row += B20;
    w2_row += B01;
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
