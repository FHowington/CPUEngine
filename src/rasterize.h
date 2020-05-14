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
#include "Window.h"

const unsigned halfW = W/2;
const unsigned halfH = H/2;

extern unsigned pixels[W*H];
extern int zbuff[W*H];


typedef std::pair<double, double> SlopePair;
inline __attribute__((always_inline)) void plot(unsigned x, unsigned y, const unsigned color)
{
  // We want to flip pos y to mean "up"
#ifdef DEBUG
  //  if(pixels[(H-y)*W+x] != focolor::blank)
  //  pixels[(H-y)*W+x] = fcolor::duplicate;
    //  else
#endif
    pixels[(H-y)*W+x] = color;
}


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

template <typename T>
inline const T min3(const T& v0, const T& v1, const T& v2)
{
  T result = v0 <= v1 ? (v0 <= v2 ? v0 : v2) : (v1 <= v2 ? v1 : v2);
  return (result > 0) ? result : 0;
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

//template <typename T, typename R = void>
//void drawTri(const ModelInstance& m, const face& f, const vertex<float>& light, const TGAImage& img, const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i);

// Implementation of Bresenham's line algo
// This code is rather long to remove as many conditions, mults, divs, and floats as possible
void line(const vertex<int>& v0, const vertex<int>& v1, const unsigned color);

const matrix<4,4> getProjection(float focalLength);

const matrix<4,4> viewport(const int x, const int y, const int w, const int h);

matrix<4,4> GetInverse(const matrix<4,4>& inM);

template<typename T, typename std::enable_if<std::is_base_of<TexturedShader, T>::value, int>::type* = nullptr>
void drawTri(const ModelInstance& m, const face& f, const vertex<float>& light, const TGAImage& img,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i) {

  printf("Textured");
  // These are reused for every triangle in the model
  static const __m256i min = _mm256_set1_epi32(-1);
  static const __m256i scale = _mm256_set_epi32(7, 6, 5, 4, 3, 2, 1, 0);
  static const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);
  static const __m256i loadOffset = _mm256_set_epi32(7*W, 6*W, 5*W, 4*W, 3*W, 2*W, W, 0);
  static const __m256i ones = _mm256_set1_epi32(-1);
  static const __m256i textureClip = _mm256_set1_epi32(img.width * img.height);

  const int x0 = v0i._x;
  const int x1 = v1i._x;
  const int x2 = v2i._x;

  const int y0 = v0i._y;
  const int y1 = v1i._y;
  const int y2 = v2i._y;

  // Prevent from wasting time on polygons that have no area
  if (colinear(x0, x1, x2, y0, y1, y2)) {
    return;
  }

  const int minX = min3(x0, x1, x2);
  const int minY = min3(y0, y1, y2);

  const int maxX = max3(x0, x1, x2, (int)W);
  const int maxY = max3(y0, y1, y2, (int)H - 1);

  // Same idea. These have no area (happens when triangle is outside of viewing area)
  if (maxX < minX || maxY < minY) {
    return;
  }

  // Bias to make sure only top or left edges fall on line
  const int bias0 = -isTopLeft(v1i, v2i);
  const int bias1 = -isTopLeft(v2i, v0i);
  const int bias2 = -isTopLeft(v0i, v1i);

  int w0Row = orient2d(x1, x2, minX, y1, y2, minY) + bias0;
  int w1Row = orient2d(x2, x0, minX, y2, y0, minY) + bias1;
  int w2Row = orient2d(x0, x1, minX, y0, y1, minY) + bias2;

  // If this number is 0, triangle has no area!
  float wTotal = w0Row + w1Row + w2Row;
  if (!wTotal) {
    return;
  }

  // Deltas for change in x or y for the 3 sides of a triangle
  const short A01 = y0 - y1;
  const short A12 = y1 - y2;
  const short A20 = y2 - y0;

  const short B01 = x1 - x0;
  const short B12 = x2 - x1;
  const short B20 = x0 - x2;

  const int z0 = v0i._z;
  const int z1 = v1i._z;
  const int z2 = v2i._z;

  // If all three are positive, the object is behind the camera
  if ((v0i._z | v1i._z | v2i._z) > 0) {
    return;
  }

  unsigned x, y, xVal, yVal, numInner, inner, numOuter;

  int w0, w1, w2, z;

  const int div = (((B20) * (-A01)) + (B01) * (A20));
  const int z10 = z1 - z0;
  const int z20 = z2 - z0;

  // Change in z for change in row/column
  // Obtained by taking partial derivative with respect to x or y from equation of a plane
  // See equation of a plane here: https://math.stackexchange.com/questions/851742/calculate-coordinate-of-any-point-on-triangle-in-3d-plane
  // Using these deltas, we interpolate over face of the whole triangle
  const int zdx = (A20 * z10 + A01 * z20) / div;
  const int zdy = (B20 * z10 + B01 * z20) / div;

  // Likewise from solving for z with equation of a plane
  int zOrig = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, minX, minY);

  // X and y values for the TEXTURE at the starting coordinates
  // w0row, w1row, w2row are weights of v0,v1,v2 at starting pos. So
  // weight their x and y values accordingly to get the coordinates.
  float xColRow = (f._t0x * w0Row + f._t1x * w1Row + f._t2x * w2Row) / wTotal;
  float yColRow = (f._t0y * w0Row + f._t1y * w1Row + f._t2y * w2Row) / wTotal;

  // Change in the texture coordinated for x/y, used for interpolation
  const float xColDx = (f._t0x * A12 + f._t1x * A20 + f._t2x * A01) / wTotal;
  const float yColDx = (f._t0y * A12 + f._t1y * A20 + f._t2y * A01) / wTotal;

  const float xColDy = (f._t0x * B12 + f._t1x * B20 + f._t2x * B01) / wTotal;
  const float yColDy = (f._t0y * B12 + f._t1y * B20 + f._t2y * B01) / wTotal;

  // Current texture coordinates
  float xCol;
  float yCol;

  // We want to always have our accessed aligned on 32 byte boundaries
  unsigned __attribute__((aligned(32))) zBuffTemp[8];
  unsigned __attribute__((aligned(32))) colors[8];

  //const unsigned xDiff = maxX - minX;
  //const unsigned yDiff = maxY - minY;

  T shader;
  shader.vertexShader(m, f, light, A12, A20, A01, B12, B20, B01, wTotal, w0Row, w1Row, w2Row);

  // If the traingle is wider than tall, we want to vectorize on x
  // Otherwise, we vectorize on y
  const __m256i zdxAdd = _mm256_set_epi32(7*zdx, 6*zdx, 5*zdx, 4*zdx, 3*zdx, 2*zdx, zdx, 0);
  const __m256i xColAdd = _mm256_mul_ps(scaleFloat,  _mm256_set1_ps(xColDx));
  const __m256i yColAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(yColDx));

  __m256i w0Init;
  __m256i w1Init;
  __m256i w2Init;

  const  int zdx8 = 8*zdx;

  const __m256i a12Add = _mm256_mullo_epi32(_mm256_set1_epi32(A12), scale);
const __m256i a12Add8 = _mm256_set1_epi32(8*A12);

  const __m256i a20Add = _mm256_mullo_epi32(_mm256_set1_epi32(A20), scale);
  const  __m256i a20Add8 = _mm256_set1_epi32(8*A20);

  const __m256i a01Add = _mm256_mullo_epi32(_mm256_set1_epi32(A01), scale);
  const __m256i a01Add8 = _mm256_set1_epi32(8*A01);


  // We will enter inner loop at least once, otherwise numInner is always 0
  unsigned offset = minY * W;
  const float yColDy4 = 4 * yColDy;

  for (y = minY; y <= maxY; ++y) {

    w0 = w0Row;
    w1 = w1Row;
    w2 = w2Row;
    z = zOrig;
    xCol = xColRow;
    yCol = yColRow;

    numInner = (maxX - minX) / 8;
    numOuter = (maxX - minX) % 8;

    w0Init = _mm256_set1_epi32(w0);
    w0Init = _mm256_add_epi32(w0Init, a12Add);

    w1Init = _mm256_set1_epi32(w1);
    w1Init = _mm256_add_epi32(w1Init, a20Add);

    w2Init = _mm256_set1_epi32(w2);
    w2Init = _mm256_add_epi32(w2Init, a01Add);

    xVal = minX;
    for (inner = 0; inner < numInner; ++inner) {
      const __m256i zbuffv = _mm256_load_si256((__m256i*)(zbuff + xVal + offset));

      const __m256i zInit = _mm256_set1_epi32(z);
      const __m256i zv = _mm256_add_epi32(zInit, zdxAdd);
      const __m256i zUpdate = _mm256_and_si256(_mm256_and_si256(_mm256_cmpgt_epi32(zv, zbuffv), _mm256_cmpgt_epi32(_mm256_or_si256(w2Init, _mm256_or_si256(w0Init, w1Init)), min)), zv);

      if (!_mm256_testz_si256(zUpdate, zUpdate)) {
        _mm256_stream_si256((__m256i *)(zBuffTemp), zUpdate);

        __m256 xColv = _mm256_add_ps(_mm256_set1_ps(xCol), xColAdd);
        __m256 yColv = _mm256_add_ps(_mm256_set1_ps(yCol), yColAdd);

        // Convert to ints
        xColv = _mm256_cvtps_epi32(xColv);
        yColv = _mm256_cvtps_epi32(yColv);

        yColv = _mm256_mullo_epi32(yColv, _mm256_set1_epi32(img.width));
        xColv = _mm256_add_epi32(xColv, yColv);

        xColv = _mm256_and_si256(xColv, _mm256_cmpgt_epi32(xColv, ones));

        __m256i colorsData = _mm256_i32gather_epi32(img.data, xColv, 4);
        _mm256_stream_si256((__m256i *)(colors), colorsData);

        for (unsigned x = 0; x < 8; ++x) {
          if (zBuffTemp[x]) {
            zbuff[xVal + offset] = zBuffTemp[x];
            plot(xVal, y, shader.fragmentShader(colors[x]));
          }

          ++xVal;
          shader.stepXForX();
        }
      } else {
        xVal += 8;
        // We must step 8 times.
        shader.stepXForX(8);
      }
      xCol += xColDx * 8;
      yCol += yColDx * 8;


      z += zdx8;

      if (inner < numInner - 1) {
        w0Init = _mm256_add_epi32(w0Init, a12Add8);
        w1Init = _mm256_add_epi32(w1Init, a20Add8);
        w2Init = _mm256_add_epi32(w2Init, a01Add8);
      }
    }

    const unsigned inner8 = 8 * numInner;
    w0 += A12 * inner8;
    w1 += A20 * inner8;
    w2 += A01 * inner8;

    for (x = 0; x < numOuter; ++x) {

      // If p is on or inside all edges, render pixel
      if ((w0 | w1 | w2) >= 0) {
        // Uncomment for exact z values
        if (zbuff[xVal + offset] < z) {
          zbuff[xVal + offset] = z;
          plot(xVal, y, shader.fragmentShader(img.fast_get(xCol, yCol)));
        }
      }
      w0 += A12;
      w1 += A20;
      w2 += A01;
      z += zdx;
      xCol += xColDx;
      yCol += yColDx;
      ++xVal;
      shader.stepXForX();
    }

    w0Row += B12;
    w1Row += B20;
    w2Row += B01;
    zOrig += zdy;
    xColRow += xColDy;
    yColRow += yColDy;
    offset += W;
    shader.stepYForX();
  }
}


template<typename T, typename std::enable_if<std::is_base_of<UntexturedShader, T>::value, int>::type* = nullptr>
void drawTri(const ModelInstance& m, const face& f, const vertex<float>& light, const TGAImage& img,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i) {
  printf("FOO\n");
}
