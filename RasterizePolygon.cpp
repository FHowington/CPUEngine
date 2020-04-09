#include "RasterizePolygon.h"
#include <immintrin.h>



void drawTri(const face& f,  const float light, const TGAImage& img)
{
#ifdef DEBUG
  int direction = directionality(f._v0, f._v1, f._v2);
  if (direction < 0) {
    printf("Failed\n");
    return;
  }
#endif


  static const __m256i min = _mm256_set1_epi32(-1);
  static const __m256i scale = _mm256_set_epi32(7, 6, 5, 4, 3, 2, 1, 0);
  static const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);
  static const __m256i loadOffset = _mm256_set_epi32(7*W, 6*W, 5*W, 4*W, 3*W, 2*W, W, 0);


  const int x0 = f._v0._x * halfW + halfW;
  const int x1 = f._v1._x * halfW + halfW;
  const int x2 = f._v2._x * halfW + halfW;

  const int y0 = f._v0._y * halfH + halfH;
  const int y1 = f._v1._y * halfH + halfH;
  const int y2 = f._v2._y * halfH + halfH;

  // Prevent from wasting time on polygons that have no area
  if (colinear(x0, x1, x2, y0, y1, y2)) {
    return;
  }

  const int minX = min3(x0, x1, x2);
  const int minY = min3(y0, y1, y2, true);

  const int maxX = max3(x0, x1, x2, (int)W);
  const int maxY = max3(y0, y1, y2, (int)H);

  const int bias0 = isTopLeft(f._v1, f._v2) ? 0 : -1;
  const int bias1 = isTopLeft(f._v2, f._v0) ? 0 : -1;
  const int bias2 = isTopLeft(f._v0, f._v1) ? 0 : -1;

  int w0Row = orient2d(x1, x2, minX, y1, y2, minY) + bias0;
  int w1Row = orient2d(x2, x0, minX, y2, y0, minY) + bias1;
  int w2Row = orient2d(x0, x1, minX, y0, y1, minY) + bias2;

  float wTotal = w0Row + w1Row + w2Row;
  if (!wTotal) {
    return;
  }

  const short A01 = y0 - y1;
  const short A12 = y1 - y2;
  const short A20 = y2 - y0;

  const short B01 = x1 - x0;
  const short B12 = x2 - x1;
  const short B20 = x0 - x2;

  const int z0 = f._v0._z * 0xFFFFF + 0xFFFFF; // Consider changing these into shifts, should be the same
  const int z1 = f._v1._z * 0xFFFFF + 0xFFFFF;
  const int z2 = f._v2._z * 0xFFFFF + 0xFFFFF;

  unsigned x, y, z, xVal, yVal, numInner, inner, numOuter;

  int w0, w1, w2;

  const int div = (((B20) * (-A01)) + (B01) * (A20));
  const int z10 = z1 - z0;
  const int z20 = z2 - z0;

  const int zdx = (A20 * z10 + A01 * z20) / div;
  const int zdy = (B20 * z10 + B01 * z20) / div;

  int zOrig = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, minX, minY);

  float xColRow = (f._t0x * w0Row + f._t1x * w1Row + f._t2x * w2Row) / wTotal;
  float yColRow = (f._t0y * w0Row + f._t1y * w1Row + f._t2y * w2Row) / wTotal;

  const float xColDx = (f._t0x * A12 + f._t1x * A20 + f._t2x * A01) / wTotal;
  const float yColDx = (f._t0y * A12 + f._t1y * A20 + f._t2y * A01) / wTotal;

  const float xColDy = (f._t0x * B12 + f._t1x * B20 + f._t2x * B01) / wTotal;
  const float yColDy = (f._t0y * B12 + f._t1y * B20 + f._t2y * B01) / wTotal;

  float xCol;
  float yCol;


  __m256i ones = _mm256_set1_epi64x(-1);

  // We want to always have our accessed aligned on 32 byte boundaries
  unsigned __attribute__((aligned(32))) zBuffTemp[8];
  float __attribute__((aligned(32))) xColArr[8];
  float __attribute__((aligned(32))) yColArr[8];

  const unsigned xDiff = maxX - minX;
  const unsigned yDiff = maxY - minY;

  if (xDiff > yDiff) {
    __m256i zdxAdd;
    __m256i xColAdd;
    __m256i yColAdd;
    __m256i w0Init;
    __m256i w1Init;
    __m256i w2Init;

    int A128 = 0;
    int A208 = 0;
    int A018 = 0;
    int zdx8 = 0;
    float xColDx8 = 0;
    float yColDx8 = 0;

    __m256i a12Add;
    __m256i a12Add8;

    __m256i a20Add;
    __m256i a20Add8;

    __m256i a01Add;
    __m256i a01Add8;


    // We will enter inner loop at least once, otherwise numInner is always 0
    if (xDiff > 7) {
      zdxAdd = _mm256_set_epi32(7*zdx, 6*zdx, 5*zdx, 4*zdx, 3*zdx, 2*zdx, zdx, 0);

      xColAdd = _mm256_set1_ps(xColDx);
      xColAdd = _mm256_mul_ps(scaleFloat, xColAdd);

      yColAdd = _mm256_set1_ps(yColDx);
      yColAdd = _mm256_mul_ps(scaleFloat, yColAdd);

      zdx8 = 8*zdx;
      xColDx8 = 8*xColDx;
      yColDx8 = 8*yColDx;

      a12Add = _mm256_set1_epi32(A12);
      a12Add = _mm256_mullo_epi32(a12Add, scale);
      a12Add8 = _mm256_set1_epi32(8*A12);

      a20Add = _mm256_set1_epi32(A20);
      a20Add = _mm256_mullo_epi32(a20Add, scale);
      a20Add8 = _mm256_set1_epi32(8*A20);

      a01Add = _mm256_set1_epi32(A01);
      a01Add = _mm256_mullo_epi32(a01Add, scale);
      a01Add8 = _mm256_set1_epi32(8*A01);
    }

    for (y = minY; y <= maxY; ++y) {
      unsigned offset = W * y;

      w0 = w0Row;
      w1 = w1Row;
      w2 = w2Row;
      z = zOrig;
      xCol = xColRow;
      yCol = yColRow;

      numInner = (maxX - minX) / 8;
      numOuter = (maxX - minX) % 8 + 1;


      if (numInner) {
        w0Init = _mm256_set1_epi32(w0);
        w0Init = _mm256_add_epi32(w0Init, a12Add);

        w1Init = _mm256_set1_epi32(w1);
        w1Init = _mm256_add_epi32(w1Init, a20Add);

        w2Init = _mm256_set1_epi32(w2);
        w2Init = _mm256_add_epi32(w2Init, a01Add);
      }

      xVal = minX;
      for (inner = 0; inner < numInner; ++inner) {
        // We have AVX2, lets take advantage of it!
        // We break the loop out to enable the compiler to vectorize it!
        //#pragma clang loop vectorize(enable) interleave(enable)

        // Step 1:
        // Calculate w0, w1, w2, xCol, yCol
        // For all 8 cases.
        // Next, Create mask of w0 | w1 w2 && zbuf thing
        // Then, calculate color for each of these
        // Finally, apply to both zbuff and plot the results


        // If z is all zeros, skip a bunch of this
        __m256i zbuffv = _mm256_load_si256((__m256i*)(zbuff + xVal + offset));
        int zAllZero = _mm256_testz_si256(ones, zbuffv);

        int route;
        __m256i zUpdate;

        if (!zAllZero) {
          __m256i zInit = _mm256_set1_epi32(z);
          __m256i zv = _mm256_add_epi32(zInit, zdxAdd);

          zUpdate = _mm256_and_si256(_mm256_and_si256(_mm256_cmpgt_epi32(zv, zbuffv), _mm256_cmpgt_epi32(_mm256_or_si256(w2Init, _mm256_or_si256(w0Init, w1Init)), min)), zv);

          // If this is 1, it indicates that all z values are zero
          route = _mm256_testz_si256(ones, zUpdate);
        } else {

          // All z values are zero
          // Only want to see what the barycentric coords are for each index
          zUpdate = _mm256_cmpgt_epi32(_mm256_or_si256(w2Init, _mm256_or_si256(w0Init, w1Init)), min);
          route = _mm256_testz_si256(ones, zUpdate);
        }

        if (!route) {
          if (zAllZero) {
            __m256i zInit = _mm256_set1_epi32(z);
            __m256i zv = _mm256_add_epi32(zInit, zdxAdd);
            zUpdate = _mm256_and_si256(zv, zUpdate);
          }
          __m256i xColInit = _mm256_set1_ps(xCol);
          __m256i xColv = _mm256_add_ps(xColInit, xColAdd);

          __m256i yColInit = _mm256_set1_ps(yCol);
          __m256i yColv = _mm256_add_ps(yColInit, yColAdd);

          _mm256_stream_si256((__m256i *)(zBuffTemp), zUpdate);
          _mm256_stream_ps(yColArr, yColv);
          _mm256_stream_ps(xColArr, xColv);

          for (unsigned x = 0; x < 8; ++x) {
            if (zBuffTemp[x]) {
              fcolor c = img.fast_get(xColArr[x], yColArr[x]);
              zbuff[xVal + offset] = zBuffTemp[x];
              plot(xVal, y, c);
            }
            ++xVal;
          }
        } else {
          xVal += 8;
        }

        z += zdx8;
        xCol += xColDx8;
        yCol += yColDx8;

        if (inner < numInner - 1) {
          w0Init = _mm256_add_epi32(w0Init, a12Add8);
          w1Init = _mm256_add_epi32(w1Init, a20Add8);
          w2Init = _mm256_add_epi32(w2Init, a01Add8);
        }
      }

      unsigned inner8 = 8 * numInner;
      w0 += A12 * inner8;
      w1 += A20 * inner8;
      w2 += A01 * inner8;

      for (x = 0; x < numOuter; ++x) {

        // If p is on or inside all edges, render pixel
        if ((w0 | w1 | w2) >= 0) {
          // Uncomment for exact z values
          //z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);
          if (zbuff[xVal + offset] < z) {
            fcolor c = img.get_and_light(xCol, yCol, 1);
            zbuff[xVal + offset] = z;
            plot(xVal, y, c);
          }
        }
        w0 += A12;
        w1 += A20;
        w2 += A01;
        z += zdx;
        xCol += xColDx;
        yCol += yColDx;
        ++xVal;
      }

      w0Row += B12;
      w1Row += B20;
      w2Row += B01;
      zOrig += zdy;
      xColRow += xColDy;
      yColRow += yColDy;
    }
  } else {
    int zdy8;
    float xColDy8;
    float yColDy8;

    __m256i zdyAdd;
    __m256i xColRowAdd;
    __m256i yColRowAdd;
    __m256i w0RowInit;
    __m256i w1RowInit;
    __m256i w2RowInit;


    __m256i b12Add;
    __m256i b12Add8;

    __m256i b20Add;
    __m256i b20Add8;

    __m256i b01Add;
    __m256i b01Add8;

    // For row only
    w0 = w0Row;
    w1 = w1Row;
    w2 = w2Row;
    xCol = xColRow;
    yCol = yColRow;

    if (yDiff > 7) {
      zdyAdd = _mm256_set_epi32(7*zdy, 6*zdy, 5*zdy, 4*zdy, 3*zdy, 2*zdy, zdy, 0);
      xColRowAdd = _mm256_set1_ps(xColDy);
      xColRowAdd = _mm256_mul_ps(scaleFloat, xColRowAdd);
      yColRowAdd = _mm256_set1_ps(yColDy);
      yColRowAdd = _mm256_mul_ps(scaleFloat, yColRowAdd);


      b12Add = _mm256_set1_epi32(B12);
      b12Add = _mm256_mullo_epi32(b12Add, scale);
      b12Add8 = _mm256_set1_epi32(8*B12);


      b20Add = _mm256_set1_epi32(B20);
      b20Add = _mm256_mullo_epi32(b20Add, scale);
      b20Add8 = _mm256_set1_epi32(8*B20);


      b01Add = _mm256_set1_epi32(B01);
      b01Add = _mm256_mullo_epi32(b01Add, scale);
      b01Add8 = _mm256_set1_epi32(8*B01);

      zdy8 = 8*zdy;
      xColDy8 = 8*xColDy;
      yColDy8 = 8*yColDy;
    }


    for (x = minX; x <= maxX; ++x) {
      w0Row = w0;
      w1Row = w1;
      w2Row = w2;
      z = zOrig;
      xColRow = xCol;
      yColRow = yCol;

      numInner = (maxY - minY) / 8;
      numOuter = (maxY - minY) % 8 + 1;

      if (numInner) {
        w0RowInit = _mm256_set1_epi32(w0Row);
        w0RowInit = _mm256_add_epi32(w0RowInit, b12Add);

        w1RowInit = _mm256_set1_epi32(w1Row);
        w1RowInit = _mm256_add_epi32(w1RowInit, b20Add);

        w2RowInit = _mm256_set1_epi32(w2Row);
        w2RowInit = _mm256_add_epi32(w2RowInit, b01Add);
      }

      yVal = minY;

      unsigned offset = minY * W + x;

      for (inner = 0; inner < numInner; ++inner) {

        __m256i zbuffv = _mm256_i32gather_epi32(zbuff + offset, loadOffset, 4);
        int zAllZero = _mm256_testz_si256(ones, zbuffv);

        int route;
        __m256i zUpdate;

        if (!zAllZero) {
          __m256i zInit = _mm256_set1_epi32(z);
          __m256i zv = _mm256_add_epi32(zInit, zdyAdd);
          zUpdate = _mm256_and_si256(_mm256_and_si256(_mm256_cmpgt_epi32(zv, zbuffv),  _mm256_cmpgt_epi32(_mm256_or_si256(w2RowInit, _mm256_or_si256(w0RowInit, w1RowInit)), min)), zv);

          // If this is 1, it indicates that all z values are zero
          route = _mm256_testz_si256(ones, zUpdate);
        } else {
          // All z values are zero
          // Only want to see what the barycentric coords are for each index
          zUpdate = _mm256_cmpgt_epi32(_mm256_or_si256(w2RowInit, _mm256_or_si256(w0RowInit, w1RowInit)), min);
          route = _mm256_testz_si256(ones, zUpdate);
        }

        if (!route) {
          if (zAllZero) {
            __m256i zInit = _mm256_set1_epi32(z);
            __m256i zv = _mm256_add_epi32(zInit, zdyAdd);
            zUpdate = _mm256_and_si256(zv, zUpdate);
          }

          __m256i xColRowInit = _mm256_set1_ps(xColRow);
          __m256i xColRowv = _mm256_add_ps(xColRowInit, xColRowAdd);

          __m256i yColRowInit = _mm256_set1_ps(yColRow);
          __m256i yColRowv = _mm256_add_ps(yColRowInit, yColRowAdd);

          _mm256_stream_si256((__m256i *)(zBuffTemp), zUpdate);
          _mm256_stream_ps(yColArr, yColRowv);
          _mm256_stream_ps(xColArr, xColRowv);

          for (unsigned y = 0; y < 8; ++y) {
            if (zBuffTemp[y]) {
              fcolor c = img.fast_get(xColArr[y], yColArr[y]);

              zbuff[yVal * W + x] = zBuffTemp[y];
              plot(x, yVal, c);
            }
            ++yVal;
          }
        } else {
          yVal += 8;
        }

        z += zdy8;
        xColRow += xColDy8;
        yColRow += yColDy8;

        if (inner < numInner - 1) {
          w0RowInit = _mm256_add_epi32(w0RowInit, b12Add8);
          w1RowInit = _mm256_add_epi32(w1RowInit, b20Add8);
          w2RowInit = _mm256_add_epi32(w2RowInit, b01Add8);
        }
        offset += 8*W;
      }

      unsigned inner8 = numInner * 8;

      w0Row += B12 * inner8;
      w1Row += B20 * inner8;
      w2Row += B01 * inner8;

      for (y = 0; y < numOuter; ++y) {
        // If p is on or inside all edges, render pixel
        if ((w0Row | w1Row | w2Row) >= 0) {
          // Uncomment for exact z values
          //z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);
          if (zbuff[yVal * W + x] < z) {
            fcolor c = img.fast_get(xColRow, yColRow);
            zbuff[yVal * W + x] = z;
            plot(x, yVal, c);
          }
        }
        w0Row += B12;
        w1Row += B20;
        w2Row += B01;
        z += zdy;
        xColRow += xColDy;
        yColRow += yColDy;
        ++yVal;
      }

      w0 += A12;
      w1 += A20;
      w2 += A01;
      zOrig += zdx;
      xCol += xColDx;
      yCol += yColDx;
    }
  }
}

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

float zPos(const int cx, const int bx, const int ax, const int cy, const int by, const int ay, const int cz, const int bz, const int az, const int x, const int y) {
  return  az + ((bx - ax) * (cz - az) - (cx - ax) * (bz - az))*(y - ay)/ ((bx-ax) * (cy-ay) - (cx-ax) * (by-ay)) - ((by-ay) * (cz-az) - (cy-ay)*(bz-az))*(x-ax)/((bx-ax)*(cy-ay) - (cx-ax) * (by-ay));
}

void plot(unsigned x, unsigned y, const unsigned color)
{
  // We want to flip pos y to mean "up"
#ifdef DEBUG
  if(pixels[(H-y)*W+x] != focolor::blank)
    pixels[(H-y)*W+x] = fcolor::duplicate;
  else
#endif
    pixels[(H-y)*W+x] = color;
}
