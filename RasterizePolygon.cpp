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

  const int A01 = y0 - y1;
  const int A12 = y1 - y2;
  const int A20 = y2 - y0;

  const int B01 = x1 - x0;
  const int B12 = x2 - x1;
  const int B20 = x0 - x2;

  const int minX = min3(x0, x1, x2);
  const int minY = min3(y0, y1, y2, true);

  const int maxX = max3(x0, x1, x2, (int)W);
  const int maxY = max3(y0, y1, y2, (int)H);

  const int bias0 = isTopLeft(f._v1, f._v2) ? 0 : -1;
  const int bias1 = isTopLeft(f._v2, f._v0) ? 0 : -1;
  const int bias2 = isTopLeft(f._v0, f._v1) ? 0 : -1;

  int w0_row = orient2d(x1, x2, minX, y1, y2, minY) + bias0;
  int w1_row = orient2d(x2, x0, minX, y2, y0, minY) + bias1;
  int w2_row = orient2d(x0, x1, minX, y0, y1, minY) + bias2;

  float wTotal = w0_row + w1_row + w2_row;
  if (!wTotal) {
    return;
  }

  const int z0 = f._v0._z * 0xFFFFF + 0xFFFFF; // Consider changing these into shifts, should be the same
  const int z1 = f._v1._z * 0xFFFFF + 0xFFFFF;
  const int z2 = f._v2._z * 0xFFFFF + 0xFFFFF;

  unsigned x, y, z, xVal, xValInner, numInner, xInner, numOuter, yValInner, yInner;

  int w0, w1, w2;

  int div = (((B20) * (-A01)) + (B01) * (A20));
  int z10 = z1 - z0;
  int z20 = z2 - z0;

  int zdx = (A20 * z10 + A01 * z20) / div;
  int zdy = (B20 * z10 + B01 * z20) / div;

  int zOrig = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, minX, minY);

  float xCol_row = (f._t0x * w0_row + f._t1x * w1_row + f._t2x * w2_row) / wTotal;
  float yCol_row = (f._t0y * w0_row + f._t1y * w1_row + f._t2y * w2_row) / wTotal;

  float xColDx = (f._t0x * A12 + f._t1x * A20 + f._t2x * A01) / wTotal;
  float yColDx = (f._t0y * A12 + f._t1y * A20 + f._t2y * A01) / wTotal;

  float xColDy = (f._t0x * B12 + f._t1x * B20 + f._t2x * B01) / wTotal;
  float yColDy = (f._t0y * B12 + f._t1y * B20 + f._t2y * B01) / wTotal;

  float xCol;
  float yCol;

  __m256i min = _mm256_set1_epi32(-1);

  __m256i a12_add = _mm256_set_epi32(7*A12, 6*A12, 5*A12, 4*A12, 3*A12, 2*A12, A12, 0);
  __m256i a12_add_8 = _mm256_set1_epi32(8*A12);

  __m256i a20_add = _mm256_set_epi32(7*A20, 6*A20, 5*A20, 4*A20, 3*A20, 2*A20, A20, 0);
  __m256i a20_add_8 = _mm256_set1_epi32(8*A20);

  __m256i a01_add = _mm256_set_epi32(7*A01, 6*A01, 5*A01, 4*A01, 3*A01, 2*A01, A01, 0);
  __m256i a01_add_8 = _mm256_set1_epi32(8*A01);

  __m256i zdx_add = _mm256_set_epi32(7*zdx, 6*zdx, 5*zdx, 4*zdx, 3*zdx, 2*zdx, zdx, 0);
  __m256i xCol_add = _mm256_set_ps(7*xColDx, 6*xColDx, 5*xColDx, 4*xColDx, 3*xColDx, 2*xColDx, xColDx, 0);
  __m256i yCol_add = _mm256_set_ps(7*yColDx, 6*yColDx, 5*yColDx, 4*yColDx, 3*yColDx, 2*yColDx, yColDx, 0);




  __m256i b12_add = _mm256_set_epi32(7*B12, 6*B12, 5*B12, 4*B12, 3*B12, 2*B12, B12, 0);
  __m256i b12_add_8 = _mm256_set1_epi32(8*B12);

  __m256i b20_add = _mm256_set_epi32(7*B20, 6*B20, 5*B20, 4*B20, 3*B20, 2*B20, B20, 0);
  __m256i b20_add_8 = _mm256_set1_epi32(8*B20);

  __m256i b01_add = _mm256_set_epi32(7*B01, 6*B01, 5*B01, 4*B01, 3*B01, 2*B01, B01, 0);
  __m256i b01_add_8 = _mm256_set1_epi32(8*B01);

  __m256i zdy_add = _mm256_set_epi32(7*zdy, 6*zdy, 5*zdy, 4*zdy, 3*zdy, 2*zdy, zdy, 0);
  __m256i xCol_row_add = _mm256_set_ps(7*xColDy, 6*xColDy, 5*xColDy, 4*xColDy, 3*xColDy, 2*xColDy, xColDy, 0);
  __m256i yCol_row_add = _mm256_set_ps(7*yColDy, 6*yColDy, 5*yColDy, 4*yColDy, 3*yColDy, 2*yColDy, yColDy, 0);



  int A12_8 = 8*A12;
  int A20_8 = 8*A20;
  int A01_8 = 8*A01;
  int zdx_8 = 8*zdx;
  float xColDx_8 = 8*xColDx;
  float yColDx_8 = 8*yColDx;


  int B12_8 = 8*B12;
  int B20_8 = 8*B20;
  int B01_8 = 8*B01;
  int zdy_8 = 8*zdy;
  float xColDy_8 = 8*xColDy;
  float yColDy_8 = 8*yColDy;

  // We want to always have our accessed aligned on 32 byte boundaries
  unsigned xDiff = maxX - minX;

  __m256i ones = _mm256_set1_epi64x(-1);

  unsigned __attribute__((aligned(32))) zBuffTemp[8];
  float __attribute__((aligned(32))) xColArr[8];
  float __attribute__((aligned(32))) yColArr[8];



  // For row only
  w0 = w0_row;
  w1 = w1_row;
  w2 = w2_row;
  xCol = xCol_row;
  yCol = yCol_row;



  for (x = minX; x <= maxX; ++x) {
    w0_row = w0;
    w1_row = w1;
    w2_row = w2;
    z = zOrig;
    xCol_row = xCol;
    yCol_row = yCol;

    numInner = (maxY - minY) / 8;
    numOuter = (maxY - minY) % 8 + 1;

    __m256i w0_row_init = _mm256_set1_epi32(w0_row);
    w0_row_init = _mm256_add_epi32(w0_row_init, b12_add);

    __m256i w1_row_init = _mm256_set1_epi32(w1_row);
    w1_row_init = _mm256_add_epi32(w1_row_init, b20_add);

    __m256i w2_row_init = _mm256_set1_epi32(w2_row);
    w2_row_init = _mm256_add_epi32(w2_row_init, b01_add);


    for (yInner = 0; yInner < numInner; ++yInner) {
      __m256i xCol_row_init = _mm256_set1_ps(xCol_row);
      __m256i xCol_rowv = _mm256_add_ps(xCol_row_init, xCol_row_add);

      __m256i yCol_row_init = _mm256_set1_ps(yCol_row);
      __m256i yCol_rowv = _mm256_add_ps(yCol_row_init, yCol_row_add);

      __m256i z_init = _mm256_set1_epi32(z);
      __m256i zv = _mm256_add_epi32(z_init, zdy_add);

      // This will NOT WORK. NEED TO LOAD IN DIFFERENT DIRECTION
      unsigned offset = (yInner * 8 + minY) * W + x;
      __m256i zbuffv =   _mm256_set_epi32(zbuff[offset + 7*W], zbuff[offset + 6*W], zbuff[offset + 5*W], zbuff[offset + 4*W], zbuff[offset + 3*W], zbuff[offset + 2*W], zbuff[offset + W], zbuff[offset]);


      // We are now inside both if statements
      // We are ANDING with 0 for every value of vector that should not be updated
      // Otherwise, we are updating
      __m256i zupdate = _mm256_and_si256(_mm256_and_si256(_mm256_cmpgt_epi32(zv, zbuffv),  _mm256_cmpgt_epi32(_mm256_or_si256(w2_row_init, _mm256_or_si256(w0_row_init, w1_row_init)), min)), zv);

      // If this is 1, it indicates that all z values are zero
      int route = _mm256_testz_si256(ones, zupdate);

      if (!route) {
        _mm256_stream_si256((__m256i *)(zBuffTemp), zupdate);
        _mm256_stream_ps(yColArr, yCol_rowv);
        _mm256_stream_ps(xColArr, xCol_rowv);

        for (unsigned y = 0; y < 8; ++y) {
          if (zBuffTemp[y]) {
            yValInner = yInner * 8 + y + minY;
            fcolor c = img.fast_get(xColArr[y], yColArr[y]);
            //fcolor c(0,255 - 30*y, 0, 255-30*y);

            zbuff[yValInner * W + x] = zBuffTemp[y];
            plot(x, yValInner, c);
          }
        }
      }

      z += zdy_8;
      xCol_row += xColDy_8;
      yCol_row += yColDy_8;


      if (yInner < numInner - 1) {
        w0_row_init = _mm256_add_epi32(w0_row_init, b12_add_8);
        w1_row_init = _mm256_add_epi32(w1_row_init, b20_add_8);
        w2_row_init = _mm256_add_epi32(w2_row_init, b01_add_8);
      }
    }

    w0_row += B12_8 * numInner;
    w1_row += B20_8 * numInner;
    w2_row += B01_8 * numInner;

    for (y = 0; y < numOuter; ++y) {
      yValInner = minY + y + 8 * numInner;

      // If p is on or inside all edges, render pixel
      if ((w0_row | w1_row | w2_row) >= 0) {
        // Uncomment for exact z values
        //z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);
        if (zbuff[yValInner * W + x] < z) {
          fcolor c = img.get_and_light(xCol_row, yCol_row, 1);
          zbuff[yValInner * W + x] = z;
          plot(x, yValInner, c);
        }
      }
      w0_row += B12;
      w1_row += B20;
      w2_row += B01;
      z += zdy;
      xCol_row += xColDy;
      yCol_row += yColDy;
    }

    w0 += A12;
    w1 += A20;
    w2 += A01;
    zOrig += zdx;
    xCol += xColDx;
    yCol += yColDx;
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
