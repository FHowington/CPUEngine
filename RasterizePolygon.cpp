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

  const short A01 = y0 - y1;
  const short A12 = y1 - y2;
  const short A20 = y2 - y0;

  const short B01 = x1 - x0;
  const short B12 = x2 - x1;
  const short B20 = x0 - x2;

  const int z0 = f._v0._z * 0xFFFFF + 0xFFFFF; // Consider changing these into shifts, should be the same
  const int z1 = f._v1._z * 0xFFFFF + 0xFFFFF;
  const int z2 = f._v2._z * 0xFFFFF + 0xFFFFF;

  unsigned x, y, z, xVal, xValInner, numInner, xInner, numOuter, yValInner, yInner;

  int w0, w1, w2;

  const int div = (((B20) * (-A01)) + (B01) * (A20));
  const int z10 = z1 - z0;
  const int z20 = z2 - z0;

  const int zdx = (A20 * z10 + A01 * z20) / div;
  const int zdy = (B20 * z10 + B01 * z20) / div;

  int zOrig = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, minX, minY);

  float xCol_row = (f._t0x * w0_row + f._t1x * w1_row + f._t2x * w2_row) / wTotal;
  float yCol_row = (f._t0y * w0_row + f._t1y * w1_row + f._t2y * w2_row) / wTotal;

  const float xColDx = (f._t0x * A12 + f._t1x * A20 + f._t2x * A01) / wTotal;
  const float yColDx = (f._t0y * A12 + f._t1y * A20 + f._t2y * A01) / wTotal;

  const float xColDy = (f._t0x * B12 + f._t1x * B20 + f._t2x * B01) / wTotal;
  const float yColDy = (f._t0y * B12 + f._t1y * B20 + f._t2y * B01) / wTotal;

  float xCol;
  float yCol;

  const __m256i min = _mm256_set1_epi32(-1);
  const __m256i scale = _mm256_set_epi32(7, 6, 5, 4, 3, 2, 1, 0);
  const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);

  __m256i loadOffset = _mm256_set1_epi32(W);
  loadOffset = _mm256_mullo_epi32(scale, loadOffset);

  __m256i ones = _mm256_set1_epi64x(-1);

  // We want to always have our accessed aligned on 32 byte boundaries
  unsigned __attribute__((aligned(32))) zBuffTemp[8];
  float __attribute__((aligned(32))) xColArr[8];
  float __attribute__((aligned(32))) yColArr[8];

  const unsigned xDiff = maxX - minX;
  const unsigned yDiff = maxY - minY;

  if (xDiff > yDiff) {
    __m256i zdx_add;
    __m256i xCol_add;
    __m256i yCol_add;
    __m256i w0_init;
    __m256i w1_init;
    __m256i w2_init;

    int A12_8 = 0;
    int A20_8 = 0;
    int A01_8 = 0;
    int zdx_8 = 0;
    float xColDx_8 = 0;
    float yColDx_8 = 0;

    __m256i a12_add;
    __m256i a12_add_8;

    __m256i a20_add;
    __m256i a20_add_8;

    __m256i a01_add;
    __m256i a01_add_8;


    // We will enter inner loop at least once, otherwise numInner is always 0
    if (xDiff > 7) {
      zdx_add = _mm256_set_epi32(7*zdx, 6*zdx, 5*zdx, 4*zdx, 3*zdx, 2*zdx, zdx, 0);

      xCol_add = _mm256_set1_ps(xColDx);
      xCol_add = _mm256_mul_ps(scaleFloat, xCol_add);
      yCol_add = _mm256_set1_ps(yColDx);
      yCol_add = _mm256_mul_ps(scaleFloat, yCol_add);

      A12_8 = 8*A12;
      A20_8 = 8*A20;
      A01_8 = 8*A01;
      zdx_8 = 8*zdx;
      xColDx_8 = 8*xColDx;
      yColDx_8 = 8*yColDx;

      a12_add = _mm256_set1_epi32(A12);
      a12_add = _mm256_mullo_epi32(a12_add, scale);
      a12_add_8 = _mm256_set1_epi32(8*A12);

      a20_add = _mm256_set1_epi32(A20);
      a20_add = _mm256_mullo_epi32(a20_add, scale);
      a20_add_8 = _mm256_set1_epi32(8*A20);

      a01_add = _mm256_set1_epi32(A01);
      a01_add = _mm256_mullo_epi32(a01_add, scale);
      a01_add_8 = _mm256_set1_epi32(8*A01);
    }

    for (y = minY; y <= maxY; ++y) {
      unsigned offset = W * y;

      w0 = w0_row;
      w1 = w1_row;
      w2 = w2_row;
      z = zOrig;
      xCol = xCol_row;
      yCol = yCol_row;

      numInner = (maxX - minX) / 8;
      numOuter = (maxX - minX) % 8 + 1;


      if (numInner) {
        w0_init = _mm256_set1_epi32(w0);
        w0_init = _mm256_add_epi32(w0_init, a12_add);

        w1_init = _mm256_set1_epi32(w1);
        w1_init = _mm256_add_epi32(w1_init, a20_add);

        w2_init = _mm256_set1_epi32(w2);
        w2_init = _mm256_add_epi32(w2_init, a01_add);
      }

      xVal = minX;
      for (xInner = 0; xInner < numInner; ++xInner) {
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
        __m256i zupdate;

        if (!zAllZero) {
          __m256i z_init = _mm256_set1_epi32(z);
          __m256i zv = _mm256_add_epi32(z_init, zdx_add);

          zupdate = _mm256_and_si256(_mm256_and_si256(_mm256_cmpgt_epi32(zv, zbuffv), _mm256_cmpgt_epi32(_mm256_or_si256(w2_init, _mm256_or_si256(w0_init, w1_init)), min)), zv);

          // If this is 1, it indicates that all z values are zero
          route = _mm256_testz_si256(ones, zupdate);
        } else {
          // All z values are zero
          // Only want to see what the barycentric coords are for each index
          zupdate = _mm256_cmpgt_epi32(_mm256_or_si256(w2_init, _mm256_or_si256(w0_init, w1_init)), min);
          route = _mm256_testz_si256(ones, zupdate);
        }

        if (!route) {
          if (zAllZero) {
            __m256i z_init = _mm256_set1_epi32(z);
            __m256i zv = _mm256_add_epi32(z_init, zdx_add);
            zupdate = _mm256_and_si256(zv, zupdate);
          }
          __m256i xCol_init = _mm256_set1_ps(xCol);
          __m256i xColv = _mm256_add_ps(xCol_init, xCol_add);

          __m256i yCol_init = _mm256_set1_ps(yCol);
          __m256i yColv = _mm256_add_ps(yCol_init, yCol_add);

          _mm256_stream_si256((__m256i *)(zBuffTemp), zupdate);
          _mm256_stream_ps(yColArr, yColv);
          _mm256_stream_ps(xColArr, xColv);

          for (unsigned x = 0; x < 8; ++x) {
            if (zBuffTemp[x]) {
              xValInner = xVal + x;
              fcolor c = img.fast_get(xColArr[x], yColArr[x]);
              zbuff[xValInner + offset] = zBuffTemp[x];
              plot(xValInner, y, c);
            }
          }
        }

        z += zdx_8;
        xCol += xColDx_8;
        yCol += yColDx_8;
        xVal += 8;

        if (xInner < numInner - 1) {
          w0_init = _mm256_add_epi32(w0_init, a12_add_8);
          w1_init = _mm256_add_epi32(w1_init, a20_add_8);
          w2_init = _mm256_add_epi32(w2_init, a01_add_8);
        }
      }

      w0 += A12_8 * numInner;
      w1 += A20_8 * numInner;
      w2 += A01_8 * numInner;

      for (x = 0; x < numOuter; ++x) {
        xValInner = xVal + x;

        // If p is on or inside all edges, render pixel
        if ((w0 | w1 | w2) >= 0) {
          // Uncomment for exact z values
          //z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);
          if (zbuff[xValInner + offset] < z) {
            fcolor c = img.get_and_light(xCol, yCol, 1);
            zbuff[xValInner + offset] = z;
            plot(xValInner, y, c);
          }
        }
        w0 += A12;
        w1 += A20;
        w2 += A01;
        z += zdx;
        xCol += xColDx;
        yCol += yColDx;
      }

      w0_row += B12;
      w1_row += B20;
      w2_row += B01;
      zOrig += zdy;
      xCol_row += xColDy;
      yCol_row += yColDy;
    }
  } else {

    int B12_8;
    int B20_8;
    int B01_8;
    int zdy_8;
    float xColDy_8;
    float yColDy_8;

    __m256i zdy_add;
    __m256i xCol_row_add;
    __m256i yCol_row_add;
    __m256i w0_row_init;
    __m256i w1_row_init;
    __m256i w2_row_init;


    __m256i b12_add;
    __m256i b12_add_8;

    __m256i b20_add;
    __m256i b20_add_8;

    __m256i b01_add;
    __m256i b01_add_8;

    // For row only
    w0 = w0_row;
    w1 = w1_row;
    w2 = w2_row;
    xCol = xCol_row;
    yCol = yCol_row;

    if (yDiff > 7) {
      zdy_add = _mm256_set_epi32(7*zdy, 6*zdy, 5*zdy, 4*zdy, 3*zdy, 2*zdy, zdy, 0);
      xCol_row_add = _mm256_set1_ps(xColDy);
      xCol_row_add = _mm256_mul_ps(scaleFloat, xCol_row_add);
      yCol_row_add = _mm256_set1_ps(yColDy);
      yCol_row_add = _mm256_mul_ps(scaleFloat, yCol_row_add);


      b12_add = _mm256_set1_epi32(B12);
      b12_add = _mm256_mullo_epi32(b12_add, scale);
      b12_add_8 = _mm256_set1_epi32(8*B12);


      b20_add = _mm256_set1_epi32(B20);
      b20_add = _mm256_mullo_epi32(b20_add, scale);
      b20_add_8 = _mm256_set1_epi32(8*B20);


      b01_add = _mm256_set1_epi32(B01);
      b01_add = _mm256_mullo_epi32(b01_add, scale);
      b01_add_8 = _mm256_set1_epi32(8*B01);

      B12_8 = 8*B12;
      B20_8 = 8*B20;
      B01_8 = 8*B01;
      zdy_8 = 8*zdy;
      xColDy_8 = 8*xColDy;
      yColDy_8 = 8*yColDy;
    }


    for (x = minX; x <= maxX; ++x) {
      w0_row = w0;
      w1_row = w1;
      w2_row = w2;
      z = zOrig;
      xCol_row = xCol;
      yCol_row = yCol;

      numInner = (maxY - minY) / 8;
      numOuter = (maxY - minY) % 8 + 1;

      if (numInner) {
        w0_row_init = _mm256_set1_epi32(w0_row);
        w0_row_init = _mm256_add_epi32(w0_row_init, b12_add);

        w1_row_init = _mm256_set1_epi32(w1_row);
        w1_row_init = _mm256_add_epi32(w1_row_init, b20_add);

        w2_row_init = _mm256_set1_epi32(w2_row);
        w2_row_init = _mm256_add_epi32(w2_row_init, b01_add);
      }

      for (yInner = 0; yInner < numInner; ++yInner) {

        unsigned offset = (yInner * 8 + minY) * W + x;

        // Switch this to a gather instruction
        //__m256i zbuffv = _mm256_set_epi32(zbuff[offset + 7*W], zbuff[offset + 6*W], zbuff[offset + 5*W], zbuff[offset + 4*W], zbuff[offset + 3*W], zbuff[offset + 2*W], zbuff[offset + W], zbuff[offset]);
        __m256i zbuffv = _mm256_i32gather_epi32(zbuff + offset, loadOffset, 4);
        //printf("This: %d %d %d %d %d %d %d %d\n", _mm256_extract_epi32(zbuffv, 0), _mm256_extract_epi32(zbuffv, 1), _mm256_extract_epi32(zbuffv, 2), _mm256_extract_epi32(zbuffv, 3), _mm256_extract_epi32(zbuffv, 4), _mm256_extract_epi32(zbuffv, 5), _mm256_extract_epi32(zbuffv, 6), _mm256_extract_epi32(zbuffv, 7));

        //printf("vs: %d %d %d %d %d %d %d %d\n", _mm256_extract_epi32(zbuffv2, 0), _mm256_extract_epi32(zbuffv2, 1), _mm256_extract_epi32(zbuffv2, 2), _mm256_extract_epi32(zbuffv2, 3), _mm256_extract_epi32(zbuffv2, 4), _mm256_extract_epi32(zbuffv2, 5), _mm256_extract_epi32(zbuffv2, 6), _mm256_extract_epi32(zbuffv2, 7));
        int zAllZero = _mm256_testz_si256(ones, zbuffv);

        int route;
        __m256i zupdate;

        if (!zAllZero) {
          __m256i z_init = _mm256_set1_epi32(z);
          __m256i zv = _mm256_add_epi32(z_init, zdy_add);
          zupdate = _mm256_and_si256(_mm256_and_si256(_mm256_cmpgt_epi32(zv, zbuffv),  _mm256_cmpgt_epi32(_mm256_or_si256(w2_row_init, _mm256_or_si256(w0_row_init, w1_row_init)), min)), zv);

          // If this is 1, it indicates that all z values are zero
          route = _mm256_testz_si256(ones, zupdate);
        } else {
          // All z values are zero
          // Only want to see what the barycentric coords are for each index
          zupdate = _mm256_cmpgt_epi32(_mm256_or_si256(w2_row_init, _mm256_or_si256(w0_row_init, w1_row_init)), min);
          route = _mm256_testz_si256(ones, zupdate);
        }

        if (!route) {
          if (zAllZero) {
            __m256i z_init = _mm256_set1_epi32(z);
            __m256i zv = _mm256_add_epi32(z_init, zdy_add);
            zupdate = _mm256_and_si256(zv, zupdate);
          }

          __m256i xCol_row_init = _mm256_set1_ps(xCol_row);
          __m256i xCol_rowv = _mm256_add_ps(xCol_row_init, xCol_row_add);

          __m256i yCol_row_init = _mm256_set1_ps(yCol_row);
          __m256i yCol_rowv = _mm256_add_ps(yCol_row_init, yCol_row_add);

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
        //if (numOuter > 3) {
        //  printf("Vecotrize me!\n");
        //}
        yValInner = minY + y + 8 * numInner;

        // If p is on or inside all edges, render pixel
        if ((w0_row | w1_row | w2_row) >= 0) {
          // Uncomment for exact z values
          //z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);
          if (zbuff[yValInner * W + x] < z) {
            fcolor c = img.fast_get(xCol_row, yCol_row);
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
