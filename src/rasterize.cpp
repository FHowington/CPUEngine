#include <immintrin.h>
#include "geometry.h"
#include "rasterize.h"

const unsigned depth = 0XFFFF;

const matrix<4,4> viewport(const int x, const int y, const int w, const int h) {
  matrix m = matrix<4,4>::identity();
  float focalLength = -2.0/3.0;
  m.set(3, 0, x);
  m.set(3, 1, y);
  m.set(3, 2, depth/2.f);
  m.set(0, 0, w);
  m.set(1, 1, h);
  m.set(2,2, depth/1.5f);
  return m;
}


#ifdef __AVX2__
template<typename T, typename std::enable_if<std::is_base_of<TexturedShader, T>::value, int>::type*>
void drawTri(const ModelInstance& m, const face& f, const vertex<float>& light,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i) {

  // These are reused for every triangle in the model
  static const __m256i min = _mm256_set1_epi32(-1);
  static const __m256i scale = _mm256_set_epi32(7, 6, 5, 4, 3, 2, 1, 0);
  static const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);
  static const __m256i ones = _mm256_set1_epi32(-1);

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

  // If all three are positive, the object is behind the camera
  if ((v0i._z | v1i._z | v2i._z) > 0) {
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
  //unsigned __attribute__((aligned(32))) zBuffTemp2[8];
  unsigned __attribute__((aligned(32))) colors[8];
  unsigned __attribute__((aligned(32))) bufferTemp[8];


  T shader(m, f, light, A12, A20, A01, B12, B20, B01, wTotal, w0Row, w1Row, w2Row);
  const TGAImage& img = *m._texture;
  const __m256i textureClip = _mm256_set1_epi32(img.width *img.height);

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
    if (numOuter) {
      ++numInner;
    }

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
      const __m256i needsUpdate = _mm256_and_si256(_mm256_cmpgt_epi32(zv, zbuffv), _mm256_cmpgt_epi32(_mm256_or_si256(w2Init, _mm256_or_si256(w0Init, w1Init)), min));


      // USE BLEND!!

      if (!_mm256_testz_si256(needsUpdate, needsUpdate)) {
        //const __m256i zUpdate2 = _mm256_blendv_epi8(zbuffv, zv, needsUpdate);
        const __m256i zUpdate = _mm256_and_si256(needsUpdate, zv);

        _mm256_stream_si256((__m256i *)(zBuffTemp), zUpdate);
        //_mm256_stream_si256((__m256i *)(zBuffTemp2), zUpdate2);

        //const __m256i colorV = _mm256_load_si256((__m256i*)(pixels + xVal + offset));

        __m256 xColv = _mm256_add_ps(_mm256_set1_ps(xCol), xColAdd);
        __m256 yColv = _mm256_add_ps(_mm256_set1_ps(yCol), yColAdd);

        // Convert to ints
        xColv = _mm256_cvtps_epi32(xColv);
        yColv = _mm256_cvtps_epi32(yColv);

        yColv = _mm256_mullo_epi32(yColv, _mm256_set1_epi32(img.width));
        xColv = _mm256_add_epi32(xColv, yColv);

        xColv = _mm256_and_si256(xColv, _mm256_cmpgt_epi32(xColv, ones));

        __m256i colorsData = _mm256_i32gather_epi32(img.data, xColv, 4);
        //colorsData = _mm256_blendv_epi8(colorV, colorsData, needsUpdate);

        _mm256_stream_si256((__m256i *)(colors), colorsData);

        //_mm256_stream_si256((__m256i*)(zbuff + xVal + offset), zUpdate2);


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
      } else {
        w0 = _mm256_extract_epi32(w0Init, 7) + A12;
        w1 = _mm256_extract_epi32(w1Init, 7) + A20;
        w2 = _mm256_extract_epi32(w2Init, 7) + A01;
      }
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
#else
template<typename T, typename std::enable_if<std::is_base_of<TexturedShader, T>::value, int>::type*>
void drawTri(const ModelInstance& m, const face& f, const vertex<float>& light,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i) {

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

  unsigned offset = minY * W;
  float textureOffset = yColRow;
  const float yColDy4 = 4 * yColDy;

  T shader(m, f, light, A12, A20, A01, B12, B20, B01, wTotal, w0Row, w1Row, w2Row);
  const TGAImage& img = *m._texture;

  for (y = minY; y <= maxY; ++y) {
    w0 = w0Row;
    w1 = w1Row;
    w2 = w2Row;
    z = zOrig;
    xCol = xColRow;
    yCol = yColRow;

    for (x = minX; x <= maxX; ++x) {
      // If p is on or inside all edges, render pixel
      if ((w0 | w1 | w2) >= 0) {
        // Uncomment for exact z values
        //z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);
        if (zbuff[x + offset] < z) {
          plot(x, y, shader.fragmentShader(img.fast_get(xCol, yCol)));
          zbuff[x + offset] = z;
        }
      }
      w0 += A12;
      w1 += A20;
      w2 += A01;
      z += zdx;
      xCol += xColDx;
      yCol += yColDx;
      shader.stepXForX();
    }

    w0Row += B12;
    w1Row += B20;
    w2Row += B01;
    zOrig += zdy;
    xColRow += xColDy;
    yColRow += yColDy;
    offset += W;
    textureOffset += yColDy4;
    shader.stepYForX();
  }
}
#endif

void line(const vertex<int>& v0, const vertex<int>& v1, const unsigned color) {
  // We must find whether x is longer, or y is longer
  // If true, indicates that x will have pixels on same row

  int x0 = v0._x;
  int y0 = v0._y;
  int x1 = v1._x;
  int y1 = v1._y;

  if (x0 > W) {
    x0 = 0;
  }
  if (x0 < 0) {
    x0 = W;
  }
  if (x1 > W) {
    x1 = 0;
  }
  if (x1 < 0) {
    x1 = W;
  }

  if (y0 > H) {
    y0 = 0;
  }
  if (y0 < 0) {
    y0 = H;
  }
  if (y1 > H) {
    y1 = 0;
  }
  if (y1 < 0) {
    y1 = H;
  }

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



template<typename T, typename std::enable_if<std::is_base_of<UntexturedShader, T>::value, int>::type*>
void drawTri(const ModelInstance& m, const face& f, const vertex<float>& light,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i) {

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

  unsigned offset = minY * W;

  T shader(m, f, light, A12, A20, A01, B12, B20, B01, wTotal, w0Row, w1Row, w2Row);

  for (y = minY; y <= maxY; ++y) {
    w0 = w0Row;
    w1 = w1Row;
    w2 = w2Row;
    z = zOrig;

    for (x = minX; x <= maxX; ++x) {
      // If p is on or inside all edges, render pixel
      if ((w0 | w1 | w2) >= 0) {
        // Uncomment for exact z values
        //z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);
        if (zbuff[x + offset] < z) {
          plot(x, y, shader.fragmentShader());
          zbuff[x + offset] = z;
        }
      }
      w0 += A12;
      w1 += A20;
      w2 += A01;
      z += zdx;
      shader.stepXForX();
    }

    w0Row += B12;
    w1Row += B20;
    w2Row += B01;
    zOrig += zdy;
    offset += W;
    shader.stepYForX();
  }
}

float zPos(const int cx, const int bx, const int ax, const int cy, const int by, const int ay, const int cz, const int bz, const int az, const int x, const int y) {
  return  az + ((bx - ax) * (cz - az) - (cx - ax) * (bz - az))*(y - ay)/ ((bx-ax) * (cy-ay) - (cx-ax) * (by-ay)) - ((by-ay) * (cz-az) - (cy-ay)*(bz-az))*(x-ax)/((bx-ax)*(cy-ay) - (cx-ax) * (by-ay));
}

template
void drawTri<FlatShader>(const ModelInstance& m, const face& f, const vertex<float>& light,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i);

template
void drawTri<GouraudShader>(const ModelInstance& m, const face& f, const vertex<float>& light,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i);

template
void drawTri<InterpFlatShader>(const ModelInstance& m, const face& f, const vertex<float>& light,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i);

template
void drawTri<InterpGouraudShader>(const ModelInstance& m, const face& f, const vertex<float>& light,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i);
