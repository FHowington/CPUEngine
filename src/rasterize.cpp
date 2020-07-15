#include "geometry.h"
#include "rasterize.h"
#include <immintrin.h>


#ifdef __AVX2__
template<typename T, typename std::enable_if<std::is_base_of<TexturedShader, T>::value, int>::type*>
void drawTri(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2) {

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

  const int maxX = max3(x0, x1, x2, (int)W - 1);
  const int maxY = max3(y0, y1, y2, (int)H - 1);

  // Same idea. These have no area (happens when triangle is outside of viewing area)
  if (maxX < minX || maxY < minY) {
    return;
  }

  // Bias to make sure only top or left edges fall on line
  const int bias0 = isTopLeft(v1i, v2i);
  const int bias1 = isTopLeft(v2i, v0i);
  const int bias2 = isTopLeft(v0i, v1i);

  int w0Row = orient2d(x1, x2, minX, y1, y2, minY) + bias0;
  int w1Row = orient2d(x2, x0, minX, y2, y0, minY) + bias1;
  int w2Row = orient2d(x0, x1, minX, y0, y1, minY) + bias2;

  // If this number is 0, triangle has no area!
  float wTotal = w0Row + w1Row + w2Row;

  pMaxX = fast_max(pMaxX, maxX);
  pMinX = fast_min(pMinX, minX);
  pMaxY = fast_max(pMaxY, maxY);
  pMinY = fast_min(pMinY, minY);

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

  unsigned y;
  unsigned xVal;
  unsigned numInner;
  unsigned inner;

  int w0;
  int w1;
  int w2;
  int depth;

  const int div = (((B20) * (-A01)) + (B01) * (A20));
  const int z10 = z1 - z0;
  const int z20 = z2 - z0;

  // Change in z for change in row/column
  // Obtained by taking partial derivative with respect to x or y from equation of a plane
  // See equation of a plane here: https://math.stackexchange.com/questions/851742/calculate-coordinate-of-any-point-on-triangle-in-3d-plane
  // Using these deltas, we interpolate over face of the whole triangle
  const int depthDx = (A20 * z10 + A01 * z20) / div;
  const int depthDy = (B20 * z10 + B01 * z20) / div;

  // Likewise from solving for z with equation of a plane
  int depthOrig = zPos(x2, x1, x0, y2, y1, y0, z2, z1, z0, minX, minY);

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

  // Current real world coordinates
  const float z0Inv = 1.0/v0i._z;
  const float z1Inv = 1.0/v1i._z;
  const float z2Inv = 1.0/v2i._z;

  const float x0Corr = v0._x * z0Inv;
  const float x1Corr = v1._x * z1Inv;
  const float x2Corr = v2._x * z2Inv;

  const float y0Corr = v0._y * z0Inv;
  const float y1Corr = v1._y * z1Inv;
  const float y2Corr = v2._y * z2Inv;

  const float z0Corr = v0._z * z0Inv;
  const float z1Corr = v1._z * z1Inv;
  const float z2Corr = v2._z * z2Inv;

  float xLocRow = (x0Corr * w0Row + x1Corr * w1Row + x2Corr * w2Row);
  float yLocRow = (y0Corr * w0Row + y1Corr * w1Row + y2Corr * w2Row);
  float zLocRow = (z0Corr * w0Row + z1Corr * w1Row + z2Corr * w2Row);

  float wTotalRRow = w0Row * z0Inv + w1Row * z1Inv + w2Row * z2Inv;
  const float wDiffX = (A12 * z0Inv + A20 * z1Inv + A01 * z2Inv);
  const float wDiffY = (B12 * z0Inv + B20 * z1Inv + B01 * z2Inv);

  const float xDx = (x0Corr * A12 + x1Corr * A20 + x2Corr * A01);
  const float yDx = (y0Corr * A12 + y1Corr * A20 + y2Corr * A01);
  const float zDx = (z0Corr * A12 + z1Corr * A20 + z2Corr * A01);

  const float xDy = (x0Corr * B12 + x1Corr * B20 + x2Corr * B01);
  const float yDy = (y0Corr * B12 + y1Corr * B20 + y2Corr * B01);
  const float zDy = (z0Corr * B12 + z1Corr * B20 + z2Corr * B01);

  float xLoc;
  float yLoc;
  float zLoc;
  float wTotalR;

  T shader(m, f, A12, A20, A01, B12, B20, B01, wTotal, w0Row, w1Row, w2Row, v0i, v1i, v2i);
  const TGAImage& img = *m._baseModel._texture;


  const __m256i depthDxAdd = _mm256_cvttps_epi32(_mm256_mul_ps(scaleFloat,  _mm256_set1_ps(depthDx)));
  const __m256i xColAdd = _mm256_mul_ps(scaleFloat,  _mm256_set1_ps(xColDx));
  const __m256i yColAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(yColDx));

  const __m256i xRAdd = _mm256_mul_ps(scaleFloat,  _mm256_set1_ps(xDx));
  const __m256i yRAdd = _mm256_mul_ps(scaleFloat,  _mm256_set1_ps(yDx));
  const __m256i zRAdd = _mm256_mul_ps(scaleFloat,  _mm256_set1_ps(zDx));
  const __m256i wTotalRAdd = _mm256_mul_ps(scaleFloat,  _mm256_set1_ps(wDiffX));

  __m256i w0Init;
  __m256i w1Init;
  __m256i w2Init;

  const __m256i a12Add = _mm256_mullo_epi32(_mm256_set1_epi32(A12), scale);
  const __m256i a12Add8 = _mm256_set1_epi32(8*A12);

  const __m256i a20Add = _mm256_mullo_epi32(_mm256_set1_epi32(A20), scale);
  const  __m256i a20Add8 = _mm256_set1_epi32(8*A20);

  const __m256i a01Add = _mm256_mullo_epi32(_mm256_set1_epi32(A01), scale);
  const __m256i a01Add8 = _mm256_set1_epi32(8*A01);


  // We will enter inner loop at least once, otherwise numInner is always 0
  unsigned offset = minY * Wt;

  numInner = (maxX - minX) / 8;
  numInner += ((maxX - minX) % 8 != 0);

  for (y = minY; y <= maxY; ++y) {
    w0 = w0Row;
    w1 = w1Row;
    w2 = w2Row;
    depth = depthOrig;
    xCol = xColRow;
    yCol = yColRow;

    xLoc = xLocRow;
    yLoc = yLocRow;
    zLoc = zLocRow;
    wTotalR = wTotalRRow;

    w0Init = _mm256_set1_epi32(w0);
    w0Init = _mm256_add_epi32(w0Init, a12Add);

    w1Init = _mm256_set1_epi32(w1);
    w1Init = _mm256_add_epi32(w1Init, a20Add);

    w2Init = _mm256_set1_epi32(w2);
    w2Init = _mm256_add_epi32(w2Init, a01Add);

    xVal = minX;
    for (inner = 0; inner < numInner; ++inner) {
      const __m256i zbuffv = _mm256_loadu_si256((__m256i*)(t_zbuff.data() + xVal + offset)); // NOLINT

      const __m256i zInit = _mm256_set1_epi32(depth);
      const __m256i zv = _mm256_add_epi32(zInit, depthDxAdd);
      const __m256i needsUpdate = _mm256_and_si256(_mm256_cmpgt_epi32(zv, zbuffv), _mm256_cmpgt_epi32(_mm256_or_si256(w2Init, _mm256_or_si256(w0Init, w1Init)), min));

      if (!_mm256_testz_si256(needsUpdate, needsUpdate)) {
        const __m256i zUpdate = _mm256_blendv_epi8(zbuffv, zv, needsUpdate);
        const __m256i colorV = _mm256_loadu_si256((__m256i*)(t_pixels.data() + xVal + offset)); // NOLINT

        __m256 xColv = _mm256_add_ps(_mm256_set1_ps(xCol), xColAdd);
        __m256 yColv = _mm256_add_ps(_mm256_set1_ps(yCol), yColAdd);

        // Convert to ints
        xColv = _mm256_cvtps_epi32(xColv);
        yColv = _mm256_cvtps_epi32(yColv);

        yColv = _mm256_mullo_epi32(yColv, _mm256_set1_epi32(img.width));
        xColv = _mm256_add_epi32(xColv, yColv);

        xColv = _mm256_and_si256(xColv, _mm256_cmpgt_epi32(xColv, ones));

        auto colorsData = _mm256_i32gather_epi32(img.data, xColv, 4); // NOLINT

        __m256 wTotalRV = _mm256_add_ps(_mm256_set1_ps(wTotalR), wTotalRAdd);
        __m256 xV = _mm256_div_ps(_mm256_add_ps(_mm256_set1_ps(xLoc), xRAdd), wTotalRV);
        __m256 yV = _mm256_div_ps(_mm256_add_ps(_mm256_set1_ps(yLoc), yRAdd), wTotalRV);
        __m256 zV = _mm256_div_ps(_mm256_add_ps(_mm256_set1_ps(zLoc), zRAdd), wTotalRV);

        shader.fragmentShader(colorsData, zUpdate, xV, yV, zV);
        colorsData = _mm256_blendv_epi8(colorV, colorsData, needsUpdate);

        _mm256_storeu_si256((__m256i*)(t_zbuff.data() + xVal + offset), zUpdate); // NOLINT
        _mm256_storeu_si256((__m256i*)(t_pixels.data() + xVal + offset), colorsData); // NOLINT
      } else {
        shader.stepXForX(8);
      }
      xVal += 8;
      // We must step 8 times.
      xCol += xColDx * 8;
      yCol += yColDx * 8;
      depth += depthDx * 8;

      xLoc += xDx * 8;
      yLoc += yDx * 8;
      zLoc += zDx * 8;
      wTotalR += wDiffX * 8;

      if (inner < numInner - 1) {
        w0Init = _mm256_add_epi32(w0Init, a12Add8);
        w1Init = _mm256_add_epi32(w1Init, a20Add8);
        w2Init = _mm256_add_epi32(w2Init, a01Add8);
      }
    }

    w0Row += B12;
    w1Row += B20;
    w2Row += B01;
    depthOrig += depthDy;
    xColRow += xColDy;
    yColRow += yColDy;
    offset += Wt;

    xLocRow += xDy;
    yLocRow += yDy;
    zLocRow += zDy;
    wTotalRRow += wDiffY;

    shader.stepYForX();
  }
}
#else
template<typename T, typename std::enable_if<std::is_base_of<TexturedShader, T>::value, int>::type*>
void drawTri(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2) {
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

  const int maxX = max3(x0, x1, x2, (int)W - 1);
  const int maxY = max3(y0, y1, y2, (int)H - 1);

  // Same idea. These have no area (happens when triangle is outside of viewing area)
  if (maxX < minX || maxY < minY) {
    return;
  }

  // Bias to make sure only top or left edges fall on line
  const int bias0 = isTopLeft(v1i, v2i);
  const int bias1 = isTopLeft(v2i, v0i);
  const int bias2 = isTopLeft(v0i, v1i);
  int w0Row = orient2d(x1, x2, minX, y1, y2, minY) + bias0;
  int w1Row = orient2d(x2, x0, minX, y2, y0, minY) + bias1;
  int w2Row = orient2d(x0, x1, minX, y0, y1, minY) + bias2;

  float wTotal = w0Row + w1Row + w2Row;

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

  pMaxX = fast_max(pMaxX, maxX);
  pMinX = fast_min(pMinX, minX);
  pMaxY = fast_max(pMaxY, maxY);
  pMinY = fast_min(pMinY, minY);

  unsigned x;
  unsigned y;

  int w0;
  int w1;
  int w2;
  int depth;

  const int div = (((B20) * (-A01)) + (B01) * (A20));
  const int z10 = z1 - z0;
  const int z20 = z2 - z0;

  // Change in z for change in row/column
  // Obtained by taking partial derivative with respect to x or y from equation of a plane
  // See equation of a plane here: https://math.stackexchange.com/questions/851742/calculate-coordinate-of-any-point-on-triangle-in-3d-plane
  // Using these deltas, we interpolate over face of the whole triangle
  const int depthDx = (A20 * z10 + A01 * z20) / div;
  const int depthDy = (B20 * z10 + B01 * z20) / div;

  // Likewise from solving for z with equation of a plane
  int depthOrig = zPos(x2, x1, x0, y2, y1, y0, z2, z1, z0, minX, minY);

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


  // Current real world coordinates
  const float z0Inv = 1.0/v0i._z;
  const float z1Inv = 1.0/v1i._z;
  const float z2Inv = 1.0/v2i._z;

  const float x0Corr = v0._x * z0Inv;
  const float x1Corr = v1._x * z1Inv;
  const float x2Corr = v2._x * z2Inv;

  const float y0Corr = v0._y * z0Inv;
  const float y1Corr = v1._y * z1Inv;
  const float y2Corr = v2._y * z2Inv;

  const float z0Corr = v0._z * z0Inv;
  const float z1Corr = v1._z * z1Inv;
  const float z2Corr = v2._z * z2Inv;

  float xLocRow = (x0Corr * w0Row + x1Corr * w1Row + x2Corr * w2Row);
  float yLocRow = (y0Corr * w0Row + y1Corr * w1Row + y2Corr * w2Row);
  float zLocRow = (z0Corr * w0Row + z1Corr * w1Row + z2Corr * w2Row);

  float wTotalRRow = w0Row * z0Inv + w1Row * z1Inv + w2Row * z2Inv;

  const float wDiffX = (A12 * z0Inv + A20 * z1Inv + A01 * z2Inv);
  const float wDiffY = (B12 * z0Inv + B20 * z1Inv + B01 * z2Inv);

  const float xDx = (x0Corr * A12 + x1Corr * A20 + x2Corr * A01);
  const float yDx = (y0Corr * A12 + y1Corr * A20 + y2Corr * A01);
  const float zDx = (z0Corr * A12 + z1Corr * A20 + z2Corr * A01);

  const float xDy = (x0Corr * B12 + x1Corr * B20 + x2Corr * B01);
  const float yDy = (y0Corr * B12 + y1Corr * B20 + y2Corr * B01);
  const float zDy = (z0Corr * B12 + z1Corr * B20 + z2Corr * B01);

  float xLoc;
  float yLoc;
  float zLoc;
  float wTotalR;

  // Current texture coordinates
  float xCol;
  float yCol;

  unsigned offset = minY * W;
  float textureOffset = yColRow;
  const float yColDy4 = 4 * yColDy;

  T shader(m, f, A12, A20, A01, B12, B20, B01, wTotal, w0Row, w1Row, w2Row, v0i, v1i, v2i);
  const TGAImage& img = *m._baseModel._texture;

  for (y = minY; y <= maxY; ++y) {
    w0 = w0Row;
    w1 = w1Row;
    w2 = w2Row;
    depth = depthOrig;
    xCol = xColRow;
    yCol = yColRow;

    xLoc = xLocRow;
    yLoc = yLocRow;
    zLoc = zLocRow;
    wTotalR = wTotalRRow;

    for (x = minX; x <= maxX; ++x) {
      // If p is on or inside all edges, render pixel
      if ((w0 | w1 | w2) >= 0) {
        // Uncomment for exact z values
        //z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);
        if (t_zbuff[x + offset] < depth) {
          t_pixels[x + offset] = shader.fragmentShader(xLoc/wTotalR, yLoc/wTotalR, zLoc/wTotalR, img.fast_get(xCol, yCol));
          t_zbuff[x + offset] = depth;
        }
      }
      w0 += A12;
      w1 += A20;
      w2 += A01;
      depth += depthDx;
      xCol += xColDx;
      yCol += yColDx;

      xLoc += xDx;
      yLoc += yDx;
      zLoc += zDx;
      wTotalR += wDiffX;

      shader.stepXForX();
    }

    w0Row += B12;
    w1Row += B20;
    w2Row += B01;
    depthOrig += depthDy;
    xColRow += xColDy;
    yColRow += yColDy;
    offset += W;
    textureOffset += yColDy4;
    xLocRow += xDy;
    yLocRow += yDy;
    zLocRow += zDy;
    wTotalRRow += wDiffY;
    shader.stepYForX();
  }
}
#endif

#ifdef __AVX2__
template<typename T, typename std::enable_if<std::is_base_of<UntexturedShader, T>::value, int>::type*>
void drawTri(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2) {
  // These are reused for every triangle in the model
  static const __m256i min = _mm256_set1_epi32(-1);
  static const __m256i scale = _mm256_set_epi32(7, 6, 5, 4, 3, 2, 1, 0);
  static const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);

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

  const int maxX = max3(x0, x1, x2, (int)W - 1);
  const int maxY = max3(y0, y1, y2, (int)H - 1);

  // Same idea. These have no area (happens when triangle is outside of viewing area)
  if (maxX < minX || maxY < minY) {
    return;
  }

  // Bias to make sure only top or left edges fall on line
  const int bias0 = isTopLeft(v1i, v2i);
  const int bias1 = isTopLeft(v2i, v0i);
  const int bias2 = isTopLeft(v0i, v1i);

  int w0Row = orient2d(x1, x2, minX, y1, y2, minY) + bias0;
  int w1Row = orient2d(x2, x0, minX, y2, y0, minY) + bias1;
  int w2Row = orient2d(x0, x1, minX, y0, y1, minY) + bias2;

  // If this number is 0, triangle has no area!
  float wTotal = w0Row + w1Row + w2Row;

  // Deltas for change in x or y for the 3 sides of a triangle
  const int A01 = y0 - y1;
  const int A12 = y1 - y2;
  const int A20 = y2 - y0;

  const int B01 = x1 - x0;
  const int B12 = x2 - x1;
  const int B20 = x0 - x2;

  const int z0 = v0i._z;
  const int z1 = v1i._z;
  const int z2 = v2i._z;

  unsigned y;
  unsigned xVal;
  unsigned numInner;
  unsigned inner;

  int w0;
  int w1;
  int w2;
  int depth;

  const int div = (((long long)(B20) * (-A01)) + (long long)(B01) * (A20));
  const int z10 = z1 - z0;
  const int z20 = z2 - z0;

  // Change in z for change in row/column
  // Obtained by taking partial derivative with respect to x or y from equation of a plane
  // See equation of a plane here: https://math.stackexchange.com/questions/851742/calculate-coordinate-of-any-point-on-triangle-in-3d-plane
  // Using these deltas, we interpolate over face of the whole triangle
  const int depthDx = ((long long)A20 * z10 + (long long)A01 * z20) / div;
  const int depthDy = ((long long)B20 * z10 + (long long)B01 * z20) / div;

  // Likewise from solving for z with equation of a plane
  int depthOrig = zPos(x2, x1, x0, y2, y1, y0, z2, z1, z0, minX, minY);

  T shader(m, f, A12, A20, A01, B12, B20, B01, wTotal, w0Row, w1Row, w2Row, v0i, v1i, v2i);

  // If the traingle is wider than tall, we want to vectorize on x
  // Otherwise, we vectorize on y
  const __m256i depthDxAdd = _mm256_cvttps_epi32(_mm256_mul_ps(scaleFloat,  _mm256_set1_ps(depthDx)));

  __m256i w0Init;
  __m256i w1Init;
  __m256i w2Init;

  const __m256i a12Add = _mm256_mullo_epi32(_mm256_set1_epi32(A12), scale);
  const __m256i a12Add8 = _mm256_set1_epi32(8*A12);

  const __m256i a20Add = _mm256_mullo_epi32(_mm256_set1_epi32(A20), scale);
  const  __m256i a20Add8 = _mm256_set1_epi32(8*A20);

  const __m256i a01Add = _mm256_mullo_epi32(_mm256_set1_epi32(A01), scale);
  const __m256i a01Add8 = _mm256_set1_epi32(8*A01);

  // Current real world coordinates
  const float z0Inv = 1.0/v0i._z;
  const float z1Inv = 1.0/v1i._z;
  const float z2Inv = 1.0/v2i._z;

  const float x0Corr = v0._x * z0Inv;
  const float x1Corr = v1._x * z1Inv;
  const float x2Corr = v2._x * z2Inv;

  const float y0Corr = v0._y * z0Inv;
  const float y1Corr = v1._y * z1Inv;
  const float y2Corr = v2._y * z2Inv;

  const float z0Corr = v0._z * z0Inv;
  const float z1Corr = v1._z * z1Inv;
  const float z2Corr = v2._z * z2Inv;

  float xLocRow = (x0Corr * w0Row + x1Corr * w1Row + x2Corr * w2Row);
  float yLocRow = (y0Corr * w0Row + y1Corr * w1Row + y2Corr * w2Row);
  float zLocRow = (z0Corr * w0Row + z1Corr * w1Row + z2Corr * w2Row);

  float wTotalRRow = w0Row * z0Inv + w1Row * z1Inv + w2Row * z2Inv;
  const float wDiffX = (A12 * z0Inv + A20 * z1Inv + A01 * z2Inv);
  const float wDiffY = (B12 * z0Inv + B20 * z1Inv + B01 * z2Inv);

  const float xDx = (x0Corr * A12 + x1Corr * A20 + x2Corr * A01);
  const float yDx = (y0Corr * A12 + y1Corr * A20 + y2Corr * A01);
  const float zDx = (z0Corr * A12 + z1Corr * A20 + z2Corr * A01);

  const float xDy = (x0Corr * B12 + x1Corr * B20 + x2Corr * B01);
  const float yDy = (y0Corr * B12 + y1Corr * B20 + y2Corr * B01);
  const float zDy = (z0Corr * B12 + z1Corr * B20 + z2Corr * B01);

  float xLoc;
  float yLoc;
  float zLoc;
  float wTotalR;

  const __m256i xRAdd = _mm256_mul_ps(scaleFloat,  _mm256_set1_ps(xDx));
  const __m256i yRAdd = _mm256_mul_ps(scaleFloat,  _mm256_set1_ps(yDx));
  const __m256i zRAdd = _mm256_mul_ps(scaleFloat,  _mm256_set1_ps(zDx));
  const __m256i wTotalRAdd = _mm256_mul_ps(scaleFloat,  _mm256_set1_ps(wDiffX));

  pMaxX = fast_max(pMaxX, maxX);
  pMinX = fast_min(pMinX, minX);
  pMaxY = fast_max(pMaxY, maxY);
  pMinY = fast_min(pMinY, minY);

  // We will enter inner loop at least once, otherwise numInner is always 0
  unsigned offset = minY * Wt;

  numInner = (maxX - minX) / 8;

  if ((maxX - minX) % 8) {
    ++numInner;
  }

  for (y = minY; y <= maxY; ++y) {
    w0 = w0Row;
    w1 = w1Row;
    w2 = w2Row;
    depth = depthOrig;

    xLoc = xLocRow;
    yLoc = yLocRow;
    zLoc = zLocRow;
    wTotalR = wTotalRRow;

    w0Init = _mm256_set1_epi32(w0);
    w0Init = _mm256_add_epi32(w0Init, a12Add);

    w1Init = _mm256_set1_epi32(w1);
    w1Init = _mm256_add_epi32(w1Init, a20Add);

    w2Init = _mm256_set1_epi32(w2);
    w2Init = _mm256_add_epi32(w2Init, a01Add);

    xVal = minX;

    for (inner = 0; inner < numInner; ++inner) {
      const __m256i zbuffv = _mm256_loadu_si256((__m256i*)(t_zbuff.data() + xVal + offset)); // NOLINT

      const __m256i zInit = _mm256_set1_epi32(depth);
      const __m256i zv = _mm256_add_epi32(zInit, depthDxAdd);
      const __m256i needsUpdate = _mm256_and_si256(_mm256_cmpgt_epi32(zv, zbuffv), _mm256_cmpgt_epi32(_mm256_or_si256(w2Init, _mm256_or_si256(w0Init, w1Init)), min));

      if (!_mm256_testz_si256(needsUpdate, needsUpdate)) {
        const __m256i zUpdate = _mm256_blendv_epi8(zbuffv, zv, needsUpdate);
        const __m256i colorV = _mm256_loadu_si256((__m256i*)(t_pixels.data() + offset + xVal)); // NOLINT

        __m256 wTotalRV = _mm256_add_ps(_mm256_set1_ps(wTotalR), wTotalRAdd);
        __m256 xV = _mm256_div_ps(_mm256_add_ps(_mm256_set1_ps(xLoc), xRAdd), wTotalRV);
        __m256 yV = _mm256_div_ps(_mm256_add_ps(_mm256_set1_ps(yLoc), yRAdd), wTotalRV);
        __m256 zV = _mm256_div_ps(_mm256_add_ps(_mm256_set1_ps(zLoc), zRAdd), wTotalRV);

        __m256i colorsData;
        shader.fragmentShader(colorsData, zUpdate, xV, yV, zV);
        colorsData = _mm256_blendv_epi8(colorV, colorsData, needsUpdate);

        _mm256_storeu_si256((__m256i*)(t_zbuff.data() + xVal + offset), zUpdate); // NOLINT
        _mm256_storeu_si256((__m256i*)(t_pixels.data() + offset + xVal), colorsData); // NOLINT
      } else {
        shader.stepXForX(8);
      }

      xVal += 8;
      // We must step 8 times.
      depth += depthDx * 8;

      xLoc += xDx * 8;
      yLoc += yDx * 8;
      zLoc += zDx * 8;
      wTotalR += wDiffX * 8;

      if (inner < numInner - 1) {
        w0Init = _mm256_add_epi32(w0Init, a12Add8);
        w1Init = _mm256_add_epi32(w1Init, a20Add8);
        w2Init = _mm256_add_epi32(w2Init, a01Add8);
      }
    }

    w0Row += B12;
    w1Row += B20;
    w2Row += B01;
    depthOrig += depthDy;
    offset += Wt;

    xLocRow += xDy;
    yLocRow += yDy;
    zLocRow += zDy;
    wTotalRRow += wDiffY;

    shader.stepYForX();
  }
}
#else
template<typename T, typename std::enable_if<std::is_base_of<UntexturedShader, T>::value, int>::type*>
void drawTri(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2) {
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

  const int maxX = max3(x0, x1, x2, (int)W - 1);
  const int maxY = max3(y0, y1, y2, (int)H - 1);

  // Same idea. These have no area (happens when triangle is outside of viewing area)
  if (maxX < minX || maxY < minY) {
    return;
  }

  // Bias to make sure only top or left edges fall on line
  const int bias0 = isTopLeft(v1i, v2i);
  const int bias1 = isTopLeft(v2i, v0i);
  const int bias2 = isTopLeft(v0i, v1i);
  int w0Row = orient2d(x1, x2, minX, y1, y2, minY) + bias0;
  int w1Row = orient2d(x2, x0, minX, y2, y0, minY) + bias1;
  int w2Row = orient2d(x0, x1, minX, y0, y1, minY) + bias2;

  float wTotal = w0Row + w1Row + w2Row;

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

  pMaxX = fast_max(pMaxX, maxX);
  pMinX = fast_min(pMinX, minX);
  pMaxY = fast_max(pMaxY, maxY);
  pMinY = fast_min(pMinY, minY);

  unsigned x;
  unsigned y;

  int w0;
  int w1;
  int w2;
  int depth;

  const int div = -((long long)(-B20) * (-A01)) + ((long long)(B01) * (A20));
  const int z10 = z1 - z0;
  const int z20 = z2 - z0;

  // Current real world coordinates
  const float z0Inv = 1.0/v0i._z;
  const float z1Inv = 1.0/v1i._z;
  const float z2Inv = 1.0/v2i._z;

  const float x0Corr = v0._x * z0Inv;
  const float x1Corr = v1._x * z1Inv;
  const float x2Corr = v2._x * z2Inv;

  const float y0Corr = v0._y * z0Inv;
  const float y1Corr = v1._y * z1Inv;
  const float y2Corr = v2._y * z2Inv;

  const float z0Corr = v0._z * z0Inv;
  const float z1Corr = v1._z * z1Inv;
  const float z2Corr = v2._z * z2Inv;


  float xLocRow = (x0Corr * w0Row + x1Corr * w1Row + x2Corr * w2Row);
  float yLocRow = (y0Corr * w0Row + y1Corr * w1Row + y2Corr * w2Row);
  float zLocRow = (z0Corr * w0Row + z1Corr * w1Row + z2Corr * w2Row);

  float wTotalRRow = w0Row * z0Inv + w1Row * z1Inv + w2Row * z2Inv;
  const float wDiffX = (A12 * z0Inv + A20 * z1Inv + A01 * z2Inv);
  const float wDiffY = (B12 * z0Inv + B20 * z1Inv + B01 * z2Inv);

  const float xDx = (x0Corr * A12 + x1Corr * A20 + x2Corr * A01);
  const float yDx = (y0Corr * A12 + y1Corr * A20 + y2Corr * A01);
  const float zDx = (z0Corr * A12 + z1Corr * A20 + z2Corr * A01);

  const float xDy = (x0Corr * B12 + x1Corr * B20 + x2Corr * B01);
  const float yDy = (y0Corr * B12 + y1Corr * B20 + y2Corr * B01);
  const float zDy = (z0Corr * B12 + z1Corr * B20 + z2Corr * B01);

  float xLoc;
  float yLoc;
  float zLoc;
  float wTotalR;

  // Change in z for change in row/column
  // Obtained by taking partial derivative with respect to x or y from equation of a plane
  // See equation of a plane here: https://math.stackexchange.com/questions/851742/calculate-coordinate-of-any-point-on-triangle-in-3d-plane
  // Using these deltas, we interpolate over face of the whole triangle
  const int depthDx = (((long long)A20 * z10) + ((long long)A01 * z20)) / div;
  const int depthDy = (((long long)B20 * z10) + ((long long)B01 * z20)) / div;

  // Likewise from solving for z with equation of a plane
  int depthOrig = zPos(x2, x1, x0, y2, y1, y0, z2, z1, z0, minX, minY);
  unsigned offset = minY * W;

  T shader(m, f, A12, A20, A01, B12, B20, B01, wTotal, w0Row, w1Row, w2Row, v0i, v1i, v2i);

  for (y = minY; y <= maxY; ++y) {
    w0 = w0Row;
    w1 = w1Row;
    w2 = w2Row;

    xLoc = xLocRow;
    yLoc = yLocRow;
    zLoc = zLocRow;
    wTotalR = wTotalRRow;
    depth= depthOrig;

    for (x = minX; x <= maxX; ++x) {
      // If p is on or inside all edges, render pixel
      if ((w0 | w1 | w2) >= 0) {
        // Uncomment for exact z values
        //z = zPos(x0, x1, x2, y0, y1, y2, z0, z1, z2, xValInner, y);
        if (t_zbuff[x + offset] < depth) {
          t_pixels[x + offset] = shader.fragmentShader(xLoc/wTotalR, yLoc/wTotalR, zLoc/wTotalR);
          t_zbuff[x + offset] = depth;
        }
      }
      w0 += A12;
      w1 += A20;
      w2 += A01;
      depth += depthDx;
      shader.stepXForX();

      xLoc += xDx;
      yLoc += yDx;
      zLoc += zDx;
      wTotalR += wDiffX;
    }

    w0Row += B12;
    w1Row += B20;
    w2Row += B01;
    depthOrig += depthDy;
    offset += W;

    xLocRow += xDy;
    yLocRow += yDy;
    zLocRow += zDy;
    wTotalRRow += wDiffY;
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

template
void drawTri<FlatShader>(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2);

template
void drawTri<GouraudShader>(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2);

template
void drawTri<InterpFlatShader>(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2);

template
void drawTri<InterpGouraudShader>(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2);

template
void drawTri<PlaneShader>(const ModelInstance& m, const face& f,
             const vertex<int>& v0i, const vertex<int>& v1i, const vertex<int>& v2i,
             const vertex<float>& v0, const vertex<float>& v1, const vertex<float>& v2);

void plot(unsigned x, unsigned y, const unsigned color)
{
    t_pixels[y*W+x] = color;
}
