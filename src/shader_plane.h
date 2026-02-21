#pragma once

// The goal of this texture is to create a grid pattern based on the global x/z coords of each pixel
class PlaneXZShader : public UntexturedShader, public BehindCamera {
 public:
  PlaneXZShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2)), _ds(m._absLight)
  { }


  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    unsigned res = ((((int)floor(x)) & 0x1) ^ (((int)floor(z)) & 0x1)) != 0U ? 8405024 : 8421504;
    illumination il = getLight(_norm, _luminance, x, y, z, _ds);

    res = fast_min(255, ((int)(((res >> 16) & 0xff) * il._R))) << 16 |
                       fast_min(255, ((int)(((res >> 8) & 0xff) * il._G))) << 8 |
                       fast_min(255, (int)(((res) & 0xff) * il._B));

    return res;
  }

#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    __m256i xVals = _mm256_round_ps (xV, (_MM_FROUND_TO_NEG_INF |_MM_FROUND_NO_EXC) );
    xVals = _mm256_cvtps_epi32 (xVals);

    __m256i oddMask = _mm256_set1_epi32(1);

    xVals = _mm256_and_si256(oddMask, xVals);
    xVals = _mm256_cmpeq_epi32(xVals, _mm256_setzero_si256());

    __m256i zVals = _mm256_round_ps (zV, (_MM_FROUND_TO_NEG_INF |_MM_FROUND_NO_EXC) );
    zVals = _mm256_cvtps_epi32 (zVals);

    zVals = _mm256_and_si256(oddMask, zVals);
    zVals = _mm256_cmpeq_epi32(zVals, _mm256_setzero_si256());

    xVals = _mm256_xor_si256(zVals, xVals);

    __m256i blue = _mm256_set1_epi32(8421504);
    __m256i green = _mm256_set1_epi32(8405024);
    colorsData = _mm256_blendv_ps(blue, green, xVals);

    __m256 rV;
    __m256 gV;
    __m256 bV;

    getLight(_mm256_set1_ps(_norm._x), _mm256_set1_ps(_norm._y),
             _mm256_set1_ps(_norm._z), _luminance,
             xV, yV, zV, rV, gV, bV, _ds);


    colorsData = vectorLight(colorsData, rV, gV, bV);
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
  }

  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {
  }

 private:
  const float _luminance;
  const vertex<float>& _norm;
  const bool _ds;
};

// The goal of this texture is to create a grid pattern based on the global x/y coords of each pixel
class PlaneXYShader : public UntexturedShader, public BehindCamera {
 public:
  PlaneXYShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2)), _ds(m._absLight)
  { }


  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    unsigned res = ((((int)floor(x)) & 0x1) ^ (((int)floor(y)) & 0x1)) != 0U ? 8405024 : 8421504;
    illumination il = getLight(_norm, _luminance, x, y, z, _ds);

    res = fast_min(255, ((int)(((res >> 16) & 0xff) * il._R))) << 16 |
                       fast_min(255, ((int)(((res >> 8) & 0xff) * il._G))) << 8 |
                       fast_min(255, (int)(((res) & 0xff) * il._B));

    return res;
  }

#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    __m256i xVals = _mm256_round_ps (xV, (_MM_FROUND_TO_NEG_INF |_MM_FROUND_NO_EXC) );
    xVals = _mm256_cvtps_epi32 (xVals);

    __m256i oddMask = _mm256_set1_epi32(1);

    xVals = _mm256_and_si256(oddMask, xVals);
    xVals = _mm256_cmpeq_epi32(xVals, _mm256_setzero_si256());

    __m256i yVals = _mm256_round_ps (yV, (_MM_FROUND_TO_NEG_INF |_MM_FROUND_NO_EXC) );
    yVals = _mm256_cvtps_epi32 (yVals);

    yVals = _mm256_and_si256(oddMask, yVals);
    yVals = _mm256_cmpeq_epi32(yVals, _mm256_setzero_si256());

    xVals = _mm256_xor_si256(yVals, xVals);

    __m256i blue = _mm256_set1_epi32(8421504);
    __m256i green = _mm256_set1_epi32(8405024);
    colorsData = _mm256_blendv_ps(blue, green, xVals);

    __m256 rV;
    __m256 gV;
    __m256 bV;

    getLight(_mm256_set1_ps(_norm._x), _mm256_set1_ps(_norm._y),
             _mm256_set1_ps(_norm._z), _luminance,
             xV, yV, zV, rV, gV, bV, _ds);


    colorsData = vectorLight(colorsData, rV, gV, bV);
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
  }

  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {
  }

 private:
  const float _luminance;
  const vertex<float>& _norm;
  const bool _ds;
};

// The goal of this texture is to create a grid pattern based on the global z/y coords of each pixel
class PlaneYZShader : public UntexturedShader, public BehindCamera {
 public:
  PlaneYZShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2)), _ds(m._absLight)
  { }


  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    unsigned res = ((((int)floor(z)) & 0x1) ^ (((int)floor(y)) & 0x1)) != 0U ? 8405024 : 8421504;
    illumination il = getLight(_norm, _luminance, x, y, z, _ds);

    res = fast_min(255, ((int)(((res >> 16) & 0xff) * il._R))) << 16 |
                       fast_min(255, ((int)(((res >> 8) & 0xff) * il._G))) << 8 |
                       fast_min(255, (int)(((res) & 0xff) * il._B));

    return res;
  }

#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& xV, const __m256i& yV, const __m256i& zV) override {
    __m256i zVals = _mm256_round_ps (zV, (_MM_FROUND_TO_NEG_INF |_MM_FROUND_NO_EXC) );
    zVals = _mm256_cvtps_epi32 (zVals);

    __m256i oddMask = _mm256_set1_epi32(1);

    zVals = _mm256_and_si256(oddMask, zVals);
    zVals = _mm256_cmpeq_epi32(zVals, _mm256_setzero_si256());

    __m256i yVals = _mm256_round_ps (yV, (_MM_FROUND_TO_NEG_INF |_MM_FROUND_NO_EXC) );
    yVals = _mm256_cvtps_epi32 (yVals);

    yVals = _mm256_and_si256(oddMask, yVals);
    yVals = _mm256_cmpeq_epi32(yVals, _mm256_setzero_si256());

    zVals = _mm256_xor_si256(yVals, zVals);

    __m256i blue = _mm256_set1_epi32(8421504);
    __m256i green = _mm256_set1_epi32(8405024);
    colorsData = _mm256_blendv_ps(blue, green, zVals);

    __m256 rV;
    __m256 gV;
    __m256 bV;

    getLight(_mm256_set1_ps(_norm._x), _mm256_set1_ps(_norm._y),
             _mm256_set1_ps(_norm._z), _luminance,
             xV, yV, zV, rV, gV, bV, _ds);


    colorsData = vectorLight(colorsData, rV, gV, bV);
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
  }

  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {
  }

 private:
  const float _luminance;
  const vertex<float>& _norm;
  const bool _ds;
};
