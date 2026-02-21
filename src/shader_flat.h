#pragma once

// NOTE: Using this flat shader causes a serious perf. impact versus the naive
// approach of just doing these calculations inline
// The difference is due to the cost of calling a virtual function MANY times
// So instead, we will NOT call the functions via a Shader ref or pointer
// The type of shader will be a compile time CONSTANT!
class FlatShader : public TexturedShader, public InFrontCamera {
 public:
  FlatShader(const ModelInstance& m, const face& f,
             const short A12, const short A20, const short A01,
             const short B12, const short B20, const short B01,
             const float wTotal, int w0, int w1, int w2,
             const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination) {

    const vertex<float> v0iLight(multToVector(m._position, m._baseModel.getVertex(f._v0)));
    const vertex<float> v1iLight(multToVector(m._position, m._baseModel.getVertex(f._v1)));
    const vertex<float> v2iLight(multToVector(m._position, m._baseModel.getVertex(f._v2)));
    _norm = cross(v2iLight, v1iLight, v0iLight);
    _norm.normalize();
  }

  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    illumination il = getLight(_norm, _luminance, x, y, z);
    return fcolor(color, il._R, il._G, il._B);
  }

#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& x, const __m256i& y, const __m256i& z) override {
    __m256 rV;
    __m256 gV;
    __m256 bV;

    getLight(_mm256_set1_ps(_norm._x), _mm256_set1_ps(_norm._y),
             _mm256_set1_ps(_norm._z), _luminance,
             x, y, z, rV, gV, bV);


    colorsData = vectorLight(colorsData, rV, gV, bV);
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 0) override { }
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override { }

 private:
  vertex<float> _norm;
  const float _luminance;
};


class InterpFlatShader : public UntexturedShader, public InFrontCamera {
 public:
  InterpFlatShader(const ModelInstance& m, const face& f,
                   const short A12, const short A20, const short A01,
                   const short B12, const short B20, const short B01,
                   const float wTotal, int w0, int w1, int w2,
                   const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination) {

    const vertex<float> v0iLight(multToVector(m._position, m._baseModel.getVertex(f._v0)));
    const vertex<float> v1iLight(multToVector(m._position, m._baseModel.getVertex(f._v1)));
    const vertex<float> v2iLight(multToVector(m._position, m._baseModel.getVertex(f._v2)));
    _norm = cross(v2iLight, v1iLight, v0iLight);
    _norm.normalize();

    // Change in the texture coordinated for x/y, used for interpolation
    const unsigned col0 = m._baseModel._texture->fast_get(f._t0x, f._t0y);
    const unsigned col1 = m._baseModel._texture->fast_get(f._t1x, f._t1y);
    const unsigned col2 = m._baseModel._texture->fast_get(f._t2x, f._t2y);

    const float R0 = (col0 >> 16) & 0xff;
    const float G0 = (col0 >> 8) & 0xff;
    const float B0 = col0 & 0xff;

    const float R1 = (col1 >> 16) & 0xff;
    const float G1 = (col1 >> 8) & 0xff;
    const float B1 = col1 & 0xff;

    const float R2 = (col2 >> 16) & 0xff;
    const float G2 = (col2 >> 8) & 0xff;
    const float B2 = col2 & 0xff;

    _Rdx = (R0 * A12 + R1 * A20 + R2 * A01) / wTotal;
    _Rdy = (R0 * B12 + R1 * B20 + R2 * B01) / wTotal;

    _Gdx = (G0 * A12 + G1 * A20 + G2 * A01) / wTotal;
    _Gdy = (G0 * B12 + G1 * B20 + G2 * B01) / wTotal;

    _Bdx = (B0 * A12 + B1 * A20 + B2 * A01) / wTotal;
    _Bdy = (B0 * B12 + B1 * B20 + B2 * B01) / wTotal;

    _R = (R0 * w0 + R1 * w1 + R2 * w2) / wTotal;
    _G = (G0 * w0 + G1 * w1 + G2 * w2) / wTotal;
    _B = (B0 * w0 + B1 * w1 + B2 * w2) / wTotal;
    _rowR = _R;
    _rowG = _G;
    _rowB = _B;
  }

  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    illumination il = getLight(_norm, _luminance, x, y, z);
    return fcolor(((unsigned)_R << 16) | ((unsigned)_G << 8) | (unsigned)_B, il._R, il._G, il._B);
  }


#ifdef __AVX2__
  const inline __attribute__((always_inline)) void fragmentShader(__m256i& colorsData, const __m256i& zv, const __m256i& x, const __m256i& y, const __m256i& z) override {
    static const __m256i scaleFloat = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);
    const __m256i rAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_Rdx));
    __m256i rRow = _mm256_add_ps(_mm256_set1_ps(_R), rAdd);
    rRow = _mm256_cvttps_epi32(rRow);
    rRow = _mm256_slli_epi32(rRow, 16);

    const __m256i gAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_Gdx));
    __m256i gRow = _mm256_add_ps(_mm256_set1_ps(_G), gAdd);
    gRow = _mm256_cvttps_epi32(gRow);
    gRow = _mm256_slli_epi32(gRow, 8);

    const __m256i bAdd = _mm256_mul_ps(scaleFloat, _mm256_set1_ps(_Bdx));
    __m256i bRow = _mm256_add_ps(_mm256_set1_ps(_B), bAdd);
    bRow = _mm256_cvttps_epi32(bRow);

    __m256 rV;
    __m256 gV;
    __m256 bV;

    getLight(_mm256_set1_ps(_norm._x), _mm256_set1_ps(_norm._y),
             _mm256_set1_ps(_norm._z), _luminance,
             x, y, z, rV, gV, bV);

    colorsData = _mm256_or_si256(rRow, _mm256_or_si256(gRow, bRow));
    colorsData = vectorLight(colorsData, rV, gV, bV);

    _R += _Rdx * 8;
    _G += _Gdx * 8;
    _B += _Bdx * 8;
  }
#endif

  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {
    _R += _Rdx * step;
    _G += _Gdx * step;
    _B += _Bdx * step;
  }
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {
    _rowR += _Rdy;
    _rowG += _Gdy;
    _rowB += _Bdy;

    _R = _rowR;
    _G = _rowG;
    _B = _rowB;
  }

 private:
  float _R;
  float _G;
  float _B;
  float _Rdx;
  float _Rdy;
  float _Gdx;
  float _Gdy;
  float _Bdx;
  float _Bdy;

  float _rowR;
  float _rowG;
  float _rowB;

  vertex<float> _norm;
  const float _luminance;
};
