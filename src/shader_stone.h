#pragma once

// --- Procedural stone wall helpers ---
inline float stoneHash(int a, int b) {
  int h = a * 374761393 + b * 668265263;
  h = (h ^ (h >> 13)) * 1274126177;
  return (float)(h & 0xFFFF) / 65535.0f;
}

inline unsigned stoneColor(float u, float v, const vertex<float>& faceNorm, const vertex<float>& tanU, const vertex<float>& tanV, float luminance, float wx, float wy, float wz) {
  const float blockW = 2.0f;
  const float blockH = 1.0f;
  const float mortar = 0.06f;

  int row = (int)floorf(v / blockH);
  float uShifted = u + (row & 1) * (blockW * 0.5f);
  int col = (int)floorf(uShifted / blockW);

  float uLocal = uShifted - col * blockW;
  float vLocal = v - row * blockH;

  bool isMortar = (uLocal < mortar || uLocal > blockW - mortar || vLocal < mortar || vLocal > blockH - mortar);

  unsigned baseR, baseG, baseB;
  vertex<float> norm = faceNorm;

  if (isMortar) {
    baseR = 60; baseG = 58; baseB = 55;
  } else {
    float h = stoneHash(col, row);
    baseR = (unsigned)(140 + h * 40);
    baseG = (unsigned)(130 + h * 30);
    baseB = (unsigned)(115 + h * 25);

    // Normal perturbation near block edges for depth illusion
    float edgeU = fminf(uLocal - mortar, blockW - mortar - uLocal);
    float edgeV = fminf(vLocal - mortar, blockH - mortar - vLocal);
    float bevel = 0.15f;

    if (edgeU < bevel) {
      float t = 1.0f - edgeU / bevel;
      float sign = (uLocal - mortar < blockW - mortar - uLocal) ? -1.0f : 1.0f;
      norm._x += tanU._x * sign * t * 0.5f;
      norm._y += tanU._y * sign * t * 0.5f;
      norm._z += tanU._z * sign * t * 0.5f;
    }
    if (edgeV < bevel) {
      float t = 1.0f - edgeV / bevel;
      float sign = (vLocal - mortar < blockH - mortar - vLocal) ? -1.0f : 1.0f;
      norm._x += tanV._x * sign * t * 0.5f;
      norm._y += tanV._y * sign * t * 0.5f;
      norm._z += tanV._z * sign * t * 0.5f;
    }
    norm.normalize();

    float noise = stoneHash((int)(u * 7.3f), (int)(v * 7.7f));
    baseR = (unsigned)fminf(255, baseR + (noise - 0.5f) * 20);
    baseG = (unsigned)fminf(255, baseG + (noise - 0.5f) * 15);
    baseB = (unsigned)fminf(255, baseB + (noise - 0.5f) * 12);
  }

  illumination il = getLight(norm, luminance, wx, wy, wz);
  unsigned r = fast_min(255, (int)(baseR * il._R));
  unsigned g = fast_min(255, (int)(baseG * il._G));
  unsigned b = fast_min(255, (int)(baseB * il._B));
  return (r << 16) | (g << 8) | b;
}

class StoneXZShader : public UntexturedShader, public BehindCamera {
 public:
  StoneXZShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }
  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    return stoneColor(x, z, _norm, vertex<float>(1,0,0), vertex<float>(0,0,1), _luminance, x, y, z);
  }
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {}
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {}
 private:
  const float _luminance;
  const vertex<float>& _norm;
};

class StoneXYShader : public UntexturedShader, public BehindCamera {
 public:
  StoneXYShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }
  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    return stoneColor(x, y, _norm, vertex<float>(1,0,0), vertex<float>(0,1,0), _luminance, x, y, z);
  }
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {}
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {}
 private:
  const float _luminance;
  const vertex<float>& _norm;
};

class StoneYZShader : public UntexturedShader, public BehindCamera {
 public:
  StoneYZShader(const ModelInstance& m, const face& f,
              const short A12, const short A20, const short A01,
              const short B12, const short B20, const short B01,
              const float wTotal, int w0, int w1, int w2,
              const vertex<int>& v0, const vertex<int>& v1, const vertex<int>& v2) : _luminance(m._globalIllumination), _norm(m._baseModel.getVertexNormal(f._v2))
  { }
  inline __attribute__((always_inline)) fcolor fragmentShader(const float x, const float y, const float z, const unsigned color = 0) override {
    return stoneColor(z, y, _norm, vertex<float>(0,0,1), vertex<float>(0,1,0), _luminance, x, y, z);
  }
  inline __attribute__((always_inline)) void stepXForX(const unsigned step = 1) override {}
  inline __attribute__((always_inline)) void stepYForX(const unsigned step = 0) override {}
 private:
  const float _luminance;
  const vertex<float>& _norm;
};
