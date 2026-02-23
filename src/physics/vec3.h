#pragma once

#include "../simd_compat.h"
#include <cmath>

struct Vec3 {
  float x, y, z;

  Vec3() : x(0), y(0), z(0) {}
  Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

  Vec3 operator+(const Vec3& r) const { return {x+r.x, y+r.y, z+r.z}; }
  Vec3 operator-(const Vec3& r) const { return {x-r.x, y-r.y, z-r.z}; }
  Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
  Vec3& operator+=(const Vec3& r) { x+=r.x; y+=r.y; z+=r.z; return *this; }
  Vec3& operator-=(const Vec3& r) { x-=r.x; y-=r.y; z-=r.z; return *this; }
  Vec3& operator*=(float s) { x*=s; y*=s; z*=s; return *this; }

  float dot(const Vec3& r) const { return x*r.x + y*r.y + z*r.z; }
  float length2() const { return dot(*this); }
  float length() const { return sqrtf(length2()); }
  Vec3 normalized() const { float l = length(); return l > 0 ? *this * (1.0f/l) : Vec3(); }
};

// ─── Batch SIMD operations on arrays of Vec3 ────────────────────────────────

#if defined(__AVX2__) && defined(__FMA__)

// Integrate N bodies: velocity += (force * invMass) * dt; position += velocity * dt; force = 0
// Data is stored as separate x/y/z arrays for SoA layout.
inline void integrateAVX(float* px, float* py, float* pz,
                         float* vx, float* vy, float* vz,
                         float* fx, float* fy, float* fz,
                         const float* invMass, unsigned n, float dt) {
  __m256 vDt = _mm256_set1_ps(dt);
  __m256 vZero = _mm256_setzero_ps();
  unsigned i = 0;
  for (; i + 8 <= n; i += 8) {
    __m256 im = _mm256_loadu_ps(invMass + i);
    // accel = force * invMass
    __m256 ax = _mm256_mul_ps(_mm256_loadu_ps(fx + i), im);
    __m256 ay = _mm256_mul_ps(_mm256_loadu_ps(fy + i), im);
    __m256 az = _mm256_mul_ps(_mm256_loadu_ps(fz + i), im);
    // velocity += accel * dt
    __m256 newVx = _mm256_fmadd_ps(ax, vDt, _mm256_loadu_ps(vx + i));
    __m256 newVy = _mm256_fmadd_ps(ay, vDt, _mm256_loadu_ps(vy + i));
    __m256 newVz = _mm256_fmadd_ps(az, vDt, _mm256_loadu_ps(vz + i));
    _mm256_storeu_ps(vx + i, newVx);
    _mm256_storeu_ps(vy + i, newVy);
    _mm256_storeu_ps(vz + i, newVz);
    // position += velocity * dt
    _mm256_storeu_ps(px + i, _mm256_fmadd_ps(newVx, vDt, _mm256_loadu_ps(px + i)));
    _mm256_storeu_ps(py + i, _mm256_fmadd_ps(newVy, vDt, _mm256_loadu_ps(py + i)));
    _mm256_storeu_ps(pz + i, _mm256_fmadd_ps(newVz, vDt, _mm256_loadu_ps(pz + i)));
    // clear force
    _mm256_storeu_ps(fx + i, vZero);
    _mm256_storeu_ps(fy + i, vZero);
    _mm256_storeu_ps(fz + i, vZero);
  }
  // Scalar tail
  for (; i < n; ++i) {
    float im2 = invMass[i];
    vx[i] += fx[i] * im2 * dt; vy[i] += fy[i] * im2 * dt; vz[i] += fz[i] * im2 * dt;
    px[i] += vx[i] * dt; py[i] += vy[i] * dt; pz[i] += vz[i] * dt;
    fx[i] = fy[i] = fz[i] = 0;
  }
}

// Apply gravity to N bodies: force += gravity * mass (skip where invMass == 0)
inline void applyGravityAVX(float* fy, const float* mass, unsigned n, float gy) {
  __m256 vGy = _mm256_set1_ps(gy);
  unsigned i = 0;
  for (; i + 8 <= n; i += 8) {
    __m256 m = _mm256_loadu_ps(mass + i);
    __m256 f = _mm256_loadu_ps(fy + i);
    _mm256_storeu_ps(fy + i, _mm256_fmadd_ps(m, vGy, f));
  }
  for (; i < n; ++i)
    fy[i] += mass[i] * gy;
}

#endif
