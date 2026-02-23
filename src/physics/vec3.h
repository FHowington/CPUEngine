#pragma once

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
