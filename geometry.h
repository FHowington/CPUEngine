#pragma once
#include <cmath>
#include <iostream>
#include <vector>

// This defines a 4x4 matrix
template <unsigned N, unsigned M>
struct fMatrix {
  float _m[N * M];

  static fMatrix identity();
  template <unsigned I, unsigned J>
  auto operator*(const fMatrix<I, J>& rhs) const;

  float at(const unsigned i, const unsigned j) const { return _m[M * i + j]; }
  void set(const unsigned i, const unsigned j, const float val) { _m[M * i + j] = val; }
};
