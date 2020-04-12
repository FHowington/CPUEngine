#include <iostream>
#include "geometry.h"

template <>
matrix<4,4> matrix<4,4>::identity() {
  matrix<4,4> res;
  for (unsigned idx = 0; idx < 4; ++idx) {
    res._m[idx * 4 + idx] = 1;
  }
  return res;
}
