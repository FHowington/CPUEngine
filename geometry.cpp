#include <vector>
#include <cassert>
#include <cmath>
#include <iostream>
#include "geometry.h"

template <>
fMatrix<4,4> fMatrix<4,4>::identity() {
  fMatrix<4,4> res;
  for (unsigned idx = 0; idx < 4; ++idx) {
    res._m[idx * 4 + idx] = 1;
  }
  return res;
}
