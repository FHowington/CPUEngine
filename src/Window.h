#pragma once

const unsigned W = 1920;
#ifdef __AVX2__
const unsigned Wt = 1920 + 7;
#else
const unsigned Wt = 1920;
#endif
const unsigned H = 1080;
const unsigned W4 = W*4;

const unsigned xZoom = 2;
const unsigned yZoom = 2;
const unsigned xFOV = 2;
const unsigned yFOV = 2;
