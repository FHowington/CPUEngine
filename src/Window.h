#pragma once

const unsigned W = 1080;
const unsigned H = 1080;
// Extra row for the Y-flip overflow in copy_to_main_buffer
const unsigned BUF_SZ = W * H + W;

// Active render resolution (always <= W/H). Arrays stay sized at W*H.
extern unsigned rW;
extern unsigned rH;

#ifdef __AVX2__
const unsigned Wt = W + 7;
#else
const unsigned Wt = W;
#endif
const unsigned W4 = W*4;

const unsigned xZoom = 2;
const unsigned yZoom = 2;
const float xFOV = .5;
const float yFOV = .5;
extern float focalLength;  // Mutable: updated by Engine::setFOV()

const unsigned depth = 0xFFFF;
