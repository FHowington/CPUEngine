#ifndef _USERS_FORBES_CLIONPROJECTS_CPUENGINE_SRC_WINDOW_H
#define _USERS_FORBES_CLIONPROJECTS_CPUENGINE_SRC_WINDOW_H

#pragma once

const unsigned W = 1080;
const unsigned H = 1080;

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
const float focalLength = 1.5;

const unsigned depth = 0xFFFFF;

#endif
