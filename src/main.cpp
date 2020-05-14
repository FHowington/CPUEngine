//
// Created by Forbes Howington on 3/27/20.
//
#include <array>
#include <chrono>
#include <iostream>
#include "rasterize.h"
#include <SDL2/SDL.h>
#include <thread>
#include "tgaimage.h"
#include <vector>
#include "Window.h"
#include "geometry.h"

#define SPEED 80000000

unsigned pixels[W * H];
int zbuff[W * H];

// TODO: Clip at far z to maximize z-buffer granularity
matrix<4,1> project(const vertex<float>& f) {
  matrix<4,1> res;
  res._m[0] = f._x / -(1.5 * f._z);
  res._m[1] = f._y / -(1.5 * f._z);
  res._m[2] = f._z;
  res._m[3] = 1;
  return res;
}

int main() {
  // Create a screen.
  SDL_Window* window = SDL_CreateWindow("Chip8", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, W*4,H*4, SDL_WINDOW_RESIZABLE);
  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, W,H);

  TGAImage headtext;
  headtext.read_tga_file("/Users/forbes/CLionProjects/CPUEngine/diablo3_pose_diffuse.tga");
  headtext.flip_vertically();
  Model head("/Users/forbes/CLionProjects/CPUEngine/diablo3_pose.obj", headtext.get_width(), headtext.get_height());

  bool wireframe = false;
  bool fps = false;

  unsigned frame = 0;
  float x = 0;
  float y = 0;
  auto start = std::chrono::high_resolution_clock::now();
  auto lastFrame = start;
  float rot = 0;

  float cameraRotX = 0;
  float cameraRotY = 0;

  bool lLeft = false;
  bool lRight = false;
  bool lUp = false;
  bool lDown = false;

  bool mForward = false;
  bool mBackward = false;
  bool mLeft = false;
  bool mRight = false;

  float cameraX = 0;
  float cameraY = 0;
  float cameraZ = 0;

  for(bool interrupted=false; !interrupted;)
  {
    for(auto& p: pixels) p = 0;
    for(auto& p: zbuff) p = std::numeric_limits<int>::min();

    SDL_Event ev;
    while(SDL_PollEvent(&ev))
      switch(ev.type)
      {
        case SDL_QUIT: interrupted = true; break;
        case SDL_KEYDOWN:
          switch (ev.key.keysym.sym)
          {
            case SDLK_DOWN:
              lDown = true;
              break;

            case SDLK_UP:
              lUp = true;
              break;

            case SDLK_LEFT:
              lLeft = true;
              break;

            case SDLK_RIGHT:
              lRight = true;
              break;

            case SDLK_p:
              wireframe = !wireframe;
              break;

            case SDLK_f:
              fps = !fps;
              break;

            case SDLK_e:
              rot+= 0.05;
              break;

            case SDLK_q:
              rot -= 0.05;
              break;

            case SDLK_w:
              mForward = true;
              break;

            case SDLK_s:
              mBackward = true;
              break;

            case SDLK_a:
              mLeft = true;
              break;

            case SDLK_d:
              mRight = true;
              break;
          }
          break;

        case SDL_KEYUP:
          switch (ev.key.keysym.sym)
          {
            case SDLK_DOWN:
              lDown = false;
              break;

            case SDLK_UP:
              lUp = false;
              break;

            case SDLK_LEFT:
              lLeft = false;
              break;

            case SDLK_RIGHT:
              lRight = false;
              break;

            case SDLK_w:
              mForward = false;
              break;

            case SDLK_s:
              mBackward = false;
              break;

            case SDLK_a:
              mLeft = false;
              break;

            case SDLK_d:
              mRight = false;
              break;
          }
      }

    if (lUp) {
      cameraRotX -= 0.03;
    }
    if (lDown) {
      cameraRotX += 0.03;
    }
    if (lRight) {
      cameraRotY -= 0.03;
    }
    if (lLeft) {
      cameraRotY += 0.03;
    }


    // TODO: Convert to more understandable numbers
    static const matrix<4,4> viewClip = viewport((W) / 2.0, (H) / 2.0, 2*W, 2*H);

    matrix<4,4> cameraRotXM = matrix<4,4>::rotationX(cameraRotX);
    matrix<4,4> cameraRotYM = matrix<4,4>::rotationY(cameraRotY);
    matrix<4,4> cameraRot(cameraRotXM * cameraRotYM);

    auto frameTime = std::chrono::high_resolution_clock::now();
    auto d = frameTime - lastFrame;
    lastFrame = frameTime;

    if (mForward) {
      cameraX -= d.count() * cameraRot._m[8]/SPEED;
      cameraY -= d.count() * cameraRot._m[9]/SPEED;
      cameraZ -= d.count() * cameraRot._m[10]/SPEED;
    } else if (mBackward) {
      cameraX += d.count() * cameraRot._m[8]/SPEED;
      cameraY += d.count() * cameraRot._m[9]/SPEED;
      cameraZ += d.count() * cameraRot._m[10]/SPEED;
    }

    if (mLeft) {
      cameraX -= d.count() * cameraRot._m[0]/SPEED;
      cameraY -= d.count() * cameraRot._m[1]/SPEED;
      cameraZ -= d.count() * cameraRot._m[2]/SPEED;
    } else if (mRight) {
      cameraX += d.count() * cameraRot._m[0]/SPEED;
      cameraY += d.count() * cameraRot._m[1]/SPEED;
      cameraZ += d.count() * cameraRot._m[2]/SPEED;
    }

    cameraRot.set(3, 0, cameraX);
    cameraRot.set(3, 1, cameraY);
    cameraRot.set(3, 2, cameraZ);

    matrix<4,4> cameraTransform = invert(cameraRot);


    // This is where the per model will be done.
    ModelInstance modInstance(head);
    modInstance.position = matrix<4,4>::rotationY(rot);
    modInstance.position.set(3, 2, -5);

    vertex<float> light(x, y, -1);

    unsigned idx = 0;
    for (auto t : head.getFaces()) {
      const vertex<int> v0i(pipeline(cameraTransform, modInstance.position, viewClip, head.getVertex(t._v0), 1.5));
      const vertex<int> v1i(pipeline(cameraTransform, modInstance.position, viewClip, head.getVertex(t._v1), 1.5));
      const vertex<int> v2i(pipeline(cameraTransform, modInstance.position, viewClip, head.getVertex(t._v2), 1.5));

      // We get the normal vector for every triangle
      vertex<float> v = cross(v0i, v1i, v2i);

      // If it is backfacing, vector will be pointing in +z, so cull it
      if (v._z < 0) {

        drawTri<GouraudShader>(modInstance, t, light, headtext, v0i, v1i, v2i);
      }
      if (wireframe) {
        line(v0i, v1i,  0xFFFFFFF);
        line(v1i, v2i,  0xFFFFFFF);
        line(v2i, v0i,  0xFFFFFFF);
      }
    }

    SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);

    if (fps) {
      ++frame;
      if (frame == 100) {
        frame = 0;
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        float fps = (100.0 / duration.count()) * 1000000;
        printf("%f FPS\n", fps);
        start = std::chrono::high_resolution_clock::now();
      }

    } else {
      //SDL_Delay(1000/60);
    }
  }

  return 0;
}
