//
// Created by Forbes Howington on 3/27/20.
//
#include <array>
#include <chrono>
#include <iostream>
#include "rasterize.h"
#include <SDL2/SDL.h>
#include "tgaimage.h"
#include <vector>
#include "Window.h"
#include "geometry.h"

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
  headtext.read_tga_file("/Users/forbes/CLionProjects/CPUEngine/african_head_diffuse.tga");
  headtext.flip_vertically();
  Model head("/Users/forbes/CLionProjects/CPUEngine/african_head.obj", headtext.get_width(), headtext.get_height());

  bool wireframe = false;
  bool fps = false;

  unsigned frame = 0;
  float x = 0;
  float y = 0;
  auto start = std::chrono::high_resolution_clock::now();
  float rot = 0;

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
              if (y <= .95)
              y += 0.05;
              break;

            case SDLK_UP:
              if (y >= -0.95 )
              y -= 0.05;
              break;

            case SDLK_LEFT:
              if (x <= .95)
              x += 0.05;
              break;

            case SDLK_RIGHT:
              if (x >= -0.95 )
              x -= 0.05;
              break;

            case SDLK_w:
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
          }
      }

    // TODO: Convert to more understandable numbers
    static const matrix<4,4> viewClip = viewport((W) / 2.0, (H) / 2.0, W*3/4, H*3/4);

    matrix<4,4> cameraPos = matrix<4,4>::rotation(0, 0, 1.6);

    cameraPos.set(3, 0, .9);
    cameraPos.set(3, 1, -0.9);
    cameraPos.set(3, 2, -5);

    matrix<4,4> cameraTransform = invert(cameraPos);


    // This is where the per model will be done..

    matrix<4,4> model = matrix<4,4>::rotation(0,0, rot);
    model.set(3, 2, -5);
    model.set(3, 1, -.9);
    model.set(3, 0, -.8);

    vertex<float> light(x, y, -1);

    unsigned idx = 0;
    for (auto t : head.getFaces()) {
      const vertex<int> v0i(pipeline(cameraTransform, model, viewClip, head.getVertex(t._v0), 1.5));
      const vertex<int> v1i(pipeline(cameraTransform, model, viewClip, head.getVertex(t._v1), 1.5));
      const vertex<int> v2i(pipeline(cameraTransform, model, viewClip, head.getVertex(t._v2), 1.5));

      // We get the normal vector for every triangle
      vertex<float> v = cross(v0i, v1i, v2i);

      // If it is backfacing, vector will be pointing in +z, so cull it
      if (v._z < 0) {
        const vertex<float> v0iLight(multToVector(model, head.getVertex(t._v0)));
        const vertex<float> v1iLight(multToVector(model, head.getVertex(t._v1)));
        const vertex<float> v2iLight(multToVector(model, head.getVertex(t._v2)));
        vertex<float> vLight = cross(v0iLight, v1iLight, v2iLight);
        vLight.normalize();
        float aoi = dot(vLight, light);

        // Effectively, this is the global illumination
        if (aoi < 0.2) {
          aoi = 0.2;
        }

        drawTri(head, t, aoi, headtext, v0i, v1i, v2i);
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
