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
          }
      }

    for (auto t : head.getFaces()) {

      // We get the normal vector for every triangle
      vertex<float> v = cross(t._v0, t._v1, t._v2);

      // Angle of the light source
      vertex<float> light(x, y, -1);

      // Angle on the camera
      vertex<float> view(0, 0, -1);

      // Consider that positive z is "out" of the screen.
      // I have no idea what the convention is on other engines
      // Makes sense based on right hand rule

      // Then take the dot between the normal and the light
      // To determine the amount of lighting
      v.normalize();

      float seen = dot(v, view);
      float aoi = dot(v, light);

      if (aoi < 0.1) {
        aoi = 0.1;
      }

      if (seen > 0) {
        drawTri(t, aoi, headtext);

        if (wireframe) {
          line(t._v0, t._v1,  0xFFFFFFF);
          line(t._v1, t._v2,  0xFFFFFFF);
          line(t._v2, t._v0,  0xFFFFFFF);
        }
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
