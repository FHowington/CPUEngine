//
// Created by Forbes Howington on 3/27/20.
//
#include <iostream>
#include "RasterizePolygon.h"
#include <vector>
#include <array>
#include <SDL2/SDL.h>
#include "Window.h"
#include "mesh.h"

unsigned pixels[W * H];
unsigned zbuff[W * H];

int main() {
  // Create a screen.
  SDL_Window* window = SDL_CreateWindow("Chip8", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, W*4,H*4, SDL_WINDOW_RESIZABLE);
  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, W,H);

  bool wireframe = false;

  // //This commented out code is for drawing triangles
  // std::vector< std::array<std::array<int,2>, 3> > triangles = CreateTriangleMesh(W,H,100);
  // for(bool interrupted=false; !interrupted; )
  // {
  //   unsigned color = 0x3B0103A5;

  //   //Process events.
  //   SDL_Event ev;
  //   while(SDL_PollEvent(&ev))
  //     switch(ev.type)
  //     {
  //       case SDL_QUIT: interrupted = true; break;
  //     }
  //   for(auto& p: pixels) p = blank;
  //   for(auto& t: triangles)
  //   {
  //     color = ((color << 1) | (color >> (32-1)));
  //     std::shared_ptr<vertex> t0 = std::make_shared<vertex>(((float)t[0][0]-halfW)/(float)halfW, ((float)t[0][1]-halfH)/(float)halfH, 0);
  //     std::shared_ptr<vertex> t1 = std::make_shared<vertex>(((float)t[1][0]-halfW)/(float)halfW, ((float)t[1][1]-halfH)/(float)halfH, 0);
  //     std::shared_ptr<vertex> t2 = std::make_shared<vertex>(((float)t[2][0]-halfW)/(float)halfW, ((float)t[2][1]-halfH)/(float)halfH, 0);

  //     drawTri(*t0, *t1, *t2, color);
  //   }

  //   SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
  //   SDL_RenderCopy(renderer, texture, nullptr, nullptr);
  //   SDL_RenderPresent(renderer);
  // }

  auto lines = getWireframe("/Users/forbes/CLionProjects/CPUEngine/african_head.obj");
  unsigned color = 0x3B0103A5;

  unsigned frame = 0;
  for(bool interrupted=false; !interrupted && frame < 1000; ++frame)
  {
    for(auto& p: zbuff) p = 0;

    SDL_Event ev;

    while(SDL_PollEvent(&ev))
      switch(ev.type)
      {
        case SDL_QUIT: interrupted = true; break;
      }

    for (auto l : lines) {


      // We get the normal vector for every triangle
      vertex v = cross(*(std::get<0>(l)), *(std::get<1>(l)), *(std::get<2>(l)));

      // Angle of the light source
      vertex light(0, -0.5, -.7);

      // Consider that positive z is "out" of the screen.
      // I have no idea what the convention is on other engines
      // Makes sense based on right hand rule

      // Then take the dot between the normal and the light
      // To determine the amount of lighting
      float aoi = dot(v.normalize(), light);

      if (aoi > 0) {
        fcolor c(0, 255 * aoi, 255 * aoi, 255 * aoi);
        drawTri(*std::get<0>(l), *std::get<1>(l), *std::get<2>(l), c);

        if (wireframe) {
          line(*std::get<0>(l), *std::get<1>(l),  0xFFFFFFF);
          line(*std::get<1>(l), *std::get<2>(l),  0xFFFFFFF);
          line(*std::get<2>(l), *std::get<0>(l),  0xFFFFFFF);
        }
      }
    }

    SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
    SDL_Delay(1000/60);
  }

  return 0;
}
