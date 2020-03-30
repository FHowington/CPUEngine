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

int main() {
    // Create a screen.
    SDL_Window* window = SDL_CreateWindow("Chip8", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, W*4,H*4, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, W,H);

    bool wireframe = false;
    std::vector< std::array<std::array<int,2>, 3> > triangles = CreateTriangleMesh(W,H,100);

    // This commented out code is for drawing triangles
    // for(bool interrupted=false; !interrupted; )
    //   {
    //     unsigned color = 0x3B0103A5;

    //     // Process events.
    //     SDL_Event ev;
    //     while(SDL_PollEvent(&ev))
    //       switch(ev.type)
    //         {
    //         case SDL_QUIT: interrupted = true; break;
    //         }
    //     for(auto& p: pixels) p = blank;
    //     for(auto& t: triangles)
    //     {
    //       color = ((color << 1) | (color >> (32-1)));
    //       std::shared_ptr<vertex> t0 = std::make_shared<vertex>(t[0][0], t[0][1], 0);
    //       std::shared_ptr<vertex> t1 = std::make_shared<vertex>(t[1][0], t[1][1], 0);
    //       std::shared_ptr<vertex> t2 = std::make_shared<vertex>(t[2][0], t[2][1], 0);
    //       //std::shared_ptr<vertex> t0 = std::make_shared<vertex>(0, 0, 0);
    //       //std::shared_ptr<vertex> t1 = std::make_shared<vertex>(0, 100, 0);
    //       //std::shared_ptr<vertex> t2 = std::make_shared<vertex>(100, 100, 0);
    //       drawTri(t0, t1, t2, color);
    //       //drawPoly(t[0], t[1], t[2], color);
    //     }
    //     SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
    //     SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    //     SDL_RenderPresent(renderer);

    //     SDL_Delay(1000/60);
    //   }

    auto lines = getWireframe("african_head.obj");


    unsigned color = 0x3B0103A5;

    for(bool interrupted=false; !interrupted; )
    {

      SDL_Event ev;

      while(SDL_PollEvent(&ev))
        switch(ev.type)
        {
          case SDL_QUIT: interrupted = true; break;
        }

      for (auto l : lines) {
        if (wireframe) {
          line(std::get<0>(l)->_x, std::get<1>(l)->_x, std::get<0>(l)->_y, std::get<1>(l)->_y, color);
          line(std::get<1>(l)->_x, std::get<2>(l)->_x, std::get<1>(l)->_y, std::get<2>(l)->_y, color);
          line(std::get<2>(l)->_x, std::get<0>(l)->_x, std::get<2>(l)->_y, std::get<0>(l)->_y, color);
        } else {
          vertex v = cross(*(std::get<0>(l)), *(std::get<1>(l)), *(std::get<2>(l)));
          printf("cross %d %d %d \n", v._x, v._y, v._z);
          printf("%d %d %d \n", std::get<0>(l)->_x, std::get<0>(l)->_y, std::get<0>(l)->_z);
          printf("%d %d %d \n", std::get<1>(l)->_x, std::get<1>(l)->_y, std::get<1>(l)->_z);
          vertex light(0, 0, 100);

          // Consider that positive x is "into" the screen.
          // I have no idea what the convention is on other engines
          int aoi = dot(v.normalize(), light);
          printf("%d \n", aoi);

          if (aoi > 0) {
            color = 0xFFFFFF - 0xF * aoi;
            drawTri(std::get<0>(l), std::get<1>(l), std::get<2>(l), color);
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
