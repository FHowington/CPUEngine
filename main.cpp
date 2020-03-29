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
    //       {
    //         color = ((color << 1) | (color >> (32-1)));
    //         drawTri(t[0], t[1], t[2], color);
    //         //drawPoly(t[0], t[1], t[2], color);
    //       }
    //     SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
    //     SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    //     SDL_RenderPresent(renderer);

    //     SDL_Delay(1000/60);
    //   }

    auto lines = getWireframe("african_head.obj");


    for(bool interrupted=false; !interrupted; )
    {
      unsigned color = 0x3B0103A5;

      SDL_Event ev;

      while(SDL_PollEvent(&ev))
        switch(ev.type)
        {
          case SDL_QUIT: interrupted = true; break;
        }
      for (auto l : lines) {
        line(l.first->_x, l.second->_x, l.first->_y, l.second->_y, color);
      }
       SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
       SDL_RenderCopy(renderer, texture, nullptr, nullptr);
       SDL_RenderPresent(renderer);
       SDL_Delay(1000/60);
    }

    return 0;
}
