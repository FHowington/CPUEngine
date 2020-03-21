#include <iostream>
#include "RasterizePolygon.h"
#include <vector>
#include <array>
#include <SDL2/SDL.h>

unsigned pixels[800 * 480];

int main() {
    const int W = 800, H = 480;
    // Create a screen.
    SDL_Window* window = SDL_CreateWindow("Chip8", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, W*4,H*4, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, W,H);


    std::vector< std::array<std::array<int,2>, 3> > triangles;
    std::array<std::array<int,2>, 3> elem1 = {{{{0,0}}, {{10,10}}, {{0,50}}}};

    triangles.push_back(elem1);

    for(bool interrupted=false; !interrupted; )
    {
        // Process events.
        SDL_Event ev;
        while(SDL_PollEvent(&ev))
            switch(ev.type)
            {
                case SDL_QUIT: interrupted = true; break;
            }
        for(auto& p: pixels) p = blank;
        for(auto& t: triangles)
        {
            drawPoly(t[0], t[1], t[2]);
        }
        SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        SDL_Delay(1000/60);
    }
}
