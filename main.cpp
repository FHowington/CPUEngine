#include <iostream>
#include "mesh.h"
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

    // Create a mesh of triangles covering the entire screen
    std::vector< std::array<std::array<int,2>, 3> > triangles;
    std::array<int, 2> p0 = {0, 0};
    std::array<int, 2> p1 = {20, 100};
    std::array<int, 2> p2 = {400, 400};

    std::array<std::array<int, 2>, 3> elem1 = {p0, p1, p2};
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
            drawPoly(triangles[0][0], triangles[0][1], triangles[0][2]);
        }
        SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        SDL_Delay(1000/60);
    }
}
