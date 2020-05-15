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

template <typename T>
inline void renderModel(const ModelInstance* model, const matrix<4,4>& cameraTransform, const matrix<4,4>& viewClip, const vertex<float>& light) {
  for (auto t : model->_baseModel.getFaces()) {
    const vertex<int> v0i(pipeline(cameraTransform, model->_position, viewClip, model->_baseModel.getVertex(t._v0), 1.5));
    const vertex<int> v1i(pipeline(cameraTransform, model->_position, viewClip, model->_baseModel.getVertex(t._v1), 1.5));
    const vertex<int> v2i(pipeline(cameraTransform, model->_position, viewClip, model->_baseModel.getVertex(t._v2), 1.5));

    // We get the normal vector for every triangle
    const vertex<float> v = cross(v0i, v1i, v2i);

    // If it is backfacing, vector will be pointing in +z, so cull it
    if (v._z < 0) {

      drawTri<T>(*model, t, light, v0i, v1i, v2i);
    }
  }
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

  // This is where the per model will be done.
  std::vector<ModelInstance*> modelsInScene;

  ModelInstance modInstance(head, &headtext, shaderType::GouraudShader);
  modelsInScene.push_back(&modInstance);

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
      float movemt = (float)d.count() / SPEED;
      cameraX -= movemt * cameraRot._m[8];
      cameraY -= movemt * cameraRot._m[9];
      cameraZ -= movemt * cameraRot._m[10];
    } else if (mBackward) {
      float movemt = (float)d.count() / SPEED;
      cameraX += movemt * cameraRot._m[8];
      cameraY += movemt * cameraRot._m[9];
      cameraZ += movemt * cameraRot._m[10];
    }

    if (mLeft) {
      float movemt = (float)d.count() / SPEED;
      cameraX -= movemt * cameraRot._m[0];
      cameraY -= movemt * cameraRot._m[1];
      cameraZ -= movemt * cameraRot._m[2];
    } else if (mRight) {
      float movemt = (float)d.count() / SPEED;
      cameraX += movemt * cameraRot._m[0];
      cameraY += movemt * cameraRot._m[1];
      cameraZ += movemt * cameraRot._m[2];
    }

    cameraRot.set(3, 0, cameraX);
    cameraRot.set(3, 1, cameraY);
    cameraRot.set(3, 2, cameraZ);

    matrix<4,4> cameraTransform = invert(cameraRot);

    modInstance._position = matrix<4,4>::rotationY(rot);
    modInstance._position.set(3, 2, -5);

    vertex<float> light(x, y, -1);

    unsigned idx = 0;
    for (ModelInstance* model : modelsInScene) {
      if (!wireframe) {
        switch (modInstance._shader) {
          case shaderType::FlatShader: {
            renderModel<FlatShader>(model, cameraTransform, viewClip, light);
            break;
          }

          case shaderType::GouraudShader: {
            renderModel<GouraudShader>(model, cameraTransform, viewClip, light);
            break;
          }

          case shaderType::InterpFlatShader: {
            renderModel<InterpFlatShader>(model, cameraTransform, viewClip, light);
            break;
          }
          case shaderType::InterpGouraudShader: {
            renderModel<InterpGouraudShader>(model, cameraTransform, viewClip, light);
            break;
          }
        }
      } else {
        for (auto t : head.getFaces()) {
          const vertex<int> v0i(pipeline(cameraTransform, model->_position, viewClip, head.getVertex(t._v0), 1.5));
          const vertex<int> v1i(pipeline(cameraTransform, model->_position, viewClip, head.getVertex(t._v1), 1.5));
          const vertex<int> v2i(pipeline(cameraTransform, model->_position, viewClip, head.getVertex(t._v2), 1.5));

          line(v0i, v1i,  0xFFFFFFF);
          line(v1i, v2i,  0xFFFFFFF);
          line(v2i, v0i,  0xFFFFFFF);
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

    }
  }
  return 0;
}
