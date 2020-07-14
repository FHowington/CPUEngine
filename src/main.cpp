//
// Created by Forbes Howington on 3/27/20.
//
#include "Window.h"
#include "geometry.h"
#include "light.h"
#include "pool.h"
#include "rasterize.h"
#include "tgaimage.h"
#include <SDL2/SDL.h>
#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

constexpr float SPEED = 80000000.0;

std::array<unsigned, W * H> pixels;
std::array<int, W*H> zbuff;

matrix<4,4> cameraTransform;
std::atomic<unsigned> remaining_models;
std::array<unsigned, W * H> pixels_flipped;


int main() {
  remaining_models = 0;
  // Create a screen.
  SDL_Window* window = SDL_CreateWindow("Chip8", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, W*4,H*4, SDL_WINDOW_RESIZABLE);
  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, W,H);

  TGAImage headtext;
  headtext.read_tga_file("/Users/forbes/CLionProjects/CPUEngine/african_head_diffuse.tga");
  headtext.flip_vertically();
  Model head("/Users/forbes/CLionProjects/CPUEngine/african_head.obj", headtext.get_width(), headtext.get_height());

  Model plane;
  std::vector<vertex<float>> planeVertices;

  planeVertices.emplace_back(10, -5, -5);
  planeVertices.emplace_back(-10, -5, -5);
  planeVertices.emplace_back(10, -5, -25);

  planeVertices.emplace_back(10, -5, -25);
  planeVertices.emplace_back(-10, -5, -5);
  planeVertices.emplace_back(-10, -5, -25);

  std::vector<vertex<float>> planeNorms;

  planeNorms.emplace_back(0,1,0);
  planeNorms.emplace_back(0,1,0);
  planeNorms.emplace_back(0,1,0);

  planeNorms.emplace_back(0,1,0);
  planeNorms.emplace_back(0,1,0);
  planeNorms.emplace_back(0,1,0);

  std::vector<face> planeFaces;
  planeFaces.emplace_back(2, 1, 0, 10, -5, -10, -5, 10, -5);
  planeFaces.emplace_back(5, 4, 3, -10, -5, -10, -5, 10, -5);

  plane.setVertices(std::move(planeVertices));
  plane.setNormals(std::move(planeNorms));
  plane.setFaces(std::move(planeFaces));

  bool wireframe = false;
  bool fps = false;

  unsigned frame = 0;
  float x = 1;
  float y = -3;
  Light::sceneLights.emplace_back(LightType::Point, -5, -3, -10, 100, 1, 0, 0);
  Light::sceneLights.emplace_back(LightType::Point, 5, -3, -10, 50, 0, 0, 1);
  Light::sceneLights.emplace_back(LightType::Directional, vertex<float>(5, y, -1.5), 1, 1, 1);
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
  std::vector<std::shared_ptr<const ModelInstance>> modelsInScene;
  std::map<const std::string, Model> models;
  std::map<const std::string, TGAImage> textures;


  loadScene(modelsInScene, models, textures, "scene1.scn");

  std::shared_ptr<ModelInstance> modInstance = std::make_shared<ModelInstance>(head, &headtext, shaderType::GouraudShader);
  modelsInScene.push_back(modInstance);

  std::shared_ptr<ModelInstance> planeInstance = std::make_shared<ModelInstance>(plane, nullptr, shaderType::PlaneShader);
  planeInstance->_position = matrix<4,4>::identity();
  modelsInScene.push_back(planeInstance);

  Pool pool(std::thread::hardware_concurrency());

  for(bool interrupted=false; !interrupted;)
  {
    for(auto& p: pixels) { p = 0;
}
    for(auto& p: zbuff) { p = std::numeric_limits<int>::min();
}

    for (const auto& m : modelsInScene) {
      ++remaining_models;
      Pool::enqueue_model(m);
    }

    // TODO(forbes): Change this to something..better. A conditional perhaps.
    while(remaining_models != 0U) {;
}

    SDL_Event ev;
    while(SDL_PollEvent(&ev) != 0) {
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

            case SDLK_i:
              y += 0.2;
              break;

            case SDLK_k:
              y -= 0.2;
              break;

            case SDLK_l:
              x += 0.2;
              break;

            case SDLK_j:
              x -= 0.2;
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


    // TODO(forbes): Convert to more understandable numbers

    matrix<4,4> cameraRotXM = matrix<4,4>::rotationX(cameraRotX);
    matrix<4,4> cameraRotYM = matrix<4,4>::rotationY(cameraRotY);
    matrix<4,4> cameraRot(cameraRotXM * cameraRotYM);

    auto frameTime = std::chrono::high_resolution_clock::now();
    auto d = frameTime - lastFrame;
    lastFrame = frameTime;

    if (mForward) {
      float movemt = static_cast<float>(d.count()) / SPEED;
      cameraX -= movemt * cameraRot._m[8];
      cameraY -= movemt * cameraRot._m[9];
      cameraZ -= movemt * cameraRot._m[10];
    } else if (mBackward) {
      float movemt = static_cast<float>(d.count()) / SPEED;
      cameraX += movemt * cameraRot._m[8];
      cameraY += movemt * cameraRot._m[9];
      cameraZ += movemt * cameraRot._m[10];
    }

    if (mLeft) {
      float movemt = static_cast<float>(d.count()) / SPEED;
      cameraX -= movemt * cameraRot._m[0];
      cameraY -= movemt * cameraRot._m[1];
      cameraZ -= movemt * cameraRot._m[2];
    } else if (mRight) {
      float movemt = static_cast<float>(d.count()) / SPEED;
      cameraX += movemt * cameraRot._m[0];
      cameraY += movemt * cameraRot._m[1];
      cameraZ += movemt * cameraRot._m[2];
    }

    cameraRot.set(3, 0, cameraX);
    cameraRot.set(3, 1, cameraY);
    cameraRot.set(3, 2, cameraZ);

    matrix<4,4> newCameraTransform = invert(cameraRot);
    matrix<4,4> newPostion = matrix<4,4>::rotationY(rot);
    static matrix<4,4> scaler;
    scaler.set(0, 0, .5);
    scaler.set(1, 1, .5);
    scaler.set(2, 2, .5);
    scaler.set(3, 3, 1);

    newPostion = scaler * newPostion;
    newPostion.set(3, 2, -5);

    SDL_UpdateTexture(texture, nullptr, pixels.data(), 4*W);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);

    modInstance->_position = newPostion;
    cameraTransform = newCameraTransform;
    if (fps) {
      ++frame;
      if (frame == 100) {
        frame = 0;
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        float fps = (100.0F / duration.count()) * 1000000;
        std::cout << fps << " FPS" << std::endl;
        start = std::chrono::high_resolution_clock::now();
      }
    }
  }
  return 0;
}
