#include "pool.h"
#include "shader.h"
#include "rasterize.h"


extern matrix<4,4> cameraTransform;
extern vertex<float> light;
extern std::atomic<unsigned> remaining_models;
extern unsigned pixels[W*H];
extern int zbuff[W*H];


const matrix<4,4> viewClip = viewport((W) / 2.0, (H) / 2.0, 2*W, 2*H);

std::mutex Pool::job_queue_mutex_;
std::condition_variable Pool::job_condition;
std::list<const ModelInstance*> Pool::model_queue;
bool Pool::terminate = false;
std::mutex Pool::pixel_buffer_lock;

thread_local unsigned pMinX;
thread_local unsigned pMaxX;
thread_local unsigned pMinY;
thread_local unsigned pMaxY;

thread_local __attribute__((aligned(32))) unsigned t_pixels[W * H + H];
thread_local __attribute__((aligned(32))) int t_zbuff[W * H + H];

Pool::Pool(const unsigned numThreads) {
  for(int i = 0; i < numThreads; ++i) {
    thread_pool.push_back(std::thread(job_wait));
  }
}

Pool::~Pool() {
  terminate = true;
  job_condition.notify_all();

  for (std::thread &t : thread_pool) {
    t.join();
  }
  thread_pool.clear();
}

void Pool::job_wait() {
  for(auto& p: t_pixels) p = 0;
  for(auto& p: t_zbuff) p = std::numeric_limits<int>::min();

  while(true) {
    std::unique_lock<std::mutex> lock(job_queue_mutex_);

    job_condition.wait(lock, []{ return !Pool::model_queue.empty() || Pool::terminate; });
    if (terminate) {
      break;
    }

    const ModelInstance* job = model_queue.front();
    model_queue.pop_front();

    lock.unlock();

    pMinX = W;
    pMaxX = 0;
    pMinY = H;
    pMaxY = 0;

    switch (job->_shader) {
      case shaderType::FlatShader: {
        renderModel<FlatShader>(job, cameraTransform, viewClip, light);
        break;
      }

      case shaderType::GouraudShader: {
        renderModel<GouraudShader>(job, cameraTransform, viewClip, light);
        break;
      }

      case shaderType::InterpFlatShader: {
        renderModel<InterpFlatShader>(job, cameraTransform, viewClip, light);
        break;
      }

      case shaderType::InterpGouraudShader: {
        renderModel<InterpGouraudShader>(job, cameraTransform, viewClip, light);
        break;
      }
    }

    Pool::copy_to_main_buffer();
    --remaining_models;

    for(auto& p: t_pixels) p = 0;
    for(auto& p: t_zbuff) p = std::numeric_limits<int>::min();
  }
}

void Pool::copy_to_main_buffer() {
  std::unique_lock<std::mutex> lock(pixel_buffer_lock);
  // TODO: Vectorize this
  for (unsigned y = pMinY; y < pMaxY; ++y) {
    for (unsigned x = pMinX; x < pMaxX; ++x) {
      if (t_zbuff[y * W + x] > zbuff[y * W + x]) {
        pixels[y * W + x] = t_pixels[y * W + x];
        zbuff[y * W + x] = t_zbuff[y * W + x];
      }
    }
  }
}

void Pool::enqueue_model(const ModelInstance* model) {
  std::unique_lock<std::mutex> lock(job_queue_mutex_);
  model_queue.push_back(model);
  job_condition.notify_one();
}
