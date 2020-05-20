#include "pool.h"
#include "shader.h"

extern matrix<4,4> cameraTransform;
extern vertex<float> light;
extern std::atomic<unsigned> remaining_models;

const matrix<4,4> viewClip = viewport((W) / 2.0, (H) / 2.0, 2*W, 2*H);

std::mutex Pool::job_queue_mutex_;
std::condition_variable Pool::job_condition;
std::list<const ModelInstance*> Pool::model_queue;

Pool::Pool(const unsigned numThreads) {
  for(int i = 0; i < numThreads; ++i) {
    thread_pool.push_back(std::thread(job_wait));
  }
}

Pool::~Pool() {
    job_condition.notify_all();

    for (std::thread &t : thread_pool)
    {
      t.join();
    }

    thread_pool.clear();
}

void Pool::job_wait() {
  while(true)
  {
    std::unique_lock<std::mutex> lock(job_queue_mutex_);

    job_condition.wait(lock, []{return !Pool::model_queue.empty();});
    const ModelInstance* job = model_queue.front();
    model_queue.pop_front();

    lock.unlock();

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
    --remaining_models;
  }
}

void Pool::enqueue_model(const ModelInstance* model) {
  std::unique_lock<std::mutex> lock(job_queue_mutex_);
  model_queue.push_back(model);
  job_condition.notify_one();
}
