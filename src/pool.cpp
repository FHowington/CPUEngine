#include "pool.h"
#include "shader.h"
#include "rasterize.h"


extern matrix<4,4> cameraTransform;
extern vertex<float> light;
extern std::atomic<unsigned> remaining_models;
extern unsigned pixels[W*H];
extern int zbuff[W*H];

std::mutex Pool::job_queue_mutex_;
std::condition_variable Pool::job_condition;
std::condition_variable Pool::render_condition;
std::list<const ModelInstance*> Pool::model_queue;
bool Pool::terminate = false;
std::mutex Pool::main_buffer_mutex_;
std::map<const std::thread::id, const std::pair<const std::pair<const unsigned, const unsigned>, const std::pair<const unsigned, const unsigned>>> Pool::buffer_zones;

thread_local unsigned pMinX;
thread_local unsigned pMaxX;
thread_local unsigned pMinY;
thread_local unsigned pMaxY;

thread_local __attribute__((aligned(32))) unsigned t_pixels[Wt * H + H];
thread_local __attribute__((aligned(32))) int t_zbuff[Wt * H + H];

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
        renderModel<FlatShader>(job, cameraTransform, light);
        break;
      }

      case shaderType::GouraudShader: {
        renderModel<GouraudShader>(job, cameraTransform, light);
        break;
      }

      case shaderType::InterpFlatShader: {
        renderModel<InterpFlatShader>(job, cameraTransform, light);
        break;
      }

      case shaderType::InterpGouraudShader: {
        renderModel<InterpGouraudShader>(job, cameraTransform, light);
        break;
      }

      case shaderType::PlaneShader: {
        renderModel<PlaneShader>(job, cameraTransform, light);
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
  std::unique_lock<std::mutex> zone_lock(main_buffer_mutex_);
  static thread_local const std::thread::id thisTID = std::this_thread::get_id();

  render_condition.wait(zone_lock, [&]{
                                     for (const std::pair<const std::thread::id, const std::pair<const std::pair<const unsigned, const unsigned>, const std::pair<const unsigned, const unsigned>>>& zone : buffer_zones) {
                                       // minX, minY
                                       const unsigned minX = zone.second.first.first;
                                       const unsigned minY = zone.second.first.second;
                                       const unsigned maxX = zone.second.second.first;
                                       const unsigned maxY = zone.second.second.second;

                                       if (pMinX <= maxX && pMaxX >= minX && pMinY <= maxY && pMaxY >= minY) {
                                         return false;
                                       }
                                     }
                                     buffer_zones.insert({thisTID, std::make_pair(std::make_pair(pMinX, pMinY), std::make_pair(pMaxX, pMaxY))});
                                     return true;
                                   });
  zone_lock.unlock();

  unsigned offset = pMinY * W;
  unsigned offsetH = (H - pMinY) * W;

#ifdef __AVX2__
  unsigned offset_t = pMinY * Wt;
  const unsigned loops = (pMaxX - pMinX) / 8;

  for (unsigned y = pMinY; y < pMaxY; ++y) {
    unsigned xVal = pMinX;

    for (unsigned loop = 0; loop < loops; ++loop) {
      const __m256i zbuffV = _mm256_load_si256((__m256i*)(zbuff + offset + xVal));
      const __m256i t_zbuffV = _mm256_load_si256((__m256i*)(t_zbuff + offset_t + xVal));
      const __m256i needsUpdate = _mm256_cmpgt_epi32(t_zbuffV, zbuffV);
      if (!_mm256_testz_si256(needsUpdate, needsUpdate)) {
        // Any pixels with X > W need to have zbuff set to 0
        const __m256i colV = _mm256_load_si256((__m256i*)(pixels + offsetH + xVal));
        const __m256i t_colV = _mm256_load_si256((__m256i*)(t_pixels + offset_t + xVal));
        const __m256i colUpdate = _mm256_blendv_epi8(colV, t_colV, needsUpdate);
        const __m256i zUpdate = _mm256_blendv_epi8(zbuffV, t_zbuffV, needsUpdate);
        _mm256_storeu_si256((__m256i*)(zbuff + offset + xVal), zUpdate);
        _mm256_storeu_si256((__m256i*)(pixels + offsetH + xVal), colUpdate);
      }
      xVal += 8;
    }

    for (unsigned x = xVal; x < pMaxX; ++x) {
      if (t_zbuff[offset_t + x] > zbuff[offset + x]) {
        pixels[offsetH + x] = t_pixels[offset_t + x];
        zbuff[offset + x] = t_zbuff[offset_t + x];
      }
    }
    offset += W;
    offset_t += Wt;
    offsetH -= W;
  }

#else
  for (unsigned y = pMinY; y < pMaxY; ++y) {
    for (unsigned x = pMinX; x < pMaxX; ++x) {
      if (t_zbuff[offset + x] > zbuff[offset + x]) {
        pixels[offsetH + x] = t_pixels[offset + x];
        zbuff[offset + x] = t_zbuff[offset + x];
      }
    }
    offset += W;
    offsetH -= W;
  }
#endif

  zone_lock.lock();
  buffer_zones.erase(buffer_zones.find(thisTID));
  zone_lock.unlock();
  render_condition.notify_all();
}

void Pool::enqueue_model(const ModelInstance* model) {
  std::unique_lock<std::mutex> lock(job_queue_mutex_);
  model_queue.push_back(model);
  job_condition.notify_one();
}
