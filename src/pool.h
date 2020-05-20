// Created by Forbes Howington 5/19/20

#pragma once

#include <list>
#include <thread>
#include "loader.h"
#include "rasterize.h"

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

class Pool {
 public:
  static void job_wait();
  Pool(const unsigned numThreads);
  ~Pool();
  void enqueue_model(const ModelInstance* model);

 private:
  static std::mutex job_queue_mutex_;
  static std::condition_variable job_condition;
  static std::list<const ModelInstance*> model_queue;
  std::vector<std::thread> thread_pool;
};
