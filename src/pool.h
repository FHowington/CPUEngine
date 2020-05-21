// Created by Forbes Howington 5/19/20

#pragma once

#include <list>
#include <thread>
#include "loader.h"
#include "Window.h"


extern thread_local unsigned t_pixels[W * H];
extern thread_local int t_zbuff[W * H];

class Pool {
 public:
  static void job_wait();
  Pool(const unsigned numThreads);
  ~Pool();
  void enqueue_model(const ModelInstance* model);
  //void clear_thread_arrays() { for (auto& b : thread_clear_pixel_array) { b = true; } }
  static std::mutex pixel_buffer_lock;

 private:
  static void copy_to_main_buffer();
  static std::mutex job_queue_mutex_;
  static std::condition_variable job_condition;
  static std::list<const ModelInstance*> model_queue;
  static bool terminate;
  //static std::vector<char> thread_clear_pixel_array;
  std::vector<std::thread> thread_pool;
};
