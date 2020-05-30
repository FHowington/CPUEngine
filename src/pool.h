// Created by Forbes Howington 5/19/20
#pragma once

#include <list>
#include <map>
#include <thread>
#include "loader.h"
#include "Window.h"


extern thread_local unsigned t_pixels[Wt * H + H];
extern thread_local int t_zbuff[Wt * H + H];
extern thread_local unsigned pMinX;
extern thread_local unsigned pMaxX;
extern thread_local unsigned pMinY;
extern thread_local unsigned pMaxY;

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
  static std::mutex main_buffer_mutex_;
  static std::condition_variable job_condition;
  static std::condition_variable render_condition;
  static std::list<const ModelInstance*> model_queue;
  static std::map<const std::thread::id, const std::pair<const std::pair<const unsigned, const unsigned>, const std::pair<const unsigned, const unsigned>>> buffer_zones;
  static bool terminate;
  std::vector<std::thread> thread_pool;
};
