// Created by Forbes Howington 5/19/20
#pragma once

#include "Window.h"
#include "loader.h"
#include <list>
#include <map>
#include <thread>


extern thread_local std::array<unsigned, Wt * H + H> t_pixels;
extern thread_local std::array<int, Wt * H + H> t_zbuff;
extern thread_local unsigned pMinX;
extern thread_local unsigned pMaxX;
extern thread_local unsigned pMinY;
extern thread_local unsigned pMaxY;

class Pool { // NOLINT
 public:
  static void job_wait();
  Pool(unsigned numThreads);
  ~Pool();
  static void enqueue_model(const ModelInstance* model);

 private:
  static void copy_to_main_buffer();
  static std::mutex job_queue_mutex_;
  static std::mutex main_buffer_mutex_;
  static std::condition_variable job_condition;
  static std::condition_variable render_condition;
  static std::map<const std::thread::id, const std::pair<const std::pair<const unsigned, const unsigned>, const std::pair<const unsigned, const unsigned>>> buffer_zones;
  static bool terminate;
  std::vector<std::thread> thread_pool;
  static std::list<const ModelInstance*> model_queue;
};
