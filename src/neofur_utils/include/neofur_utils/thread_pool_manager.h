#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <thread>

#include "external/BS_thread_pool.hpp"

namespace utils {

/**
 * @class ThreadPoolManager
 * @brief 提供一个全局单例的 BS::thread_pool 实例。
 *
 */
class ThreadPoolManager {
 public:
  static BS::thread_pool<>& Instance() {
    static BS::thread_pool instance;
    return instance;
  }

 private:
  ThreadPoolManager()=delete;
  ThreadPoolManager(const ThreadPoolManager&)=delete; 
  ThreadPoolManager& operator=(const ThreadPoolManager&)=delete;
};
}  // namespace utils

#endif  // THREAD_POOL_H
