#ifndef UTILS_H
#define UTILS_H

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <functional>
#include <utility>

namespace utils {
// 辅助类：ScopeGuard，用于在作用域退出时执行清理函数
// 这是一个通用的 RAII 模式，可以避免重复的错误清理代码
class ScopeGuard {
 public:
  // 构造函数接收一个在作用域退出时执行的 lambda 或可调用对象
  explicit ScopeGuard(std::function<void()> on_exit)
      : on_exit_(std::move(on_exit)), dismissed_(false) {}

  // 析构函数：如果未被解除，则执行清理函数
  ~ScopeGuard() {
    if (!dismissed_) {
      on_exit_();
    }
  }

  // 显式解除清理函数，防止其在作用域退出时执行
  void Dismiss() { dismissed_ = true; }

  // 禁止拷贝和移动，确保清理函数只执行一次
  ScopeGuard(const ScopeGuard&) = delete;
  ScopeGuard& operator=(const ScopeGuard&) = delete;
  ScopeGuard(ScopeGuard&&) = delete;
  ScopeGuard& operator=(ScopeGuard&&) = delete;

 private:
  std::function<void()> on_exit_;
  bool dismissed_;
};

/**
 * @class MappedFile
 * @brief 使用 RAII 风格封装内存映射文件 (mmap)。
 *
 * 它是可移动的，但不可拷贝。
 */
class MappedFile {
 public:
  explicit MappedFile(std::string_view path) {
    fd_ = open(std::string(path).c_str(), O_RDONLY);
    if (fd_ == -1) {
      return;
    }

    struct stat sb;
    if (fstat(fd_, &sb) == -1) {
      cleanup();
      return;
    }
    size_ = sb.st_size;

    if (size_ == 0) {
      return;
    }

    addr_ = mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd_, 0);
    if (addr_ == MAP_FAILED) {
      addr_ = nullptr;
      cleanup();
    }
  }

  ~MappedFile() { cleanup(); }

  // 禁用拷贝
  MappedFile(const MappedFile&) = delete;
  MappedFile& operator=(const MappedFile&) = delete;

  // 启用移动
  MappedFile(MappedFile&& other) noexcept
      : fd_(std::exchange(other.fd_, -1)),
        addr_(std::exchange(other.addr_, nullptr)),
        size_(std::exchange(other.size_, 0)) {}

  MappedFile& operator=(MappedFile&& other) noexcept {
    if (this != &other) {
      cleanup();
      fd_ = std::exchange(other.fd_, -1);
      addr_ = std::exchange(other.addr_, nullptr);
      size_ = std::exchange(other.size_, 0);
    }
    return *this;
  }

  const char* data() const { return static_cast<const char*>(addr_); }
  size_t size() const { return size_; }
  bool is_valid() const { return addr_ != nullptr; }

 private:
  inline void cleanup() {
    if (addr_) {
      munmap(addr_, size_);
      addr_ = nullptr;
    }
    if (fd_ != -1) {
      close(fd_);
      fd_ = -1;
    }
    size_ = 0;
  }

  int fd_ = -1;
  void* addr_ = nullptr;
  size_t size_ = 0;
};

}  // namespace utils

#endif  // UTILS_H
