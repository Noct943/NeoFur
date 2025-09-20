#ifndef LOG_MANAGER_H
#define LOG_MANAGER_H

#include <functional>
#include <string>
#include <string_view>

namespace utils {

// 1. 定义日志级别
enum class LogLevel { Info, Warning, Error };

// 2. 更新 LogFunc 的签名，使其接受日志级别
using LogFunc = std::function<void(LogLevel level, std::string_view message)>;

/**
 * @class LogManager
 * @brief 提供一个全局的、可重定向的日志记录器单例。
 */
class LogManager {
 public:
  // 3. 简化为单个 SetLogger
  static void SetLogger(LogFunc logger) {
    Instance().logger_ = std::move(logger);
  }

  // 4. 调用时传入对应的日志级别
  static void Info(std::string_view msg) {
    if (Instance().logger_) {
      Instance().logger_(LogLevel::Info, msg);
    }
  }

  static void Warning(std::string_view msg) {
    if (Instance().logger_) {
      Instance().logger_(LogLevel::Warning, msg);
    }
  }

  static void Error(std::string_view msg) {
    if (Instance().logger_) {
      Instance().logger_(LogLevel::Error, msg);
    }
  }

 private:
  static LogManager& Instance() {
    static LogManager instance;
    return instance;
  }

  LogManager() = default;
  ~LogManager() = default;
  LogManager(const LogManager&) = delete;
  LogManager& operator=(const LogManager&) = delete;

  // 5. 只有一个 logger 成员
  LogFunc logger_;
};

}  // namespace utils
#endif  // LOG_MANAGER_H
