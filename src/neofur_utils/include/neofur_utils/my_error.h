/**
 * @file my_error.h
 * @brief 定义了一个通用的、富上下文的错误处理框架。
 *
 * 包含错误的分类(ErrorCode)、承载所有错误信息的结构体(Error)，
 * 以及用于便捷创建和传播错误对象的辅助宏。该框架与具体业务逻辑无关，
 * 可在任何项目模块中复用。
 *
 * @author Gemini
 * @date 2025-08-02
 */
#ifndef MY_ERROR_H_H
#define MY_ERROR_H_H

#include <cerrno>
#include <cstring>
#include <string>

#include "external/expected.hpp"
#include "source_location.h"

namespace neofur {
namespace utils {

/**
 * @enum ErrorCode
 * @brief 对通用错误进行高度概括的分类，用于程序化的错误处理。
 */
enum class ErrorCode {
  /** @brief 表示无效的输入参数、ID或状态。通常是调用者的错误。*/
  InvalidArguments,

  /** @brief 表示请求的资源或对象不存在。*/
  NotFound,

  /** @brief 表示与外部资源或I/O相关的操作失败（如文件、设备、网络）。*/
  IOError,

  /** @brief 表示一个运行时操作流程失败、超时或进入非法状态。*/
  RuntimeError,

  ResourceExhausted,

  // @brief 渲染错误
  RenderError,

  /** @brief 表示未知的、未分类的错误。*/
  Unknown,
};

/**
 * @struct Error
 * @brief 一个富错误信息对象，封装了错误处理所需的所有上下文。
 */
struct Error {
  ErrorCode code;
  int saved_errno;
  source_location location;
  std::string context_message;

  Error(ErrorCode cod, int err_no, const source_location &loc,
        std::string ctx_msg = "")
      : code(cod),
        saved_errno(err_no),
        location(loc),
        context_message(std::move(ctx_msg)) {}

  /**
   * @brief 生成一份结构化的、人类可读的详细错误报告。
   */
  std::string message() const {
    std::string msg;

    // 步骤 1: 确定核心错误描述。
    if (!context_message.empty()) {
      msg = context_message;
    } else {
      switch (code) {
        case ErrorCode::InvalidArguments:
          msg = "Invalid arguments provided";
          break;
        case ErrorCode::NotFound:
          msg = "The requested item was not found";
          break;
        case ErrorCode::IOError:
          msg = "An I/O or resource-related operation failed";
          break;
        case ErrorCode::RuntimeError:
          msg = "A runtime operation failed or timed out";
          break;
        case ErrorCode::RenderError:
          msg = "渲染错误";
          break;
        case ErrorCode::Unknown:
        default:
          msg = "An unknown error occurred";
          break;
      }
    }

    // 步骤 2: 组装结构化的最终报告。
    std::string final_report = "Error: " + msg + " (ErrorCode: " +
                               std::to_string(static_cast<int>(code)) + ")";

    if (saved_errno != 0) {
      final_report +=
          "\n  - System Error: " + std::string(strerror(saved_errno)) +
          " (errno: " + std::to_string(saved_errno) + ")";
    }

    final_report +=
        "\n  - In function:  " + std::string(location.function_name) +
        "\n  - At location:  " + std::string(location.file_name) + ":" +
        std::to_string(location.line);

    return final_report;
  }
};

// 统一的结果/错误返回类型，使用tl::expected封装
template <typename T>
using ResultErr = tl::expected<T, utils::Error>;
}  // namespace utils
}  // namespace neofur
/**
 * @def MAKE_ERROR(code)
 * @brief 创建一个标准错误对象的便捷宏。
 * @usage return
 * tl::make_unexpected(MAKE_ERROR(utils::ErrorCode::RuntimeError));
 */
#define MAKE_ERROR(code) utils::Error(code, errno, CURRENT_LOCATION())

/**
 * @def MAKE_ERROR_CTX(code, msg)
 * @brief 创建一个带自定义上下文消息的错误对象的便捷宏。
 * @usage return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::NotFound,
 * "Item 'XYZ' not found"));
 */
#define MAKE_ERROR_CTX(code, msg) \
  utils::Error(code, errno, CURRENT_LOCATION(), msg)

/**
 * @def PROPAGATE_ERROR_CTX(expr)
 * @brief 检查一个返回 tl::expected
 * 的表达式，如果它包含错误，则立即在当前函数中返回该错误。
 *
 * 这个宏简化了错误传播的样板代码，使业务逻辑更清晰。
 * 它假设当前函数的返回类型也是一个能承载 utils::Error 的 tl::expected。
 *
 * @usage
 * ResultErr<void> MyFunction() {
 * PROPAGATE_ERROR_CTX(SomeOtherFunctionThatReturnsExpected());
 * // ... 如果没有错误，继续执行
 * return {};
 * }
 */
#define PROPAGATE_ERROR_CTX(expr)              \
  do {                                         \
    if (auto res = (expr); !res) {             \
      return tl::make_unexpected(res.error()); \
    }                                          \
  } while (0)

#endif /* MY_ERROR_H_H */
