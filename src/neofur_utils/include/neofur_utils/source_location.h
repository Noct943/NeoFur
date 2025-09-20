/**
 * @file source_location.h
 * @brief 提供一个C++17兼容的源代码位置信息捕获工具。
 *
 * 该工具旨在模仿 C++20 的 std::source_location，为旧版标准的项目
 * 提供一种在编译时捕获文件名、函数名和行号的机制。
 *
 * @author Gemini
 * @date 2025-08-02
 */
#ifndef SOURCE_LOCATION_H_H
#define SOURCE_LOCATION_H_H

#include <cstdint>

namespace utils {

/**
 * @struct source_location
 * @brief 存储源代码位置信息的结构体。
 *
 * 设计为普通旧数据(POD)类型，以避免在捕获信息时产生额外开销。
 */
struct source_location {
  const char* file_name;
  const char* function_name;
  std::uint_least32_t line;
};

}  // namespace utils

/**
 * @def CURRENT_LOCATION()
 * @brief 在调用点捕获并创建一个 util::source_location 对象的便捷宏。
 *
 * 它会根据编译器选择最佳的宏来获取最详细的函数签名。
 */
#if defined(__GNUC__) || defined(__clang__)
   // 对于GCC和Clang，__PRETTY_FUNCTION__ 提供了最详尽的函数签名信息，
// 包括类名、命名空间和参数类型，非常利于调试。
#define CURRENT_LOCATION()                                                    \
  utils::source_location {                                                    \
    __FILE__, __PRETTY_FUNCTION__, static_cast<std::uint_least32_t>(__LINE__) \
  }
#else
   // 对于其他编译器（如MSVC），回退到C++11标准的 __func__，
// 它只提供函数名，但保证了更好的可移植性。
#define CURRENT_LOCATION()                                         \
  util::source_location {                                          \
    __FILE__, __func__, static_cast<std::uint_least32_t>(__LINE__) \
  }
#endif

#endif  // SOURCE_LOCATION_H_H
