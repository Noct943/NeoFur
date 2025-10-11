#pragma once

#include "external/expected.hpp"
#include "my_error.h"

namespace neofur {

// 使用模板别名 (template alias) 来定义通用的 ResultErr 类型。
template <typename T>
using ResultErr = tl::expected<T, utils::Error>;

}  // namespace neofur
