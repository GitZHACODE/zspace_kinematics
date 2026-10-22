#pragma once

#include <cmath>
#include <limits>

namespace boost::math::tools {
template <typename T>
constexpr T epsilon() noexcept {
  return std::numeric_limits<T>::epsilon();
}
}  // namespace boost::math::tools

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
