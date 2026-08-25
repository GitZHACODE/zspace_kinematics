#pragma once

// kdl_parser only uses these macros for diagnostics.  This compatibility header
// keeps its parser usable as a standalone C++ library without ROS runtime code.
#include <cstdio>

#define RCUTILS_LOG_WARN_NAMED(name, format, ...)                              \
  std::fprintf(stderr, "[%s] warning: " format "\n", name, __VA_ARGS__)
#define RCUTILS_LOG_DEBUG_NAMED(name, format, ...) ((void)0)
#define RCUTILS_LOG_ERROR_NAMED(name, format, ...)                             \
  std::fprintf(stderr, "[%s] error: " format "\n", name, __VA_ARGS__)
