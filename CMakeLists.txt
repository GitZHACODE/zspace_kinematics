cmake_minimum_required(VERSION 3.16)

project(abb_kinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ABB_KINEMATICS_ENABLE_TRAC_IK "Enable TRAC-IK inverse kinematics backend" ON)
option(ABB_KINEMATICS_USE_VENDORED_DEPS
  "Build KDL and kdl_parser from the cloned third_party sources when present" ON)

if(ABB_KINEMATICS_USE_VENDORED_DEPS AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/orocos_kinematics_dynamics/orocos_kdl/CMakeLists.txt" AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/kdl_parser/src/kdl_parser.cpp")
  list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  set(BUILD_TESTING OFF CACHE BOOL "Disable third-party tests" FORCE)
  set(ENABLE_TESTS OFF CACHE BOOL "Disable Orocos KDL tests" FORCE)
  set(ENABLE_EXAMPLES OFF CACHE BOOL "Disable Orocos KDL examples" FORCE)

  add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")
  set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")

  add_subdirectory(third_party/tinyxml2 EXCLUDE_FROM_ALL)
  add_subdirectory(third_party/console_bridge EXCLUDE_FROM_ALL)
  add_library(console_bridge::console_bridge ALIAS console_bridge)
  target_include_directories(console_bridge PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/third_party/console_bridge>")

  add_library(urdfdom_headers INTERFACE)
  target_include_directories(urdfdom_headers INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom_headers/include>"
    "$<INSTALL_INTERFACE:include/urdfdom_headers>")
  add_library(urdfdom_headers::urdfdom_headers ALIAS urdfdom_headers)
  install(TARGETS urdfdom_headers EXPORT urdfdom)
  install(DIRECTORY third_party/urdfdom_headers/include/
    DESTINATION include/urdfdom_headers)
  add_subdirectory(third_party/urdfdom EXCLUDE_FROM_ALL)

  # Orocos KDL treats only MSVC as Windows.  clang++ targets the MSVC ABI here
  # but does not set CMake's MSVC variable, which would otherwise produce an
  # un-exported DLL.  Use KDL's intended static Windows branch for this subtree.
  set(ABB_KINEMATICS_SAVED_MSVC "${MSVC}")
  set(MSVC TRUE)
  add_subdirectory(third_party/orocos_kinematics_dynamics/orocos_kdl EXCLUDE_FROM_ALL)
  set(MSVC "${ABB_KINEMATICS_SAVED_MSVC}")
  set(ABB_KINEMATICS_KDL_INCLUDE_DIR "${CMAKE_BINARY_DIR}/vendored_include")
  file(MAKE_DIRECTORY "${ABB_KINEMATICS_KDL_INCLUDE_DIR}/kdl")
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/orocos_kinematics_dynamics/orocos_kdl/src/"
    DESTINATION "${ABB_KINEMATICS_KDL_INCLUDE_DIR}/kdl"
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.inl")
  file(COPY
    "${CMAKE_BINARY_DIR}/third_party/orocos_kinematics_dynamics/orocos_kdl/src/config.h"
    DESTINATION "${ABB_KINEMATICS_KDL_INCLUDE_DIR}/kdl")

  add_library(abb_kinematics_kdl INTERFACE)
  target_link_libraries(abb_kinematics_kdl INTERFACE orocos-kdl Eigen3::Eigen)
  target_include_directories(abb_kinematics_kdl INTERFACE
    "${ABB_KINEMATICS_KDL_INCLUDE_DIR}")
  set(ABB_KINEMATICS_KDL_TARGET abb_kinematics_kdl)

  add_library(abb_kinematics_kdl_parser STATIC
    third_party/kdl_parser/src/kdl_parser.cpp)
  target_include_directories(abb_kinematics_kdl_parser
    PUBLIC
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/standalone_compat/include"
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/kdl_parser/include")
  target_link_libraries(abb_kinematics_kdl_parser
    PUBLIC abb_kinematics_kdl urdfdom_model)
  set(ABB_KINEMATICS_KDL_PARSER_TARGET abb_kinematics_kdl_parser)
  message(STATUS "Using vendored Orocos KDL and kdl_parser sources")
else()
  find_package(orocos_kdl CONFIG QUIET)
  if(NOT TARGET orocos-kdl AND NOT TARGET orocos_kdl AND NOT TARGET Orocos-KDL::orocos-kdl)
    find_package(orocos_kdl REQUIRED)
  endif()
  if(TARGET orocos-kdl)
    set(ABB_KINEMATICS_KDL_TARGET orocos-kdl)
  elseif(TARGET orocos_kdl)
    set(ABB_KINEMATICS_KDL_TARGET orocos_kdl)
  elseif(TARGET Orocos-KDL::orocos-kdl)
    set(ABB_KINEMATICS_KDL_TARGET Orocos-KDL::orocos-kdl)
  else()
    message(FATAL_ERROR "Orocos KDL was found but did not export a recognised CMake target.")
  endif()

  find_package(kdl_parser REQUIRED)
  if(TARGET kdl_parser::kdl_parser)
    set(ABB_KINEMATICS_KDL_PARSER_TARGET kdl_parser::kdl_parser)
  elseif(TARGET kdl_parser)
    set(ABB_KINEMATICS_KDL_PARSER_TARGET kdl_parser)
  else()
    message(FATAL_ERROR "kdl_parser was found but did not export a recognised CMake target.")
  endif()
endif()

set(ABB_KINEMATICS_TRAC_IK_TARGET "")
if(ABB_KINEMATICS_ENABLE_TRAC_IK)
  find_package(trac_ik_lib QUIET CONFIG)
  find_package(trac_ik QUIET CONFIG)
  if(TARGET trac_ik_lib::trac_ik_lib)
    set(ABB_KINEMATICS_TRAC_IK_TARGET trac_ik_lib::trac_ik_lib)
  elseif(TARGET trac_ik::trac_ik)
    set(ABB_KINEMATICS_TRAC_IK_TARGET trac_ik::trac_ik)
  elseif(TARGET trac_ik_lib)
    set(ABB_KINEMATICS_TRAC_IK_TARGET trac_ik_lib)
  elseif(TARGET trac_ik)
    set(ABB_KINEMATICS_TRAC_IK_TARGET trac_ik)
  endif()
endif()

add_library(abb_kinematics STATIC src/robot_kinematics.cpp)
add_library(abb_kinematics::abb_kinematics ALIAS abb_kinematics)
target_include_directories(abb_kinematics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(abb_kinematics
  PRIVATE ${ABB_KINEMATICS_KDL_PARSER_TARGET}
  PUBLIC ${ABB_KINEMATICS_KDL_TARGET})

if(ABB_KINEMATICS_TRAC_IK_TARGET)
  target_link_libraries(abb_kinematics PRIVATE ${ABB_KINEMATICS_TRAC_IK_TARGET})
  target_compile_definitions(abb_kinematics PUBLIC ABB_KINEMATICS_HAS_TRAC_IK)
  message(STATUS "TRAC-IK found: enabling TRAC-IK backend")
elseif(ABB_KINEMATICS_ENABLE_TRAC_IK)
  message(STATUS "TRAC-IK not found: building with KDL IK backend only")
else()
  message(STATUS "TRAC-IK backend disabled by ABB_KINEMATICS_ENABLE_TRAC_IK")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(abb_kinematics PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(ikfk_example examples/ikfk_example.cpp)
target_link_libraries(ikfk_example PRIVATE abb_kinematics)

enable_testing()
add_executable(test_kinematics tests/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE abb_kinematics)
target_compile_definitions(test_kinematics
  PRIVATE ABB_KINEMATICS_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/test_6dof.urdf")
add_test(NAME kinematics COMMAND test_kinematics)

if(WIN32 AND ABB_KINEMATICS_USE_VENDORED_DEPS)
  foreach(consumer ikfk_example test_kinematics)
    foreach(runtime_dependency urdfdom_model console_bridge tinyxml2)
      add_custom_command(TARGET ${consumer} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:${runtime_dependency}> $<TARGET_FILE_DIR:${consumer}>)
    endforeach()
  endforeach()
endif()
