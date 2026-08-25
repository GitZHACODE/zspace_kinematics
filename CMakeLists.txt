cmake_minimum_required(VERSION 3.16)

project(zKinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ZKINEMATICS_ENABLE_TRAC_IK "Enable TRAC-IK inverse kinematics backend" ON)
option(ZKINEMATICS_USE_VENDORED_DEPS
  "Build KDL and kdl_parser from the cloned third_party sources when present" ON)
option(ZKINEMATICS_BUILD_SHARED
  "Build zKinematics as a shared library (DLL on Windows)" ON)
set(ZKINEMATICS_VCPKG_INSTALLED_DIR "C:/dev/vcpkg/installed/x64-windows"
  CACHE PATH "vcpkg x64-windows installed directory used for optional TRAC-IK dependencies")

if(EXISTS "${ZKINEMATICS_VCPKG_INSTALLED_DIR}")
  list(PREPEND CMAKE_PREFIX_PATH "${ZKINEMATICS_VCPKG_INSTALLED_DIR}")
endif()

if(ZKINEMATICS_USE_VENDORED_DEPS AND
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

  # Keep URDF parsing dependencies static so zKinematics.dll is the only
  # project-specific DLL that a consuming application needs to deploy.
  set(BUILD_SHARED_LIBS OFF)
  add_subdirectory(third_party/tinyxml2 EXCLUDE_FROM_ALL)
  add_subdirectory(third_party/console_bridge EXCLUDE_FROM_ALL)
  add_library(console_bridge::console_bridge ALIAS console_bridge)
  target_include_directories(console_bridge PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/third_party/console_bridge>")
  target_compile_definitions(console_bridge PUBLIC CONSOLE_BRIDGE_STATIC_DEFINE)

  add_library(urdfdom_headers INTERFACE)
  target_include_directories(urdfdom_headers INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom_headers/include>"
    "$<INSTALL_INTERFACE:include/urdfdom_headers>")
  add_library(urdfdom_headers::urdfdom_headers ALIAS urdfdom_headers)
  install(TARGETS urdfdom_headers EXPORT urdfdom)
  install(DIRECTORY third_party/urdfdom_headers/include/
    DESTINATION include/urdfdom_headers)
  add_library(urdfdom_model STATIC
    third_party/urdfdom/urdf_parser/src/pose.cpp
    third_party/urdfdom/urdf_parser/src/model.cpp
    third_party/urdfdom/urdf_parser/src/link.cpp
    third_party/urdfdom/urdf_parser/src/joint.cpp)
  target_include_directories(urdfdom_model PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom/urdf_parser/include")
  target_link_libraries(urdfdom_model PUBLIC
    urdfdom_headers console_bridge tinyxml2)
  target_compile_definitions(urdfdom_model PUBLIC URDFDOM_EXPORTS)

  # Orocos KDL treats only MSVC as Windows.  clang++ targets the MSVC ABI here
  # but does not set CMake's MSVC variable, which would otherwise produce an
  # un-exported DLL.  Use KDL's intended static Windows branch for this subtree.
  set(ZKINEMATICS_SAVED_MSVC "${MSVC}")
  set(MSVC TRUE)
  add_subdirectory(third_party/orocos_kinematics_dynamics/orocos_kdl EXCLUDE_FROM_ALL)
  set(MSVC "${ZKINEMATICS_SAVED_MSVC}")
  set(ZKINEMATICS_KDL_INCLUDE_DIR "${CMAKE_BINARY_DIR}/vendored_include")
  file(MAKE_DIRECTORY "${ZKINEMATICS_KDL_INCLUDE_DIR}/kdl")
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/orocos_kinematics_dynamics/orocos_kdl/src/"
    DESTINATION "${ZKINEMATICS_KDL_INCLUDE_DIR}/kdl"
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.inl")
  file(COPY
    "${CMAKE_BINARY_DIR}/third_party/orocos_kinematics_dynamics/orocos_kdl/src/config.h"
    DESTINATION "${ZKINEMATICS_KDL_INCLUDE_DIR}/kdl")

  add_library(zKinematics_kdl INTERFACE)
  target_link_libraries(zKinematics_kdl INTERFACE orocos-kdl Eigen3::Eigen)
  target_include_directories(zKinematics_kdl INTERFACE
    "${ZKINEMATICS_KDL_INCLUDE_DIR}")
  set(ZKINEMATICS_KDL_TARGET zKinematics_kdl)

  add_library(zKinematics_kdl_parser STATIC
    third_party/kdl_parser/src/kdl_parser.cpp)
  target_include_directories(zKinematics_kdl_parser
    PUBLIC
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/standalone_compat/include"
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/kdl_parser/include")
  target_link_libraries(zKinematics_kdl_parser
    PUBLIC zKinematics_kdl urdfdom_model)
  set(ZKINEMATICS_KDL_PARSER_TARGET zKinematics_kdl_parser)
  message(STATUS "Using vendored Orocos KDL and kdl_parser sources")
else()
  find_package(orocos_kdl CONFIG QUIET)
  if(NOT TARGET orocos-kdl AND NOT TARGET orocos_kdl AND NOT TARGET Orocos-KDL::orocos-kdl)
    find_package(orocos_kdl REQUIRED)
  endif()
  if(TARGET orocos-kdl)
    set(ZKINEMATICS_KDL_TARGET orocos-kdl)
  elseif(TARGET orocos_kdl)
    set(ZKINEMATICS_KDL_TARGET orocos_kdl)
  elseif(TARGET Orocos-KDL::orocos-kdl)
    set(ZKINEMATICS_KDL_TARGET Orocos-KDL::orocos-kdl)
  else()
    message(FATAL_ERROR "Orocos KDL was found but did not export a recognised CMake target.")
  endif()

  find_package(kdl_parser REQUIRED)
  if(TARGET kdl_parser::kdl_parser)
    set(ZKINEMATICS_KDL_PARSER_TARGET kdl_parser::kdl_parser)
  elseif(TARGET kdl_parser)
    set(ZKINEMATICS_KDL_PARSER_TARGET kdl_parser)
  else()
    message(FATAL_ERROR "kdl_parser was found but did not export a recognised CMake target.")
  endif()
endif()

set(ZKINEMATICS_TRAC_IK_TARGET "")
if(ZKINEMATICS_ENABLE_TRAC_IK)
  find_package(trac_ik_lib QUIET CONFIG)
  find_package(trac_ik QUIET CONFIG)
  if(TARGET trac_ik_lib::trac_ik_lib)
    set(ZKINEMATICS_TRAC_IK_TARGET trac_ik_lib::trac_ik_lib)
  elseif(TARGET trac_ik::trac_ik)
    set(ZKINEMATICS_TRAC_IK_TARGET trac_ik::trac_ik)
  elseif(TARGET trac_ik_lib)
    set(ZKINEMATICS_TRAC_IK_TARGET trac_ik_lib)
  elseif(TARGET trac_ik)
    set(ZKINEMATICS_TRAC_IK_TARGET trac_ik)
  endif()

  if(NOT ZKINEMATICS_TRAC_IK_TARGET AND
     EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/trac_ik/trac_ik_lib/src/trac_ik.cpp")
    find_package(NLopt QUIET CONFIG)
    find_package(Boost QUIET COMPONENTS date_time)
    if(TARGET NLopt::nlopt AND TARGET Boost::date_time)
      add_library(zKinematics_trac_ik STATIC
        third_party/trac_ik/trac_ik_lib/src/kdl_tl.cpp
        third_party/trac_ik/trac_ik_lib/src/nlopt_ik.cpp
        third_party/trac_ik/trac_ik_lib/src/trac_ik.cpp)
      target_include_directories(zKinematics_trac_ik
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/third_party/trac_ik/trac_ik_lib/include"
        PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/standalone_compat/include")
      target_link_libraries(zKinematics_trac_ik
        PUBLIC zKinematics_kdl ${ZKINEMATICS_KDL_PARSER_TARGET}
        NLopt::nlopt Boost::date_time)
      target_compile_definitions(zKinematics_trac_ik PUBLIC uint=unsigned)
      if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(zKinematics_trac_ik PRIVATE
          "-include${CMAKE_CURRENT_SOURCE_DIR}/third_party/standalone_compat/include/trac_ik/standalone_compat.hpp")
      elseif(MSVC)
        target_compile_options(zKinematics_trac_ik PRIVATE
          "/FI${CMAKE_CURRENT_SOURCE_DIR}/third_party/standalone_compat/include/trac_ik/standalone_compat.hpp")
      endif()
      set(ZKINEMATICS_TRAC_IK_TARGET zKinematics_trac_ik)
      set(ZKINEMATICS_TRAC_IK_NLOPT_TARGET NLopt::nlopt)
      message(STATUS "Using vendored TRAC-IK sources with vcpkg NLopt and Boost")
    else()
      message(STATUS "Vendored TRAC-IK requires NLopt and Boost Date-Time; backend unavailable")
    endif()
  endif()
endif()

if(ZKINEMATICS_BUILD_SHARED)
  set(ZKINEMATICS_LIBRARY_TYPE SHARED)
else()
  set(ZKINEMATICS_LIBRARY_TYPE STATIC)
endif()

add_library(zKinematics ${ZKINEMATICS_LIBRARY_TYPE} src/robot_kinematics.cpp)
add_library(zKinematics::zKinematics ALIAS zKinematics)
set_target_properties(zKinematics PROPERTIES
  OUTPUT_NAME zKinematics
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>")
target_include_directories(zKinematics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(zKinematics
  PRIVATE ${ZKINEMATICS_KDL_PARSER_TARGET}
  PUBLIC ${ZKINEMATICS_KDL_TARGET})

if(ZKINEMATICS_BUILD_SHARED)
  target_compile_definitions(zKinematics
    PRIVATE ZKINEMATICS_BUILDING_DLL
    PUBLIC ZKINEMATICS_SHARED)
  message(STATUS "zKinematics: building shared library")
else()
  target_compile_definitions(zKinematics PUBLIC ZKINEMATICS_STATIC)
  message(STATUS "zKinematics: building static library")
endif()

if(ZKINEMATICS_TRAC_IK_TARGET)
  target_link_libraries(zKinematics PRIVATE ${ZKINEMATICS_TRAC_IK_TARGET})
  target_compile_definitions(zKinematics PUBLIC ZKINEMATICS_HAS_TRAC_IK)
  message(STATUS "TRAC-IK found: enabling TRAC-IK backend")
elseif(ZKINEMATICS_ENABLE_TRAC_IK)
  message(STATUS "TRAC-IK not found: building with KDL IK backend only")
else()
  message(STATUS "TRAC-IK backend disabled by ZKINEMATICS_ENABLE_TRAC_IK")
endif()

if(WIN32 AND ZKINEMATICS_TRAC_IK_NLOPT_TARGET)
  add_custom_command(TARGET zKinematics POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:${ZKINEMATICS_TRAC_IK_NLOPT_TARGET}>
      $<TARGET_FILE_DIR:zKinematics>)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(zKinematics PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(ikfk_example examples/ikfk_example.cpp)
target_link_libraries(ikfk_example PRIVATE zKinematics)

enable_testing()
add_executable(test_kinematics tests/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE zKinematics)
target_compile_definitions(test_kinematics
  PRIVATE ZKINEMATICS_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/test_6dof.urdf")
add_test(NAME kinematics COMMAND test_kinematics)

if(WIN32 AND ZKINEMATICS_USE_VENDORED_DEPS)
  set(ZKINEMATICS_RUNTIME_DEPENDENCIES)
  if(ZKINEMATICS_BUILD_SHARED)
    list(APPEND ZKINEMATICS_RUNTIME_DEPENDENCIES zKinematics)
  endif()
  foreach(consumer ikfk_example test_kinematics)
    foreach(runtime_dependency IN LISTS ZKINEMATICS_RUNTIME_DEPENDENCIES)
      add_custom_command(TARGET ${consumer} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:${runtime_dependency}> $<TARGET_FILE_DIR:${consumer}>)
    endforeach()
  endforeach()
endif()
