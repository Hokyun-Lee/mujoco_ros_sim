# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)

# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

# This line has to appear before 'PROJECT' in order to be able to disable incremental linking
set(MSVC_INCREMENTAL_DEFAULT ON)

set(MUJOCO_DEP_VERSION_lodepng
    b4ed2cd7ecf61d29076169b49199371456d4f90b
    CACHE STRING "Version of `lodepng` to be fetched."
)

project(
  mujoco_simulate
  VERSION 3.2.7
  DESCRIPTION "MuJoCo simulate binaries"
  HOMEPAGE_URL "https://mujoco.org"
)

enable_language(C)
enable_language(CXX)

option(SIMULATE_BUILD_EXECUTABLE "Build the simulate executable binary." ON)
option(SIMULATE_GLFW_DYNAMIC_SYMBOLS "Whether to resolve GLFW symbols dynamically." OFF)

# Check if we are building as standalone project.
set(SIMULATE_STANDALONE ON)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(FetchContent)
include(FindOrFetch)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)

# findorfetch(
#   USE_SYSTEM_PACKAGE
#   MUJOCO_SIMULATE_USE_SYSTEM_GLFW
#   PACKAGE_NAME
#   glfw3
#   LIBRARY_NAME
#   glfw3
#   GIT_REPO
#   https://github.com/glfw/glfw.git
#   GIT_TAG
#   ${MUJOCO_DEP_VERSION_glfw3}
#   TARGETS
#   glfw
#   EXCLUDE_FROM_ALL
# )

# if(MUJOCO_EXTRAS_STATIC_GLFW)
#   set(BUILD_SHARED_LIBS
#       ${BUILD_SHARED_LIBS_OLD}
#       CACHE BOOL "Build SHARED libraries" FORCE
#   )
#   unset(BUILD_SHARED_LIBS_OLD)
# endif()

# if(NOT SIMULATE_STANDALONE)
#   target_compile_options(glfw PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
#   target_link_options(glfw PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
# endif()

# include(SimulateOptions)
# include(SimulateDependencies)

set(MUJOCO_SIMULATE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
set(MUJOCO_SIMULATE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

# Fetch lodepng dependency.
if(NOT TARGET lodepng)
  FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )

  FetchContent_GetProperties(lodepng)
  if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    # target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    # target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
  endif()
endif()

include_directories(
    include
    src/simulate)

file(GLOB LIB_MUJOCO "lib/libmujoco.so")
# Simulate library
add_library(platform_ui_adapter OBJECT)
target_sources(
  platform_ui_adapter
  PUBLIC  src/simulate/glfw_adapter.h src/simulate/glfw_dispatch.h src/simulate/platform_ui_adapter.h
  PRIVATE src/simulate/glfw_adapter.cc src/simulate/glfw_dispatch.cc src/simulate/platform_ui_adapter.cc
)
target_compile_options(platform_ui_adapter PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})

target_include_directories(
  platform_ui_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                             $<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(platform_ui_adapter PUBLIC ${LIB_MUJOCO})
if(SIMULATE_GLFW_DYNAMIC_SYMBOLS)
  target_compile_definitions(platform_ui_adapter PUBLIC mjGLFW_DYNAMIC_SYMBOLS)
endif()
add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

add_library(libsimulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
set_target_properties(libsimulate PROPERTIES OUTPUT_NAME simulate)
add_library(mujoco::libsimulate ALIAS libsimulate)

target_sources(
  libsimulate
  PUBLIC src/simulate/simulate.h
  PRIVATE src/simulate/simulate.cc src/simulate/array_safety.h
)
target_include_directories(libsimulate PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_link_libraries(libsimulate PUBLIC lodepng mujoco::platform_ui_adapter ${LIB_MUJOCO})
target_link_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})

# Build simulate executable
if(SIMULATE_BUILD_EXECUTABLE)

  add_executable(simulate src/simulate/main.cc src/simulate/custom.cc src/simulate/array_safety.h ${SIMULATE_RESOURCE_FILES})
  target_compile_options(simulate PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})

  target_link_libraries(
    simulate
    libsimulate
    ${LIB_MUJOCO}
    glfw
    Threads::Threads
    lodepng
  )

  target_link_options(simulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})

  if(_INSTALL_SIMULATE)

    include(TargetAddRpath)

    # Add support to RPATH for the samples.
    target_add_rpath(
      TARGETS
      simulate
      INSTALL_DIRECTORY
      "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
      LIB_DIRS
      "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
      DEPENDS
      MUJOCO_ENABLE_RPATH
    )

    install(
      TARGETS simulate
      EXPORT ${PROJECT_NAME}
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT simulate
      LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
      ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
      BUNDLE DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT simulate
      PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT simulate
    )

    if(NOT MUJOCO_SIMULATE_USE_SYSTEM_GLFW)
      # We downloaded GLFW. Depending if it is a static or shared LIBRARY we might
      # need to install it.
      get_target_property(MJ_GLFW_LIBRARY_TYPE glfw TYPE)
      if(MJ_GLFW_LIBRARY_TYPE STREQUAL SHARED_LIBRARY)
        install(
          TARGETS glfw
          EXPORT ${PROJECT_NAME}
          RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT simulate
          LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
          ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
          PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT simulate
        )
      endif()
    endif()
  endif()
endif()