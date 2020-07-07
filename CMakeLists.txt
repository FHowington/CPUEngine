cmake_minimum_required(VERSION 3.13)

# set the project name
project(CPUEngine VERSION 0.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# includes cmake/FindSDL2.cmake
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


option(SIMD "Use AVX2/FMA instructions" ON)
option(DEBUG "Build debug variant with no optimizations" OFF)
if(SIMD)
  SET(CLANG_INSTRUCTIONS "-mavx2 -mfma -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize")
else()
  SET(CLANG_INSTRUCTIONS "-mno-avx")
endif()

if(DEBUG)
  SET(CLANG_SPEED "-O0 -g -DDEBUG")
else()
  SET(CLANG_SPEED "-Ofast -mavx2 -mfma -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize")
  message("DEBUG")
endif()

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${CLANG_SPEED} ${CLANG_INSTRUCTIONS}")

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIR})
add_library(engineLib "")
add_subdirectory(src)

add_executable(engine src/main.cpp)
target_link_libraries(engine ${SDL2_LIBRARY})
target_link_libraries(engine engineLib)
