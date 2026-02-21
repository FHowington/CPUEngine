cmake_minimum_required(VERSION 3.13)

# set the project name
project(CPUEngine VERSION 0.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# includes cmake/FindSDL2.cmake
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=cppcoreguidelines*,clang-analyzer*,boost*,llvm*,llvmlibc*,modernize*,performance*,readability*,-readability-magic-numbers,-modernize-use-trailing-return-type,-cppcoreguidelines-pro-bounds-pointer-arithmetic,-cppcoreguidelines-pro-bounds-constant-array-index,-cppcoreguidelines-pro-type-union-access,-llvm-header-guard,-cppcoreguidelines-pro-type-member-init,-cppcoreguidelines-avoid-magic-numbers,-cppcoreguidelines-narrowing-conversions,-cppcoreguidelines-pro-type-cstyle-cast,-cppcoreguidelines-macro-usage -fix)



option(SIMD "Use SIMD instructions (portable via SIMDe)" OFF)
option(DEBUG "Build debug variant with no optimizations" OFF)
if(SIMD)
  SET(CLANG_INSTRUCTIONS "-march=native -Rpass=loop-vectorize")
  add_definitions(-DUSE_SIMD)
endif()

if(DEBUG)
  SET(CLANG_SPEED "-O0 -g -DDEBUG")
else()
  SET(CLANG_SPEED "-Ofast")
  message("DEBUG")
endif()

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${CLANG_SPEED} ${CLANG_INSTRUCTIONS} -Werror -Wall")

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIR})
if(SIMD)
  include_directories(${PROJECT_SOURCE_DIR}/third_party/simde)
endif()
add_library(engineLib "")
add_subdirectory(src)

add_executable(engine src/main.cpp)
target_link_libraries(engine ${SDL2_LIBRARY})
target_link_libraries(engine engineLib)
