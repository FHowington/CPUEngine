cmake_minimum_required(VERSION 3.15)
project(CPUEngine)

set(CMAKE_CXX_STANDARD 14)

find_package(SDL2 REQUIRED)
include_directories(${PROJECT_NAME} ${SDL2_INCLUDE_DIR})
add_executable(CPUEngine main.cpp RasterizePolygon.h mesh.h)
set(
        SOURCE_FILES
        main.cpp
        ${SDL_INCLUDE_DIR}/SDL.h
)
target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARY})

