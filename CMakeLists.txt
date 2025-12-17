cmake_minimum_required(VERSION 3.15)
project(uasisi VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(third_party/yaml-cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(src)
