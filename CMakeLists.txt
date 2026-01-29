cmake_minimum_required(VERSION 3.15)
project(uasisi VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(third_party/yaml-cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(src)

# FOR TESTING  
add_executable(test_types tests/test_types.cpp)
target_include_directories(test_types PRIVATE include)
target_link_libraries(test_types gsl gslcblas)

# Optional: add as a test target
enable_testing()
add_test(NAME TypesTest COMMAND test_types)
