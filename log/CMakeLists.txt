cmake_minimum_required(VERSION 3.5)
add_compile_options(--std=c++11)
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../third-party/spdlog-1.7.0/include")
include_directories("./")

add_executable(logger example/main.cpp)
target_link_libraries(logger pthread)