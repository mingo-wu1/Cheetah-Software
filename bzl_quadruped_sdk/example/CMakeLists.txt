cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 14)

find_package(bzl_quadruped_sdk REQUIRED)

add_executable(body_cmd body_cmd.cpp)
target_link_libraries(body_cmd bzl_quadruped_sdk)

add_executable(gamepad_cmd gamepad_cmd.cpp)
target_link_libraries(gamepad_cmd bzl_quadruped_sdk)