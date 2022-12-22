cmake_minimum_required(VERSION 3.5)

project(bzl_quadruped_sdk)

set(_VERSION_MAJOR 0)
set(_VERSION_MINOR 0)
set(_VERSION_PATCH 1)

set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_BUILD_TYPE "Release")

include(${CMAKE_CURRENT_SOURCE_DIR}/pack2deb.cmake)

include_directories("/usr/include/lcm")
include_directories("../include/")
include_directories("../../lcm-types/cpp")

file(GLOB sources "../src/bzl_quadruped.cpp" "../src/comm_client.cpp")
add_library(${PROJECT_NAME} SHARED ${sources})
target_link_libraries(${PROJECT_NAME} lcm pthread)

add_executable(body_cmd ../example/body_cmd.cpp)
target_link_libraries(body_cmd ${PROJECT_NAME})

add_executable(gamepad_cmd ../example/gamepad_cmd.cpp)
target_link_libraries(gamepad_cmd ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_cmake
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ../include/${PROJECT_NAME} DESTINATION include)
install(DIRECTORY ../example DESTINATION "/opt/${PROJECT_NAME}"
        PATTERN "build" EXCLUDE
        PATTERN "example/*"
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ 
                    GROUP_EXECUTE GROUP_WRITE GROUP_READ
                    WORLD_EXECUTE WORLD_WRITE WORLD_READ
        )
install(FILES ../README.md DESTINATION "/opt/${PROJECT_NAME}")

install(EXPORT ${PROJECT_NAME}_cmake
        FILE ${PROJECT_NAME}-config.cmake
        DESTINATION lib/cmake/${PROJECT_NAME})