cmake_minimum_required(VERSION 3.5)
project(test1)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)


execute_process(COMMAND ${CMAKE_SOURCE_DIR}/scripts/find_qt_path.sh OUTPUT_VARIABLE CMAKE_PREFIX_PATH_1)
SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH_1})
# message("CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 OLD)
endif()

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
find_package(Qt5Core CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
get_target_property(QtCore_location Qt5::Core LOCATION)
message(STATUS ${QtCore_location})
include_directories("/usr/include/lcm/")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})

find_package(Qt5Core COMPONENTS QtGamepad REQUIRED)
find_package(Qt5Gamepad REQUIRED)

add_executable(gamepad main.cpp gamecontroller.cpp gamecontroller.h)
target_link_libraries(gamepad lcm pthread Qt5::Core Qt5::Gamepad ${QT_LIBRARIES} ${OPENGL_LIBRARIES})

install(TARGETS gamepad RUNTIME DESTINATION bin)

