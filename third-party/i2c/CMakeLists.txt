cmake_minimum_required(VERSION 2.8)

project(i2c LANGUAGES C)

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(I2C_BUILD_EXAMPLES "Build I2C examples" OFF)

message(STATUS "I2C CMake Info")
message(STATUS "=======================================================")
message(STATUS "    Operation System :  ${CMAKE_SYSTEM}")
message(STATUS "    CPU Architecture : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "    Build Type : ${CMAKE_BUILD_TYPE}${CMAKE_CONFIGURATION_TYPES}")
message(STATUS "    Shared Library  : ${BUILD_SHARED_LIBS}")
message(STATUS "    Build Examples  : ${I2C_BUILD_EXAMPLES}")
message(STATUS "=======================================================")

# make lib
include_directories(${CMAKE_BINARY_DIR})
add_library(${PROJECT_NAME} STATIC i2c.c)

# examples
if (I2C_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif ()
