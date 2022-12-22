cmake_minimum_required(VERSION 3.5)
project(bzl_quadruped)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

add_compile_options(-std=c++11)

include(${CMAKE_CURRENT_SOURCE_DIR}/cpack/pack2deb.cmake)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)
#execute_process(COMMAND ../scripts/make_types.sh)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#set(CMAKE_VERBOSE_MAKEFILE ON)

option(MINI_CHEETAH_BUILD "use compiler flags for mini cheetah computer" OFF)
set(BUILD_TYPE_RELEASE TRUE)

option(NO_SIM "Do not build simulator" OFF)

if(MINI_CHEETAH_BUILD)
  SET (THIS_COM "../" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
  set(CMAKE_CXX_FLAGS "-O3 -no-pie -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wsign-promo -Werror")
  set(CMAKE_C_FLAGS "-O3 -std=gnu99 -I.")
  message("**** Mini-Cheetah build enabled ****")
else(MINI_CHEETAH_BUILD)
  SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)

  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 ")
    #-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    #-Wsign-promo -Werror"
  elseif(APPLE)
    set(CMAKE_CXX_FLAGS "-O3 -march=native -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wsign-promo")
    include_directories("/usr/local/include/")   # lcm includes
  endif()

  set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
  message("**** Mini-Cheetah build disabled ****")
endif(MINI_CHEETAH_BUILD)

set(CMAKE_CXX_STANDARD 14)

#find_package(lcm)

### Add by hanyuanqiang, 2021-03-26, Add the link of 485 A1 motor driver
### Change by hanyuanqiang, 2021-08=09, Add USE_RS485_A1
option(USE_RS485_A1 "unitree RS485 A1 motor" OFF)
if (USE_RS485_A1)
  add_definitions("-DUSE_RS485_A1=1")
  set(BITNESS 32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(BITNESS 64)
  endif()

  IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
      IF(BITNESS EQUAL 64)
          set(RS485_A1_LIBS libUnitree_motor_SDK_Linux64.so)
      ELSEIF(BITNESS EQUAL 32)
          set(RS485_A1_LIBS libUnitree_motor_SDK_Linux32.so)
      ENDIF()
  ENDIF()  

  link_directories(robot/src/rt/rs485_a1_lib)
endif()
### Add End

option(USE_SPI_DATA_CURRENT "Get the motor current through the SPIne board" ON)
if (USE_SPI_DATA_CURRENT)
add_definitions("-DUSE_SPI_DATA_CURRENT=1")
endif()

option(USE_LOGGER "使用日志升级，建议打开吧，关闭日志系统总不是长久之计" ON)
if (USE_LOGGER)
add_definitions("-DUSE_LOGGER=1")
endif()

option(USE_LINKAGE "leg linkage" OFF)
if (USE_LINKAGE)
add_definitions("-DUSE_LINKAGE=1")
endif()

option(USE_LINKAGE_INDUSTRIAL "leg linkage industrial" OFF)
if (USE_LINKAGE_INDUSTRIAL)
add_definitions("-DUSE_LINKAGE_INDUSTRIAL=1")
endif()

option(USE_LPMSUSBAL2 "lpmsusbal2 IMU" OFF)
if (USE_LPMSUSBAL2)
add_definitions("-DUSE_LPMSUSBAL2=1")
endif()

add_subdirectory(robot)
add_subdirectory(third-party)
add_subdirectory(common)
add_subdirectory(bzl_quadruped_sdk)
#add_subdirectory(googletest-release-1.10.0)

if(NO_SIM)

else(NO_SIM)
  add_subdirectory(sim)
endif()

add_subdirectory(auto_task_example)
add_subdirectory(user)
add_subdirectory(rc_test)

install(DIRECTORY config/ DESTINATION config)
install(DIRECTORY resources/ DESTINATION resources)
install(DIRECTORY lcm-types/java lcm-types/python DESTINATION lcm-types)
install(DIRECTORY scripts/lcm-log2smat DESTINATION bin)
install(PROGRAMS  scripts/00_run_gamepad.sh DESTINATION bin)
install(PROGRAMS  scripts/01_run_robot.sh DESTINATION bin)
install(PROGRAMS  scripts/02_run_sim.sh DESTINATION bin)
install(PROGRAMS  scripts/03_run_launch_spy.sh DESTINATION bin)
install(PROGRAMS  scripts/04_run_log_convert.sh DESTINATION bin)
