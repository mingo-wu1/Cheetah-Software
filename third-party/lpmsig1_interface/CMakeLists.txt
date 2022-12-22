cmake_minimum_required(VERSION 2.8.3)
project(LpIG1Imu)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_BUILD_TYPE "Release")
set(BUILD_ARCHITECTURE "32-bit" CACHE STRING "")
set_property(CACHE BUILD_ARCHITECTURE PROPERTY STRINGS "32-bit" "64-bit")

set(IG1_INC "./lpmsig1")
set(IG1_LIB "./lpmsig1/lib")

include_directories("${IG1_INC}")
link_directories("${IG1_LIB}")

ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++11") 
endif()

set(headers
    ${IG1_INC}/LpMatrix.h
    ${IG1_INC}/LpmsIG1Registers.h
    ${IG1_INC}/LpmsIG1I.h
    ${IG1_INC}/SensorDataI.h
)

set(LpIG1Imu_SRCS
    ${IG1_INC}/LpMatrix.c
    LpIG1Imu.cpp
)

#Add Begin by wuchunming02@countrygarden.com.cn, 2021-02-07, make libLpmsIG1_OpenSourceLib.so
set(LpmsIG1_OpenSourceLib_sources
    lpmsig1/MicroMeasure.cpp
    lpmsig1/SerialPortLinux.cpp
    lpmsig1/LpmsIG1.cpp
    lpmsig1/LpMatrix.c
    lpmsig1/LpUtil.cpp
    lpmsig1/version.rc
)

set(LpmsIG1_OpenSourceLib_headers
    lpmsig1/MicroMeasure.h
    lpmsig1/SerialPort.h
    lpmsig1/SensorData.h
    lpmsig1/SensorDataI.h
    lpmsig1/LpUtil.h
    lpmsig1/LpmsIG1.h
    lpmsig1/LpmsIG1I.h
    lpmsig1/LpmsIG1Registers.h
    lpmsig1/LpMatrix.h
    lpmsig1/LpLog.h
)

add_library(LpmsIG1_OpenSourceLib SHARED ${LpmsIG1_OpenSourceLib_sources} ${LpmsIG1_OpenSourceLib_headers})
target_link_libraries(LpmsIG1_OpenSourceLib pthread)
target_link_libraries(LpmsIG1_OpenSourceLib rt)
target_link_libraries(LpmsIG1_OpenSourceLib udev)
#Add End

# LpIG1Imu
add_library(lpIG1Imu SHARED ${LpIG1Imu_SRCS}  ${headers})
target_link_libraries(lpIG1Imu LpmsIG1_OpenSourceLib)

#add_executable(test_lpig1imu_interface test_lpig1imu_interface.cpp ${headers})
#target_link_libraries(test_lpig1imu_interface lpIG1Imu)

install(TARGETS LpmsIG1_OpenSourceLib lpIG1Imu LIBRARY DESTINATION lib)
