include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("include/")
include_directories("./")
include_directories("../common/include/")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("../third-party/vectornav/include")
include_directories("../third-party/vectornav/include/vn")
include_directories("../third-party/lord_imu/Include")
include_directories("../third-party/CSerialPort/include")
include_directories("../third-party/i2c")

if(USE_LPMSUSBAL2)
include_directories("../third-party/lpmsusbal2_interface/include")
endif()

include_directories("../lcm-types/cpp")
include_directories("/usr/include/lcm/")   # lcm includes
include_directories("../third-party/SOEM/osal")
include_directories("../third-party/SOEM/osal/linux")
include_directories("../third-party/SOEM/oshw")
include_directories("../third-party/SOEM/oshw/linux")

file(GLOB sources "src/*.cpp" "src/rt/*.c"  "src/rt/*.cpp")

### Add by hanyuanqiang, 2021-08-09, Add USE_RS485_A1
if (USE_RS485_A1)
else (USE_RS485_A1)
    file(GLOB rs485_source "src/rt/rt_rs485_a1.cpp")
    list(REMOVE_ITEM sources ${rs485_source})
endif(USE_RS485_A1)
### Add End

add_library(robot SHARED ${sources})

target_link_libraries(robot biomimetics pthread lcm inih dynacore_param_handler lord_imu soem lpIG1Imu libcserialport -lstdc++fs)

if(USE_LPMSUSBAL2)
target_link_libraries(robot LpUSBAL2Imu)
endif()

if(CMAKE_SYSTEM_NAME MATCHES Linux)
target_link_libraries(robot libvnc rt)
endif()

### Add by hanyuanqiang, 2021-03-26, Add the link of 485 A1 motor driver
### Change by hanyuanqiang, 2021-08=09, Add USE_RS485_A1
if (USE_RS485_A1)
    target_link_libraries(robot ${RS485_A1_LIBS})
    install(FILES ./src/rt/rs485_a1_lib/${RS485_A1_LIBS} DESTINATION lib)
endif()
### Add End

install(TARGETS robot LIBRARY DESTINATION lib)
