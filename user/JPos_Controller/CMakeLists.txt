include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")
include_directories("../../third-party/CSerialPort/include")

add_executable(jpos_ctrl main.cpp JPos_Controller.cpp)
target_link_libraries(jpos_ctrl robot biomimetics dynacore_param_handler qpOASES)

install(TARGETS jpos_ctrl RUNTIME DESTINATION bin)