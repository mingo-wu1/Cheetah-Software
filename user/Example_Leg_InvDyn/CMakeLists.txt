include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")
include_directories("../../third-party/CSerialPort/include")

add_executable(leg_invdyn_ctrl main.cpp Leg_InvDyn_Controller.cpp)
target_link_libraries(leg_invdyn_ctrl robot)

install(TARGETS leg_invdyn_ctrl RUNTIME DESTINATION bin)
