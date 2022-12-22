include_directories("./")
include_directories("./Controllers")
include_directories("./FSM_States")
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/FootstepPlanner")
include_directories("../../third-party/")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")
include_directories("../../bzl_quadruped_sdk/include")
include_directories("../../third-party/CSerialPort/include")
include_directories("../../auto_task_example/include")

include_directories("../../lcm-types/cpp")
include_directories("/usr/include/lcm")   # lcm includes
include_directories("/usr/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp" 
"Controllers/BalanceController/*.cpp" 
"Controllers/convexMPC/*.cpp")

add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)
add_subdirectory(Controllers/VisionMPC)

add_executable(bzl_ctrl ${sources} BZL_Controller.cpp main.cpp)
target_link_libraries(bzl_ctrl robot biomimetics)
target_link_libraries(bzl_ctrl dynacore_param_handler qpOASES)
target_link_libraries(bzl_ctrl Goldfarb_Optimizer osqp)
target_link_libraries(bzl_ctrl WBC_Ctrl)
target_link_libraries(bzl_ctrl VisionMPC)
target_link_libraries(bzl_ctrl bzl_quadruped_sdk_server)
target_link_libraries(bzl_ctrl libcserialport)
target_link_libraries(bzl_ctrl autoTaskExample)

install(TARGETS  bzl_ctrl RUNTIME DESTINATION bin)
