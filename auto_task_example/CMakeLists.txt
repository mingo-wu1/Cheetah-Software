
include_directories("./")
include_directories("./include")
include_directories("../robot/include")
include_directories("../common/include/")
include_directories("../third-party/")
include_directories("../user/BZL_Controller")
include_directories("../user/BZL_Controller/FSM_States")
include_directories("../common/FootstepPlanner")
include_directories("../third-party/")
include_directories(SYSTEM "../third-party/qpOASES/include")
include_directories("../third-party/ParamHandler")
include_directories("../bzl_quadruped_sdk/include")
include_directories("../third-party/CSerialPort/include")

include_directories("../lcm-types/cpp")
include_directories("/usr/include/lcm")   # lcm includes
include_directories("/usr/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE autoTaskExampleSources "./src/*.cpp")

add_library(autoTaskExample SHARED ${autoTaskExampleSources})
target_link_libraries(autoTaskExample biomimetics)

install(TARGETS  autoTaskExample LIBRARY DESTINATION lib)
