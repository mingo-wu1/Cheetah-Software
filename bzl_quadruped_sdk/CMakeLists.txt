include_directories("include/")
include_directories("../lcm-types/cpp")
include_directories("/usr/include/lcm/")   # lcm includes

include_directories("../common/include/")
include_directories("../user/BZL_Controller/")
include_directories("../")
include_directories("../third-party/CSerialPort/include/")

# file(GLOB sources "src/comm_server_impl.cpp" "src/comm_server.cpp")
file(GLOB sources "src/comm_server.cpp")
add_library(bzl_quadruped_sdk_server SHARED ${sources})
target_link_libraries(bzl_quadruped_sdk_server lcm pthread biomimetics)

install(TARGETS bzl_quadruped_sdk_server LIBRARY DESTINATION lib)