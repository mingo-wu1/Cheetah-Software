cmake_minimum_required(VERSION 3.10)
PROJECT(imu)
set (CMAKE_CXX_STANDARD 14)

#find_package(OpenZen REQUIRED)
set(oriCMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -w")
add_subdirectory(openzen)
set(CMAKE_CXX_FLAGS "${oriCMAKE_CXX_FLAGS}")
#add_subdirectory(gsl)
set(curDir ${CMAKE_SOURCE_DIR}/third-party/lpmsusbal2_interface)
include_directories(${curDir}/include/)
message(${curDir}/include)
include_directories("../../common/include/")

link_directories(${curDir}/lib)
message(${curDir}/lib)

#add_library(OpenZen SHARED IMPORTED)

#set_target_properties(
#        OpenZen
#        PROPERTIES IMPORTED_LOCATION
#        ${curDir}/lib/libOpenZen.so
#)


set(example_sources
   LpUSBAL2Imu_test.cpp
)


add_library(LpUSBAL2Imu SHARED
      LpUSBAL2Imu.cpp
)

include(FindThreads)

target_compile_features(LpUSBAL2Imu
    PRIVATE
        cxx_std_14
)




target_link_libraries(LpUSBAL2Imu
       GSL
       Threads::Threads
       OpenZen
)


add_executable(LpUSBAL2Imu_test
    ${example_sources}
)


target_compile_features(LpUSBAL2Imu_test
    PRIVATE
        cxx_std_14
)


target_link_libraries(LpUSBAL2Imu_test
    PRIVATE
       LpUSBAL2Imu
)

# target_link_libraries(LpUSBAL2Imu
#     PRIVATE
#        GSL
#        Threads::Threads
# #        spdlog
#        OpenZen
# )


install(TARGETS  LpUSBAL2Imu
        LIBRARY DESTINATION lib)

# Change by hanyuanqian, 2021-10-19, The 'lib' folder does not exist
# install(DIRECTORY ${curDir}/lib/ DESTINATION /usr/local/lib)
# install(DIRECTORY ${curDir}/lib/ DESTINATION /usr/lib)

#if(ZEN_USE_STATIC_LIBS)
#    target_compile_definitions(LpUSBAL2Imu_test
#        PRIVATE
#           ZEN_API_STATIC
#    )
#else()
#    add_custom_command(TARGET LpUSBAL2Imu_test POST_BUILD
#       COMMAND ${CMAKE_COMMAND} -E copy_if_different
#            $<TARGET_FILE:OpenZen>
#            $<TARGET_FILE_DIR:LpUSBAL2Imu_test>
#    )
#endif()

# only copy Qt libraries if Bluetooth BLE support is enabled
#if(ZEN_BLUETOOTH_BLE)
#    add_custom_command(TARGET LpUSBAL2Imu_test POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy_if_different
#            $<TARGET_FILE:Qt5::Bluetooth>
#            $<TARGET_FILE_DIR:LpUSBAL2Imu_test>
#        COMMAND ${CMAKE_COMMAND} -E copy_if_different
#           $<TARGET_FILE:Qt5::Core>
#            $<TARGET_FILE_DIR:LpUSBAL2Imu_test>
#    )
#endif()

if (MSVC)
    target_compile_options(LpUSBAL2Imu_test
        PRIVATE
            /Zc:__cplusplus
    )
endif()


