cmake_minimum_required(VERSION 3.10)
project(motion_sdk_example)


set(SOURCES 
    motion_sdk_example.cpp
    main.cpp
)

add_executable(motion_sdk_example ${SOURCES})
target_link_libraries(motion_sdk_example deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR})

