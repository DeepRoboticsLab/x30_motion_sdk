cmake_minimum_required(VERSION 3.10)
project(motion_example)


set(SOURCES 
    motion_example.cpp
    main.cpp
)


add_executable(motion_example ${SOURCES})
target_link_libraries(motion_example deeprobotics_motion_sdk_${CMAKE_SYSTEM_PROCESSOR})

