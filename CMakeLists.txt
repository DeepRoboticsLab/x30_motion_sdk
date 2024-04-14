cmake_minimum_required(VERSION 3.10)

set(ROBOT_NAME "x30")
project(${ROBOT_NAME}_motion_sdk)
# SET(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
# SET(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
set(CMAKE_BUILD_TYPE Release)
set(SRC_DIR_LIST "." src)
add_definitions(-w) # warning ignore
set(CMAKE_CXX_FLAGS "-std=c++11 ${CAMKE_CXX_FLAGS}")
message("CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

option(BUILD_EXAMPLE "Example for motion sdk in C++" ON)
option(BUILD_PYTHON "Motion sdk python version" ON)

foreach(VAR ${SRC_DIR_LIST})
  set(TEMP)
  aux_source_directory(./src/ TEMP)
  set(SRC_LIST ${RCS_SRC_LIST} ${TEMP})
endforeach(VAR)

include_directories(
  ./include/
)


add_library(deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} SHARED ${SRC_LIST})
target_link_libraries(deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} -lpthread -lm -lrt -ldl -lstdc++)

add_custom_command(TARGET deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR}> ${CMAKE_BINARY_DIR}/../python/lib/
    COMMENT "Copying deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} to python/lib"
)



if(BUILD_EXAMPLE)
  add_subdirectory(example)
endif()

if(BUILD_PYTHON)
  add_subdirectory(python)
endif()