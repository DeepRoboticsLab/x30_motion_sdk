cmake_minimum_required(VERSION 3.10)

project(x30_motion_sdk)
# SET(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
# SET(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
set(CMAKE_BUILD_TYPE Release)
set(SRC_DIR_LIST "." src)
add_definitions(-w) # warning ignore
set(CMAKE_CXX_FLAGS "-std=c++11 ${CAMKE_CXX_FLAGS}")
message("CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

foreach(VAR ${SRC_DIR_LIST})
  set(TEMP)
  aux_source_directory(./src/ TEMP)
  set(SRC_LIST ${RCS_SRC_LIST} ${TEMP})
endforeach(VAR)

include_directories(
  ./include/
)


add_library(deeprobotics_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} SHARED ${SRC_LIST})
target_link_libraries(deeprobotics_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} -lpthread -lm -lrt -ldl -lstdc++)

add_custom_command(TARGET deeprobotics_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:deeprobotics_motion_sdk_${CMAKE_SYSTEM_PROCESSOR}> ${CMAKE_BINARY_DIR}/../python/lib/
    COMMENT "Copying deeprobotics_motion_sdk_${CMAKE_SYSTEM_PROCESSOR} to python/lib"
)

option(BUILD_WITH_EXAMPLE ON)
option(BUILD_PYTHON ON)

if(BUILD_WITH_EXAMPLE)
  add_subdirectory(example)
endif()

if(BUILD_PYTHON)
  add_subdirectory(python)
endif()