cmake_minimum_required(VERSION 3.10)

project(x30_motion)
# SET(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
# SET(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
set(CMAKE_BUILD_TYPE Release)
set(SRC_DIR_LIST "." src)
add_definitions(-w) # warning ignore
set(CMAKE_CXX_FLAGS "-std=c++11 ${CAMKE_CXX_FLAGS}")

foreach(VAR ${SRC_DIR_LIST})
  set(TEMP)
  aux_source_directory(./src/ TEMP)
  set(SRC_LIST ${RCS_SRC_LIST} ${TEMP})
endforeach(VAR)

include_directories(
  ./include/
  # ./lib/eigen3
)

add_executable(${PROJECT_NAME} "main.cpp" ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} -lpthread -lm -lrt -ldl -lstdc++ )

