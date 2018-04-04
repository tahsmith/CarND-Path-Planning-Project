project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/catch.hpp src/path_planner.cpp src/path_planner.hpp src/map_data.hpp src/utilities.hpp src/sensor_fusion_data.hpp src/jerk_minimal_trajectory.cpp src/jerk_minimal_trajectory.hpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(test_all ${sources} src/test_main.cpp)
target_link_libraries(test_all z ssl uv uWS)

add_executable(path_planning src/main.cpp ${sources})
add_definitions(-DCATCH_CONFIG_DISABLE)
target_link_libraries(path_planning z ssl uv uWS)