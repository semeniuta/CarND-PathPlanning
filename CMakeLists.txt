project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(core_sources src/helpers.cpp src/geometry.cpp src/PathPlanner.cpp src/steps.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)

    file(GLOB LIBUV_DIR /usr/local/Cellar/libuv/1*/lib)
    link_directories(${LIBUV_DIR})

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(path_planning src/main.cpp ${core_sources})
target_link_libraries(path_planning z ssl uv uWS)

add_executable(playground src/playground.cpp ${core_sources})
target_link_libraries(playground z ssl uv uWS)

