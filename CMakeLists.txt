cmake_minimum_required(VERSION 2.8)
project(icp_hw)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL REQUIRED)

include_directories(
        "/usr/include/eigen3/"
        ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_executable(icp_main
        icp_main.cpp icp_g2o.cpp icp_g2o.h)
target_link_libraries(icp_main
        ${PCL_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/3rdParty/g2o/lib/libg2o.so)
