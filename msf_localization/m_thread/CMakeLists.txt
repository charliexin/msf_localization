cmake_minimum_required(VERSION 3.10.2)
project(m_thread)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")



find_package(Threads REQUIRED)

#include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(main_node src/main_node.cpp)

target_link_libraries(main_node Threads::Threads)