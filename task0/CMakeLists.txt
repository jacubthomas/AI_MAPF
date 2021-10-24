project(task0)

cmake_minimum_required (VERSION 2.6)
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall --std=c++0x")

include_directories("../")
file(GLOB SOURCES "../*.cpp" "*.cpp")
add_executable(task0 ${SOURCES})