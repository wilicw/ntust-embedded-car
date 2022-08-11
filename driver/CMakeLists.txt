cmake_minimum_required(VERSION 3.10)
project(driver)

set(CMAKE_CXX_STANDARD 11)

include_directories(core)

file(GLOB_RECURSE SRC_FILES core *.cpp)

add_executable(driver ${SRC_FILES} main.cpp)
