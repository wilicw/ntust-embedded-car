cmake_minimum_required(VERSION 3.10)
project(driver)

set(CMAKE_CXX_STANDARD 11)

#find_package(OpenCV REQUIRED)
find_package(Threads)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

include_directories(core)

file(GLOB_RECURSE SRC_FILES "core/*.cpp")

add_executable(driver ${SRC_FILES} main.cpp)
target_link_libraries(driver ${WIRINGPI_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
