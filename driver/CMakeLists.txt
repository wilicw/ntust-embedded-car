cmake_minimum_required(VERSION 3.10)
project(driver)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "-O3")
SET(CMAKE_C_FLAGS "-O3")

find_package(OpenCV REQUIRED)
find_package(Threads)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_library(TF_LIBRARIES NAMES tensorflow-lite)
find_package(Boost REQUIRED)

include_directories(core ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

file(GLOB_RECURSE SRC_FILES "core/*.cpp")

add_executable(driver ${SRC_FILES} main.cpp)
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(driver
        ${WIRINGPI_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        ${OpenCV_LIBS}
        ${CMAKE_DL_LIBS}
        ${TF_LIBRARIES}
        ${Boost_LIBRARIES})
