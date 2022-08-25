cmake_minimum_required(VERSION 3.23)
project(CppLenfVersion)

set(CMAKE_CXX_STANDARD 14)

set(OpenCV_DIR "C:/opencv/build/x64/vc15/lib")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

find_package(OpenCV REQUIRED)
#set(OpenCV_FOUND true)

include_directories(core ${OpenCV_INCLUDE_DIRS})
file(GLOB_RECURSE SRC_FILES "core/*.cpp")


add_executable(CppLenfVersion ${SRC_FILES} main.cpp)
link_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(CppLenfVersion
        ${CMAKE_THREAD_LIBS_INIT}
        ${OpenCV_LIBS}
        ${CMAKE_DL_LIBS})
