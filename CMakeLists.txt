cmake_minimum_required(VERSION 3.1)
project(cpp_math)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES math.cpp matrix3.cpp matrix4.cpp quaternion.cpp vector.cpp)
add_library(cpp_math ${SOURCE_FILES})