cmake_minimum_required (VERSION 2.6)
SET (CMAKE_CXX_FLAGS "-g -Wall")
project (Robovision)
find_package (OpenCV REQUIRED)
add_executable (Robovision main.cpp)
target_link_libraries (Robovision ${OpenCV_LIBS})
