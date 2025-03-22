cmake_minimum_required( VERSION 3.0 )
project(vo)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV not found.")
   endif()

# Find YAML-CPP
find_package(yaml-cpp REQUIRED)

# set (CMAKE_CXX_FLAGS "-lstdc++fs -std=c++17")

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable( vo src/vo_main.cpp src/LatLong-UTMconversion.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} yaml-cpp stdc++fs )