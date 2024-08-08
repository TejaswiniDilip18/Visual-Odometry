cmake_minimum_required( VERSION 3.0 )
project(vo)

find_package(OpenCV 4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4 not found.")
   endif()

set (CMAKE_CXX_FLAGS "-lstdc++fs -std=c++17")

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable( vo vo_main.cpp LatLong-UTMconversion.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} )