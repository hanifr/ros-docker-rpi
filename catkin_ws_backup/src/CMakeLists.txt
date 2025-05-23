cmake_minimum_required(VERSION 3.0.2)
project(Project)

find_package(catkin REQUIRED)

# Find all packages in subdirectories
file(GLOB packages RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(package ${packages})
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${package}/CMakeLists.txt)
    message(STATUS "Adding subdirectory: ${package}")
    add_subdirectory(${package})
  endif()
endforeach()