cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(random_velocity_plugin2)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(random_velocity_plugin2 SHARED random_velocity_plugin2.cpp)
target_link_libraries(random_velocity_plugin2 ${GAZEBO_LIBRARIES})

#message (STATUS "GAZEBO_LIBRARIES: " ${GAZEBO_LIBRARIES})
#install (TARGETS ${PROJECT_NAME} DESTINATION ${GAZEBO_PACKAGE_LIB_DESTINATION})

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${GAZEBO_PACKAGE_INCLUDE_DESTINATION}
#)
