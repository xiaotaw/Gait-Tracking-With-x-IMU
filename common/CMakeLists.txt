cmake_minimum_required(VERSION 2.8.0)

add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
    rosbag
    eigen_conversions
)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(exec_name "test_RosbagReader")
add_executable(${exec_name}
    test/test_RosbagReader.cpp
)
target_link_libraries(${exec_name} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
