cmake_minimum_required(VERSION 2.8.0)

find_package(catkin REQUIRED COMPONENTS
    rosbag
)

# need c++14 for auto in lambda
add_definitions(-std=c++11)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
)

set(exec_name "test_RosbagReader")
add_executable(${exec_name} test/test_RosbagReader.cpp)
target_link_libraries(${exec_name} ${catkin_LIBRARIES})