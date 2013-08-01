cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(aeas7000_driver src/aeas7000_driver.cpp)

rosbuild_add_executable(aeas7000_node src/aeas7000_node.cpp)
target_link_libraries(aeas7000_node raspi_interface aeas7000_driver)