cmake_minimum_required(VERSION 2.8.3)
project(grizzly_base)

find_package(catkin REQUIRED COMPONENTS roscpp diagnostic_updater grizzly_msgs roboteq_msgs geometry_msgs)
catkin_package()

include_directories(${catkin_INCLUDES})

add_subdirectory(src)
add_subdirectory(nodes)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
