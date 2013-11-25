cmake_minimum_required(VERSION 2.8.3)
project(grizzly_bringup)

find_package(catkin)

catkin_package()

install(DIRECTORY launch config udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/install
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
