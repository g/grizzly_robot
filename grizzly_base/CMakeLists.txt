cmake_minimum_required(VERSION 2.8.3)
project(grizzly_base)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  controller_manager diagnostic_updater geometry_msgs hardware_interface grizzly_motor_driver grizzly_motor_msgs
  realtime_tools grizzly_msgs roscpp rosserial_server sensor_msgs std_msgs topic_tools)
find_package(Boost REQUIRED COMPONENTS thread chrono)

catkin_package(INCLUDE_DIRS include)
 catkin_add_env_hooks(50.grizzly_find_mag_config
   SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(grizzly_node
  src/grizzly_base.cpp
  src/grizzly_hardware.cpp
  src/grizzly_diagnostic_updater.cpp
  src/grizzly_cooling.cpp
  src/grizzly_lighting.cpp
  src/grizzly_indicators.cpp
  )
target_link_libraries(grizzly_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(grizzly_node grizzly_msgs_gencpp rosserial_msgs_gencpp)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

 install(PROGRAMS scripts/calibrate_compass scripts/compute_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS grizzly_node
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/base.launch)

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
