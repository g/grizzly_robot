cmake_minimum_required(VERSION 2.8.3)
project(grizzly_base)

find_package(catkin REQUIRED COMPONENTS roscpp diagnostic_updater grizzly_msgs roboteq_msgs geometry_msgs)
catkin_package()

include_directories(${catkin_INCLUDES})
add_subdirectory(src)

install(PROGRAMS
  # nodes/dead_reckoning.py
  nodes/diagnostics.py
  nodes/energy.py
  nodes/estop_mux.py
  nodes/fan_control.py
  # nodes/motion_generator.py
  # nodes/motion_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


