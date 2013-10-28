cmake_minimum_required(VERSION 2.8.3)
project(grizzly_node)

find_package(catkin)

catkin_package()

add_executable(hardware_safety hardware_safety.cpp)
target_link_libraries(hardware_safety ${catkin_LIBRARIES})

install(TARGETS hardware_safety
  RUNTIME_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(PROGRAMS
  nodes/dead_reckoning.py
  nodes/diagnostics.py
  nodes/energy.py
  nodes/estop_mux.py
  nodes/fan_control.py
  nodes/motion_generator.py
  nodes/motion_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


