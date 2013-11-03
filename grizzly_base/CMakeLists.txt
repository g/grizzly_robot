cmake_minimum_required(VERSION 2.8.3)
project(grizzly_base)

find_package(catkin REQUIRED COMPONENTS roscpp diagnostic_updater roboteq_msgs )
catkin_package()

add_executable(${PROJECT_NAME}_motion_generator src/motion_generator)
target_link_libraries(${PROJECT_NAME}_motion_generator ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_generator
                      PROPERTIES OUTPUT_NAME motion_generator PREFIX "")

install(TARGETS ${PROJECT_NAME}_motion_generator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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


