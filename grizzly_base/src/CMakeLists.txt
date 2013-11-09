add_executable(${PROJECT_NAME}_motor_fanout motor_fanout)
add_dependencies(${PROJECT_NAME}_motor_fanout
  grizzly_msgs_generate_messages_cpp
  roboteq_msgs_generate_messages_cpp
)
target_link_libraries(${PROJECT_NAME}_motor_fanout ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motor_fanout
                        PROPERTIES OUTPUT_NAME motor_fanout PREFIX "")

install(TARGETS
  ${PROJECT_NAME}_motor_fanout
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
