cmake_minimum_required(VERSION 3.8)
project(fsm_impedance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(
  PACKAGE_DEPENDENCIES

  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  tf2_ros
  hardware_interface
  realtime_tools
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rclcpp_components
  rosidl_default_generators
  visualization_msgs
  action_msgs
  franka_hardware
  franka_msgs
  franka_semantic_components
  Eigen3 
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(controller_manager REQUIRED)

foreach(PACKAGE_DEPENDENCY IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE_DEPENDENCY} REQUIRED)
endforeach()

add_executable(equilibrium_pose_publisher src/equilibrium_pose_publisher.cpp)

# Link the executable with the necessary libraries
ament_target_dependencies(equilibrium_pose_publisher
  rclcpp
  geometry_msgs
)

# Describe the compilation of the controller
add_library(
  ${PROJECT_NAME} SHARED
  src/fsm_impedance_controller.cpp
)

# Include necessary header files
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>
)


# Specify ament dependencies
ament_target_dependencies(
  ${PROJECT_NAME} ${PACKAGE_DEPENDENCIES}
)

# Use pluginlib to expose the controller as visible to the rest of ros2_control
pluginlib_export_plugin_description_file(controller_interface fsm_impedance_controller.xml)

# Describe installation procedures
install(TARGETS
  equilibrium_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS fsm_impedance_controller
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_load_fsm_ic test/test_load_fsm_impedance_controller.cpp)
  ament_target_dependencies(
    test_load_fsm_ic controller_manager
  )

endif()

# Define exports for the package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${PACKAGE_DEPENDENCIES} controller_manager
)

ament_package()
