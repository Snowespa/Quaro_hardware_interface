cmake_minimum_required(VERSION 3.8)
project(quaro_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hw_pi_hat_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/quaro_system.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(
  ${PROJECT_NAME}
  PUBLIC c_std_99 cxx_std_17
) # Require C99 and C++17

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp  
)

target_link_libraries(
  ${PROJECT_NAME}
  hw_pi_hat_interface::hw_pi_hat_interface
)

pluginlib_export_plugin_description_file(
  hardware_interface
  quaro_control_plugins.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES quaro_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY ros2_control
  DESTINATION share/${PROJECT_NAME}/ros2_control
)

ament_export_include_directories(include)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
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
endif()

ament_package()
