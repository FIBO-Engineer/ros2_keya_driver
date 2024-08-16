cmake_minimum_required(VERSION 3.8)
project(ros2_keya_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

# Add service files
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/EmptyTrigger.srv"
# )

add_library(
  keya_driver
  SHARED
  src/keya_codec.cpp
  src/keya_driver_hw.cpp
)

target_compile_features(keya_driver PUBLIC cxx_std_17)

ament_target_dependencies( keya_driver
  ament_cmake
  rclcpp
  rclcpp_lifecycle
  rcl
  transmission_interface
  hardware_interface
  controller_interface
  controller_manager
  diagnostic_updater
  pluginlib
  std_srvs

)

pluginlib_export_plugin_description_file(hardware_interface keya_driver.xml)

# INSTALL
install(TARGETS
  keya_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include 
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY 
  config 
  description 
  DESTINATION share/${PROJECT_NAME}
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
