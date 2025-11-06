cmake_minimum_required(VERSION 3.16)
project(allegro_hand_grasp_controllers LANGUAGES CXX)

# Set the default build type to Release if it's not specified.
# This enables optimizations and disables debug information by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Defaulting build type to 'Release'")
endif()

if(APPLE OR WIN32)
  message(WARNING "gripper controllers are not available on OSX or Windows")
  return()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  allegro_hand_control_msgs
  allegro_hand_grasp_library
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  realtime_tools
  spdlog
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

message(STATUS "bhand_INCLUDE_DIRS: ${bhand_INCLUDE_DIRS}")

generate_parameter_library(grasp_action_controller_parameters
  src/grasp_action_controller_parameters.yaml
)

add_library(grasp_action_controller SHARED
  src/grasp_action_controller.cpp
  src/grasp_action_position_controller.cpp
  src/grasp_action_effort_controller.cpp
)
target_compile_features(grasp_action_controller PUBLIC cxx_std_17)
target_include_directories(grasp_action_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${bhand_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include/grasp_action_controller>
)
target_link_libraries(grasp_action_controller PUBLIC
  grasp_action_controller_parameters
)
ament_target_dependencies(grasp_action_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_compile_definitions(grasp_action_controller PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

pluginlib_export_plugin_description_file(controller_interface ros_control_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/grasp_action_controller
)
install(
  TARGETS
    grasp_action_controller
    grasp_action_controller_parameters
  EXPORT export_gripper_action_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_gripper_action_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
