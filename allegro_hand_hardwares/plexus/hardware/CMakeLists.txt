cmake_minimum_required(VERSION 3.8)
project(allegro_hand_plexus_hardware)

# Set the default build type to Release if it's not specified.
# This enables optimizations and disables debug information by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Defaulting build type to 'Release'")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(spdlog REQUIRED) 
find_package(diagnostic_updater REQUIRED)
find_package(allegro_hand_io REQUIRED)
find_package(allegro_hand_utility REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  hardware_interface 
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  spdlog 
  diagnostic_updater
  allegro_hand_io
  allegro_hand_utility
)

# set(SRC_DIR EXP_SRC_DIR)
# file(GLOB SRC_DIR ./src/command_server.cpp ./src/command_server_node.cpp)
# file(GLOB EXP_SRC_DIR ./examples/src/*.cpp)

#compile
# add_executable(ros2_cmd_server ${SRC_DIR})
# add_executable(srv_test_client ${EXP_SRC_DIR})
add_library(
  allegro_hand_plexus_hardware 
  SHARED  
  ./src/plexus_sdk.cpp 
  ./src/plexus_spec.cpp 
  ./src/plexus_hardware_interface.cpp
  ./src/generic_system.cpp
)

target_compile_definitions(allegro_hand_plexus_hardware PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

add_executable(interface_test ./src/plexus_interface_test.cpp)

target_compile_definitions(interface_test PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

#add headfile source
# target_include_directories(ros2_cmd_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
# target_include_directories(srv_test_client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/examples/include)
target_include_directories(allegro_hand_plexus_hardware PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/allegro_hand_plexus_hardware>
)

#add dependencies
ament_target_dependencies(allegro_hand_plexus_hardware ${dependencies})

target_link_libraries(interface_test spdlog::spdlog )
ament_target_dependencies(interface_test ${dependencies}) 


#export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface allegro_hand_plexus_hardware.xml)

#install all files
#command server
# install(TARGETS ros2_cmd_server 
#   DESTINATION lib/${PROJECT_NAME}
# )

#test example
# install(TARGETS srv_test_client
#   DESTINATION lib/${PROJECT_NAME}
# )

#interface test 
install(TARGETS interface_test
  DESTINATION lib/${PROJECT_NAME}
)

#ros2_control hardware
install(TARGETS allegro_hand_plexus_hardware
  EXPORT allegro_hand_plexus_hardware
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#export
ament_export_targets(allegro_hand_plexus_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle)

ament_package()
