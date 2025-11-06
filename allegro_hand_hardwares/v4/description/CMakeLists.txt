cmake_minimum_required(VERSION 3.5.0)

project(allegro_hand_v4_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
	urdf
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()
install(
  TARGETS
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()