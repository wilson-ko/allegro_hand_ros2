cmake_minimum_required(VERSION 3.16)
project(allegro_hand_utility LANGUAGES CXX)

# Set the default build type to Release if it's not specified.
# This enables optimizations and disables debug information by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Defaulting build type to 'Release'")
endif()

# Set the default build type to Release if it's not specified.
# This enables optimizations and disables debug information by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Defaulting build type to 'Release'")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)

# --- Build Library ---
set(LIB_SOURCES
  src/utility.cpp
  src/ros_utility.cpp
)

# Create a shared library
set(SHARED_LIB_NAME ${PROJECT_NAME})
add_library(${SHARED_LIB_NAME} SHARED ${LIB_SOURCES})

# --- Configure Library ---
target_compile_features(${SHARED_LIB_NAME} PUBLIC cxx_std_17)

target_include_directories(${SHARED_LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# For Debug
# target_compile_definitions(${SHARED_LIB_NAME} PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

# For system dependencies, it's more robust to use target_link_libraries with imported targets.
target_link_libraries(${SHARED_LIB_NAME} PUBLIC
  spdlog::spdlog
  ${rclcpp_LIBRARIES}
  ${hardware_interface_LIBRARIES}
)

# --- Test Executable for Linkage Check ---
add_executable(test_library test/test_library.cpp)
target_link_libraries(test_library PUBLIC ${SHARED_LIB_NAME})

# --- Installation ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${SHARED_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# --- Exporting ---
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

set(LIB_DEPS
  spdlog
  Boost 
)
ament_export_dependencies(${LIB_DEPS})

install(
  TARGETS test_library
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
