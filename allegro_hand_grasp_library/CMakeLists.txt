cmake_minimum_required(VERSION 3.16)
project(allegro_hand_grasp_library LANGUAGES CXX)

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
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
# Tell pinocchio to not look for python bindings, which are not needed.
set(PINOCCHIO_SKIP_PYTHON_LIBS TRUE)
find_package(pinocchio REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# --- Configure Pre-compiled Libraries ---
set(SHARED_LIB_NAME ${PROJECT_NAME})
set(STATIC_LIB_NAME ${PROJECT_NAME}_static)

# Define paths to the pre-compiled libraries based on system architecture
set(PRECOMPILED_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
set(SHARED_LIB_PATH ${PRECOMPILED_LIB_DIR}/lib${SHARED_LIB_NAME}.so)
set(STATIC_LIB_PATH ${PRECOMPILED_LIB_DIR}/lib${STATIC_LIB_NAME}.a)

# Create IMPORTED library targets for the pre-compiled libraries
add_library(${SHARED_LIB_NAME} SHARED IMPORTED)
set_property(TARGET ${SHARED_LIB_NAME} PROPERTY IMPORTED_LOCATION ${SHARED_LIB_PATH})

add_library(${STATIC_LIB_NAME} STATIC IMPORTED)
set_property(TARGET ${STATIC_LIB_NAME} PROPERTY IMPORTED_LOCATION ${STATIC_LIB_PATH})

set(TARGETS_TO_CONFIGURE ${SHARED_LIB_NAME} ${STATIC_LIB_NAME})
 
# For IMPORTED targets, we must define their usage requirements (include directories,
# linked libraries, etc.) using the INTERFACE scope. This tells any downstream
# target that links against this imported library what it needs.
foreach(TARGET ${TARGETS_TO_CONFIGURE})
  target_include_directories(${TARGET} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
  )

  # pinocchio::pinocchio will transitively link against Eigen3 and provide its include directories.
  target_link_libraries(${TARGET} INTERFACE
    pinocchio::pinocchio
    spdlog::spdlog
    yaml-cpp
  )
endforeach()

# --- Installation ---
install(
  DIRECTORY include/
  DESTINATION include
)

# Use install(FILES ...) for pre-compiled libraries instead of install(TARGETS ...).
install(
  FILES ${SHARED_LIB_PATH} ${STATIC_LIB_PATH}
  DESTINATION lib
)

# --- Exporting ---
set(LIB_DEPS
  Eigen3
  spdlog
  Boost
  pinocchio
  yaml_cpp_vendor
)
ament_export_dependencies(${LIB_DEPS})

# Export the include directory for downstream packages
ament_export_include_directories(include)
# Export the library targets so downstream packages can link against them
ament_export_libraries(${SHARED_LIB_NAME} ${STATIC_LIB_NAME})

ament_package()
