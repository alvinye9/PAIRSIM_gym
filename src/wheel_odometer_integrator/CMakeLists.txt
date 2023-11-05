cmake_minimum_required(VERSION 3.5)
project(wheel_odometer_integrator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(raptor_dbw_msgs)
ament_auto_find_build_dependencies()

include_directories(
    include
    ${raptor_dbw_msgs_INCLUDE_DIRS}
    )

ament_auto_add_executable(wheel_odom_integrator_node
  src/wheel_odometer_integrator.cpp
)

# export information to downstream packages
ament_auto_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
  INSTALL_TO_SHARE launch config
)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
