cmake_minimum_required(VERSION 3.5)
project(li_battery_diagnostic_module)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Ensure that we explicitly add the C++ exceptions flag.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fexceptions)
endif()

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# Add any additional package you need here

# Include C++ header files
include_directories(include)

# Declare a library with the source files
add_library(${PROJECT_NAME} SHARED
  src/diagnostic_algorithm/battery_module.cpp
  src/diagnostic_algorithm/cell_voltage_analysis.cpp
  src/battery_diagnostic_core.cpp
)

# Link library with required ROS 2 packages
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  # Add any additional package you need here
)

# Declare an executable for your node
add_executable(battery_diagnostic_node src/battery_diagnostic_core.cpp)

# Link executable with your library
target_link_libraries(battery_diagnostic_node ${PROJECT_NAME})

# Install executables and/or libraries
install(TARGETS
  battery_diagnostic_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Uncomment if you have include directories to export
# ament_export_include_directories(include)

# Uncomment if you have libraries to export
# ament_export_libraries(${PROJECT_NAME})

# Uncomment if you have dependencies to export
# ament_export_dependencies(rclcpp sensor_msgs std_msgs)

# Include CMake testing for your package
if(BUILD_TESTING)
  # Add any tests here
endif()

# Must be called at the end to set up the proper environment for ROS 2
ament_package()

