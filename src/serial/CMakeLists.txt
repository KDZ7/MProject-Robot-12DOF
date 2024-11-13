cmake_minimum_required(VERSION 3.8)
project(serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Define macros for conditional compilation
add_compile_definitions(__DEBUG_SERIAL=1)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

# Includes the headers of the serial package
include_directories(include)

# Create dynamic library (SHARED) for serial.cpp
add_library(serial_lib SHARED
  src/serial.cpp
)

# Create executable for testlib_node.cpp
add_executable(testlib_node
  src/testlib_node.cpp
)

# Create static library (STATIC) for serial.cpp
# add_library(serial_lib STATIC
#   src/serial_com.cpp
# )

# ament_target_dependencies(serial_lib rclcpp ...)
# Export the library to make it available to other packages
ament_export_targets(serial_lib HAS_LIBRARY_TARGET)

# Link the serial_lib library to the testlib_node executable
target_link_libraries(testlib_node serial_lib)
ament_target_dependencies(testlib_node rclcpp)

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include)

install(
  TARGETS serial_lib
  EXPORT serial_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS testlib_node
  DESTINATION lib/${PROJECT_NAME}
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
