cmake_minimum_required(VERSION 3.8)
project(dgz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Define macros for conditional compilation
add_compile_definitions(__DEBUG_DGZ=1)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(space_interfaces REQUIRED)
find_package(serial REQUIRED)

# Includes the headers of dgz package
include_directories(include)

# Create dynamic library (SHARED) for dgz.cpp
add_library(dgz_frame_lib SHARED
  src/dgz_frame.cpp
)

add_executable(move_ctrl_node src/move_ctrl_node.cpp)
ament_target_dependencies(move_ctrl_node rclcpp space_interfaces)

add_executable(dgz_ctrl_node src/dgz_ctrl_node.cpp)
ament_target_dependencies(dgz_ctrl_node rclcpp serial space_interfaces)

# Link the libraries to the executables
# target_link_libraries(dgz_ctrl_node dgz_frame_lib serial_lib)
target_link_libraries(dgz_ctrl_node dgz_frame_lib)

# install(
#   DIRECTORY include
#   DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

# install(TARGETS dgz_frame_lib
#   EXPORT dgz_frame_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(
  TARGETS dgz_frame_lib move_ctrl_node dgz_ctrl_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS move_ctrl_node dgz_ctrl_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/)

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
