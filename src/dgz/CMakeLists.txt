cmake_minimum_required(VERSION 3.8)
project(dgz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(space_interfaces REQUIRED)
find_package(serial REQUIRED)

# Inclut les en-têtes nécessaires
include_directories(include)

# Crée la bibliothèque
add_library(dgz_frame_lib SHARED
  src/dgz_frame.cpp
)

# Dépendances de la bibliothèque
ament_target_dependencies(dgz_frame_lib rclcpp space_interfaces)

# Crée les exécutables
add_executable(move_ctrl_node src/move_ctrl_node.cpp)
ament_target_dependencies(move_ctrl_node rclcpp space_interfaces)

add_executable(dgz_ctrl_node src/dgz_ctrl_node.cpp)
ament_target_dependencies(dgz_ctrl_node rclcpp space_interfaces serial)

# Lie les bibliothèques aux exécutables
target_link_libraries(dgz_ctrl_node dgz_frame_lib serial_lib)

# Installation
install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS dgz_frame_lib
  EXPORT dgz_frame_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS move_ctrl_node dgz_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
