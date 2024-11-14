cmake_minimum_required(VERSION 3.8)
project(serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(__DEBUG_SERIAL=1)

# Trouver les dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Inclure les en-têtes du package
include_directories(include)

# Créer une bibliothèque dynamique (SHARED) pour serial.cpp
add_library(serial_lib SHARED
  src/serial.cpp
)

# Cible de l'exécutable de test
add_executable(testlib_node src/testlib_node.cpp)
target_link_libraries(testlib_node serial_lib)
ament_target_dependencies(testlib_node rclcpp)

# Définir les répertoires d'inclusion pour la bibliothèque
target_include_directories(serial_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Exporter la bibliothèque pour d'autres packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp) # non ajouté par KIM
ament_export_include_directories(include)
ament_export_libraries(serial_lib)

# Installer les fichiers d'en-tête
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Installer les cibles
install(
  TARGETS serial_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS testlib_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()