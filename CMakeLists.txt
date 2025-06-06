cmake_minimum_required(VERSION 3.5)
project(sicktoolbox_wrapper)

# Requerir C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Encontrar paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sicktoolbox REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Incluir directorios
include_directories(
  include
  ${sicktoolbox_INCLUDE_DIRS}
)

# ========================
# Ejecutables ROS
# ========================
add_executable(sickld ros/sickld.cpp)
ament_target_dependencies(sickld rclcpp sensor_msgs sicktoolbox diagnostic_updater)

add_executable(sicklms ros/sicklms.cpp)
ament_target_dependencies(sicklms rclcpp sensor_msgs sicktoolbox diagnostic_updater)

# ========================
# Ejecutables Standalone
# ========================
add_executable(log_scans standalone/log_scans.cpp)
ament_target_dependencies(log_scans rclcpp sicktoolbox)

add_executable(time_scans standalone/time_scans.cpp)
ament_target_dependencies(time_scans rclcpp sicktoolbox)

add_executable(print_scans standalone/print_scans.cpp)
ament_target_dependencies(print_scans sicktoolbox)

# ========================
# Instalar ejecutables
# ========================
install(TARGETS
  sickld
  sicklms
  log_scans
  time_scans
  print_scans
  DESTINATION lib/${PROJECT_NAME}
)

# ========================
# Instalar launch files
# ========================
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ========================
# Exportar dependencias
# ========================
ament_export_dependencies(rclcpp sensor_msgs sicktoolbox diagnostic_updater)

ament_package()

