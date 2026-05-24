cmake_minimum_required(VERSION 3.8)
project(fastslam_thesis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Dependências do ROS 2
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Inclui os headers da pasta src
include_directories(include src)

# Cria o executável com todos os arquivos .cpp
add_executable(fastslam_node
  src/fastslam_node.cpp
  src/motion_model.cpp
  src/measurement_model.cpp
  src/grid_mapper.cpp
)

# Linka as dependências do ROS ao executável
ament_target_dependencies(fastslam_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Instala o executável (o nó C++)
install(TARGETS fastslam_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- A PARTE NOVA QUE FALTAVA ---
# Instala a pasta launch para o ROS encontrar o script Python
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# --------------------------------

ament_package()