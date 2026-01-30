cmake_minimum_required(VERSION 3.8)
project(fastslam_thesis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Aqui encontramos as bibliotecas que tiram o erro vermelho
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Inclui os headers da pasta src (onde você moveu os .hpp)
include_directories(include src)

# Cria o executável
add_executable(fastslam_node
  src/fastslam_node.cpp
  src/motion_model.cpp
  src/measurement_model.cpp
  src/grid_mapper.cpp
)

# Diz ao CMake onde procurar os headers do ROS
ament_target_dependencies(fastslam_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS fastslam_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()