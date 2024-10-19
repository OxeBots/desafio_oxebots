cmake_minimum_required(VERSION 3.5)
project(desafio_oxebots)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

install(DIRECTORY launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(gazebo_ros_pkgs)

ament_package()
