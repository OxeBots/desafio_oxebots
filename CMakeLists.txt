cmake_minimum_required(VERSION 3.8)
project(desafio_oxebots)

find_package(ament_cmake REQUIRED)
find_package(ros_gz REQUIRED)

install(DIRECTORY launch models urdf worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ros_gz)

set(ignoreMe
  "${CATKIN_INSTALL_INTO_PREFIX_ROOT}${CATKIN_SYMLINK_INSTALL}${CMAKE_EXPORT_COMPILE_COMMANDS}"
)

ament_package()
