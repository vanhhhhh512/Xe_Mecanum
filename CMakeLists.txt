cmake_minimum_required(VERSION 3.8)
project(Xe)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch meshes urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/mecanum_numpad.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
