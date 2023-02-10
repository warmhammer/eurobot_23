cmake_minimum_required(VERSION 2.8.3)
project(aruco_description)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/generate_aruco_mesh.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)