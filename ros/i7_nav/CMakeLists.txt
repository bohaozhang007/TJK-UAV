cmake_minimum_required(VERSION 3.0.2)
project(i7_nav)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(
  PROGRAMS scripts/i7_nav_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
