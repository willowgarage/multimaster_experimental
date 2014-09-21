cmake_minimum_required(VERSION 2.8.3)
project(app_manager_android)

find_package(catkin REQUIRED COMPONENTS roslib pyqrnative)

catkin_package(CATKIN_DEPENDS roslib pyqrnative)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
