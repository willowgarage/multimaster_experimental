cmake_minimum_required(VERSION 2.8.3)
project(app_manager_tutorial)

find_package(catkin REQUIRED COMPONENTS app_manager rospy)
catkin_package(CATKIN_DEPENDS app_manager rospy)

install(DIRECTORY apps applist
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(FILES monolithic.launch test1.launch test2.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})