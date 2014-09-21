cmake_minimum_required(VERSION 2.8.3)
project(rosproxy)
find_package(catkin REQUIRED COMPONENTS
  rospy rosgraph rosservice rostopic
)
catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy rosgraph rosservice rostopic)


install(DIRECTORY scripts nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
