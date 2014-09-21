cmake_minimum_required(VERSION 2.8.3)
project(foreign_relay)

find_package(catkin REQUIRED COMPONENTS
  roscpp topic_tools)
catkin_package(
  CATKIN_DEPENDS roscpp topic_tools)

add_executable(unreliable_relay src/nodes/unreliable_relay.cpp)
target_link_libraries(unreliable_relay ${catkin_LIBRARIES})

add_executable(foreign_relay src/nodes/foreign_relay.cpp)
target_link_libraries(foreign_relay ${catkin_LIBRARIES})

add_executable(foreign_relay_latched src/nodes/foreign_relay_latched.cpp)
target_link_libraries(foreign_relay_latched ${catkin_LIBRARIES})

install(TARGETS foreign_relay_latched foreign_relay unreliable_relay
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
