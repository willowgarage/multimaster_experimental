cmake_minimum_required(VERSION 2.8.3)
project(app_manager)
find_package(catkin REQUIRED COMPONENTS
  rospy rosunit roslaunch rosgraph message_generation
)
catkin_python_setup()

add_message_files(FILES
  AppInstallationState.msg
  AppList.msg
  App.msg
  AppStatus.msg
  ClientApp.msg
  ExchangeApp.msg
  Icon.msg
  KeyValue.msg
  StatusCodes.msg
  )

add_service_files(FILES
  GetAppDetails.srv
  GetInstallationState.srv
  InstallApp.srv
  ListApps.srv
  StartApp.srv
  StopApp.srv
  UninstallApp.srv
  )

generate_messages()
catkin_package(CATKIN_DEPENDS rospy rosunit roslaunch rosgraph message_generation)
catkin_add_nosetests(test/test_app.py)
catkin_add_nosetests(test/test_app_list.py)

install(DIRECTORY bin launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
