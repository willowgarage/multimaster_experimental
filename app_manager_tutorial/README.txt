Create 4 terminals.

Building Terminal:
export ROS_MASTER_URI=http://localhost:11368
roscore -p 11368

Robot1 Terminal:
export ROS_MASTER_URI=http://localhost:11312
roslaunch app_manager_tutorial test1.launch

Robot2 Terminal:
export ROS_MASTER_URI=http://localhost:11313
roslaunch app_manager_tutorial test2.launch

Client Terminal:
export ROS_MASTER_URI=http://localhost:11368
rosservice list
rosservice call robot1/start_app app_manager_tutorial/example
rostopic list
rosservice call robot2/start_app app_manager_tutorial/example
rostopic list
rostopic echo /robot1/application/gorilla/baz -n1
rostopic echo /robot2/application/gorilla/baz -n1
rostopic pub /robot1/application/gorilla/foo std_msgs/Float32 1.5 -1
rostopic pub /robot2/application/gorilla/foo std_msgs/Float32 2.6 -1
rostopic echo /robot1/application/gorilla/baz -n1
rostopic echo /robot2/application/gorilla/baz -n1
export ROS_MASTER_URI=http://localhost:11312
rostopic pub bar std_msgs/Int32 7 -1
export ROS_MASTER_URI=http://localhost:11313
rostopic pub bar std_msgs/Int32 9 -1
export ROS_MASTER_URI=http://localhost:11368
rostopic echo /robot1/application/gorilla/baz -n1
rostopic echo /robot2/application/gorilla/baz -n1
rosservice call robot1/stop_app
rosservice call robot2/stop_app
rostopic list

