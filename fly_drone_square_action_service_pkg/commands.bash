# //Create package
roscd
cd ../src
catkin_create_pkg fly_drone_square_action_service_pkg roscpp actionlib ardrone_as geometry_msgs

# // Build
cd ~/catkin_ws
catkin_make --only-pkg-with-deps fly_drone_square_action_service_pkg
source devel/setup.bash

# Launch action server
roslaunch fly_drone_square_action_service_pkg start_fly_drone_square_action_server.launch

rostopic pub -1 /fly_drone_action/goal actionlib/TestActionGoal '{"goal": {"goal": 2}}'

rostopic echo /fly_drone_action/feedback
rostopic echo /fly_drone_action/result

rostopic pub  /fly_drone_action/cancel actionlib_msgs/GoalID "{}"

