# //Create package
roscd
cd ../src
catkin_create_pkg custom_action_ardrone_pkg roscpp actionlib 

# // Build
cd ~/catkin_ws
catkin_make --only-pkg-with-deps custom_action_ardrone_pkg
source devel/setup.bash

# Launch action server
roslaunch custom_action_ardrone_pkg start_custom_action_ardrone.launch
rostopic pub /custom_action_ardrone/goal custom_action_ardrone_pkg/CustomActionGoal '{"goal":{"goal":"TAKEOFF"}}'
ostopic pub /custom_action_ardrone/goal custom_action_ardrone_pkg/CustomActionGoal '{"goal":{"goal":"LAND"}}'

rostopic echo /custom_action_ardrone/feedback











rostopic pub -1 /fly_drone_action/goal actionlib/TestActionGoal '{"goal": {"goal": 2}}'

rostopic echo /fly_drone_action/feedback
rostopic echo /fly_drone_action/result

rostopic pub  /fly_drone_action/cancel actionlib_msgs/GoalID "{}"

