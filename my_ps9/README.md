# my_ps9

Your description goes here
####very important here:
In order to run this package correctly, you should first download file "sticky_fingers_manip_fncs.cpp"
in "ps9_external".
Then you should replace the file same name in "learning_ros"(part5/object_manipulation_properties) with it.

roscd &&catkin_make
## Example usage
#1st load ur10 and other parts to be grabbed
roslaunch ur10_launch ur10_w_gripper.launch
roslaunch cwru_ariac_launch cwru_ariac.launch
roslaunch ariac_models add_parts.launch
#2nd load server and client to do grabbing
rosrun my_ps9 ps9_server
rosrun my_ps9 ps9_client
## Running tests/demos
    
