export ROS_MASTER_URI=http://ubiquityrobot.local:11311
export ROS_HOSTNAME=$(hostname).local
rosrun image_view image_view image:=/camera/image_raw _image_transport:=compressed &
roslaunch teleop_twist_joy teleop.launch
