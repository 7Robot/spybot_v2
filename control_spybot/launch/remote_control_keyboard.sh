export ROS_MASTER_URI=http://ubiquityrobot.local:11311
export ROS_HOSTNAME=$(hostname).local
rosrun image_view image_view image:=/camera/image_raw _image_transport:=compressed &
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
