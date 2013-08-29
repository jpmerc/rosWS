rosWS
=====

The code for the adc (adcDriver.cpp and adcClient.cpp) comes from ros4mat  (https://code.google.com/p/ros4mat/). 

You need to add some text at the end of your ~/.bashrc file :
  export TURTLEBOT_BASE=create
  export TURTLEBOT_STACKS=circles
  export TURTLEBOT_3D_SENSOR=kinect
  export ROS_MASTER_URI = http://MASTER_IP::11311
  export ROS_HOSTNAME = http://OWN_IP
  source /opt/ros/groovy/setup.bash
  export ROS_PACKAGE_PATH= PATH_TO_rosbuild_ws:PATH_TO_custom_navigation:$ROS_PACKAGE_PATH
