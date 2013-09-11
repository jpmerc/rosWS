export TURTLEBOT_LABEL=robot_0
export TURTLEBOT_BASE=create
export TURTLEBOT_STACKS=circles
export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT_BATTERY=/sys/class/power_supply/BAT0
ipadr=$(hostname -I)
export ROS_HOSTNAME=$ipadr
export ROS_MASTER_URI=http://orleans.gel.ulaval.ca:11311
#export ROS_MASTER_URI=http://$ipadr:11311
source /opt/ros/groovy/setup.bash
source ~/Devel/rosWS/custom_ros/devel/setup.sh
export ROS_PACKAGE_PATH=~/Documents/rosWS/custom_ros:~/Workspace/src/iCreate_Custom:$ROS_PACKAGE_PATH
export HOKUYO_PORT=$(ls -l /dev/serial/by-id/usb-Hokuyo* | awk '{print $11}' | awk -F"/" '{print $3}')
