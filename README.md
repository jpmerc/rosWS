rosWS
=====

The code for the adc (adcDriver.cpp and adcClient.cpp) in custom_navigation package comes from ros4mat  (https://code.google.com/p/ros4mat/). 

First, you have to modify the custom_navigation/src/custom.navigation.sh script. Here's what you have to modify :

      export TURTLEBOT_LABEL=robot_X #Replace X by a number from 0 to 5
      export ROS_MASTER_URI=IP_ADDR_OF_MASTER:11311 #replace IP_ADDR_OF_MASTER by the ip address of the computer you want to act as the master
      source PATH_TO_ROSWS/custom_ros/devel/setup.sh #replace PATH_TO_ROSWS with the path of rosWS workspace (devel will only be available after catkin_make)

      
Then, you need to source custom_navigation/src/custom.navigation.sh at the end of the ~/.bashrc file.
After that, you build the repository by doing the command "catkin_make"  at the base of custom_navigation repository.


Necessary commands in terminal to make the custom navigation system works (each command in separate tabs/windows):

      roscore
      roslaunch custom_navigation create.launch
      roslaunch custom_navigation custom_navigation.launch
      
Additional commands to make a map of the environment:

      roslaunch custom_navigation gmapping_demo.launch
      roslaunch custom_navigation keyboard_teleop.launch
      rosrun map_server map_saver -f /path_where_you_wanna_save_the_map (to do at the end when the map is completed)

Additionnal command to navigate in a known map :
      
      roslaunch custom_navigation amcl_demo.launch map_file:=/path_where_you_saved_the_map.yaml

Additionnal command to control the robot

      rosrun custom_navigation move_robot
      
Additional command to visualize the map, robot, path, laser scan, etc.

      export LIBGL_ALWAYS_SOFTWARE=1 #only if you get a segmentation fault when starting rviz
      roslaunch turtlebot_rviz_launchers view_navigation.launch (or rosrun rviz rviz and you set everything manually)
      
      
Git commands to load submodules :

      git submodule init
      git submodule foreach git pull origin master
      git submodule update
      
