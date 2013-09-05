rosWS
=====

The code for the adc (adcDriver.cpp and adcClient.cpp) in custom_navigation package comes from ros4mat  (https://code.google.com/p/ros4mat/). 

You need to add some code at the end of your ~/.bashrc file : 
   
    export ROS_MASTER_URI = http://MASTER_IP::11311  
    export ROS_HOSTNAME = http://OWN_IP  
    source /opt/ros/groovy/setup.bash  
    export ROS_PACKAGE_PATH= PATH_TO_OVERLAYS_ROS:PATH_TO_CUSTOM_ROS:$ROS_PACKAGE_PATH
  
Necessary commands in terminal to make the custom navigation system works (each command in separate tabs/windows):

      roscore
      roslaunch turtlebot_bringup minimal.launch
      roslaunch hokuyo_node hokuyo_test.launch
      roslaunch custom_navigation custom_navigation.launch
      
Additional commands to make a map of the environment:

      roslaunch turtlebot_navigation gmapping_demo.launch
      roslaunch turtlebot_teleop keyboard_teleop.launch
      rosrun map_server map_saver -f /path_where_you_wanna_save_the_map (to do at the end when the map is completed)

Additionnal command to navigate in a known map :
      
      roslaunch turtlebot_navigation amcl_demo.launch map_file:=/path_where_you_saved_the_map.yaml

Additionnal command to control the robot

      rosrun custom_navigation move_robot
      
Additional command to visualize the map, robot, path, laser scan, etc.

      export LIBGL_ALWAYS_SOFTWARE=1
      roslaunch turtlebot_rviz_launchers view_navigation.launch (or rosrun rviz rviz and you set everything manually)
