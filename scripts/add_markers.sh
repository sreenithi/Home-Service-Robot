#!/bin/sh
xterm -e " export ROBOT_INITIAL_POSE='-x 3.3 -y 3 -Y 1.5707'; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find maps)/map/sreenithi.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find maps)/map/map5.yaml" &
sleep 5
xterm -e " rosrun rviz rviz -d $(rospack find rviz_config)/rviz/config.rviz" &
sleep 20
xterm -e " rosrun add_markers add_markers_display"
