#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find maps)/map/sreenithi.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find maps)/map/map.yaml initial_pose_a:=-1.5707" &
sleep 5
xterm -e " rosrun rviz rviz -d $(rospack find rviz_config)/rviz/config.rviz" &
sleep 5
xterm -e " rosrun add_markers add_markers" &
sleep 10
xterm -e " rosrun pick_objects pick_objects"
