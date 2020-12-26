#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  
  ros::NodeHandle n;
  ros::Publisher p = n.advertise<std_msgs::Int8>("/pick_objects/reach_goal", 1);
  std_msgs::Int8 reachedFlag;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_zone, dropoff_zone;

  // set up the frame parameters
  pickup_zone.target_pose.header.frame_id = "map";
  pickup_zone.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickup_zone.target_pose.pose.position.x = 1.0;
  pickup_zone.target_pose.pose.position.y = 1.0;
  pickup_zone.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending Pick-up goal");
  ac.sendGoal(pickup_zone);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The robot has reached the set pickup zone! Picking up object!");
    reachedFlag.data = 1;
    p.publish(reachedFlag);    
  }
  else
  {
    ROS_INFO("The robot failed to reach the pickup zone");
    reachedFlag.data = 0;
    p.publish(reachedFlag);    
  }
  
  ros::Duration(5).sleep();
  
  // set up the frame parameters
  dropoff_zone.target_pose.header.frame_id = "map";
  dropoff_zone.target_pose.header.stamp = ros::Time::now(); 
  
  dropoff_zone.target_pose.pose.position.x = 3.0;
  dropoff_zone.target_pose.pose.position.y = 3.0;
  dropoff_zone.target_pose.pose.orientation.w = 1.0;
  
  ROS_INFO("Sending Drop-off goal");
  ac.sendGoal(dropoff_zone);
  
  ac.waitForResult();
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The robot has reached the set dropoff zone! Dropping off object!");
    reachedFlag.data = 2;
    p.publish(reachedFlag);
  }
  else
  {
    ROS_INFO("The robot failed to reached the dropoff zone!");
    reachedFlag.data = 0;
    p.publish(reachedFlag);
  }

  ros::spin();
  
  return 0;
}
