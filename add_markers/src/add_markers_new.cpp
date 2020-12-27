#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>

class MarkerVisualisation
{
  private:
	ros::NodeHandle n;
	ros::Publisher marker_pub;
	ros::Subscriber sub;
	visualization_msgs::Marker marker;
  	geometry_msgs::Pose pickup_pose, dropoff_pose;
  
  	void initialize_marker()
    {
      // Set our initial shape type to be a cube
      uint32_t shape = visualization_msgs::Marker::CYLINDER;
      
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";
      marker.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.2;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
	
      marker.lifetime = ros::Duration();
    }

  void checkPos(const std_msgs::Int8 flag)
  {
  	if(flag.data == 1)
    {
      deleteMarker();
      ROS_INFO("Picking up object");
      ros::Duration(5).sleep();    
    }
    else if(flag.data == 2)
    {
      publishMarker(dropoff_pose, "Dropping off object!");
    }
  }
  
  public:
  	MarkerVisualisation(geometry_msgs::Pose pickup, geometry_msgs::Pose dropoff)
    {
        marker_pub = n.advertise<visualization_msgs::Marker>("marker_visualisation", 1);
      	sub = n.subscribe<std_msgs::Int8>("/pick_objects/reach_goal", 1, &MarkerVisualisation::checkPos, this);
      	
		pickup_pose = pickup;
      	dropoff_pose = dropoff;
      
      	initialize_marker();
      
      	publishMarker(pickup, "Object ready for pickup!");
    }
  
  	void publishMarker(geometry_msgs::Pose pose, char* publish_msg)
    {
    	marker.action = visualization_msgs::Marker::ADD;
      	marker.pose = pose;
      
        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
          if (!ros::ok())
          {
            ROS_WARN_ONCE("Returning");
            return;
          }
          ROS_WARN_ONCE("Please create a subscriber to the marker");
          sleep(1);
        }

//       	ROS_INFO("Publishing marker");
        ROS_INFO("%s", publish_msg);
        marker_pub.publish(marker);    
    }
  
  	void deleteMarker()
    {
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);    
    }
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers"); 

  geometry_msgs::Pose marker_pickup_pose, marker_dropoff_pose;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker_pickup_pose.position.x = -5.0;
  marker_pickup_pose.position.y = -5.0;
  marker_pickup_pose.orientation.w = 1.0;


  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker_dropoff_pose.position.x = 1.0;
  marker_dropoff_pose.position.y = 1.0;
  marker_dropoff_pose.orientation.w = 1.0;    

  MarkerVisualisation marker(marker_pickup_pose, marker_dropoff_pose);
  
  ros::spin();

}
