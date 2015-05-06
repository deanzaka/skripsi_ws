
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#define _USE_MATH_DEFINES

int main (int argc, char** argv)
{
    	// Initialize ROS
    	ros::init (argc, argv, "camshift");
    	ros::NodeHandle nh;  

    	while(nh.ok()) {
	      	    ros::spinOnce();
  	}
}
