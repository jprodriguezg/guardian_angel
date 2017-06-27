#include <iostream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
# define PI 3.14159265358979323846

// Some global variables

geometry_msgs::Twist velocityMsg;
std::vector<double> visual_features(3,0); // [u,v]

void hasReceivedVisualFeatures(const geometry_msgs::PoseStamped::ConstPtr& msg){

	// Obtaining target point info
	visual_features[0] = msg->pose.position.x;
	visual_features[1] = msg->pose.position.y;
	visual_features[2] = msg->pose.position.z;

  return;
}


int main(int argc, char** argv){

ros::init(argc, argv, "IBSV_controller_node");
ros::NodeHandle nh_;
ros::NodeHandle nhp_("~");
ros::Rate rate(20.0);


ros::Subscriber visual_tracker_sub_=nh_.subscribe("visual_tracker", 10, hasReceivedVisualFeatures);
//ros::Publisher vel_pub_=nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	while (ros::ok()){

   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
