#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
# define PI 3.14159265358979323846

// Some global variables
std::vector<double> drone_pose(7,0), visual_features(3,0); // [u,v]
double camera_depth = 0.0;

void hasReceivedVisualFeatures(const geometry_msgs::PoseStamped::ConstPtr& msg){

	visual_features[0] = msg->pose.position.x;
	visual_features[1] = msg->pose.position.y;
	visual_features[2] = msg->pose.position.z;
  return;
}

void hasReceivedDronePosition(const geometry_msgs::PoseStamped::ConstPtr& msg){

	drone_pose[2] = msg->pose.position.z;
  return;
}



Eigen::MatrixXd computeImageJacobian(std::vector<double> vf, std::vector<double> uav_pose){

	Eigen::MatrixXd Ji(2,6);
	double u = vf[1];
	double v = vf[2];
	double Z = uav_pose[3];
	Ji(0,0) = -1/Z;
	Ji(0,1) = 0.0;
	Ji(0,2) = u/Z;
	Ji(0,3) = u*v;
	Ji(0,4) = -(1+ sqrt(u));
	Ji(0,5) = v;
	Ji(1,0) = 0;
	Ji(1,1) = -1/Z;
	Ji(1,2) = v/Z;
	Ji(1,3) = (1 + sqrt(v));
	Ji(1,4) = u*v;
	Ji(1,5) = -u;

	return Ji;
}

int main(int argc, char** argv){

ros::init(argc, argv, "ibvs_controller_node");
ros::NodeHandle nh_;
ros::NodeHandle nhp_("~");
ros::Rate rate(20.0);


// Initialize varialbes
Eigen::MatrixXd Ji(2,6);
std_msgs::Float32MultiArray control_angles;

// Create Subscribers and Publishers
ros::Subscriber visual_tracker_sub_=nh_.subscribe("visual_tracker", 10, hasReceivedVisualFeatures);
ros::Subscriber vrep_drone_pose_sub_=nh_.subscribe("vrep_drone_pose", 10, hasReceivedDronePosition);
ros::Publisher control_output_pub_=nh_.advertise<std_msgs::Float32MultiArray>("ibvs_control_output", 1);

	while (ros::ok()){

		Ji = computeImageJacobian(visual_features,drone_pose);

		control_output_pub_.publish(control_angles);
   		ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
