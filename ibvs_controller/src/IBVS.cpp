#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
# define PI 3.14159265358979323846

// Some global variables
std::vector<double> drone_velocity(6,0),drone_pose(6,0), visual_features(3,0), drone2cameraPose(3,0); // [u,v]
double camera_depth = 0.0;


// -------- Callbacks to retrieve topics information  --------------------
void hasReceivedVisualFeatures(const geometry_msgs::PoseStamped::ConstPtr& msg){

	visual_features[0] = msg->pose.position.x;
	visual_features[1] = msg->pose.position.y;
	visual_features[2] = msg->pose.position.z;
  return;
}

void hasReceivedDronePosition(const geometry_msgs::PoseStamped::ConstPtr& msg){

	drone_pose[0] = msg->pose.position.x;
	drone_pose[1] = msg->pose.position.y;
	drone_pose[2] = msg->pose.position.z;
	drone_pose[3] = msg->pose.orientation.x;
	drone_pose[4] = msg->pose.orientation.y;
	drone_pose[5] = msg->pose.orientation.z;
  return;
}


void hasReceivedDroneVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg){

	drone_velocity[0] = msg->twist.linear.x;
	drone_velocity[1] = msg->twist.linear.y;
	drone_velocity[2] = msg->twist.linear.z;
	drone_velocity[3] = msg->twist.angular.x;
	drone_velocity[4] = msg->twist.angular.y;
	drone_velocity[5] = msg->twist.angular.z;
  return;
}

void hasReceivedDrone2CameraPosition(const geometry_msgs::PointStamped::ConstPtr& msg){

	drone2cameraPose[0] = msg->point.x;
	drone2cameraPose[1] = msg->point.y;
	drone2cameraPose[2] = msg->point.z;
	return;
}



// -------- Functions --------------------
/* This function retrives the variable and its derivatives. It is lazy function but saves space. Unfortunately I
 	did not find how to get a subverctor from a vector in c++ */
void VariablesandDerivatives(std::vector<double> x, std::vector<double> xd, Eigen::RowVectorXd &var, Eigen::RowVectorXd &dvar, int starting_ref){

	int j = 0;
	for (int i = starting_ref; i<i+3; i++){
		var(j) = x[i];
		dvar(j) = xd[i];
		j++;
	}
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

/* This functions makes the transformation between the derivatives of the euler angles and the angular velocities
	NOTE: Be careful with the notation of the angles, otherwise the function returns a wrong transformation */

Eigen::RowVectorXd fromAngleDerivatives2AngularVelocity(Eigen::RowVectorXd euler_derivatives, Eigen::RowVectorXd euler_angles){
	Eigen::RowVectorXd angular_vel (3);
	// NOTE: The relation of the Euler angles to RPY is the following (The same as in my notebook)
	// gm -> Roll, ph -> yaw, th -> pitch.
	double gm = euler_angles[0];
	double gmd = euler_derivatives[0];
	double th = euler_angles[1];
	double thd = euler_derivatives[1];
	double ph = euler_angles[2];
	double phd = euler_derivatives[2];

	// Computes the correspondent angular velocity
	angular_vel(0) = phd*sin(th)*sin(gm) + thd*cos(gm);
	angular_vel(1) = phd*sin(th)*cos(gm) - thd*sin(gm);
	angular_vel(2) = phd*cos(th) + gmd;


	return angular_vel;
}

Eigen::MatrixXd PseudoInverse(Eigen::MatrixXd matrix){
	Eigen::MatrixXd aux, pseudoInv;
	aux = matrix*matrix.transpose();
	pseudoInv = matrix.transpose()*aux.inverse();
	return pseudoInv;
}

Eigen::MatrixXd SkewSymmetricMatrix(std::vector<double> input_vec){
	Eigen::MatrixXd S;
	S << 0.0, -input_vec[2], input_vec[1],
    input_vec[2], 0.0, -input_vec[0],
    -input_vec[1], input_vec[0], 0.0;
	return S;
}

Eigen::MatrixXd computeDrone2CameraJacobian(std::vector<double> drone2cameraVec){
	Eigen::MatrixXd Jcd(6,6);

	// Skew symmetric of the vector from the drone to the camera
	Eigen::MatrixXd Sdc = SkewSymmetricMatrix(drone2cameraVec)*-1.0;

	// Rotation between the drone and the camera
	// NOTE: This rotation was computed based on the orientations in the V-REP scene
	Eigen::MatrixXd Rotationdc;
	Rotationdc << 1.0, 0.0,	0.0,
    				0.0, -1.0, 0.0,
    				0.0, 0.0, -1.0;
	// Filling the Jcd matrix
	Jcd.topLeftCorner(3,3) = Eigen::MatrixXd::Identity(3, 3);
	Jcd.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
	Jcd.topRightCorner(3,3) = Sdc;
	Jcd.bottomRightCorner(3, 3) = Rotationdc;
	return Jcd;
}

int main(int argc, char** argv){

ros::init(argc, argv, "ibvs_controller_node");
ros::NodeHandle nh_;
ros::NodeHandle nhp_("~");
ros::Rate rate(20.0);

// Initialize the publisher message
std_msgs::Float32MultiArray msg;
std::vector<double> control_angles(2,0);
msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
msg.layout.dim[0].size = control_angles.size();
msg.layout.dim[0].stride = 1;


// Intialize some local variables
Eigen::MatrixXd Jcd, J, Ji, Jv, Jw, Jvps;
Eigen::RowVectorXd angular_vel(3), imagef(2), imgefd(2), error(3), angles(3), angles_derivatives(3), position(3), linear_vel(3);

// Control gain
double K = 1.0;
// Visual gain
double Ka = 1.0;
// Quadrotor mass
double mass = 1.0;
// Current trusht force
double Thrust =1.0;
// Create Subscribers and Publishers
ros::Subscriber visual_tracker_sub_=nh_.subscribe("visual_tracker", 10, hasReceivedVisualFeatures);
ros::Subscriber vrep_drone_pose_sub_=nh_.subscribe("vrep_drone_pose", 10, hasReceivedDronePosition);
ros::Subscriber vrep_drone2camera_pose_sub_=nh_.subscribe("vrep_drone2camera_pose", 10, hasReceivedDrone2CameraPosition);
ros::Subscriber vrep_drone_velocity_sub_=nh_.subscribe("vrep_drone_velocity", 10, hasReceivedDroneVelocity);
ros::Publisher control_output_pub_=nh_.advertise<std_msgs::Float32MultiArray>("ibvs_control_output", 1);

	while (ros::ok()){

		// Update the local variables
		VariablesandDerivatives(drone_pose, drone_velocity,angles,angles_derivatives,3);
		VariablesandDerivatives(drone_pose, drone_velocity,position,linear_vel,0);
		//Compute the angular velocity
		angular_vel = fromAngleDerivatives2AngularVelocity(angles_derivatives,angles);
		// Compute the Jacobian matrices
		Ji = computeImageJacobian(visual_features,drone_pose);
		Jcd = computeDrone2CameraJacobian(drone2cameraPose);
		J = Ji*Jcd;

		// Take the velocity and angular part of the Jacobian (both dimension 2x3)
		Jv = J.topLeftCorner(2,3);
		Jw = J.topRightCorner(2,3);
		// Computes the pseudo inverse of Jv with dimension 3x2
		Jvps = PseudoInverse(Jv);

		// Computes the derivatives of the image features
		imgefd = Jv*linear_vel + Jw* angular_vel;
		// Assigning the virtual features to the eigen vector
		imagef(0) = visual_features[0];
		imagef(1) = visual_features[1];

		// Takes the error in velocity (ex,ey,ez)
		error = Jvps*(-K*imagef-imgefd);

		// Compute the IBVS control outputs
		control_angles[0] = asin((mass*Ka/Thrust)*(error[0]/cos(drone_pose[3])));
		control_angles[1] = asin((mass*Ka/Thrust)*error[1]);

		// Insert the value of the control outputs in the ros message
		msg.data.insert(msg.data.end(), control_angles.begin(), control_angles.end());
		// Publish the data
		control_output_pub_.publish(msg);
		// Clearing the data for the next iteration
		msg.data.clear();
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
